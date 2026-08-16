/*
 * parameter_store.c - Power-loss-tolerant S4.9 parameter persistence.
 *
 * Sector 6/7 remain an A/B, commit-last store. New saves write a fixed v2
 * record containing accel calibration plus RC, Rate PID, Angle, motor idle and
 * App names. The loader still accepts the frozen 48-byte v1 record, keeps its
 * accel bias, fills every new field from safe defaults and marks migration
 * pending until the next successful save. No in-place migration is attempted.
 *
 * The module owns CRC, validation, erase/program/verify and sequence ordering.
 * It has no RTOS dependency; ImuTask remains the only caller allowed to save.
 */
#include "storage/parameter_store.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "stm32f7xx_hal.h"

#define PARAMETER_RECORD_MAGIC 0x47465052UL
#define PARAMETER_RECORD_VERSION_V1 1U
#define PARAMETER_RECORD_VERSION_V2 2U
#define PARAMETER_RECORD_COMMIT 0x434F4D54UL
#define PARAMETER_RECORD_FLAG_ACCEL_CALIBRATED (1UL << 0U)
#define PARAMETER_RECORD_KNOWN_FLAGS PARAMETER_RECORD_FLAG_ACCEL_CALIBRATED
#define PARAMETER_MAX_ACCEL_BIAS_M_S2 (0.2f * 9.80665f)
#define PARAMETER_SLOT_A_INVALID_MASK (1U << 0U)
#define PARAMETER_SLOT_B_INVALID_MASK (1U << 1U)
#define PARAMETER_CACHE_LINE_SIZE 32U
#define PARAMETER_MOTOR_IDLE_MAX 2000U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    uint32_t flags;
    float accel_bias_m_s2[PARAMETER_STORE_AXIS_COUNT];
    uint32_t reserved[3];
    uint32_t crc32;
    uint32_t commit;
} parameter_record_v1_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    uint32_t flags;
    float accel_bias_m_s2[PARAMETER_STORE_AXIS_COUNT];
    rc_setpoint_profile_t rc_profile;
    rate_pid_profile_t rate_pid_profile;
    angle_outer_loop_profile_t angle_profile;
    uint16_t motor_idle_percent_x100;
    uint16_t reserved_u16;
    char craft_name[PARAMETER_STORE_NAME_LENGTH];
    char pilot_name[PARAMETER_STORE_NAME_LENGTH];
    uint32_t reserved[2];
    uint32_t crc32;
    uint32_t commit;
} parameter_record_v2_t;

typedef union
{
    parameter_record_v1_t v1;
    parameter_record_v2_t v2;
    uint8_t bytes[sizeof(parameter_record_v2_t)];
} parameter_record_any_t;

typedef struct
{
    parameter_record_any_t record;
    bool blank;
    bool valid;
    uint16_t version;
    uint32_t sequence;
} parameter_slot_record_t;

extern const uint8_t __parameter_slot_a_start__[];
extern const uint8_t __parameter_slot_a_end__[];
extern const uint8_t __parameter_slot_b_start__[];
extern const uint8_t __parameter_slot_b_end__[];

_Static_assert(sizeof(parameter_record_v1_t) == 48U,
               "parameter record v1 layout changed");
_Static_assert(offsetof(parameter_record_v1_t, crc32) == 40U,
               "parameter record v1 CRC coverage changed");
_Static_assert(offsetof(parameter_record_v1_t, commit) == 44U,
               "parameter record v1 commit offset changed");
_Static_assert((sizeof(parameter_record_v2_t) % sizeof(uint32_t)) == 0U,
               "parameter record v2 must be word aligned");
_Static_assert((offsetof(parameter_record_v2_t, commit) %
                sizeof(uint32_t)) == 0U,
               "parameter record v2 commit must be word aligned");

static parameter_store_values_t current_values;
static parameter_store_status_t current_status;

static uint32_t slot_address(parameter_store_slot_t slot)
{
    return slot == PARAMETER_STORE_SLOT_A
               ? (uint32_t)(uintptr_t)__parameter_slot_a_start__
               : (uint32_t)(uintptr_t)__parameter_slot_b_start__;
}

static uint32_t slot_end_address(parameter_store_slot_t slot)
{
    return slot == PARAMETER_STORE_SLOT_A
               ? (uint32_t)(uintptr_t)__parameter_slot_a_end__
               : (uint32_t)(uintptr_t)__parameter_slot_b_end__;
}

static uint32_t slot_sector(parameter_store_slot_t slot)
{
    return slot == PARAMETER_STORE_SLOT_A ? FLASH_SECTOR_6 : FLASH_SECTOR_7;
}

static uint32_t crc32_ieee(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = UINT32_MAX;
    size_t index;

    for (index = 0U; index < length; ++index) {
        uint8_t bit;

        crc ^= bytes[index];
        for (bit = 0U; bit < 8U; ++bit) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static bool bytes_are_erased(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    size_t index;

    for (index = 0U; index < length; ++index) {
        if (bytes[index] != 0xFFU) {
            return false;
        }
    }
    return true;
}

static bool acceleration_bias_is_valid(
    const float bias_m_s2[PARAMETER_STORE_AXIS_COUNT])
{
    uint32_t axis;

    for (axis = 0U; axis < PARAMETER_STORE_AXIS_COUNT; ++axis) {
        if (!isfinite(bias_m_s2[axis]) ||
            (fabsf(bias_m_s2[axis]) > PARAMETER_MAX_ACCEL_BIAS_M_S2)) {
            return false;
        }
    }
    return true;
}

static bool name_is_valid(const char name[PARAMETER_STORE_NAME_LENGTH + 1U])
{
    bool terminated = false;
    uint32_t index;

    if (name[PARAMETER_STORE_NAME_LENGTH] != '\0') {
        return false;
    }
    for (index = 0U; index < PARAMETER_STORE_NAME_LENGTH; ++index) {
        const uint8_t value = (uint8_t)name[index];

        if (terminated) {
            if (value != 0U) {
                return false;
            }
        } else if (value == 0U) {
            terminated = true;
        } else if ((value < 0x20U) || (value > 0x7EU)) {
            return false;
        }
    }
    return true;
}

void parameter_store_values_set_defaults(parameter_store_values_t *values)
{
    static const char default_craft_name[] = "GETFUN F722";

    if (values == NULL) {
        return;
    }
    memset(values, 0, sizeof(*values));
    values->rc_profile = *rc_setpoint_default_profile();
    values->rate_pid_profile = *rate_pid_default_profile();
    values->angle_profile = *angle_outer_loop_default_profile();
    values->motor_idle_percent_x100 = PARAMETER_STORE_MOTOR_IDLE_DEFAULT;
    memcpy(values->craft_name, default_craft_name,
           sizeof(default_craft_name));
}

bool parameter_store_values_are_valid(const parameter_store_values_t *values)
{
    return (values != NULL) &&
           (!values->accel_calibration_valid ||
            acceleration_bias_is_valid(values->accel_bias_m_s2)) &&
           rc_setpoint_profile_is_valid(&values->rc_profile) &&
           rate_pid_profile_is_valid(&values->rate_pid_profile) &&
           angle_outer_loop_profile_is_valid(&values->angle_profile) &&
           (values->motor_idle_percent_x100 <= PARAMETER_MOTOR_IDLE_MAX) &&
           name_is_valid(values->craft_name) &&
           name_is_valid(values->pilot_name);
}

static bool record_v1_body_is_valid(const parameter_record_v1_t *record)
{
    uint32_t reserved_index;

    if ((record->magic != PARAMETER_RECORD_MAGIC) ||
        (record->version != PARAMETER_RECORD_VERSION_V1) ||
        (record->length != sizeof(*record)) ||
        (record->sequence == 0U) ||
        ((record->flags & ~PARAMETER_RECORD_KNOWN_FLAGS) != 0U)) {
        return false;
    }
    for (reserved_index = 0U;
         reserved_index < sizeof(record->reserved) / sizeof(record->reserved[0]);
         ++reserved_index) {
        if (record->reserved[reserved_index] != 0U) {
            return false;
        }
    }
    if (((record->flags & PARAMETER_RECORD_FLAG_ACCEL_CALIBRATED) != 0U) &&
        !acceleration_bias_is_valid(record->accel_bias_m_s2)) {
        return false;
    }
    return record->crc32 ==
           crc32_ieee(record, offsetof(parameter_record_v1_t, crc32));
}

static void values_from_v2(const parameter_record_v2_t *record,
                           parameter_store_values_t *values)
{
    parameter_store_values_set_defaults(values);
    values->accel_calibration_valid =
        (record->flags & PARAMETER_RECORD_FLAG_ACCEL_CALIBRATED) != 0U;
    memcpy(values->accel_bias_m_s2, record->accel_bias_m_s2,
           sizeof(values->accel_bias_m_s2));
    values->rc_profile = record->rc_profile;
    values->rate_pid_profile = record->rate_pid_profile;
    values->angle_profile = record->angle_profile;
    values->motor_idle_percent_x100 = record->motor_idle_percent_x100;
    memset(values->craft_name, 0, sizeof(values->craft_name));
    memset(values->pilot_name, 0, sizeof(values->pilot_name));
    memcpy(values->craft_name, record->craft_name,
           sizeof(record->craft_name));
    memcpy(values->pilot_name, record->pilot_name,
           sizeof(record->pilot_name));
}

static bool record_v2_body_is_valid(const parameter_record_v2_t *record)
{
    parameter_store_values_t values;
    uint32_t reserved_index;

    if ((record->magic != PARAMETER_RECORD_MAGIC) ||
        (record->version != PARAMETER_RECORD_VERSION_V2) ||
        (record->length != sizeof(*record)) ||
        (record->sequence == 0U) ||
        ((record->flags & ~PARAMETER_RECORD_KNOWN_FLAGS) != 0U) ||
        (record->reserved_u16 != 0U)) {
        return false;
    }
    for (reserved_index = 0U;
         reserved_index < sizeof(record->reserved) / sizeof(record->reserved[0]);
         ++reserved_index) {
        if (record->reserved[reserved_index] != 0U) {
            return false;
        }
    }
    values_from_v2(record, &values);
    return parameter_store_values_are_valid(&values) &&
           (record->crc32 ==
            crc32_ieee(record, offsetof(parameter_record_v2_t, crc32)));
}

static void invalidate_record_cache(parameter_store_slot_t slot)
{
    const uint32_t address =
        slot_address(slot) & ~(PARAMETER_CACHE_LINE_SIZE - 1U);
    const int32_t length = (int32_t)((sizeof(parameter_record_v2_t) +
                                      PARAMETER_CACHE_LINE_SIZE - 1U) &
                                     ~(PARAMETER_CACHE_LINE_SIZE - 1U));

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)(uintptr_t)address, length);
    }
    __DSB();
    __ISB();
}

static void read_slot(parameter_store_slot_t slot,
                      parameter_slot_record_t *slot_record)
{
    const uint32_t address = slot_address(slot);
    const parameter_record_v1_t *v1 = &slot_record->record.v1;
    const parameter_record_v2_t *v2 = &slot_record->record.v2;

    memset(slot_record, 0, sizeof(*slot_record));
    invalidate_record_cache(slot);
    memcpy(slot_record->record.bytes, (const void *)(uintptr_t)address,
           sizeof(slot_record->record.bytes));
    slot_record->blank = bytes_are_erased(slot_record->record.bytes,
                                          sizeof(slot_record->record.bytes));
    if ((v2->version == PARAMETER_RECORD_VERSION_V2) &&
        (v2->commit == PARAMETER_RECORD_COMMIT) &&
        record_v2_body_is_valid(v2)) {
        slot_record->valid = true;
        slot_record->version = PARAMETER_RECORD_VERSION_V2;
        slot_record->sequence = v2->sequence;
    } else if ((v1->version == PARAMETER_RECORD_VERSION_V1) &&
               (v1->commit == PARAMETER_RECORD_COMMIT) &&
               record_v1_body_is_valid(v1)) {
        slot_record->valid = true;
        slot_record->version = PARAMETER_RECORD_VERSION_V1;
        slot_record->sequence = v1->sequence;
    }
}

static bool sequence_is_newer(uint32_t candidate, uint32_t reference)
{
    return (int32_t)(candidate - reference) > 0;
}

static void values_from_slot(const parameter_slot_record_t *slot,
                             parameter_store_values_t *values)
{
    parameter_store_values_set_defaults(values);
    if (slot->version == PARAMETER_RECORD_VERSION_V2) {
        values_from_v2(&slot->record.v2, values);
    } else if ((slot->record.v1.flags &
                PARAMETER_RECORD_FLAG_ACCEL_CALIBRATED) != 0U) {
        values->accel_calibration_valid = true;
        memcpy(values->accel_bias_m_s2, slot->record.v1.accel_bias_m_s2,
               sizeof(values->accel_bias_m_s2));
    }
}

static void select_slot(const parameter_slot_record_t *slot,
                        parameter_store_slot_t slot_id,
                        parameter_store_load_result_t load_result)
{
    current_status.active_slot = slot_id;
    current_status.load_result = load_result;
    current_status.sequence = slot->sequence;
    current_status.loaded_record_version = slot->version;
    current_status.migration_pending =
        slot->version == PARAMETER_RECORD_VERSION_V1;
    current_status.storage_valid = true;
    values_from_slot(slot, &current_values);
}

static void increment_saturating(uint32_t *counter)
{
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static void note_save_failure(parameter_store_save_result_t result)
{
    current_status.last_save_result = result;
    current_status.last_hal_error = HAL_FLASH_GetError();
    increment_saturating(&current_status.save_error_count);
}

void parameter_store_init(void)
{
    parameter_slot_record_t slot_a;
    parameter_slot_record_t slot_b;
    const bool layout_valid =
        (slot_address(PARAMETER_STORE_SLOT_A) == 0x08040000UL) &&
        (slot_end_address(PARAMETER_STORE_SLOT_A) == 0x08060000UL) &&
        (slot_address(PARAMETER_STORE_SLOT_B) == 0x08060000UL) &&
        (slot_end_address(PARAMETER_STORE_SLOT_B) == 0x08080000UL);

    parameter_store_values_set_defaults(&current_values);
    memset(&current_status, 0, sizeof(current_status));
    current_status.last_save_result = PARAMETER_STORE_SAVE_NOT_ATTEMPTED;

    if (!layout_valid) {
        current_status.load_result = PARAMETER_STORE_LOAD_DEFAULTS_CORRUPT;
        current_status.invalid_slot_mask =
            PARAMETER_SLOT_A_INVALID_MASK | PARAMETER_SLOT_B_INVALID_MASK;
        return;
    }
    read_slot(PARAMETER_STORE_SLOT_A, &slot_a);
    read_slot(PARAMETER_STORE_SLOT_B, &slot_b);
    if (!slot_a.blank && !slot_a.valid) {
        current_status.invalid_slot_mask |= PARAMETER_SLOT_A_INVALID_MASK;
    }
    if (!slot_b.blank && !slot_b.valid) {
        current_status.invalid_slot_mask |= PARAMETER_SLOT_B_INVALID_MASK;
    }

    if (slot_a.valid && slot_b.valid) {
        if (sequence_is_newer(slot_b.sequence, slot_a.sequence)) {
            select_slot(&slot_b, PARAMETER_STORE_SLOT_B,
                        PARAMETER_STORE_LOAD_SLOT_B);
        } else {
            select_slot(&slot_a, PARAMETER_STORE_SLOT_A,
                        PARAMETER_STORE_LOAD_SLOT_A);
        }
    } else if (slot_a.valid) {
        select_slot(&slot_a, PARAMETER_STORE_SLOT_A,
                    (!slot_b.blank && !slot_b.valid)
                        ? PARAMETER_STORE_LOAD_RECOVERED_SLOT_A
                        : PARAMETER_STORE_LOAD_SLOT_A);
    } else if (slot_b.valid) {
        select_slot(&slot_b, PARAMETER_STORE_SLOT_B,
                    (!slot_a.blank && !slot_a.valid)
                        ? PARAMETER_STORE_LOAD_RECOVERED_SLOT_B
                        : PARAMETER_STORE_LOAD_SLOT_B);
    } else {
        current_status.storage_valid = slot_a.blank && slot_b.blank;
        current_status.load_result = current_status.storage_valid
                                         ? PARAMETER_STORE_LOAD_DEFAULTS_EMPTY
                                         : PARAMETER_STORE_LOAD_DEFAULTS_CORRUPT;
    }
}

void parameter_store_get_values(parameter_store_values_t *values)
{
    if (values != NULL) {
        *values = current_values;
    }
}

void parameter_store_get_status(parameter_store_status_t *status)
{
    if (status != NULL) {
        *status = current_status;
    }
}

static void build_record(const parameter_store_values_t *values,
                         uint32_t sequence,
                         parameter_record_v2_t *record)
{
    memset(record, 0, sizeof(*record));
    record->magic = PARAMETER_RECORD_MAGIC;
    record->version = PARAMETER_RECORD_VERSION_V2;
    record->length = sizeof(*record);
    record->sequence = sequence;
    if (values->accel_calibration_valid) {
        record->flags = PARAMETER_RECORD_FLAG_ACCEL_CALIBRATED;
    }
    memcpy(record->accel_bias_m_s2, values->accel_bias_m_s2,
           sizeof(record->accel_bias_m_s2));
    record->rc_profile = values->rc_profile;
    record->rate_pid_profile = values->rate_pid_profile;
    record->angle_profile = values->angle_profile;
    record->motor_idle_percent_x100 = values->motor_idle_percent_x100;
    memcpy(record->craft_name, values->craft_name,
           sizeof(record->craft_name));
    memcpy(record->pilot_name, values->pilot_name,
           sizeof(record->pilot_name));
    record->crc32 =
        crc32_ieee(record, offsetof(parameter_record_v2_t, crc32));
    record->commit = PARAMETER_RECORD_COMMIT;
}

static bool erase_slot(parameter_store_slot_t slot)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = UINT32_MAX;

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = slot_sector(slot);
    erase.NbSectors = 1U;
    return HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;
}

static bool program_record_body(parameter_store_slot_t slot,
                                const parameter_record_v2_t *record)
{
    const uint32_t address = slot_address(slot);
    const uint32_t *words = (const uint32_t *)record;
    const size_t word_count =
        offsetof(parameter_record_v2_t, commit) / sizeof(uint32_t);
    size_t index;

    for (index = 0U; index < word_count; ++index) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              address + (uint32_t)(index * sizeof(uint32_t)),
                              words[index]) != HAL_OK) {
            return false;
        }
    }
    return true;
}

static bool program_commit(parameter_store_slot_t slot)
{
    return HAL_FLASH_Program(
               FLASH_TYPEPROGRAM_WORD,
               slot_address(slot) + offsetof(parameter_record_v2_t, commit),
               PARAMETER_RECORD_COMMIT) == HAL_OK;
}

parameter_store_save_result_t parameter_store_save(
    const parameter_store_values_t *values)
{
    parameter_record_v2_t record;
    parameter_slot_record_t verify;
    const parameter_store_slot_t target_slot =
        current_status.active_slot == PARAMETER_STORE_SLOT_A
            ? PARAMETER_STORE_SLOT_B
            : PARAMETER_STORE_SLOT_A;
    uint32_t next_sequence;

    if (!parameter_store_values_are_valid(values)) {
        note_save_failure(PARAMETER_STORE_SAVE_BAD_ARGUMENT);
        return current_status.last_save_result;
    }
    next_sequence = current_status.sequence + 1U;
    if (next_sequence == 0U) {
        next_sequence = 1U;
    }
    build_record(values, next_sequence, &record);
    if (HAL_FLASH_Unlock() != HAL_OK) {
        note_save_failure(PARAMETER_STORE_SAVE_FLASH_UNLOCK_FAILED);
        return current_status.last_save_result;
    }
    if (!erase_slot(target_slot)) {
        (void)HAL_FLASH_Lock();
        note_save_failure(PARAMETER_STORE_SAVE_ERASE_FAILED);
        return current_status.last_save_result;
    }
    if (!program_record_body(target_slot, &record)) {
        (void)HAL_FLASH_Lock();
        invalidate_record_cache(target_slot);
        note_save_failure(PARAMETER_STORE_SAVE_PROGRAM_FAILED);
        return current_status.last_save_result;
    }
    invalidate_record_cache(target_slot);
    memcpy(&verify.record.v2, (const void *)(uintptr_t)slot_address(target_slot),
           sizeof(verify.record.v2));
    if (!record_v2_body_is_valid(&verify.record.v2) ||
        (verify.record.v2.commit != UINT32_MAX)) {
        (void)HAL_FLASH_Lock();
        note_save_failure(PARAMETER_STORE_SAVE_VERIFY_FAILED);
        return current_status.last_save_result;
    }
    if (!program_commit(target_slot)) {
        (void)HAL_FLASH_Lock();
        invalidate_record_cache(target_slot);
        note_save_failure(PARAMETER_STORE_SAVE_PROGRAM_FAILED);
        return current_status.last_save_result;
    }
    (void)HAL_FLASH_Lock();
    read_slot(target_slot, &verify);
    if (!verify.valid ||
        (verify.version != PARAMETER_RECORD_VERSION_V2) ||
        (verify.sequence != next_sequence)) {
        note_save_failure(PARAMETER_STORE_SAVE_VERIFY_FAILED);
        return current_status.last_save_result;
    }

    current_values = *values;
    current_status.storage_valid = true;
    current_status.active_slot = target_slot;
    current_status.sequence = next_sequence;
    current_status.loaded_record_version = PARAMETER_RECORD_VERSION_V2;
    current_status.migration_pending = false;
    current_status.invalid_slot_mask &=
        target_slot == PARAMETER_STORE_SLOT_A
            ? (uint8_t)~PARAMETER_SLOT_A_INVALID_MASK
            : (uint8_t)~PARAMETER_SLOT_B_INVALID_MASK;
    current_status.last_save_result = PARAMETER_STORE_SAVE_OK;
    current_status.last_hal_error = HAL_FLASH_ERROR_NONE;
    return PARAMETER_STORE_SAVE_OK;
}
