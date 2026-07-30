/*
 * parameter_store.c - Power-loss-tolerant STM32F722 parameter persistence.
 *
 * This module stores one fixed 48-byte v1 record in each of Flash sectors 6
 * and 7. A save erases only the inactive sector, writes and verifies the
 * record body, then programs the commit marker last. The previous sector
 * therefore remains bootable through erase/program interruption.
 *
 * Record validation checks commit, magic, version, length, sequence, known
 * flags, finite bounded acceleration bias and IEEE CRC-32. Startup selects the
 * newer valid sequence, recovers the remaining valid slot after partial
 * writes, or exposes safe defaults with an explicit corrupt state.
 *
 * The module uses STM32 HAL Flash primitives and linker-exported slot
 * boundaries. It has no RTOS dependency and performs no dynamic allocation.
 */
#include "storage/parameter_store.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "stm32f7xx_hal.h"

#define PARAMETER_RECORD_MAGIC 0x47465052UL
#define PARAMETER_RECORD_VERSION 1U
#define PARAMETER_RECORD_COMMIT 0x434F4D54UL
#define PARAMETER_RECORD_FLAG_ACCEL_CALIBRATED (1UL << 0U)
#define PARAMETER_RECORD_KNOWN_FLAGS \
    PARAMETER_RECORD_FLAG_ACCEL_CALIBRATED
#define PARAMETER_MAX_ACCEL_BIAS_M_S2 (0.2f * 9.80665f)
#define PARAMETER_SLOT_A_INVALID_MASK (1U << 0U)
#define PARAMETER_SLOT_B_INVALID_MASK (1U << 1U)
#define PARAMETER_CACHE_LINE_SIZE 32U

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
    parameter_record_v1_t record;
    bool blank;
    bool valid;
} parameter_slot_record_t;

extern const uint8_t __parameter_slot_a_start__[];
extern const uint8_t __parameter_slot_a_end__[];
extern const uint8_t __parameter_slot_b_start__[];
extern const uint8_t __parameter_slot_b_end__[];

_Static_assert(sizeof(parameter_record_v1_t) == 48U,
               "parameter record v1 layout changed");
_Static_assert(offsetof(parameter_record_v1_t, crc32) == 40U,
               "parameter record CRC coverage changed");
_Static_assert(offsetof(parameter_record_v1_t, commit) == 44U,
               "parameter record commit offset changed");

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
    return slot == PARAMETER_STORE_SLOT_A
               ? FLASH_SECTOR_6
               : FLASH_SECTOR_7;
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
            const uint32_t mask =
                (uint32_t)-(int32_t)(crc & 1U);
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

static bool record_body_is_valid(const parameter_record_v1_t *record)
{
    uint32_t reserved_index;

    if ((record->magic != PARAMETER_RECORD_MAGIC) ||
        (record->version != PARAMETER_RECORD_VERSION) ||
        (record->length != sizeof(*record)) ||
        (record->sequence == 0U) ||
        ((record->flags & ~PARAMETER_RECORD_KNOWN_FLAGS) != 0U)) {
        return false;
    }

    for (reserved_index = 0U;
         reserved_index <
         (sizeof(record->reserved) / sizeof(record->reserved[0]));
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

static bool record_is_valid(const parameter_record_v1_t *record)
{
    return (record->commit == PARAMETER_RECORD_COMMIT) &&
           record_body_is_valid(record);
}

static void invalidate_record_cache(parameter_store_slot_t slot)
{
    const uint32_t address =
        slot_address(slot) & ~(PARAMETER_CACHE_LINE_SIZE - 1U);
    const int32_t length =
        (int32_t)((sizeof(parameter_record_v1_t) +
                   PARAMETER_CACHE_LINE_SIZE - 1U) &
                  ~(PARAMETER_CACHE_LINE_SIZE - 1U));

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)(uintptr_t)address,
                                    length);
    }
    __DSB();
    __ISB();
}

static void read_slot(parameter_store_slot_t slot,
                      parameter_slot_record_t *slot_record)
{
    const uint32_t address = slot_address(slot);

    invalidate_record_cache(slot);
    memcpy(&slot_record->record,
           (const void *)(uintptr_t)address,
           sizeof(slot_record->record));
    slot_record->blank =
        bytes_are_erased(&slot_record->record,
                         sizeof(slot_record->record));
    slot_record->valid = record_is_valid(&slot_record->record);
}

static bool sequence_is_newer(uint32_t candidate, uint32_t reference)
{
    return (int32_t)(candidate - reference) > 0;
}

static void values_from_record(const parameter_record_v1_t *record,
                               parameter_store_values_t *values)
{
    memset(values, 0, sizeof(*values));
    if ((record->flags &
         PARAMETER_RECORD_FLAG_ACCEL_CALIBRATED) != 0U) {
        values->accel_calibration_valid = true;
        memcpy(values->accel_bias_m_s2,
               record->accel_bias_m_s2,
               sizeof(values->accel_bias_m_s2));
    }
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

    memset(&current_values, 0, sizeof(current_values));
    memset(&current_status, 0, sizeof(current_status));
    current_status.last_save_result =
        PARAMETER_STORE_SAVE_NOT_ATTEMPTED;

    if (!layout_valid) {
        current_status.load_result =
            PARAMETER_STORE_LOAD_DEFAULTS_CORRUPT;
        current_status.storage_valid = false;
        current_status.invalid_slot_mask =
            PARAMETER_SLOT_A_INVALID_MASK |
            PARAMETER_SLOT_B_INVALID_MASK;
        return;
    }

    read_slot(PARAMETER_STORE_SLOT_A, &slot_a);
    read_slot(PARAMETER_STORE_SLOT_B, &slot_b);

    if (!slot_a.blank && !slot_a.valid) {
        current_status.invalid_slot_mask |=
            PARAMETER_SLOT_A_INVALID_MASK;
    }
    if (!slot_b.blank && !slot_b.valid) {
        current_status.invalid_slot_mask |=
            PARAMETER_SLOT_B_INVALID_MASK;
    }

    if (slot_a.valid && slot_b.valid) {
        if (sequence_is_newer(slot_b.record.sequence,
                              slot_a.record.sequence)) {
            current_status.active_slot = PARAMETER_STORE_SLOT_B;
            current_status.load_result = PARAMETER_STORE_LOAD_SLOT_B;
            current_status.sequence = slot_b.record.sequence;
            values_from_record(&slot_b.record, &current_values);
        } else {
            current_status.active_slot = PARAMETER_STORE_SLOT_A;
            current_status.load_result = PARAMETER_STORE_LOAD_SLOT_A;
            current_status.sequence = slot_a.record.sequence;
            values_from_record(&slot_a.record, &current_values);
        }
        current_status.storage_valid = true;
        return;
    }

    if (slot_a.valid) {
        current_status.active_slot = PARAMETER_STORE_SLOT_A;
        current_status.load_result =
            (!slot_b.blank && !slot_b.valid)
                ? PARAMETER_STORE_LOAD_RECOVERED_SLOT_A
                : PARAMETER_STORE_LOAD_SLOT_A;
        current_status.sequence = slot_a.record.sequence;
        current_status.storage_valid = true;
        values_from_record(&slot_a.record, &current_values);
        return;
    }

    if (slot_b.valid) {
        current_status.active_slot = PARAMETER_STORE_SLOT_B;
        current_status.load_result =
            (!slot_a.blank && !slot_a.valid)
                ? PARAMETER_STORE_LOAD_RECOVERED_SLOT_B
                : PARAMETER_STORE_LOAD_SLOT_B;
        current_status.sequence = slot_b.record.sequence;
        current_status.storage_valid = true;
        values_from_record(&slot_b.record, &current_values);
        return;
    }

    current_status.storage_valid = slot_a.blank && slot_b.blank;
    current_status.load_result =
        current_status.storage_valid
            ? PARAMETER_STORE_LOAD_DEFAULTS_EMPTY
            : PARAMETER_STORE_LOAD_DEFAULTS_CORRUPT;
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
                         parameter_record_v1_t *record)
{
    memset(record, 0, sizeof(*record));
    record->magic = PARAMETER_RECORD_MAGIC;
    record->version = PARAMETER_RECORD_VERSION;
    record->length = sizeof(*record);
    record->sequence = sequence;
    if (values->accel_calibration_valid) {
        record->flags |=
            PARAMETER_RECORD_FLAG_ACCEL_CALIBRATED;
        memcpy(record->accel_bias_m_s2,
               values->accel_bias_m_s2,
               sizeof(record->accel_bias_m_s2));
    }
    record->crc32 =
        crc32_ieee(record, offsetof(parameter_record_v1_t, crc32));
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
                                const parameter_record_v1_t *record)
{
    const uint32_t address = slot_address(slot);
    const uint32_t *words = (const uint32_t *)record;
    const size_t word_count =
        offsetof(parameter_record_v1_t, commit) / sizeof(uint32_t);
    size_t index;

    for (index = 0U; index < word_count; ++index) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              address +
                                  (uint32_t)(index * sizeof(uint32_t)),
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
               slot_address(slot) +
                   offsetof(parameter_record_v1_t, commit),
               PARAMETER_RECORD_COMMIT) == HAL_OK;
}

parameter_store_save_result_t parameter_store_save(
    const parameter_store_values_t *values)
{
    parameter_record_v1_t record;
    parameter_slot_record_t verify;
    const parameter_store_slot_t target_slot =
        current_status.active_slot == PARAMETER_STORE_SLOT_A
            ? PARAMETER_STORE_SLOT_B
            : PARAMETER_STORE_SLOT_A;
    uint32_t next_sequence;

    if ((values == NULL) ||
        (values->accel_calibration_valid &&
         !acceleration_bias_is_valid(values->accel_bias_m_s2))) {
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
    memcpy(&verify.record,
           (const void *)(uintptr_t)slot_address(target_slot),
           sizeof(verify.record));
    if (!record_body_is_valid(&verify.record) ||
        (verify.record.commit != UINT32_MAX)) {
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
        (verify.record.sequence != next_sequence)) {
        note_save_failure(PARAMETER_STORE_SAVE_VERIFY_FAILED);
        return current_status.last_save_result;
    }

    current_values = *values;
    current_status.storage_valid = true;
    current_status.active_slot = target_slot;
    current_status.sequence = next_sequence;
    current_status.invalid_slot_mask &=
        target_slot == PARAMETER_STORE_SLOT_A
            ? (uint8_t)~PARAMETER_SLOT_A_INVALID_MASK
            : (uint8_t)~PARAMETER_SLOT_B_INVALID_MASK;
    current_status.last_save_result = PARAMETER_STORE_SAVE_OK;
    current_status.last_hal_error = HAL_FLASH_ERROR_NONE;
    return PARAMETER_STORE_SAVE_OK;
}
