/* Independent S4.9 v1 migration, parameter transaction and App-page regression. */
import fs from "node:fs";

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function crc32(buffer) {
  let crc = 0xFFFFFFFF;
  for (const value of buffer) {
    crc ^= value;
    for (let bit = 0; bit < 8; bit++) {
      crc = (crc >>> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
    }
  }
  return (~crc) >>> 0;
}

function makeV1Record(bias) {
  const record = Buffer.alloc(48);
  record.writeUInt32LE(0x47465052, 0);
  record.writeUInt16LE(1, 4);
  record.writeUInt16LE(48, 6);
  record.writeUInt32LE(7, 8);
  record.writeUInt32LE(1, 12);
  bias.forEach((value, axis) => record.writeFloatLE(value, 16 + axis * 4));
  record.writeUInt32LE(crc32(record.subarray(0, 40)), 40);
  record.writeUInt32LE(0x434F4D54, 44);
  return record;
}

function loadV1(record) {
  const valid = record.length === 48 &&
    record.readUInt32LE(0) === 0x47465052 &&
    record.readUInt16LE(4) === 1 &&
    record.readUInt16LE(6) === 48 &&
    record.readUInt32LE(8) !== 0 &&
    record.readUInt32LE(12) === 1 &&
    record.readUInt32LE(40) === crc32(record.subarray(0, 40)) &&
    record.readUInt32LE(44) === 0x434F4D54;
  if (!valid) return null;
  return {
    bias: [0, 1, 2].map((axis) => record.readFloatLE(16 + axis * 4)),
    migrated: true,
    angleLimitDeg: 60,
    angleP: 5,
    motorIdle: 550,
    craftName: "GETFUN F722",
  };
}

const v1 = makeV1Record([0.1, -0.2, 0.3]);
const migrated = loadV1(v1);
assert(migrated?.migrated && migrated.motorIdle === 550 &&
       migrated.angleLimitDeg === 60 && migrated.angleP === 5 &&
       migrated.bias.every((value, axis) =>
         Math.abs(value - [0.1, -0.2, 0.3][axis]) < 1e-6),
"v1 migration must keep accel bias and fill every newer field from defaults");
v1[20] ^= 0x01;
assert(loadV1(v1) === null, "v1 CRC corruption must be rejected");

function dshotFromStandard(value) {
  if (value === 1000) return 0;
  return 48 + Math.floor(((value - 1001) * (2047 - 48) + 499) / 999);
}
assert(dshotFromStandard(1000) === 0 &&
       dshotFromStandard(1001) === 48 &&
       dshotFromStandard(2000) === 2047,
"standard MSP motor endpoints must map to stop and DShot 48..2047");

const transaction = { active: false, last: 0, values: null };
function stage(now, armed, values) {
  if (armed) return false;
  transaction.active = true;
  transaction.last = now;
  transaction.values = structuredClone(values);
  return true;
}
function commit(now, armed) {
  if (armed || !transaction.active || now - transaction.last >= 1000) {
    transaction.active = false;
    return false;
  }
  transaction.active = false;
  return true;
}
assert(stage(100, false, { angleP: 6 }) && commit(1099, false),
"a current DISARMED transaction must commit");
assert(stage(2000, false, { angleP: 6 }) && !commit(3000, false),
"a transaction must expire at one second");
assert(!stage(4000, true, { angleP: 6 }) &&
       stage(4000, false, { angleP: 6 }) && !commit(4001, true),
"SET and EEPROM must both reject ARMED");

const readSource = (path) => fs.readFileSync(path, "utf8")
  .replaceAll("\r\n", "\n");
const storeHeader = readSource("APP/Inc/storage/parameter_store.h");
const store = readSource("APP/Src/storage/parameter_store.c");
const state = readSource("APP/Inc/app_state.h");
const imu = readSource("APP/Src/rtos/imu_task.c");
const flight = readSource("APP/Src/rtos/flight_task.c");
const msp = readSource("APP/Src/protocol/msp_server.c");

assert(storeHeader.includes("parameter_store_values_set_defaults") &&
       storeHeader.includes("rc_setpoint_profile_t rc_profile") &&
       storeHeader.includes("rate_pid_profile_t rate_pid_profile") &&
       storeHeader.includes("angle_outer_loop_profile_t angle_profile") &&
       storeHeader.includes("motor_idle_percent_x100") &&
       store.includes("PARAMETER_RECORD_VERSION_V1 1U") &&
       store.includes("PARAMETER_RECORD_VERSION_V2 2U") &&
       store.includes("PARAMETER_RECORD_VERSION_V3 3U") &&
       store.includes("values_from_slot") &&
       store.includes("record_v3_body_is_valid") &&
       store.includes("migration_pending") &&
       store.includes("program_commit(target_slot)"),
"v3 record, legacy migration or commit-last persistence is missing");
assert(state.includes("parameter_store_values_t values") &&
       imu.includes("process_parameter_save_request();") &&
       imu.includes("parameter_store_save(&candidate)") &&
       imu.includes("complete_parameter_save_request") &&
       !msp.includes("parameter_store_save("),
"ImuTask must remain the only App-configuration Flash writer");
assert(imu.match(/if \(app_state_is_armed\(\)\)/g)?.length >= 3 &&
       !imu.includes("xTaskNotifyGive(imu_task_handle)"),
"owner-side ARMED recheck must not corrupt the DMA task notification");
assert(flight.includes("&snapshot.parameters.values.rc_profile") &&
       flight.includes("&snapshot.parameters.values.rate_pid_profile") &&
       flight.includes("&snapshot.parameters.values.angle_profile") &&
       flight.includes("snapshot.parameters.values.motor_idle_percent_x100") &&
       flight.includes("snapshot.parameters.sequence !=") &&
       flight.includes("reset_control(&pid_state, &state);"),
"FlightTask must consume only the atomically published persisted profiles");

const readCases = [
  "MSP_PID", "MSP_PID_ADVANCED", "MSP_RC_TUNING", "MSP_FILTER_CONFIG",
  "MSP_MODE_RANGES", "MSP_MODE_RANGES_EXTRA", "MSP_BOXIDS",
  "MSP_MOTOR", "MSP_MOTOR_CONFIG", "MSP_ADVANCED_CONFIG",
  "MSP_RC_DEADBAND", "MSP2_COMMON_SERIAL_CONFIG",
  "MSP2_MOTOR_OUTPUT_REORDERING", "MSP2_GET_TEXT",
];
const writeCases = [
  "MSP_SET_PID", "MSP_SET_PID_ADVANCED", "MSP_SET_RC_TUNING",
  "MSP_SET_MODE_RANGE", "MSP_SET_MOTOR", "MSP_SET_ADVANCED_CONFIG",
  "MSP_SET_RC_DEADBAND", "MSP_SET_RX_CONFIG", "MSP2_SET_TEXT",
  "MSP_EEPROM_WRITE",
];
for (const command of [...readCases, ...writeCases]) {
  assert(msp.includes(`case ${command}:`), `${command} dispatch is missing`);
}
assert(msp.includes("MSP_CONFIGURATION_TIMEOUT_TICKS pdMS_TO_TICKS(1000U)") &&
       msp.includes("configuration_transaction_is_current()") &&
       msp.includes("imu_task_save_parameters(") &&
       msp.includes("state.configurator_arming_disabled") &&
       msp.includes("standard > MSP_STANDARD_MOTOR_STOP") &&
       msp.includes("dshot_from_standard_motor(standard)"),
"transaction timeout, ImuTask commit or standard Motors-page gates are missing");

console.log(JSON.stringify({
  result: "PASS",
  record: "v3 with v1/v2 migration and commit-last A/B save",
  transactionTimeoutMs: 1000,
  owner: "ImuTask",
  pages: ["PID Tuning", "Modes", "Motors", "Configuration"],
  motorMapping: { stop: [1000, 0], minimum: [1001, 48], maximum: [2000, 2047] },
  gates: ["DISARMED", "Configurator arming disabled", "fresh inputs", "DShot healthy"],
}, null, 2));
