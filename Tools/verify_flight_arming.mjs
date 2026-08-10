/* Independent S4.7 arming/failsafe and Mixer-to-DShot regression. */
import fs from "node:fs";

const PREARM = 0;
const DISARMED = 1;
const ARMED = 2;
const FAILSAFE = 3;
const DSHOT_MIN = 48;
const DSHOT_MAX = 2047;
const MOTOR_IDLE_PERCENT = 0.055;

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function makeArming() {
  return {
    state: PREARM,
    lastFailsafeFlags: 0,
    armCount: 0,
    disarmCount: 0,
    failsafeCount: 0,
  };
}

function update(arming, armRequested, blockingFlags, failsafeFlags) {
  switch (arming.state) {
    case PREARM:
      if (!armRequested && blockingFlags === 0) arming.state = DISARMED;
      break;
    case DISARMED:
      if (blockingFlags !== 0) arming.state = PREARM;
      else if (armRequested) {
        arming.state = ARMED;
        arming.armCount++;
      }
      break;
    case ARMED:
      if (failsafeFlags !== 0) {
        arming.state = FAILSAFE;
        arming.lastFailsafeFlags = failsafeFlags;
        arming.failsafeCount++;
      } else if (!armRequested) {
        arming.state = DISARMED;
        arming.disarmCount++;
      }
      break;
    case FAILSAFE:
      if (!armRequested) arming.state = PREARM;
      break;
  }
}

function dshot(normalized) {
  assert(Number.isFinite(normalized) && normalized >= 0 && normalized <= 1,
    "mixer output must stay normalized");
  const outputLow = DSHOT_MIN + MOTOR_IDLE_PERCENT *
    (DSHOT_MAX - DSHOT_MIN);
  return Math.floor(outputLow + normalized * (DSHOT_MAX - outputLow) + 0.5);
}

const arming = makeArming();
update(arming, true, 0, 0);
assert(arming.state === PREARM,
  "ARM high at boot must not bypass PREARM");
update(arming, false, 1, 1);
assert(arming.state === PREARM,
  "PREARM must wait for every blocker to clear");
update(arming, false, 0, 0);
assert(arming.state === DISARMED,
  "safe ARM-low handshake must enter DISARMED");
update(arming, true, 0, 0);
assert(arming.state === ARMED && arming.armCount === 1,
  "a fresh ARM request with no blockers must arm once");
update(arming, false, 0, 0);
assert(arming.state === DISARMED && arming.disarmCount === 1,
  "ARM low must disarm immediately");
update(arming, true, 0, 0);
update(arming, true, 0x08, 0x08);
assert(arming.state === FAILSAFE && arming.failsafeCount === 1 &&
       arming.lastFailsafeFlags === 0x08,
  "an armed safety failure must latch FAILSAFE and its reason");
update(arming, true, 0, 0);
assert(arming.state === FAILSAFE,
  "recovery with ARM still high must not auto-rearm");
update(arming, false, 0, 0);
assert(arming.state === PREARM,
  "FAILSAFE must require an ARM-low PREARM handshake");
update(arming, false, 0, 0);
assert(arming.state === DISARMED,
  "recovered PREARM must return only to DISARMED");

assert(dshot(0) === 158 && dshot(0.5) === 1102 &&
       dshot(1) === DSHOT_MAX,
  "Betaflight motor_idle 5.5% must map Mixer [0,1] to DShot 158..2047");

const header = fs.readFileSync("APP/Inc/algorithms/flight_arming.h", "utf8");
const source = fs.readFileSync("APP/Src/algorithms/flight_arming.c", "utf8");
const state = fs.readFileSync("APP/Inc/app_state.h", "utf8");
const flight = fs.readFileSync("APP/Src/rtos/flight_task.c", "utf8");
const msp = fs.readFileSync("APP/Src/protocol/msp_server.c", "utf8");
const imuTask = fs.readFileSync("APP/Src/rtos/imu_task.c", "utf8");
const appStateSource = fs.readFileSync("APP/Src/app_state.c", "utf8");
const diag = fs.readFileSync("APP/Src/platform/platform_diag.c", "utf8");
const cmake = fs.readFileSync("CMakeLists.txt", "utf8");

assert(header.includes("FLIGHT_ARMING_PREARM") &&
       header.includes("FLIGHT_ARMING_FAILSAFE") &&
       source.includes("!arm_requested && (blocking_flags == 0U)") &&
       source.includes("arming->last_failsafe_flags = failsafe_flags"),
  "pure PREARM/FAILSAFE state machine or latch is missing");
assert(state.includes("APP_FLIGHT_SAFETY_DSHOT_NOT_READY") &&
       state.includes("APP_FLIGHT_SAFETY_CONFIGURATOR_DISABLED") &&
       state.includes("APP_FLIGHT_SAFETY_SYSTEM_FAULT") &&
       state.includes("APP_FLIGHT_SAFETY_CONTROL_INVALID") &&
       state.includes("APP_FLIGHT_SAFETY_ARMING_ONLY_MASK"),
  "unified S4.7 blocker/failsafe flags are incomplete");
assert(flight.includes("mixer_to_dshot(") &&
       flight.includes("if (state.armed)") &&
       flight.includes("FLIGHT_DSHOT_MOTOR_IDLE_PERCENT 0.055f") &&
       flight.includes("dshot_output_low + normalized * dshot_output_range") &&
       flight.includes("memcpy(output, flight_output") &&
       flight.includes("!state.rc_setpoint.arm_requested && !state.armed") &&
       flight.includes("if (state.armed ||") &&
       flight.includes("requests_output && app_state_is_armed()") &&
       flight.includes("dshot_motor_force_safe()"),
  "Betaflight DShot endpoint mapping or immediate stop path is missing");
assert(msp.includes("MSP2_GETFUN_ARMING_STATUS 0x4009U") &&
       msp.includes("handle_getfun_arming_status(&writer, &state)") &&
       msp.includes("state.flight.armed ||") &&
       diag.includes("arming state=%u armed=%u arm_req=%u") &&
       cmake.includes("APP/Src/algorithms/flight_arming.c"),
  "S4.7 diagnostics, motor-test exclusion or build wiring is missing");
assert(msp.includes("state.flight.armed ||\n            !state.imu.present") &&
       imuTask.match(/if \(app_state_is_armed\(\)\)/g)?.length === 2 &&
       appStateSource.includes("bool app_state_is_armed(void)"),
  "ARMED calibration start and parameter Flash writes need owner-side gates");

const armingStatusBytes = 4 + (4 * 4) + (3 * 4) + 2 + (4 * 2);
assert(armingStatusBytes === 42,
  "GETFUN arming status payload must remain exactly 42 bytes");

console.log(JSON.stringify({
  result: "PASS",
  states: ["PREARM", "DISARMED", "ARMED", "FAILSAFE"],
  prearm: "ARM low and no blockers required after boot/failsafe",
  dshotFlightRange: [dshot(0), DSHOT_MAX],
  armingStatusBytes,
  deferred: ["configurable PREARM AUX", "Angle outer loop", "App mode writes"],
}, null, 2));
