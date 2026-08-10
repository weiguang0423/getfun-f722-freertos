/* Independent S4.4 RC normalization, Actual Rates and AUX mode regression. */
import fs from "node:fs";

const profile = {
  min: 1000,
  mid: 1500,
  max: 2000,
  deadband: 5,
  yawDeadband: 5,
  center: [7, 7, 7],
  maxRate: [67, 67, 67],
  expo: [0, 0, 0],
  armAux: 4,
  angleAux: 5,
  armRange: [1700, 2000],
  angleRange: [1700, 2000],
};

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function close(actual, expected, tolerance, message) {
  assert(Math.abs(actual - expected) <= tolerance,
    `${message}: expected ${expected}, got ${actual}`);
}

function clamp(value, minimum, maximum) {
  return Math.min(Math.max(value, minimum), maximum);
}

function normalizeAxis(value, deadband, config = profile) {
  const delta = clamp(value, config.min, config.max) - config.mid;
  const magnitude = Math.abs(delta);
  if (magnitude <= deadband) return 0;
  const span = delta < 0
    ? config.mid - config.min - deadband
    : config.max - config.mid - deadband;
  return Math.sign(delta) * Math.min((magnitude - deadband) / span, 1);
}

function actualRate(normalized, center, maximum, expoPercent) {
  const expo = expoPercent / 100;
  const curve = Math.abs(normalized) *
    (normalized ** 5 * expo + normalized * (1 - expo));
  const centerDps = center * 10;
  return normalized * centerDps + (maximum * 10 - centerDps) * curve;
}

function rangeActive(value, range, config = profile) {
  const bounded = clamp(value, config.min, config.max);
  return bounded >= range[0] && bounded <= range[1];
}

function validProfile(config) {
  const left = config.mid - config.min;
  const right = config.max - config.mid;
  return config.min < config.mid && config.mid < config.max &&
    config.deadband < left && config.deadband < right &&
    config.yawDeadband < left && config.yawDeadband < right &&
    config.armAux >= 4 && config.armAux < 16 &&
    config.angleAux >= 4 && config.angleAux < 16 &&
    config.armAux !== config.angleAux &&
    [config.armRange, config.angleRange].every((range) =>
      range[0] <= range[1] && range[0] >= config.min && range[1] <= config.max) &&
    config.center.every((value, axis) => value > 0 &&
      value <= config.maxRate[axis] && config.maxRate[axis] <= 200 &&
      config.expo[axis] <= 100);
}

function compute(channels, config = profile) {
  const stick = channels.slice(0, 3).map((value, axis) =>
    normalizeAxis(value, axis === 2 ? config.yawDeadband : config.deadband, config));
  return {
    stick,
    throttle: (clamp(channels[3], config.min, config.max) - config.min) /
      (config.max - config.min),
    rateDps: stick.map((value, axis) =>
      actualRate(value, config.center[axis], config.maxRate[axis], config.expo[axis])),
    armRequested: rangeActive(channels[config.armAux], config.armRange, config),
    mode: rangeActive(channels[config.angleAux], config.angleRange, config)
      ? "ANGLE" : "RATE",
  };
}

const centered = compute([
  1500, 1503, 1495, 1000, 1000, 1000,
  ...new Array(10).fill(1000),
]);
assert(validProfile(profile), "default S4.4 profile must be valid");
assert(!validProfile({ ...profile, mid: 1000 }) &&
       !validProfile({ ...profile, armAux: 5 }) &&
       !validProfile({ ...profile, expo: [0, 101, 0] }) &&
       !validProfile({ ...profile, center: [68, 7, 7] }),
  "invalid endpoint, AUX, expo or center/max profiles must be rejected");
assert(centered.stick.every((value) => value === 0),
  "center deadband must produce zero on all axes");
assert(centered.throttle === 0 && centered.mode === "RATE" &&
       !centered.armRequested,
  "low throttle/AUX defaults changed");

const endpoints = compute([
  1000, 2000, 2000, 2000, 2011, 2011,
  ...new Array(10).fill(1000),
]);
assert(endpoints.stick.join(",") === "-1,1,1",
  "axis endpoints must normalize to -1/+1");
assert(endpoints.rateDps.join(",") === "-670,670,670",
  "default Actual Rates endpoints must be +/-670 dps");
assert(endpoints.throttle === 1 && endpoints.armRequested &&
       endpoints.mode === "ANGLE",
  "clamped high CRSF endpoints must activate AUX ranges");

assert(normalizeAxis(1495, profile.deadband) === 0 &&
       normalizeAxis(1505, profile.deadband) === 0,
  "deadband boundary must remain zero");
assert(normalizeAxis(1494, profile.deadband) < 0 &&
       normalizeAxis(1506, profile.deadband) > 0,
  "first value outside deadband must preserve sign");
close(actualRate(0.5, 7, 67, 0), 185, 1e-6,
  "Actual Rates midpoint without expo changed");
close(actualRate(0.5, 7, 67, 50), 114.6875, 1e-6,
  "Actual Rates expo curve changed");
assert(compute([1500, 1500, 1500, 885, 1000, 1000,
  ...new Array(10).fill(1000)]).throttle === 0,
  "throttle below configured endpoint must clamp to zero");
assert(compute([1500, 1500, 1500, 2115, 1000, 1000,
  ...new Array(10).fill(1000)]).throttle === 1,
  "throttle above configured endpoint must clamp to one");

const header = fs.readFileSync("APP/Inc/algorithms/rc_setpoint.h", "utf8");
const source = fs.readFileSync("APP/Src/algorithms/rc_setpoint.c", "utf8");
const stateHeader = fs.readFileSync("APP/Inc/app_state.h", "utf8");
const flight = fs.readFileSync("APP/Src/rtos/flight_task.c", "utf8");
const msp = fs.readFileSync("APP/Src/protocol/msp_server.c", "utf8");
const diag = fs.readFileSync("APP/Src/platform/platform_diag.c", "utf8");
const cmake = fs.readFileSync("CMakeLists.txt", "utf8");

assert(header.includes("RC_SETPOINT_MODE_RATE = 0") &&
       header.includes("RC_SETPOINT_MODE_ANGLE"),
  "RATE/ANGLE mode contract changed");
assert(source.includes(".input_min_us = 1000U") &&
       source.includes(".input_mid_us = 1500U") &&
       source.includes(".input_max_us = 2000U") &&
       source.includes(".actual_center_sensitivity = {7U, 7U, 7U}") &&
       source.includes(".actual_max_rate = {67U, 67U, 67U}"),
  "default input or Actual Rates profile changed");
assert(source.includes(".arm_aux_channel = 4U") &&
       source.includes(".angle_aux_channel = 5U") &&
       source.includes(".arm_range_min_us = 1700U") &&
       source.includes(".angle_range_min_us = 1700U"),
  "AUX1 ARM-request or AUX2 ANGLE-request defaults changed");
assert(source.includes("magnitude <= deadband_us") &&
       source.includes("maximum_dps - center_dps") &&
       source.includes("fifth * expo"),
  "deadband or Actual Rates implementation changed");
assert(stateHeader.includes("rc_setpoint_output_t rc_setpoint") &&
       flight.includes("rc_setpoint_compute(setpoint_profile") &&
       flight.includes("APP_FLIGHT_SAFETY_RC_INVALID |") &&
       flight.includes("APP_FLIGHT_SAFETY_RC_STALE"),
  "FlightTask no longer consumes fresh mapped RC channels into setpoints");
assert(flight.includes("mixer_to_dshot(") &&
       flight.includes("if (state.armed)") &&
       flight.includes("!state.rc_setpoint.arm_requested && !state.armed"),
  "S4.7 must keep RC flight output and motor test behind exclusive gates");
assert(msp.includes("MSP2_GETFUN_RC_SETPOINT_STATUS 0x4007U") &&
       msp.includes("handle_getfun_rc_setpoint_status(&writer, &state)") &&
       msp.includes("rc_setpoint_default_profile()->deadband_us"),
  "S4.4 MSP diagnostics or shared deadband projection changed");
assert(diag.includes("setpoint valid=%u mode=%u arm_req=%u") &&
       cmake.includes("APP/Src/algorithms/rc_setpoint.c"),
  "S4.4 diagnostics or build wiring is missing");

const setpointStatusBytes = 4 + 6 + 2 + 9 + 3 + 8 + 6 + 2 + 6 +
  4 + 4 + 4 + 4 + 2;
assert(setpointStatusBytes === 64,
  "GETFUN RC setpoint status payload must be exactly 64 bytes");

console.log(JSON.stringify({
  result: "PASS",
  inputUs: [profile.min, profile.mid, profile.max],
  deadbandUs: { rollPitch: profile.deadband, yaw: profile.yawDeadband },
  actualRates: {
    centerSensitivityDps: profile.center.map((value) => value * 10),
    maximumDps: profile.maxRate.map((value) => value * 10),
    expoPercent: profile.expo,
  },
  modes: {
    armRequest: "AUX1 1700..2000 us",
    angleRequest: "AUX2 1700..2000 us",
    default: "RATE",
  },
  statusPayloadBytes: setpointStatusBytes,
  deferred: ["configurable PREARM AUX", "Angle outer loop", "App writes"],
}, null, 2));
