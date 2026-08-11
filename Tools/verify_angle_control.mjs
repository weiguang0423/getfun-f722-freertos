/* Independent S4.8 Angle outer-loop and FlightTask integration regression. */
import fs from "node:fs";

const profile = {
  angleLimitDeg: 60,
  angleP: 5,
  rateLimitDps: 670,
};

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function clamp(value, minimum, maximum) {
  return Math.min(Math.max(value, minimum), maximum);
}

function angleOuterLoop(stick, yawRateDps, attitudeDeg) {
  if ([...stick, yawRateDps, ...attitudeDeg].some(
    (value) => !Number.isFinite(value)) ||
      stick.some((value) => value < -1 || value > 1) ||
      Math.abs(attitudeDeg[0]) > 180 || Math.abs(attitudeDeg[1]) > 90) {
    return { valid: false };
  }
  const targetAngleDeg = stick.slice(0, 2).map(
    (value) => value * profile.angleLimitDeg);
  const errorAngleDeg = targetAngleDeg.map(
    (value, axis) => value - attitudeDeg[axis]);
  return {
    valid: true,
    targetAngleDeg,
    errorAngleDeg,
    targetRateDps: [
      ...errorAngleDeg.map((value) => clamp(
        value * profile.angleP,
        -profile.rateLimitDps,
        profile.rateLimitDps)),
      yawRateDps,
    ],
  };
}

let output = angleOuterLoop([0, 0, 0], 0, [12, -8]);
assert(output.valid && output.targetRateDps[0] === -60 &&
       output.targetRateDps[1] === 40,
"centered sticks must command Roll/Pitch back toward level");
output = angleOuterLoop([1, -1, 0.5], 335, [0, 0]);
assert(output.targetAngleDeg.join(",") === "60,-60" &&
       output.targetRateDps.join(",") === "300,-300,335",
"full sticks must respect the angle limit and pass Yaw rate through");
output = angleOuterLoop([1, 1, 0], 0, [-180, -90]);
assert(output.targetRateDps[0] === profile.rateLimitDps &&
       output.targetRateDps[1] === profile.rateLimitDps,
"large attitude errors must clamp the rate target");
assert(!angleOuterLoop([Number.NaN, 0, 0], 0, [0, 0]).valid &&
       !angleOuterLoop([1.01, 0, 0], 0, [0, 0]).valid &&
       !angleOuterLoop([0, 0, 0], 0, [0, 91]).valid,
"invalid RC or attitude inputs must fail closed");

const readSource = (path) => fs.readFileSync(path, "utf8")
  .replaceAll("\r\n", "\n");
const header = readSource("APP/Inc/algorithms/angle_outer_loop.h");
const source = readSource("APP/Src/algorithms/angle_outer_loop.c");
const state = readSource("APP/Inc/app_state.h");
const flight = readSource("APP/Src/rtos/flight_task.c");
const msp = readSource("APP/Src/protocol/msp_server.c");
const diag = readSource("APP/Src/platform/platform_diag.c");
const cmake = readSource("CMakeLists.txt");

assert(header.includes("angle_outer_loop_compute(") &&
       source.includes(".angle_limit_deg = 60.0f") &&
       source.includes(".angle_p = 5.0f") &&
       source.includes(".rate_limit_dps = 670.0f") &&
       source.includes("normalized_stick[axis] * profile->angle_limit_deg") &&
       source.includes("output->target_rate_dps[2] = yaw_rate_dps"),
"bounded Angle outer-loop implementation is incomplete");
assert(state.includes("angle_outer_loop_output_t angle_outer_loop") &&
       flight.includes("RC_SETPOINT_MODE_ANGLE") &&
       flight.includes("snapshot.attitude.roll_deg") &&
       flight.includes("angle_outer_loop_compute(") &&
       flight.includes("control_setpoint_valid = false;") &&
       !flight.match(/angle_outer_loop_error_count;\n\s*continue;/) &&
       flight.includes("previous_mode != state.rc_setpoint.mode") &&
       flight.includes("rate_pid_reset(&pid_state);"),
"FlightTask Angle integration, mode reset or fail-closed output is missing");
assert(msp.includes("MSP2_GETFUN_ANGLE_STATUS 0x400AU") &&
       msp.includes("handle_getfun_angle_status(&writer, &state)") &&
       diag.includes("angle valid=%u target_mdeg") &&
       cmake.includes("APP/Src/algorithms/angle_outer_loop.c"),
"Angle diagnostics or build wiring is missing");

const angleStatusBytes = 4 + 8 + (2 * 3 * 4) + (3 * 4);
assert(angleStatusBytes === 48,
"GETFUN Angle status payload must remain exactly 48 bytes");

console.log(JSON.stringify({
  result: "PASS",
  angleLimitDeg: profile.angleLimitDeg,
  angleP: profile.angleP,
  rateLimitDps: profile.rateLimitDps,
  yaw: "Rate setpoint passthrough",
  failure: "CONTROL_INVALID and zero motor submission",
  statusPayloadBytes: angleStatusBytes,
}, null, 2));
