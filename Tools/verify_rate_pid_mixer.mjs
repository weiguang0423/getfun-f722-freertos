/* Independent S4.5/S4.6 Rate PID and Quad-X mixer regression. */
import fs from "node:fs";

const pidProfile = {
  kp: [0.10, 0.10, 0.12],
  ki: [0.0, 0.0, 0.0],
  kd: [0.0, 0.0, 0.0],
  dtermLpf1Hz: 75,
  dtermLpf2Hz: 150,
  integralLimit: 0.20,
  outputLimit: 0.50,
  minimumDt: 0.0005,
  maximumDt: 0.002,
};

const quadX = [
  [1, -1, 1],
  [1, 1, -1],
  [-1, -1, -1],
  [-1, 1, 1],
];

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

function makePidState() {
  return {
    integral: [0, 0, 0],
    previous: [0, 0, 0],
    dtermLpf1: [0, 0, 0],
    dtermLpf2: [0, 0, 0],
    initialized: false,
  };
}

function resetPid(state) {
  state.integral.fill(0);
  state.previous.fill(0);
  state.dtermLpf1.fill(0);
  state.dtermLpf2.fill(0);
  state.initialized = false;
}

function updatePid(state, setpoint, measurement, dt,
  integratorEnabled, profile = pidProfile) {
  if (!Number.isFinite(dt) || dt < profile.minimumDt ||
      dt > profile.maximumDt ||
      [...setpoint, ...measurement].some((value) => !Number.isFinite(value))) {
    resetPid(state);
    return { valid: false };
  }

  const output = {
    valid: true,
    saturatedMask: 0,
    error: [0, 0, 0],
    p: [0, 0, 0],
    i: [0, 0, 0],
    d: [0, 0, 0],
    correction: [0, 0, 0],
  };
  const lpf1Alpha = 2 * Math.PI * profile.dtermLpf1Hz * dt /
    (1 + 2 * Math.PI * profile.dtermLpf1Hz * dt);
  const lpf2Alpha = 2 * Math.PI * profile.dtermLpf2Hz * dt /
    (1 + 2 * Math.PI * profile.dtermLpf2Hz * dt);
  for (let axis = 0; axis < 3; axis++) {
    output.error[axis] = setpoint[axis] - measurement[axis];
    output.p[axis] = profile.kp[axis] * output.error[axis];
    const rawD = state.initialized
      ? -profile.kd[axis] * (measurement[axis] - state.previous[axis]) / dt
      : 0;
    state.dtermLpf1[axis] += lpf1Alpha *
      (rawD - state.dtermLpf1[axis]);
    state.dtermLpf2[axis] += lpf2Alpha *
      (state.dtermLpf1[axis] - state.dtermLpf2[axis]);
    output.d[axis] = state.dtermLpf2[axis];
    let candidate = integratorEnabled
      ? clamp(state.integral[axis] +
          profile.ki[axis] * output.error[axis] * dt,
        -profile.integralLimit, profile.integralLimit)
      : 0;
    let raw = output.p[axis] + candidate + output.d[axis];
    if ((raw > profile.outputLimit && output.error[axis] > 0) ||
        (raw < -profile.outputLimit && output.error[axis] < 0)) {
      candidate = integratorEnabled ? state.integral[axis] : 0;
      raw = output.p[axis] + candidate + output.d[axis];
    }
    state.integral[axis] = candidate;
    state.previous[axis] = measurement[axis];
    output.i[axis] = candidate;
    output.correction[axis] =
      clamp(raw, -profile.outputLimit, profile.outputLimit);
    if (output.correction[axis] !== raw) output.saturatedMask |= 1 << axis;
  }
  state.initialized = true;
  return output;
}

function mixQuadX(throttle, correction) {
  if (!Number.isFinite(throttle) || throttle < 0 || throttle > 1 ||
      correction.some((value) => !Number.isFinite(value))) {
    return { valid: false };
  }
  const raw = quadX.map((row) =>
    row.reduce((sum, coefficient, axis) =>
      sum + coefficient * correction[axis], 0));
  const minimum = Math.min(...raw);
  const maximum = Math.max(...raw);
  let scale = 1;
  if (minimum < 0 && -minimum > throttle) {
    scale = throttle / -minimum;
  }
  if (maximum > 0 && maximum * scale > 1 - throttle) {
    scale = (1 - throttle) / maximum;
  }
  const appliedThrottle = throttle;
  return {
    valid: true,
    saturated: scale < 1 || appliedThrottle !== throttle,
    scale,
    appliedThrottle,
    motor: raw.map((value) =>
      clamp(appliedThrottle + value * scale, 0, 1)),
  };
}

const pidState = makePidState();
let pid = updatePid(pidState, [0, 0, 0], [0, 0, 0], 0.001, false);
assert(pid.valid && pid.d.every((value) => value === 0),
  "first PID sample must seed measurement derivative");
pid = updatePid(pidState, [1, 0, 0], [0, 0, 0], 0.001, false);
assert(pid.correction[0] > 0 && pid.d[0] === 0,
  "positive rate error must correct positive without setpoint D kick");
pid = updatePid(pidState, [0, 0, 0], [1, 0, 0], 0.001, false);
assert(pid.correction[0] < 0 && pid.d.every((value) => value === 0),
  "positive measured Roll rate must produce negative P correction with D disabled");

const disabledIntegrating = makePidState();
for (let sample = 0; sample < 2000; sample++) {
  pid = updatePid(disabledIntegrating, [1, 0, 0], [0, 0, 0], 0.001, true);
}
assert(pid.i.every((value) => value === 0),
  "bench profile must not accumulate a persistent motor split");

const integralProfile = { ...pidProfile, ki: [0.25, 0.25, 0.25] };
const integrating = makePidState();
for (let sample = 0; sample < 2000; sample++) {
  pid = updatePid(integrating, [1, 0, 0], [0, 0, 0], 0.001, true,
    integralProfile);
}
close(pid.i[0], integralProfile.integralLimit, 1e-9,
  "integral must stop at its configured limit");
pid = updatePid(integrating, [100, 0, 0], [0, 0, 0], 0.001, true,
  integralProfile);
assert(pid.correction[0] === integralProfile.outputLimit &&
       pid.i[0] === integralProfile.integralLimit,
  "output saturation must not wind the integral further");
pid = updatePid(integrating, [0, 0, 0], [0, 0, 0], 0.001, false);
assert(pid.i.every((value) => value === 0),
  "low-throttle/disarmed gate must reset all integrators");
assert(!updatePid(integrating, [0, 0, 0], [0, 0, 0], 0.003, true).valid &&
       !updatePid(integrating, [Number.NaN, 0, 0], [0, 0, 0],
         0.001, true).valid,
  "invalid dt or non-finite input must invalidate and reset PID");

let mixed = mixQuadX(0.5, [0.1, 0, 0]);
assert(mixed.motor.join(",") === "0.6,0.6,0.4,0.4",
  "positive Roll correction must increase left and decrease right motors");
mixed = mixQuadX(0.5, [0, 0.1, 0]);
assert(mixed.motor.join(",") === "0.4,0.6,0.4,0.6",
  "positive Pitch correction must decrease front and increase rear motors");
const noseUpState = makePidState();
const noseUpPid = updatePid(noseUpState, [0, 0, 0], [0, -1, 0],
  0.001, false);
mixed = mixQuadX(0.5, noseUpPid.correction);
assert(mixed.motor.join(",") === "0.4,0.6,0.4,0.6",
  "measured nose-up rotation must slow front and speed rear motors");
mixed = mixQuadX(0.5, [0, 0, 0.1]);
assert(mixed.motor.join(",") === "0.6,0.4,0.4,0.6",
  "positive Yaw correction must include Betaflight's default yaw inversion");
mixed = mixQuadX(0.5, [0.5, -0.5, 0.5]);
assert(mixed.valid && mixed.saturated &&
       Math.abs(mixed.scale - 1 / 3) < 1e-12 &&
       mixed.appliedThrottle === 0.5 &&
       mixed.motor.every((value, index) =>
         Math.abs(value - [1, 1 / 3, 1 / 3, 1 / 3][index]) < 1e-12),
  "desaturation must reduce correction without raising throttle");
mixed = mixQuadX(0.005, [-0.153, 0.082, -0.022]);
assert(mixed.valid && mixed.saturated &&
       mixed.appliedThrottle === 0.005 &&
       Math.max(...mixed.motor) < 0.012,
  "logged 0.5% throttle disturbance must never raise collective throttle");
assert(!mixQuadX(1.1, [0, 0, 0]).valid &&
       !mixQuadX(0.5, [0, Number.NaN, 0]).valid,
  "invalid mixer input must be rejected");

const pidHeader = fs.readFileSync("APP/Inc/algorithms/rate_pid.h", "utf8");
const pidSource = fs.readFileSync("APP/Src/algorithms/rate_pid.c", "utf8");
const mixerHeader = fs.readFileSync("APP/Inc/algorithms/quad_x_mixer.h", "utf8");
const mixerSource = fs.readFileSync("APP/Src/algorithms/quad_x_mixer.c", "utf8");
const stateHeader = fs.readFileSync("APP/Inc/app_state.h", "utf8");
const flight = fs.readFileSync("APP/Src/rtos/flight_task.c", "utf8");
const msp = fs.readFileSync("APP/Src/protocol/msp_server.c", "utf8");
const diag = fs.readFileSync("APP/Src/platform/platform_diag.c", "utf8");
const cmake = fs.readFileSync("CMakeLists.txt", "utf8");

assert(pidHeader.includes("rate_pid_update(") &&
       pidSource.includes(".ki = {0.0f, 0.0f, 0.0f}") &&
       pidSource.includes("state->previous_measurement_rad_s[axis]") &&
       pidSource.includes("profile->dterm_lpf1_hz") &&
       pidSource.includes("state->dterm_lpf2[axis]") &&
       pidSource.includes("candidate_integral = integrator_enabled") &&
       pidSource.includes("profile->output_limit"),
  "S4.5 measurement-D, integrator gate or limits are missing");
assert(mixerHeader.includes("QUAD_X_MOTOR_FRONT_LEFT = 0") &&
       mixerSource.includes("{1.0f, -1.0f, 1.0f}") &&
       mixerSource.includes("{-1.0f, 1.0f, 1.0f}") &&
       mixerSource.includes("output->applied_throttle = throttle") &&
       mixerSource.includes("output->correction_scale = throttle / -minimum"),
  "S4.6 measured motor order or desaturation changed");
assert(stateHeader.includes("rate_pid_output_t rate_pid") &&
       stateHeader.includes("quad_x_mixer_output_t mixer") &&
       flight.includes("snapshot.imu.filtered_angular_rate_rad_s") &&
       flight.includes("snapshot.imu.sample_count !=") &&
       !flight.includes("FLIGHT_PID_ACTIVE_THROTTLE_MIN") &&
       !flight.includes("Keep all motors equal") &&
       flight.includes("state.rc_setpoint.throttle > 0.0f") &&
       flight.includes("rate_pid_reset(&pid_state);") &&
       flight.includes("if (!rate_pid_update("),
  "FlightTask no longer updates control once per fresh filtered IMU sample");
assert(flight.includes("mixer_to_dshot(") &&
       flight.includes("if (state.armed)") &&
       flight.includes("memcpy(output, flight_output") &&
       flight.includes("!state.rc_setpoint.arm_requested && !state.armed"),
  "S4.7 must connect Mixer to DShot only through the armed gate");
assert(msp.includes("MSP2_GETFUN_CONTROL_STATUS 0x4008U") &&
       msp.includes("handle_getfun_control_status(&writer, &state)") &&
       diag.includes("control pid=%u int=%u sat=0x%02X") &&
       diag.includes("pid_terms p_milli=[%ld,%ld,%ld]") &&
       diag.includes("mixer valid=%u saturated=%u") &&
       cmake.includes("APP/Src/algorithms/rate_pid.c") &&
       cmake.includes("APP/Src/algorithms/quad_x_mixer.c"),
  "S4.5/S4.6 diagnostics or build wiring is missing");

const controlStatusBytes = 4 + 4 + 4 + 16 + 18 + 4 + 18 + 24 + 6 + 8;
assert(controlStatusBytes === 106,
  "GETFUN control status payload must be exactly 106 bytes");

console.log(JSON.stringify({
  result: "PASS",
  ratePid: {
    kp: pidProfile.kp,
    ki: pidProfile.ki,
    kd: pidProfile.kd,
    dtermLpfHz: [pidProfile.dtermLpf1Hz, pidProfile.dtermLpf2Hz],
    integralLimit: pidProfile.integralLimit,
    outputLimit: pidProfile.outputLimit,
    dtUs: [pidProfile.minimumDt * 1e6, pidProfile.maximumDt * 1e6],
    derivative: "measurement",
  },
  quadXOrder: ["M1 front-left", "M2 rear-left", "M3 front-right", "M4 rear-right"],
  saturation: "keep requested throttle and scale correction into [0,1]",
  statusPayloadBytes: controlStatusBytes,
  dshotBoundary: "Mixer output reaches DShot only while the S4.7 state is ARMED",
}, null, 2));
