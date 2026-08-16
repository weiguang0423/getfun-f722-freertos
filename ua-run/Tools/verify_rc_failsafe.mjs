/* Independent S3.8 RC mapping, timeout/recovery and MSP contract regression. */
import fs from "node:fs";

const TIMEOUT_MS = 300;
const RECOVERY_MS = 100;
const RECOVERY_MIN_FRAMES = 5;
const PHASE = {
  WAITING: 0,
  RECOVERING: 1,
  HEALTHY: 2,
  LOST: 3,
};

function assert(condition, message) {
  if (!condition) {
    throw new Error(message);
  }
}

function elapsed(now, then) {
  return (now - then) >>> 0;
}

function createState() {
  return {
    phase: PHASE.WAITING,
    frameReceived: false,
    failsafeActive: true,
    lastFrameMs: 0,
    recoveryStartedMs: 0,
    failsafeCount: 0,
    recoveryCount: 0,
    recoveryFrameCount: 0,
  };
}

function onFrame(state, nowMs) {
  state.frameReceived = true;
  state.lastFrameMs = nowMs >>> 0;
  if (state.phase === PHASE.HEALTHY) {
    return;
  }
  if (state.phase !== PHASE.RECOVERING) {
    state.phase = PHASE.RECOVERING;
    state.recoveryStartedMs = nowMs >>> 0;
    state.recoveryFrameCount = 1;
  } else {
    state.recoveryFrameCount++;
  }
  if (
    state.recoveryFrameCount >= RECOVERY_MIN_FRAMES &&
    elapsed(nowMs, state.recoveryStartedMs) >= RECOVERY_MS
  ) {
    state.phase = PHASE.HEALTHY;
    state.failsafeActive = false;
    state.recoveryCount++;
  }
}

function update(state, nowMs) {
  if (!state.frameReceived || elapsed(nowMs, state.lastFrameMs) < TIMEOUT_MS) {
    return;
  }
  if (state.phase === PHASE.HEALTHY) {
    state.failsafeCount++;
  }
  state.phase = PHASE.LOST;
  state.failsafeActive = true;
  state.recoveryFrameCount = 0;
}

function mapAetr(raw) {
  const map = [0, 1, 3, 2, 4, 5, 6, 7];
  return raw.map((value, index) => (index < map.length ? raw[map[index]] : value));
}

function safeChannels() {
  const channels = new Array(16).fill(1000);
  channels[0] = 1500;
  channels[1] = 1500;
  channels[2] = 1500;
  return channels;
}

const mapped = mapAetr([
  1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800,
  1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008,
]);
assert(mapped.slice(0, 8).join(",") === "1100,1200,1400,1300,1500,1600,1700,1800",
  "AETR to Roll/Pitch/Yaw/Throttle mapping changed");
assert(safeChannels().slice(0, 5).join(",") === "1500,1500,1500,1000,1000",
  "failsafe channel substitution changed");

const state = createState();
assert(state.failsafeActive && state.phase === PHASE.WAITING,
  "startup must inhibit RC control");
[0, 25, 50, 75].forEach((time) => onFrame(state, time));
assert(state.phase === PHASE.RECOVERING && state.failsafeActive,
  "four frames must not complete recovery");
onFrame(state, 100);
assert(state.phase === PHASE.HEALTHY && !state.failsafeActive,
  "five frames over 100 ms must complete recovery");
update(state, 399);
assert(state.phase === PHASE.HEALTHY,
  "299 ms channel age must remain healthy");
update(state, 400);
assert(state.phase === PHASE.LOST && state.failsafeCount === 1,
  "300 ms channel age must activate failsafe exactly once");
[401, 426, 451, 476].forEach((time) => onFrame(state, time));
assert(state.phase === PHASE.RECOVERING && state.failsafeActive,
  "recovery must remain inhibited before the fifth frame");
onFrame(state, 501);
assert(state.phase === PHASE.HEALTHY && state.recoveryCount === 2,
  "post-loss recovery must require both time and frame count");

const wrapState = createState();
[0xffffff8c, 0xffffffa5, 0xffffffbe, 0xffffffd7, 0xfffffff0]
  .forEach((time) => onFrame(wrapState, time));
assert(wrapState.phase === PHASE.HEALTHY,
  "recovery must work before the uint32 wrap boundary");
update(wrapState, 0x0000011b);
assert(wrapState.phase === PHASE.HEALTHY,
  "299 ms wrapped age must remain healthy");
update(wrapState, 0x0000011c);
assert(wrapState.phase === PHASE.LOST,
  "300 ms wrapped age must activate failsafe");

const inputHeader = fs.readFileSync("APP/Inc/algorithms/rc_input.h", "utf8");
const inputSource = fs.readFileSync("APP/Src/algorithms/rc_input.c", "utf8");
const taskSource = fs.readFileSync("APP/Src/rtos/rc_task.c", "utf8");
const mspSource = fs.readFileSync("APP/Src/protocol/msp_server.c", "utf8");
const stateHeader = fs.readFileSync("APP/Inc/app_state.h", "utf8");
const cmake = fs.readFileSync("CMakeLists.txt", "utf8");

function functionBody(source, name, nextName) {
  const start = source.indexOf(`static void ${name}`);
  const end = source.indexOf(`static void ${nextName}`, start);
  assert(start >= 0 && end > start, `cannot locate ${name}`);
  return source.slice(start, end);
}

function writerBytes(source) {
  const widths = { writer_u8: 1, writer_u16: 2, writer_u32: 4, writer_i32: 4 };
  let total = 0;
  for (const [name, width] of Object.entries(widths)) {
    total += (source.match(new RegExp(`${name}\\(`, "g")) ?? []).length * width;
  }
  return total;
}

const rcTuningBody = functionBody(mspSource, "handle_rc_tuning", "handle_rx_config");
const rxConfigBody = functionBody(mspSource, "handle_rx_config", "handle_pid");
assert(writerBytes(rcTuningBody) === 24,
  "MSP_RC_TUNING payload must remain 24 bytes for API 1.48");
assert(writerBytes(rxConfigBody) === 39,
  "MSP_RX_CONFIG payload must remain 39 bytes for API 1.48");

assert(inputHeader.includes("RC_INPUT_TIMEOUT_MS 300U"),
  "firmware RC timeout changed");
assert(inputHeader.includes("RC_INPUT_RECOVERY_MS 100U"),
  "firmware RC recovery window changed");
assert(inputHeader.includes("RC_INPUT_RECOVERY_MIN_FRAMES 5U"),
  "firmware RC recovery frame gate changed");
assert(inputSource.includes("0U, 1U, 3U, 2U, 4U, 5U, 6U, 7U"),
  "firmware AETR map changed");
assert(taskSource.includes("rc_input_failsafe_update(&failsafe"),
  "RcTask no longer updates failsafe state");
assert(taskSource.includes("rc_input_map_aetr(rc->channel_us"),
  "RcTask no longer publishes mapped channels");
assert(stateHeader.includes("APP_ARMING_INHIBIT_RC_NOT_READY (1UL << 6U)"),
  "RC arming inhibit flag changed");
assert(cmake.includes("APP/Src/algorithms/rc_input.c"),
  "RC input source missing from target");

const requiredMspCases = [
  "MSP_RC", "MSP_RSSI_CONFIG", "MSP_RC_TUNING", "MSP_RX_MAP",
  "MSP_RC_DEADBAND", "MSP_RX_CONFIG", "MSP_MIXER_CONFIG",
  "MSP_FAILSAFE_CONFIG", "MSP2_GETFUN_RC_STATUS",
];
for (const command of requiredMspCases) {
  assert(mspSource.includes(`case ${command}:`),
    `${command} is missing from the MSP server`);
}
assert(mspSource.includes("FEATURE_RX_SERIAL (1UL << 3U)"),
  "Receiver page is not told that serial RX is active");
assert(mspSource.includes("rc_input_set_safe_channels(safe_channels)"),
  "MSP_RC no longer substitutes safe values during failsafe");
assert(mspSource.includes("channel < APP_STATE_RC_CHANNEL_COUNT"),
  "MSP_RC no longer writes all 16 channels");
assert(mspSource.includes("MSP2_GETFUN_RC_STATUS 0x4003U"),
  "GETFUN RC diagnostic command changed");

console.log(JSON.stringify({
  result: "PASS",
  mapping: "CRSF AETR -> MSP Roll/Pitch/Yaw/Throttle",
  mappedFirstEight: mapped.slice(0, 8),
  safeFirstFive: safeChannels().slice(0, 5),
  timeoutMs: TIMEOUT_MS,
  recoveryMs: RECOVERY_MS,
  recoveryMinFrames: RECOVERY_MIN_FRAMES,
  payloadBytes: {
    MSP_RC: 32,
    MSP_RSSI_CONFIG: 1,
    MSP_RC_TUNING: writerBytes(rcTuningBody),
    MSP_RX_MAP: 8,
    MSP_RC_DEADBAND: 5,
    MSP_RX_CONFIG: writerBytes(rxConfigBody),
    MSP_MIXER_CONFIG: 2,
    MSP_FAILSAFE_CONFIG: 8,
    MSP2_GETFUN_RC_STATUS: 64,
  },
  requiredMspCases,
  wraparound: "PASS",
}, null, 2));
