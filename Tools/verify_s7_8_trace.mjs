/* S7.8 host-side contract check for Linux JSONL trace records. */
import fs from "node:fs";

function fail(message) {
  throw new Error(message);
}

function assert(condition, message) {
  if (!condition) fail(message);
}

function readRecords(path) {
  const text = fs.readFileSync(path, "utf8");
  return text.split(/\r?\n/).filter(Boolean).map((line, index) => {
    try {
      return JSON.parse(line);
    } catch (error) {
      fail(`${path}:${index + 1}: invalid JSON (${error.message})`);
    }
  });
}

function checkLinuxTrace(records) {
  assert(records.length > 0, "Linux trace is empty");
  let previousSequence = -1;
  let previousHeartbeat = -1;
  let sawActive = false;
  let sawRelease = false;

  for (const [index, record] of records.entries()) {
    assert(Number.isInteger(record.frame_index), `record ${index}: frame_index missing`);
    if (record.sequence !== undefined) {
      assert(record.sequence >= previousSequence,
             `record ${index}: camera sequence went backwards`);
      previousSequence = record.sequence;
    }
    if (record.virtual_rc_heartbeat !== undefined) {
      assert(record.virtual_rc_heartbeat > previousHeartbeat,
             `record ${index}: virtual RC heartbeat did not advance`);
      previousHeartbeat = record.virtual_rc_heartbeat;
    }
    if (record.virtual_rc_valid === true) {
      sawActive = true;
      assert(record.virtual_rc_source_sequence === record.sequence,
             `record ${index}: RC source sequence does not match camera sequence`);
      assert(Array.isArray(record.virtual_rc_channels) &&
             record.virtual_rc_channels.length === 5,
             `record ${index}: active RC channels are missing`);
      const [roll, pitch, yaw, throttle, aux] = record.virtual_rc_channels;
      assert(throttle === 0 && aux === 0,
             `record ${index}: active frame changed throttle/AUX`);
      assert(["UNKNOWN", "OPEN_PALM", "POINT", "V_SIGN"].includes(record.virtual_rc_gesture_name),
             `record ${index}: active frame has an invalid command gesture`);
      const matchesGesture =
        (record.virtual_rc_gesture_name === "UNKNOWN" && roll === 0 && pitch === 0 && yaw === 0) ||
        (record.virtual_rc_gesture_name === "OPEN_PALM" && roll === 0 && yaw === 0 &&
         pitch >= -300 && pitch <= 0) ||
        (record.virtual_rc_gesture_name === "POINT" && roll === 0 && yaw === 0 &&
         pitch >= 0 && pitch <= 300) ||
        (record.virtual_rc_gesture_name === "V_SIGN" && pitch === 0 && yaw === 0 &&
         roll >= -300 && roll <= 0);
      assert(matchesGesture, `record ${index}: channels do not match event gesture`);
    } else if (record.virtual_rc_channels) {
      sawRelease = true;
      assert(record.virtual_rc_channels.every((value) => value === 0),
             `record ${index}: invalid frame did not release all channels`);
    }
  }
  assert(sawActive, "trace contains no valid virtual RC frame");
  assert(sawRelease, "trace contains no invalid/release virtual RC frame");
}

const paths = process.argv.slice(2);
if (paths.length === 1 && paths[0] === "--self-test") {
  checkLinuxTrace([
    { frame_index: 1, sequence: 1, virtual_rc_heartbeat: 1,
      virtual_rc_valid: true, virtual_rc_source_sequence: 1,
      virtual_rc_gesture_name: "OPEN_PALM", virtual_rc_channels: [0, -30, 0, 0, 0] },
    { frame_index: 2, sequence: 2, virtual_rc_heartbeat: 2,
      virtual_rc_valid: true, virtual_rc_source_sequence: 2,
      virtual_rc_gesture_name: "UNKNOWN", virtual_rc_channels: [0, 0, 0, 0, 0] },
    { frame_index: 3, sequence: 3, virtual_rc_heartbeat: 3,
      virtual_rc_valid: true, virtual_rc_source_sequence: 3,
      virtual_rc_gesture_name: "V_SIGN", virtual_rc_channels: [-30, 0, 0, 0, 0] },
    { frame_index: 4, sequence: 4, virtual_rc_heartbeat: 4,
      virtual_rc_valid: false, virtual_rc_channels: [0, 0, 0, 0, 0] },
  ]);
  console.log("S7.8 Linux trace: self-test PASS");
  process.exit(0);
}
if (paths.length === 0) {
  console.error("usage: node Tools/verify_s7_8_trace.mjs LINUX_TRACE.jsonl");
  process.exit(2);
}

for (const path of paths) checkLinuxTrace(readRecords(path));
console.log(`S7.8 Linux trace: PASS (${paths.length} file${paths.length === 1 ? "" : "s"})`);
