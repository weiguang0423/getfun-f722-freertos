/* S7.8 Linux-to-flight-controller end-to-end trace check. */
import fs from "node:fs";

function fail(message) {
  throw new Error(message);
}

function assert(condition, message) {
  if (!condition) fail(message);
}

function readLinuxRecords(path) {
  const text = fs.readFileSync(path, "utf8");
  return text.split(/\r?\n/).filter(Boolean).map((line, index) => {
    try {
      return JSON.parse(line);
    } catch (error) {
      fail(`${path}:${index + 1}: Linux JSON 格式错误 (${error.message})`);
    }
  });
}

function parseE2eLines(path) {
  const text = fs.readFileSync(path, "utf8");
  const records = [];

  for (const [index, line] of text.split(/\r?\n/).entries()) {
    if (!line.startsWith("e2e ")) continue;
    const values = Object.fromEntries(
      [...line.matchAll(/([a-z_]+)=([^\s\r\n]+)/g)].map((match) => [match[1], match[2]]),
    );
    const required = [
      "rc_source", "rc_valid", "rc_seq", "rc_hb", "setpoint_valid", "pid_valid",
      "mixer_valid", "flight_ready", "armed", "failsafe", "dshot_ready", "dshot_busy",
    ];
    for (const field of required) {
      assert(values[field] !== undefined, `${path}:${index + 1}: e2e 缺少字段 ${field}`);
      assert(/^\d+$/.test(values[field]), `${path}:${index + 1}: e2e 字段 ${field} 不是数字`);
    }
    records.push({
      line: index + 1,
      ...Object.fromEntries(required.map((field) => [field, Number(values[field])])),
    });
  }

  assert(records.length > 0, `${path}: 没有找到 e2e 汇总行`);
  return records;
}

function latestLinuxSourceAtOrBefore(records, sequence) {
  let latest;
  for (const record of records) {
    if (record.virtual_rc_valid === true &&
        Number.isInteger(record.virtual_rc_source_sequence) &&
        record.virtual_rc_source_sequence <= sequence) {
      latest = record;
    }
  }
  return latest;
}

function verify(linuxRecords, flightRecords, linuxPath, flightPath) {
  const linuxActive = linuxRecords.filter((record) => record.virtual_rc_valid === true);
  assert(linuxActive.length > 0, `${linuxPath}: 没有找到有效的 Linux 虚拟 RC 帧`);

  const selected = flightRecords.filter((record) => record.rc_source === 1);
  assert(selected.length > 0, `${flightPath}: 飞控没有选中 Linux RC（rc_source=1）`);

  let firstSequence;
  let lastSequence;
  for (const record of selected) {
    const badFields = [
      ["rc_valid", 1], ["setpoint_valid", 1], ["pid_valid", 1], ["mixer_valid", 1],
      ["flight_ready", 1], ["failsafe", 0], ["dshot_ready", 1],
    ].filter(([field, expected]) => record[field] !== expected);
    assert(badFields.length === 0,
           `${flightPath}:${record.line}: 已选中 Linux RC，但控制链不完整：` +
           badFields.map(([field, expected]) => `${field} 应为 ${expected}`).join("，"));

    const matched = latestLinuxSourceAtOrBefore(linuxActive, record.rc_seq);
    assert(matched !== undefined,
           `${flightPath}:${record.line}: rc_seq=${record.rc_seq} 前没有有效的 Linux 源序号`);
    if (firstSequence === undefined) firstSequence = matched.virtual_rc_source_sequence;
    lastSequence = matched.virtual_rc_source_sequence;
  }

  return {
    linuxActive: linuxActive.length,
    selected: selected.length,
    firstSequence,
    lastSequence,
  };
}

function selfTest() {
  const linux = [
    { virtual_rc_valid: false, virtual_rc_channels: [0, 0, 0, 0, 0] },
    { virtual_rc_valid: true, virtual_rc_source_sequence: 42 },
    { virtual_rc_valid: true, virtual_rc_source_sequence: 43 },
  ];
  const flight = [{
    line: 1, rc_source: 1, rc_valid: 1, rc_seq: 43, rc_hb: 8,
    setpoint_valid: 1, pid_valid: 1, mixer_valid: 1, flight_ready: 1,
    armed: 1, failsafe: 0, dshot_ready: 1, dshot_busy: 0,
  }];
  const result = verify(linux, flight, "self-test Linux", "self-test flight");
  assert(result.lastSequence === 43, "内置样例的序号关联失败");
}

const paths = process.argv.slice(2);
if (paths.length === 1 && paths[0] === "--self-test") {
  selfTest();
  console.log("S7.8 端到端检查：内置样例通过");
} else {
  if (paths.length !== 2) {
    console.error("用法：node Tools/verify_s7_8_e2e.mjs Linux日志.jsonl 飞控日志.log");
    process.exit(2);
  }
  try {
    const result = verify(readLinuxRecords(paths[0]), parseE2eLines(paths[1]), paths[0], paths[1]);
    console.log("S7.8 端到端日志检查：通过");
    console.log(`Linux 有效虚拟 RC 帧：${result.linuxActive}`);
    console.log(`飞控选中 Linux RC 的 e2e 样本：${result.selected}`);
    console.log(`已关联的源序号范围：${result.firstSequence} 到 ${result.lastSequence}`);
  } catch (error) {
    console.error(`S7.8 端到端日志检查：不通过 - ${error.message}`);
    process.exit(1);
  }
}
