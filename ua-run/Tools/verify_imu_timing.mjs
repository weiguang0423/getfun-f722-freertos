/*
 * verify_imu_timing.mjs
 *
 * 对platform_time使用的32位DWT回绕扩展、周期余数累计、uint32微秒回绕和
 * S3.5 dt门限执行离线参考回归。
 */
import fs from "node:fs";

const cyclesPerMicrosecond = 216;
const minimumIntervalUs = 500;
const maximumIntervalUs = 2000;

function assertCondition(condition, message) {
  if (!condition) {
    throw new Error(message);
  }
}

class TimeExtension {
  constructor(previousCycles = 0) {
    this.previousCycles = previousCycles >>> 0;
    this.fractionalCycles = 0;
    this.elapsedMicroseconds = 0;
  }

  resolve(currentCycles) {
    const normalizedCycles = currentCycles >>> 0;
    const deltaCycles =
      (normalizedCycles - this.previousCycles) >>> 0;
    this.previousCycles = normalizedCycles;
    const totalCycles = deltaCycles + this.fractionalCycles;
    this.elapsedMicroseconds =
      (this.elapsedMicroseconds +
        Math.floor(totalCycles / cyclesPerMicrosecond)) >>>
      0;
    this.fractionalCycles =
      totalCycles % cyclesPerMicrosecond;
    return this.elapsedMicroseconds;
  }
}

const fractional = new TimeExtension(0);
assertCondition(
  fractional.resolve(215) === 0,
  "sub-microsecond cycles must remain in the remainder",
);
assertCondition(
  fractional.resolve(216) === 1,
  "cycle remainder must carry into the next update",
);

const rawWrap = new TimeExtension(0xfffffff0);
const wrappedCycles = (0xfffffff0 + 216000) >>> 0;
assertCondition(
  rawWrap.resolve(wrappedCycles) === 1000,
  "DWT raw wrap must preserve a 1000 us interval",
);

const maintained = new TimeExtension(0);
let rawCycles = 0;
for (let second = 1; second <= 25; second += 1) {
  rawCycles = (rawCycles + 216_000_000) >>> 0;
  const timestamp = maintained.resolve(rawCycles);
  assertCondition(
    timestamp === second * 1_000_000,
    `one-second maintenance failed at second ${second}`,
  );
}

const microsecondBeforeWrap = 0xffffff00;
const microsecondAfterWrap =
  (microsecondBeforeWrap + 1000) >>> 0;
assertCondition(
  ((microsecondAfterWrap - microsecondBeforeWrap) >>> 0) === 1000,
  "uint32 microsecond subtraction must be wrap-safe",
);

const intervalCases = [
  [499, false],
  [500, true],
  [1000, true],
  [2000, true],
  [2001, false],
];
for (const [intervalUs, expectedValid] of intervalCases) {
  const actualValid =
    intervalUs >= minimumIntervalUs &&
    intervalUs <= maximumIntervalUs;
  assertCondition(
    actualValid === expectedValid,
    `unexpected validity for ${intervalUs} us`,
  );
}

const imuTask = fs.readFileSync("APP/Src/rtos/imu_task.c", "utf8");
const notReadyBranch = imuTask.slice(
  imuTask.indexOf("if (!ready)"),
  imuTask.indexOf("(void)ulTaskNotifyTake", imuTask.indexOf("if (!ready)")),
);
assertCondition(
  !notReadyBranch.includes("sample.timing_valid = false") &&
    !notReadyBranch.includes("sample.filter_ready = false") &&
    !notReadyBranch.includes("publish_imu_and_attitude"),
  "one early DRDY poll must retain the last valid sample until the age gate expires",
);

console.log(
  JSON.stringify(
    {
      result: "PASS",
      cyclesPerMicrosecond,
      rawCycleWrapSeconds: Number(
        (2 ** 32 / 216_000_000).toFixed(6),
      ),
      microsecondWrapMinutes: Number(
        (2 ** 32 / 1_000_000 / 60).toFixed(6),
      ),
      maintainedSeconds: 25,
      dtWindowUs: [minimumIntervalUs, maximumIntervalUs],
    },
    null,
    2,
  ),
);
