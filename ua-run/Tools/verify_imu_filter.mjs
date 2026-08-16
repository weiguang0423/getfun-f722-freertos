/*
 * verify_imu_filter.mjs
 *
 * 读取固件头文件中的S3.5 PT1参数，执行DC、阶跃和离散幅相参考回归。
 * 该脚本验证固定参数与数学验收值；C实现本身由ARM Debug/Release构建覆盖。
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const scriptDirectory = path.dirname(fileURLToPath(import.meta.url));
const repositoryRoot = path.resolve(scriptDirectory, "..");
const headerPath = path.join(
  repositoryRoot,
  "APP",
  "Inc",
  "algorithms",
  "imu_filter.h",
);
const header = fs.readFileSync(headerPath, "utf8");

function macroNumber(name) {
  const match = header.match(
    new RegExp(`^#define\\s+${name}\\s+([0-9]+(?:\\.[0-9]+)?)`, "m"),
  );
  if (!match) {
    throw new Error(`missing numeric macro ${name}`);
  }
  return Number(match[1]);
}

function assertCondition(condition, message) {
  if (!condition) {
    throw new Error(message);
  }
}

function alphaFor(cutoffHz, dtSeconds) {
  const omegaDt = 2 * Math.PI * cutoffHz * dtSeconds;
  return omegaDt / (1 + omegaDt);
}

function theoreticalResponse(alpha, frequencyHz, dtSeconds) {
  const omega = 2 * Math.PI * frequencyHz * dtSeconds;
  const retained = 1 - alpha;
  const denominatorReal = 1 - retained * Math.cos(omega);
  const denominatorImaginary = retained * Math.sin(omega);
  return {
    amplitude:
      alpha /
      Math.hypot(denominatorReal, denominatorImaginary),
    phaseRadians: -Math.atan2(
      denominatorImaginary,
      denominatorReal,
    ),
  };
}

function measuredResponse(cutoffHz, frequencyHz, dtSeconds) {
  const alpha = alphaFor(cutoffHz, dtSeconds);
  const settleSamples = 5000;
  const measuredSamples = 20000;
  let state = 0;
  let sineProjection = 0;
  let cosineProjection = 0;

  for (
    let sample = 0;
    sample < settleSamples + measuredSamples;
    sample += 1
  ) {
    const angle = 2 * Math.PI * frequencyHz * dtSeconds * sample;
    const input = Math.sin(angle);
    state += alpha * (input - state);
    if (sample >= settleSamples) {
      sineProjection += state * Math.sin(angle);
      cosineProjection += state * Math.cos(angle);
    }
  }

  const sineCoefficient = (2 * sineProjection) / measuredSamples;
  const cosineCoefficient = (2 * cosineProjection) / measuredSamples;
  return {
    amplitude: Math.hypot(sineCoefficient, cosineCoefficient),
    phaseRadians: Math.atan2(cosineCoefficient, sineCoefficient),
  };
}

function verifyResponse(label, cutoffHz, frequencyHz, dtSeconds) {
  const alpha = alphaFor(cutoffHz, dtSeconds);
  const expected = theoreticalResponse(alpha, frequencyHz, dtSeconds);
  const measured = measuredResponse(cutoffHz, frequencyHz, dtSeconds);
  const amplitudeError = Math.abs(
    measured.amplitude - expected.amplitude,
  );
  const phaseErrorDegrees =
    (Math.abs(measured.phaseRadians - expected.phaseRadians) * 180) /
    Math.PI;

  assertCondition(
    amplitudeError < 0.001,
    `${label} amplitude error ${amplitudeError}`,
  );
  assertCondition(
    phaseErrorDegrees < 0.1,
    `${label} phase error ${phaseErrorDegrees} deg`,
  );

  return {
    label,
    frequencyHz,
    amplitude: Number(measured.amplitude.toFixed(6)),
    phaseDegrees: Number(
      ((measured.phaseRadians * 180) / Math.PI).toFixed(3),
    ),
  };
}

function verifyStep(cutoffHz, dtSeconds) {
  const alpha = alphaFor(cutoffHz, dtSeconds);
  let state = 0;

  for (let sample = 0; sample < 10000; sample += 1) {
    state += alpha * (1 - state);
  }
  assertCondition(
    Math.abs(state - 1) < 1e-12,
    `step response did not converge for ${cutoffHz} Hz`,
  );
}

const axisCount = macroNumber("IMU_FILTER_AXIS_COUNT");
const gyroCutoffHz = macroNumber("IMU_FILTER_GYRO_CUTOFF_HZ");
const accelCutoffHz = macroNumber("IMU_FILTER_ACCEL_CUTOFF_HZ");
const minimumIntervalUs = macroNumber(
  "IMU_FILTER_MIN_INTERVAL_US",
);
const maximumIntervalUs = macroNumber(
  "IMU_FILTER_MAX_INTERVAL_US",
);
const nominalDtSeconds = 0.001;

assertCondition(axisCount === 3, "filter must remain three-axis");
assertCondition(
  minimumIntervalUs === 500 && maximumIntervalUs === 2000,
  "S3.5 dt acceptance window changed",
);
assertCondition(
  gyroCutoffHz === 100 && accelCutoffHz === 30,
  "S3.5 cutoff frequencies changed",
);

for (const cutoffHz of [gyroCutoffHz, accelCutoffHz]) {
  for (const intervalUs of [
    minimumIntervalUs,
    1000,
    maximumIntervalUs,
  ]) {
    const alpha = alphaFor(cutoffHz, intervalUs / 1_000_000);
    assertCondition(
      Number.isFinite(alpha) && alpha > 0 && alpha <= 1,
      `invalid alpha ${alpha} for ${cutoffHz} Hz at ${intervalUs} us`,
    );
  }
  verifyStep(cutoffHz, nominalDtSeconds);
}

const responses = [
  verifyResponse("gyro-passband", gyroCutoffHz, 20, nominalDtSeconds),
  verifyResponse(
    "gyro-cutoff",
    gyroCutoffHz,
    gyroCutoffHz,
    nominalDtSeconds,
  ),
  verifyResponse("accel-passband", accelCutoffHz, 5, nominalDtSeconds),
  verifyResponse(
    "accel-cutoff",
    accelCutoffHz,
    accelCutoffHz,
    nominalDtSeconds,
  ),
];

console.log(
  JSON.stringify(
    {
      result: "PASS",
      axisCount,
      dtWindowUs: [minimumIntervalUs, maximumIntervalUs],
      gyroCutoffHz,
      accelCutoffHz,
      responses,
    },
    null,
    2,
  ),
);
