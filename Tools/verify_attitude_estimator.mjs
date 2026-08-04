/*
 * verify_attitude_estimator.mjs
 *
 * Independent S3.6 mathematical regression for the pure-C Mahony contract.
 * It reads the firmware constants and checks coordinate signs, gravity
 * seeding, three-axis rotations, gyro-only behavior, recovery, reset
 * semantics, integral limiting, and long-run quaternion normalization.
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
  "attitude_estimator.h",
);
const header = fs.readFileSync(headerPath, "utf8");

function macroNumber(name) {
  const match = header.match(
    new RegExp(
      `^#define\\s+${name}\\s+` +
        `([0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)`,
      "m",
    ),
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

function assertNear(actual, expected, tolerance, label) {
  const error = Math.abs(actual - expected);
  assertCondition(
    error <= tolerance,
    `${label}: ${actual} vs ${expected}, error ${error}`,
  );
}

const axisCount = macroNumber("ATTITUDE_ESTIMATOR_AXIS_COUNT");
const quaternionCount = macroNumber(
  "ATTITUDE_ESTIMATOR_QUATERNION_COUNT",
);
const gravity = macroNumber(
  "ATTITUDE_ESTIMATOR_STANDARD_GRAVITY_M_S2",
);
const kp = macroNumber("ATTITUDE_ESTIMATOR_KP");
const ki = macroNumber("ATTITUDE_ESTIMATOR_KI");
const integralLimit = macroNumber(
  "ATTITUDE_ESTIMATOR_INTEGRAL_LIMIT_RAD_S",
);
const integralSpinLimit = macroNumber(
  "ATTITUDE_ESTIMATOR_INTEGRAL_SPIN_LIMIT_RAD_S",
);
const minimumAccelG = macroNumber("ATTITUDE_ESTIMATOR_MIN_ACCEL_G");
const maximumAccelG = macroNumber("ATTITUDE_ESTIMATOR_MAX_ACCEL_G");
const minimumDt = macroNumber("ATTITUDE_ESTIMATOR_MIN_DT_S");
const maximumDt = macroNumber("ATTITUDE_ESTIMATOR_MAX_DT_S");

assertCondition(axisCount === 3, "attitude estimator must be three-axis");
assertCondition(
  quaternionCount === 4,
  "attitude estimator quaternion must have four elements",
);
assertCondition(
  minimumDt === 0.0005 && maximumDt === 0.002,
  "S3.6 dt gate changed",
);
assertCondition(
  minimumAccelG === 0.8 && maximumAccelG === 1.2,
  "S3.6 acceleration gate changed",
);

function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function quaternionNormalize(quaternion) {
  const norm = Math.hypot(...quaternion);
  assertCondition(Number.isFinite(norm) && norm > 1e-12, "bad q norm");
  return quaternion.map((value) => value / norm);
}

function quaternionFromEuler(roll, pitch, yaw) {
  const halfRoll = roll / 2;
  const halfPitch = pitch / 2;
  const halfYaw = yaw / 2;
  const cr = Math.cos(halfRoll);
  const sr = Math.sin(halfRoll);
  const cp = Math.cos(halfPitch);
  const sp = Math.sin(halfPitch);
  const cy = Math.cos(halfYaw);
  const sy = Math.sin(halfYaw);
  return quaternionNormalize([
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ]);
}

function estimatedDown(quaternion) {
  const [w, x, y, z] = quaternion;
  return [
    2 * (x * z - w * y),
    2 * (w * x + y * z),
    w * w - x * x - y * y + z * z,
  ];
}

function specificForceForEuler(roll, pitch, yaw = 0) {
  return estimatedDown(quaternionFromEuler(roll, pitch, yaw)).map(
    (component) => -gravity * component,
  );
}

function eulerFromQuaternion(quaternion) {
  const [w, x, y, z] = quaternion;
  const roll = Math.atan2(
    2 * (w * x + y * z),
    1 - 2 * (x * x + y * y),
  );
  const pitch = Math.asin(clamp(2 * (w * y - z * x), -1, 1));
  let yaw = Math.atan2(
    2 * (w * z + x * y),
    1 - 2 * (y * y + z * z),
  );
  if (yaw < 0) {
    yaw += 2 * Math.PI;
  }
  if (yaw >= 2 * Math.PI) {
    yaw -= 2 * Math.PI;
  }
  return {
    rollDegrees: (roll * 180) / Math.PI,
    pitchDegrees: (pitch * 180) / Math.PI,
    yawDegrees: (yaw * 180) / Math.PI,
  };
}

function makeEstimator() {
  return {
    initialized: false,
    ready: false,
    quaternion: [1, 0, 0, 0],
    integral: [0, 0, 0],
    euler: {rollDegrees: 0, pitchDegrees: 0, yawDegrees: 0},
    updateCount: 0,
    resetCount: 0,
    invalidInputCount: 0,
    accelRejectionCount: 0,
    gyroOnlyUpdateCount: 0,
  };
}

function reset(estimator) {
  estimator.initialized = false;
  estimator.ready = false;
  estimator.quaternion = [1, 0, 0, 0];
  estimator.integral = [0, 0, 0];
  estimator.euler = {
    rollDegrees: 0,
    pitchDegrees: 0,
    yawDegrees: 0,
  };
  estimator.resetCount += 1;
}

function update(estimator, acceleration, angularRate, dt) {
  const finiteVectors =
    acceleration.every(Number.isFinite) &&
    angularRate.every(Number.isFinite);
  if (
    !finiteVectors ||
    !Number.isFinite(dt) ||
    dt < minimumDt ||
    dt > maximumDt
  ) {
    estimator.invalidInputCount += 1;
    reset(estimator);
    return false;
  }

  const accelerationMagnitude = Math.hypot(...acceleration);
  const accelerationValid =
    accelerationMagnitude >= minimumAccelG * gravity &&
    accelerationMagnitude <= maximumAccelG * gravity;
  let measuredDown = [0, 0, 0];
  if (accelerationValid) {
    measuredDown = acceleration.map(
      (value) => -value / accelerationMagnitude,
    );
  } else {
    estimator.accelRejectionCount += 1;
  }

  if (!estimator.initialized) {
    if (!accelerationValid) {
      return false;
    }
    const roll = Math.atan2(measuredDown[1], measuredDown[2]);
    const pitch = Math.atan2(
      -measuredDown[0],
      Math.hypot(measuredDown[1], measuredDown[2]),
    );
    estimator.quaternion = quaternionFromEuler(roll, pitch, 0);
    estimator.integral = [0, 0, 0];
    estimator.euler = eulerFromQuaternion(estimator.quaternion);
    estimator.initialized = true;
    estimator.ready = true;
    estimator.updateCount += 1;
    return true;
  }

  const correctedRate = [...angularRate];
  if (accelerationValid) {
    const down = estimatedDown(estimator.quaternion);
    const error = [
      measuredDown[1] * down[2] - measuredDown[2] * down[1],
      measuredDown[2] * down[0] - measuredDown[0] * down[2],
      measuredDown[0] * down[1] - measuredDown[1] * down[0],
    ];
    if (Math.hypot(...angularRate) < integralSpinLimit) {
      estimator.integral = estimator.integral.map((value, axis) =>
        clamp(
          value + ki * error[axis] * dt,
          -integralLimit,
          integralLimit,
        ),
      );
    }
    for (let axis = 0; axis < axisCount; axis += 1) {
      correctedRate[axis] += kp * error[axis] + estimator.integral[axis];
    }
  } else {
    estimator.gyroOnlyUpdateCount += 1;
  }

  const [w, x, y, z] = estimator.quaternion;
  const [gx, gy, gz] = correctedRate;
  estimator.quaternion = quaternionNormalize([
    w + 0.5 * dt * (-x * gx - y * gy - z * gz),
    x + 0.5 * dt * (w * gx + y * gz - z * gy),
    y + 0.5 * dt * (w * gy - x * gz + z * gx),
    z + 0.5 * dt * (w * gz + x * gy - y * gx),
  ]);
  estimator.euler = eulerFromQuaternion(estimator.quaternion);
  estimator.ready = true;
  estimator.updateCount += 1;
  return true;
}

const dt = 0.001;
const radiansPerDegree = Math.PI / 180;
const zeroRate = [0, 0, 0];
const seedCases = [
  {name: "level", roll: 0, pitch: 0},
  {name: "right-roll", roll: 30, pitch: 0},
  {name: "nose-up", roll: 0, pitch: 20},
  {name: "combined", roll: 25, pitch: -15},
];

for (const testCase of seedCases) {
  const estimator = makeEstimator();
  const acceleration = specificForceForEuler(
    testCase.roll * radiansPerDegree,
    testCase.pitch * radiansPerDegree,
  );
  assertCondition(
    update(estimator, acceleration, zeroRate, dt),
    `${testCase.name} did not seed`,
  );
  assertNear(
    estimator.euler.rollDegrees,
    testCase.roll,
    0.001,
    `${testCase.name} roll`,
  );
  assertNear(
    estimator.euler.pitchDegrees,
    testCase.pitch,
    0.001,
    `${testCase.name} pitch`,
  );
  assertNear(
    estimator.euler.yawDegrees,
    0,
    0.001,
    `${testCase.name} yaw`,
  );
}

function verifyAxisRotation(axis, expectedField, expectedDegrees) {
  const estimator = makeEstimator();
  update(estimator, [0, 0, -gravity], zeroRate, dt);
  const rate = [0, 0, 0];
  rate[axis] = expectedDegrees * radiansPerDegree;
  for (let sample = 1; sample <= 1000; sample += 1) {
    const angle = expectedDegrees * radiansPerDegree * sample * dt;
    const euler = [0, 0, 0];
    euler[axis] = angle;
    update(
      estimator,
      specificForceForEuler(euler[0], euler[1], euler[2]),
      rate,
      dt,
    );
  }
  assertNear(
    estimator.euler[expectedField],
    expectedDegrees,
    0.15,
    `positive axis ${axis}`,
  );
  return estimator.euler[expectedField];
}

const positiveRotations = {
  rollDegrees: verifyAxisRotation(0, "rollDegrees", 45),
  pitchDegrees: verifyAxisRotation(1, "pitchDegrees", 30),
  yawDegrees: verifyAxisRotation(2, "yawDegrees", 90),
};

const gyroOnly = makeEstimator();
update(gyroOnly, [0, 0, -gravity], zeroRate, dt);
for (let sample = 0; sample < 1000; sample += 1) {
  update(
    gyroOnly,
    [0, 0, -2 * gravity],
    [0, 0, 45 * radiansPerDegree],
    dt,
  );
}
assertCondition(gyroOnly.ready, "gyro-only update lost READY");
assertCondition(
  gyroOnly.gyroOnlyUpdateCount === 1000,
  "gyro-only update counter mismatch",
);
assertNear(
  gyroOnly.euler.yawDegrees,
  45,
  0.05,
  "gyro-only yaw",
);

const recovery = makeEstimator();
update(recovery, [0, 0, -gravity], zeroRate, dt);
for (let sample = 0; sample < 500; sample += 1) {
  update(
    recovery,
    [0, 0, -gravity],
    [60 * radiansPerDegree, 0, 0],
    dt,
  );
}
const disturbedRoll = Math.abs(recovery.euler.rollDegrees);
for (let sample = 0; sample < 3000; sample += 1) {
  update(recovery, [0, 0, -gravity], zeroRate, dt);
}
assertCondition(disturbedRoll > 10, "recovery stimulus was too small");
assertCondition(
  Math.abs(recovery.euler.rollDegrees) < 0.2,
  `roll did not recover: ${recovery.euler.rollDegrees}`,
);

const biasCorrection = makeEstimator();
update(biasCorrection, [0, 0, -gravity], zeroRate, dt);
for (let sample = 0; sample < 120000; sample += 1) {
  update(
    biasCorrection,
    [0, 0, -gravity],
    [0.5 * radiansPerDegree, 0, 0],
    dt,
  );
}
assertCondition(
  biasCorrection.integral.every(
    (value) => Math.abs(value) <= integralLimit + 1e-12,
  ),
  "integral feedback exceeded its limit",
);
assertCondition(
  Math.abs(biasCorrection.euler.rollDegrees) < 0.3,
  "integral feedback did not reject the static gyro bias",
);

const longRun = makeEstimator();
update(longRun, [0, 0, -gravity], zeroRate, dt);
for (let sample = 0; sample < 200000; sample += 1) {
  update(
    longRun,
    [0, 0, -gravity],
    [0, 0, 15 * radiansPerDegree],
    dt,
  );
}
const longRunNorm = Math.hypot(...longRun.quaternion);
assertNear(longRunNorm, 1, 1e-12, "long-run quaternion norm");
assertCondition(
  Object.values(longRun.euler).every(Number.isFinite),
  "long-run Euler output is not finite",
);

const invalid = makeEstimator();
update(invalid, [0, 0, -gravity], zeroRate, dt);
assertCondition(
  !update(invalid, [0, 0, -gravity], zeroRate, maximumDt + dt),
  "invalid dt was accepted",
);
assertCondition(
  !invalid.ready &&
    invalid.resetCount === 1 &&
    invalid.invalidInputCount === 1,
  "invalid dt did not produce a deterministic reset",
);
assertCondition(
  update(invalid, [0, 0, -gravity], zeroRate, dt),
  "estimator did not reseed after reset",
);

console.log(
  JSON.stringify(
    {
      result: "PASS",
      frame: "FRD body, NED earth",
      quaternion: "body-to-NED [w,x,y,z]",
      gains: {kp, ki, integralLimit, integralSpinLimit},
      gates: {
        dtSeconds: [minimumDt, maximumDt],
        accelerationG: [minimumAccelG, maximumAccelG],
      },
      positiveRotations,
      gyroOnlyYawDegrees: gyroOnly.euler.yawDegrees,
      recovery: {
        disturbedRollDegrees: disturbedRoll,
        finalRollDegrees: recovery.euler.rollDegrees,
      },
      biasCorrectionRollDegrees:
        biasCorrection.euler.rollDegrees,
      longRunQuaternionNorm: longRunNorm,
      invalidResetCount: invalid.resetCount,
    },
    null,
    2,
  ),
);
