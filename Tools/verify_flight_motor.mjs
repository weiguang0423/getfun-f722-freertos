/* Independent S4.1-S4.3 freshness, DShot300 and motor-timeout regression. */
import fs from "node:fs";

const MOTOR_COUNT = 4;
const MIN_THROTTLE = 48;
const MAX_VALUE = 2047;
const TEST_TIMEOUT_MS = 250;

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function encodeFrame(value, telemetry = false) {
  value = Math.min(value, MAX_VALUE);
  const payload = (value << 1) | Number(telemetry);
  const checksum = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0f;
  return ((payload << 4) | checksum) & 0xffff;
}

function validMotorValue(value) {
  return value === 0 || (value >= MIN_THROTTLE && value <= MAX_VALUE);
}

function requestActive(now, requestTick) {
  return ((now - requestTick) >>> 0) < TEST_TIMEOUT_MS;
}

assert(encodeFrame(0) === 0x0000, "DShot stop frame changed");
assert(encodeFrame(48) === 0x0606, "DShot minimum throttle frame changed");
assert(encodeFrame(2047) === 0xffee, "DShot maximum frame changed");
assert(validMotorValue(0) && validMotorValue(48) && validMotorValue(2047),
  "valid DShot motor values rejected");
assert(!validMotorValue(1) && !validMotorValue(47) && !validMotorValue(2048),
  "reserved DShot command or overflow value accepted");
assert(requestActive(249, 0) && !requestActive(250, 0),
  "250 ms motor-test timeout boundary changed");
assert(requestActive(0x20, 0xfffffff0),
  "motor-test timeout is not uint32-wrap safe");

const dshot = fs.readFileSync("APP/Src/bsp/dshot_motor.c", "utf8");
const flight = fs.readFileSync("APP/Src/rtos/flight_task.c", "utf8");
const msp = fs.readFileSync("APP/Src/protocol/msp_server.c", "utf8");
const irq = fs.readFileSync("Core/Src/stm32f7xx_it.c", "utf8");
const cmake = fs.readFileSync("CMakeLists.txt", "utf8");
const testTool = fs.readFileSync("Tools/motor_test.ps1", "utf8");

assert(dshot.includes("TIM1_PERIOD_TICKS == 720U") &&
       dshot.includes("TIM2_PERIOD_TICKS == 360U") &&
       dshot.includes("TIM1_DUTY_ZERO_TICKS") &&
       dshot.includes("TIM1_DUTY_ONE_TICKS"),
  "DShot300 216/108 MHz timing contract changed");
assert(dshot.includes("DMA2_Stream5") &&
       dshot.includes("TIM1_DMA_CHANNEL 6UL") &&
       dshot.includes("DMA1_Stream1") &&
       dshot.includes("TIM2_DMA_CHANNEL 3UL"),
  "TIM1_UP/TIM2_UP DMA resource contract changed");
assert(!dshot.includes("DMA1_Stream5") &&
       !dshot.includes("DMA2_Stream1") &&
       !dshot.includes("DMA2_Stream3"),
  "DShot conflicts with CRSF, ADC3 or SPI1 DMA streams");
assert(flight.includes("app_state_get_snapshot(&snapshot)") &&
       flight.includes("APP_FLIGHT_SAFETY_IMU_STALE") &&
       flight.includes("APP_FLIGHT_SAFETY_RC_STALE") &&
       flight.includes("FLIGHT_TEST_TIMEOUT_TICKS") &&
       flight.includes("dshot_motor_force_safe()"),
  "FlightTask freshness or hard-stop path changed");
assert(msp.includes("MSP2_GETFUN_FLIGHT_MOTOR_STATUS 0x4005U") &&
       msp.includes("MSP2_SET_GETFUN_MOTOR_TEST 0x4006U") &&
       msp.includes("request->payload_length != (DSHOT_MOTOR_COUNT * 2U)"),
  "S4 motor-test MSP2 contract changed");
assert(irq.includes("void DMA1_Stream1_IRQHandler(void)") &&
       irq.includes("void DMA2_Stream5_IRQHandler(void)"),
  "DShot DMA IRQ routing changed");
assert(cmake.includes("APP/Src/bsp/dshot_motor.c") &&
       cmake.includes("APP/Src/rtos/flight_task.c"),
  "S4 sources are missing from the target");
assert(testTool.includes("[switch]$PropsRemoved") &&
       testTool.includes("finally") &&
       testTool.includes("@(0, 0, 0, 0)"),
  "motor test tool lost its prop-removal or final-stop guard");

const flightMotorStatusBytes = 4 + (10 * 4) +
  (2 * MOTOR_COUNT * 2) + 4;
assert(flightMotorStatusBytes === 64,
  "GETFUN Flight/Motor status payload must remain 64 bytes");

console.log(JSON.stringify({
  result: "PASS",
  dshot300: {
    frameBits: 16,
    timer1: { clockHz: 216000000, periodTicks: 720, t0hTicks: 270, t1hTicks: 540 },
    timer2: { clockHz: 108000000, periodTicks: 360, t0hTicks: 135, t1hTicks: 270 },
    vectors: { stop: "0x0000", minThrottle: "0x0606", max: "0xffee" },
  },
  motorCount: MOTOR_COUNT,
  acceptedValues: `0 or ${MIN_THROTTLE}..${MAX_VALUE}`,
  motorTestTimeoutMs: TEST_TIMEOUT_MS,
  statusPayloadBytes: flightMotorStatusBytes,
  dma: "TIM1_UP DMA2 Stream5/Channel6; TIM2_UP DMA1 Stream1/Channel3",
  deferred: ["PID", "Mixer", "ARM state machine", "Betaflight Motors page"],
}, null, 2));
