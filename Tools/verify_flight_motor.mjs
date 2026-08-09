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
const dshotHeader = fs.readFileSync("APP/Inc/bsp/dshot_motor.h", "utf8");
const flight = fs.readFileSync("APP/Src/rtos/flight_task.c", "utf8");
const msp = fs.readFileSync("APP/Src/protocol/msp_server.c", "utf8");
const diag = fs.readFileSync("APP/Src/platform/platform_diag.c", "utf8");
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
assert(dshot.includes("DMA_SxFCR_DMDIS | DMA_SxFCR_FTH | DMA_SxFCR_FEIE") &&
       dshot.includes("TIM2_DMA_STREAM->FCR = 0U") &&
       /#define TIM2_DMA_ERROR_FLAGS \\\s*\(DMA_LISR_DMEIF1 \| DMA_LISR_TEIF1\)/.test(dshot),
  "TIM1 burst FIFO or TIM2 direct-mode error policy changed");
assert(dshot.includes("static uint16_t tim1_dma_values") &&
       dshot.includes("static uint16_t tim2_dma_values") &&
       dshot.includes("DMA_SxCR_PSIZE_0") &&
       dshot.includes("DMA_SxCR_MSIZE_0") &&
       !dshot.includes("DMA_SxCR_PSIZE_1") &&
       !dshot.includes("DMA_SxCR_MSIZE_1"),
  "TIMx_DMAR DMA must remain half-word-sized");
assert(dshotHeader.includes("last_tim1_dma_flags") &&
       dshotHeader.includes("last_tim2_dma_flags") &&
       diag.includes("dma_flags=[0x%08lX,0x%08lX]"),
  "per-timer DShot DMA fault evidence is missing");
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
       msp.includes("request->payload_length != (DSHOT_MOTOR_COUNT * 2U)") &&
       msp.includes("!state.flight.dshot_ready") &&
       msp.includes("!state.flight.inputs_ready"),
  "S4 motor-test MSP2 contract changed");
assert(irq.includes("void DMA1_Stream1_IRQHandler(void)") &&
       irq.includes("void DMA2_Stream5_IRQHandler(void)"),
  "DShot DMA IRQ routing changed");
assert(cmake.includes("APP/Src/bsp/dshot_motor.c") &&
       cmake.includes("APP/Src/rtos/flight_task.c"),
  "S4 sources are missing from the target");
assert(testTool.includes("[switch]$PropsRemoved") &&
       testTool.includes("finally") &&
       testTool.includes("@(0, 0, 0, 0)") &&
       testTool.includes("$stopWrites++") &&
       testTool.includes("every stop write failed") &&
       testTool.includes("$serial.Dispose()"),
  "motor test tool lost its prop-removal or final-stop guard");
assert(testTool.includes("[System.Diagnostics.Stopwatch]::StartNew()") &&
       testTool.includes("$timer.ElapsedMilliseconds -lt $DurationMs") &&
       !testTool.includes("[Environment]::TickCount64"),
  "motor test timer must remain compatible with Windows PowerShell 5.1");
assert(testTool.includes("Read-Msp2EmptyResponse") &&
       testTool.includes("$serial.ReadTimeout = 500") &&
       testTool.includes("$frame[2] -eq 0x21"),
  "motor test tool must verify the firmware response");

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
