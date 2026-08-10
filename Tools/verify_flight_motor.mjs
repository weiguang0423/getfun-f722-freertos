/* Independent S4.1-S4.3 freshness, DShot600 and motor-timeout regression. */
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

function crc8DvbS2(bytes) {
  let crc = 0;
  for (const byte of bytes) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) !== 0
        ? ((crc << 1) ^ 0xd5) & 0xff
        : (crc << 1) & 0xff;
    }
  }
  return crc;
}

function requestActive(now, requestTick) {
  return ((now - requestTick) >>> 0) < TEST_TIMEOUT_MS;
}

function buildBitbangWords(frames) {
  const pins = [15, 10, 9, 8];
  const pinMask = pins.reduce((mask, pin) => mask | (1 << pin), 0) >>> 0;
  const resetMask = (pinMask << 16) >>> 0;
  const words = [];

  for (let bit = 0; bit < 16; bit++) {
    const bitMask = 1 << (15 - bit);
    let middle = 0;
    for (let motor = 0; motor < MOTOR_COUNT; motor++) {
      if ((frames[motor] & bitMask) === 0) {
        middle = (middle | (1 << (pins[motor] + 16))) >>> 0;
      }
    }
    words.push(pinMask, middle, resetMask);
  }
  words.push(resetMask, 0, 0);
  return words;
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
assert(crc8DvbS2([0x00, 0x06, 0x40, 0x00, 0x00]) === 0x2f,
  "MSP2 0x4006 empty-response CRC changed");
const bitbangWords = buildBitbangWords([
  encodeFrame(200), encodeFrame(0), encodeFrame(0), encodeFrame(0),
]);
assert(bitbangWords.length === 51,
  "Betaflight bitbang frame must contain 48 symbol states and 3 hold states");
assert(bitbangWords[0] === 0x00008700 &&
       bitbangWords[1] === 0x87000000 &&
       bitbangWords[2] === 0x87000000,
  "first DShot bit must set all four pins then reset all four for zero");
assert(bitbangWords[48] === 0x87000000 &&
       bitbangWords[49] === 0 && bitbangWords[50] === 0,
  "DShot hold states must leave all four motor pins low");

const dshot = fs.readFileSync("APP/Src/bsp/dshot_motor.c", "utf8");
const dshotHeader = fs.readFileSync("APP/Inc/bsp/dshot_motor.h", "utf8");
const flight = fs.readFileSync("APP/Src/rtos/flight_task.c", "utf8");
const imuTask = fs.readFileSync("APP/Src/rtos/imu_task.c", "utf8");
const msp = fs.readFileSync("APP/Src/protocol/msp_server.c", "utf8");
const diag = fs.readFileSync("APP/Src/platform/platform_diag.c", "utf8");
const irq = fs.readFileSync("Core/Src/stm32f7xx_it.c", "utf8");
const cmake = fs.readFileSync("CMakeLists.txt", "utf8");
const testTool = fs.readFileSync("Tools/motor_test.ps1", "utf8");

assert(dshot.includes("#define DSHOT_BIT_RATE_HZ 600000UL") &&
       dshot.includes("#define DSHOT_STATES_PER_BIT 3U") &&
       dshot.includes("DSHOT_PACER_PERIOD_TICKS == 120U") &&
       dshot.includes("DSHOT_DMA_WORD_COUNT == 51U"),
  "Betaflight STM32F7 bitbang DShot600 pacing contract changed");
assert(dshot.includes("#define DSHOT_DMA_STREAM DMA2_Stream2") &&
       dshot.includes("#define DSHOT_DMA_CHANNEL 7UL") &&
       dshot.includes("TIM8->DIER |= TIM_DIER_CC1DE") &&
       dshot.includes("DSHOT_DMA_STREAM->PAR = (uint32_t)&GPIOA->BSRR"),
  "Betaflight TIM8_CH1 pacer or GPIOA BSRR DMA resource changed");
assert(dshot.includes("static uint32_t dma_values[DSHOT_DMA_WORD_COUNT]") &&
       dshot.includes("dma_values[base] = DSHOT_MOTOR_PIN_MASK") &&
       dshot.includes("dma_values[base + 2U] = DSHOT_MOTOR_PIN_RESET_MASK") &&
       /DSHOT_DMA_STREAM->CR\s*=\s*[\s\S]*?DMA_SxCR_PSIZE_1\s*\|\s*DMA_SxCR_MSIZE_1/.test(dshot),
  "DShot bitbang buffer or equal word-width DMA contract changed");
assert(!dshot.includes("TIM_DMABURSTLENGTH") &&
       !dshot.includes("TIM1_DMA_STREAM") &&
       !dshot.includes("TIM2_DMA_STREAM"),
  "obsolete TIM1/TIM2 DMAR implementation returned");
assert(dshotHeader.includes("last_dma_flags") &&
       diag.includes("dma_flags=0x%08lX") &&
       /if \(flags == 0U\) \{\s*return;\s*\}\s*DMA2->LIFCR = flags;/.test(dshot) &&
       /if \(\(flags & DSHOT_DMA_ERROR_FLAGS\) != 0U\) \{\s*diagnostics\.last_dma_flags = flags;/.test(dshot),
  "DMA IRQ must clear only observed flags and latch fatal errors");
assert(!dshot.includes("DMA1_Stream5") &&
       !dshot.includes("DMA2_Stream1") &&
       !dshot.includes("DMA2_Stream3"),
  "DShot conflicts with CRSF, ADC3 or SPI1 DMA streams");
assert(flight.includes("app_state_get_snapshot(&snapshot)") &&
       flight.includes("APP_FLIGHT_SAFETY_IMU_STALE") &&
       flight.includes("APP_FLIGHT_SAFETY_RC_STALE") &&
       flight.includes("FLIGHT_TEST_TIMEOUT_TICKS") &&
       flight.includes("dshot_motor_force_safe()") &&
       flight.includes("!state.rc_setpoint.arm_requested && !state.armed"),
  "FlightTask freshness or hard-stop path changed");
assert(!imuTask.includes("platform_motor_outputs_force_safe()"),
  "calibration save must not steal DShot motor pins");
assert(msp.includes("MSP2_GETFUN_FLIGHT_MOTOR_STATUS 0x4005U") &&
       msp.includes("MSP2_SET_GETFUN_MOTOR_TEST 0x4006U") &&
       msp.includes("request->payload_length != (DSHOT_MOTOR_COUNT * 2U)") &&
       msp.includes("!state.flight.dshot_ready") &&
       msp.includes("!state.flight.inputs_ready"),
  "S4 motor-test MSP2 contract changed");
assert(irq.includes("void DMA2_Stream2_IRQHandler(void)") &&
       /void DMA2_Stream2_IRQHandler\(void\)[\s\S]*?dshot_motor_dma_irq_handler\(\);/.test(irq),
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
assert(testTool.includes("Read-Msp2Response") &&
       testTool.includes("$serial.ReadTimeout = 500") &&
       testTool.includes("$header[2] -eq 0x21"),
  "motor test tool must verify the firmware response");
assert(testTool.includes("function Open-SerialPort") &&
       testTool.includes("catch [System.UnauthorizedAccessException]") &&
       testTool.includes("retrying open") &&
       testTool.includes("Open-SerialPort -Serial $serial"),
  "motor test tool must tolerate delayed Windows COM-port release");
assert(testTool.includes("$MotorStatusCommand = 0x4005") &&
       testTool.includes("Get-MotorStatus") &&
       testTool.includes("[byte[]]$payload = [byte[]]::new(0)") &&
       !testTool.includes("[byte[]]$payload = if") &&
       testTool.includes("submit_err_delta") &&
       testTool.includes("dma_err_delta") &&
       testTool.includes("Firmware sustained clean DShot submission"),
  "motor test tool must distinguish MSP acceptance from clean DShot submission");
assert(testTool.includes("Flight controller rejected motor test: dshot={0} inputs={1}") &&
       testTool.includes("$status = Get-MotorStatus -Serial $Serial"),
  "motor-test rejection must report the gate state that caused it");

const flightMotorStatusBytes = 4 + (10 * 4) +
  (2 * MOTOR_COUNT * 2) + 4;
assert(flightMotorStatusBytes === 64,
  "GETFUN Flight/Motor status payload must remain 64 bytes");

console.log(JSON.stringify({
  result: "PASS",
  dshot600: {
    frameBits: 16,
    implementation: "Betaflight STM32F7 GPIO bitbang",
    pacer: { timer: "TIM8_CH1", clockHz: 216000000, stateTicks: 120 },
    timingUs: { bit: 1.6667, t0h: 0.5556, t1h: 1.1111 },
    dmaWords: bitbangWords.length,
    vectors: { stop: "0x0000", minThrottle: "0x0606", max: "0xffee" },
  },
  motorCount: MOTOR_COUNT,
  acceptedValues: `0 or ${MIN_THROTTLE}..${MAX_VALUE}`,
  motorTestTimeoutMs: TEST_TIMEOUT_MS,
  statusPayloadBytes: flightMotorStatusBytes,
  dma: "TIM8_CH1 pacer; DMA2 Stream2/Channel7 -> GPIOA BSRR",
  deferred: ["Betaflight Motors page", "configurable motor/arming parameters"],
}, null, 2));
