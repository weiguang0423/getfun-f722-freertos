/* Independent S3.9 ADC conversion, battery-state and MSP contract regression. */
import fs from "node:fs";

const ADC_MAX = 4095;
const VREF_MV = 3300;
const VBAT_SCALE = 110;
const VBAT_DIVIDER = 10;
const CURRENT_SCALE = 100;
const PRESENT_ON_CV = 100;
const MAX_CELL_CV = 430;
const WARNING_CELL_CV = 350;
const CRITICAL_CELL_CV = 330;

function assert(condition, message) {
  if (!condition) {
    throw new Error(message);
  }
}

function voltageCv(raw) {
  const numerator = raw * VBAT_SCALE * VREF_MV;
  const denominator = ADC_MAX * 10 * VBAT_DIVIDER;
  return Math.floor((numerator + denominator / 2) / denominator);
}

function currentCa(raw) {
  const millivolts = Math.floor(raw * VREF_MV / 4096);
  return Math.trunc(Math.trunc(millivolts * 10000 / CURRENT_SCALE) / 10);
}

function cellCount(voltage) {
  if (voltage < PRESENT_ON_CV) return 0;
  return Math.min(8, Math.max(1, Math.ceil(voltage / MAX_CELL_CV)));
}

assert(voltageCv(0) === 0, "zero VBAT raw must convert to zero");
assert(voltageCv(4095) === 3630,
  "full-scale VBAT conversion must match scale 110 at 3.3 V");
assert(currentCa(0) === 0, "zero Current raw must convert to zero");
assert(currentCa(1242) === 10000,
  "about 1 V Current input must convert to 100 A at scale 100");
assert(cellCount(0) === 0 && cellCount(420) === 1 &&
       cellCount(840) === 2 && cellCount(1680) === 4,
  "cell-count ceiling rule changed");
assert(4 * WARNING_CELL_CV === 1400 &&
       4 * CRITICAL_CELL_CV === 1320,
  "4S low-voltage thresholds changed");

const BATTERY = { OK: 0, WARNING: 1, CRITICAL: 2, NOT_PRESENT: 3, INIT: 4 };
const debounce = {
  present: false,
  cells: 0,
  state: BATTERY.INIT,
  presentMs: 0,
  candidate: BATTERY.INIT,
  candidateMs: 0,
};

function classifyDebounced(state, voltage) {
  const warning = state.cells * WARNING_CELL_CV;
  const critical = state.cells * CRITICAL_CELL_CV;
  const hysteresis = state.cells * 10;
  if (state.state === BATTERY.CRITICAL && voltage < critical + hysteresis) {
    return BATTERY.CRITICAL;
  }
  if (voltage <= critical) return BATTERY.CRITICAL;
  if ((state.state === BATTERY.WARNING || state.state === BATTERY.CRITICAL) &&
      voltage < warning + hysteresis) {
    return BATTERY.WARNING;
  }
  if (voltage <= warning) return BATTERY.WARNING;
  return BATTERY.OK;
}

function updateDebounced(state, voltage, elapsedMs = 20) {
  if (!state.present) {
    state.presentMs = voltage >= PRESENT_ON_CV ? state.presentMs + elapsedMs : 0;
    if (state.presentMs >= 200) {
      state.present = true;
      state.cells = cellCount(voltage);
      state.state = classifyDebounced(state, voltage);
      state.candidate = state.state;
      state.candidateMs = 0;
    }
    return;
  }
  const classified = classifyDebounced(state, voltage);
  if (classified === state.state) {
    state.candidate = classified;
    state.candidateMs = 0;
  } else if (classified !== state.candidate) {
    state.candidate = classified;
    state.candidateMs = elapsedMs;
  } else {
    state.candidateMs += elapsedMs;
    if (state.candidateMs >= 1000) {
      state.state = classified;
      state.candidateMs = 0;
    }
  }
}

for (let i = 0; i < 9; i++) updateDebounced(debounce, 1680);
assert(!debounce.present, "battery presence must not assert before 200 ms");
updateDebounced(debounce, 1680);
assert(debounce.present && debounce.cells === 4 && debounce.state === BATTERY.OK,
  "4S battery presence/cell detection changed");
for (let i = 0; i < 49; i++) updateDebounced(debounce, 1390);
assert(debounce.state === BATTERY.OK,
  "warning must not assert before the 1 s debounce");
updateDebounced(debounce, 1390);
assert(debounce.state === BATTERY.WARNING,
  "warning must assert after the 1 s debounce");
for (let i = 0; i < 50; i++) updateDebounced(debounce, 1310);
assert(debounce.state === BATTERY.CRITICAL,
  "critical must assert after the 1 s debounce");
for (let i = 0; i < 100; i++) updateDebounced(debounce, 1350);
assert(debounce.state === BATTERY.CRITICAL,
  "critical recovery must honor 0.10 V/cell hysteresis");
for (let i = 0; i < 50; i++) updateDebounced(debounce, 1370);
assert(debounce.state === BATTERY.WARNING,
  "critical recovery must debounce into warning");

const monitorHeader = fs.readFileSync(
  "APP/Inc/algorithms/power_monitor.h", "utf8");
const monitorSource = fs.readFileSync(
  "APP/Src/algorithms/power_monitor.c", "utf8");
const adcSource = fs.readFileSync("APP/Src/bsp/power_adc.c", "utf8");
const taskSource = fs.readFileSync("APP/Src/rtos/battery_task.c", "utf8");
const stateHeader = fs.readFileSync("APP/Inc/app_state.h", "utf8");
const mspSource = fs.readFileSync("APP/Src/protocol/msp_server.c", "utf8");
const irqSource = fs.readFileSync("Core/Src/stm32f7xx_it.c", "utf8");
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

const voltageMeters = functionBody(
  mspSource, "handle_voltage_meters", "handle_current_meters");
const currentMeters = functionBody(
  mspSource, "handle_current_meters", "handle_voltage_meter_config");
const voltageConfig = functionBody(
  mspSource, "handle_voltage_meter_config", "handle_current_meter_config");
const currentConfig = functionBody(
  mspSource, "handle_current_meter_config", "handle_battery_config");
const batteryConfig = functionBody(
  mspSource, "handle_battery_config", "handle_getfun_power_status");
const powerStatus = functionBody(
  mspSource, "handle_getfun_power_status", "handle_get_text");

assert(writerBytes(voltageMeters) === 2,
  "MSP_VOLTAGE_METERS payload must remain 2 bytes for one ADC meter");
assert(writerBytes(currentMeters) === 5,
  "MSP_CURRENT_METERS payload must remain 5 bytes for one ADC meter");
assert(writerBytes(voltageConfig) === 7,
  "MSP_VOLTAGE_METER_CONFIG payload must remain 7 bytes");
assert(writerBytes(currentConfig) === 8,
  "MSP_CURRENT_METER_CONFIG payload must remain 8 bytes");
assert(writerBytes(batteryConfig) === 13,
  "MSP_BATTERY_CONFIG payload must remain 13 bytes");
const powerStatusBytes = writerBytes(powerStatus) + 12;
assert(powerStatusBytes === 64,
  "GETFUN power status payload must remain 64 bytes");

assert(monitorHeader.includes("POWER_MONITOR_STATE_CONFIRM_MS 1000U"),
  "low-voltage debounce changed");
assert(monitorSource.includes("consumed_ca_ms_remainder"),
  "mAh integration is missing");
assert(adcSource.includes("DMA2_Stream1") &&
       adcSource.includes("DMA_SxCR_CHSEL_1"),
  "ADC3 must use DMA2 Stream1/Channel2");
assert(!adcSource.includes("DMA_SxCR_CIRC") &&
       !adcSource.includes("ADC_CR2_CONT"),
  "ADC3 must remain a bounded one-shot scan");
assert(adcSource.includes("ADC_CR2_DMA | ADC_CR2_DDS"),
  "ADC3 must preserve DMA requests across repeated one-shot scans");
assert(adcSource.includes("if (!disable_dma_stream())"),
  "DMA2 Stream1 must be disabled before its transfer count is reloaded");
assert(adcSource.includes("10UL << 0U") &&
       adcSource.includes("13UL << 15U"),
  "ADC3 channel 10..13 scan order changed");
assert(taskSource.includes("POWER_MONITOR_SAMPLE_PERIOD_MS"),
  "BatteryTask no longer drives the 50 Hz scan");
assert(stateHeader.includes(
  "APP_ARMING_INHIBIT_BATTERY_NOT_READY (1UL << 7U)"),
  "battery arming inhibit flag changed");
assert(irqSource.includes("power_adc_dma_irq_handler();"),
  "DMA2 Stream1 IRQ is not routed to the ADC transport");
assert(cmake.includes("APP/Src/algorithms/power_monitor.c") &&
       cmake.includes("APP/Src/bsp/power_adc.c") &&
       cmake.includes("APP/Src/rtos/battery_task.c"),
  "S3.9 sources are missing from the target");

const requiredMspCases = [
  "MSP_BATTERY_CONFIG", "MSP_VOLTAGE_METER_CONFIG",
  "MSP_CURRENT_METER_CONFIG", "MSP_VOLTAGE_METERS",
  "MSP_CURRENT_METERS", "MSP_BATTERY_STATE",
  "MSP2_GETFUN_POWER_STATUS",
];
for (const command of requiredMspCases) {
  assert(mspSource.includes(`case ${command}:`),
    `${command} is missing from the MSP server`);
}

console.log(JSON.stringify({
  result: "PASS",
  adc: "ADC3 PC0..PC3, one-shot DMA2 Stream1/Channel2 at 50 Hz",
  acceptedCalibration: {
    voltageScale: VBAT_SCALE,
    currentScale: CURRENT_SCALE,
    currentOffsetMa: 0,
  },
  conversion: {
    fullScaleVoltageCv: voltageCv(4095),
    oneVoltCurrentCaAtScale100: currentCa(1242),
  },
  thresholdsCvPerCell: {
    warning: WARNING_CELL_CV,
    critical: CRITICAL_CELL_CV,
  },
  debounce: "200 ms presence, 1 s low state, 0.10 V/cell recovery hysteresis",
  payloadBytes: {
    MSP_VOLTAGE_METERS: writerBytes(voltageMeters),
    MSP_CURRENT_METERS: writerBytes(currentMeters),
    MSP_VOLTAGE_METER_CONFIG: writerBytes(voltageConfig),
    MSP_CURRENT_METER_CONFIG: writerBytes(currentConfig),
    MSP_BATTERY_CONFIG: writerBytes(batteryConfig),
    MSP2_GETFUN_POWER_STATUS: powerStatusBytes,
  },
  requiredMspCases,
}, null, 2));
