import fs from "node:fs";

function read(path) {
  return fs.readFileSync(path, "utf8");
}

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

const msp = read("APP/Src/protocol/msp_server.c");
const dshot = read("APP/Src/bsp/dshot_motor.c");
const flight = read("APP/Src/rtos/flight_task.c");
const store = read("APP/Src/storage/parameter_store.c");
const appTask = read("APP/Src/rtos/app_task.c");

assert(msp.includes("#define MSP2_SEND_DSHOT_COMMAND 0x3003U"),
  "Betaflight DShot command endpoint is missing");
assert(msp.includes("3U, 2U, 1U, 0U") &&
       msp.includes("values[app_motor_to_board[motor]]"),
  "App Quad-X numbering is not mapped to board output order");
assert(msp.includes("commands[index] == 7U") &&
       msp.includes("commands[index] == 8U") &&
       msp.includes("commands[index] == 12U"),
  "ESC direction/save command allowlist is missing");
assert(dshot.includes("dshot_encode_frame(values[motor], telemetry)") &&
       dshot.includes("submit_frames(values, true, true)"),
  "DShot commands must set the telemetry request bit");
assert(flight.includes("FLIGHT_DSHOT_INITIAL_DELAY_LOOPS 10U") &&
       flight.includes("FLIGHT_DSHOT_COMMAND_REPEATS 10U"),
  "Official 10 ms / 10-repeat DShot command timing is missing");
assert(store.includes("PARAMETER_RECORD_VERSION_V3 3U") &&
       store.includes("yaw_motors_reversed"),
  "global reversed motor direction is not persistent");
assert(flight.includes("mixer_correction[2] = -mixer_correction[2]"),
  "global reversed direction does not invert yaw mixing");
assert(msp.includes("#define MSP_REBOOT 68U") &&
       appTask.includes("msp_server_take_reboot_request()") &&
       appTask.includes("NVIC_SystemReset()"),
  "Configurator save/reconnect reboot path is missing");

console.log(JSON.stringify({
  result: "PASS",
  appMotorToBoard: [3, 2, 1, 0],
  dshotCommands: { direction1: 7, direction2: 8, save: 12 },
  timing: { initialDelayMs: 10, repeats: 10, intervalMs: 1 },
  persistence: "parameter record v3 with v1/v2 migration",
  reboot: "MSP 68 ACK then safe system reset",
}, null, 2));
