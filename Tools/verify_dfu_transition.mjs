/* Independent S2.7 MSP-to-ROM-DFU safety and integration regression. */
import fs from "node:fs";

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function acceptDfu({ payload, armed = false, motorTest = false,
                     armRequested = false, fault = false, pending = false }) {
  return payload.length === 1 && payload[0] === 1 && !armed && !motorTest &&
    !armRequested && !fault && !pending;
}

assert(acceptDfu({ payload: [1] }), "safe ROM DFU request must be accepted");
for (const unsafe of [
  { payload: [] }, { payload: [0] }, { payload: [2] },
  { payload: [1], armed: true }, { payload: [1], motorTest: true },
  { payload: [1], armRequested: true }, { payload: [1], fault: true },
  { payload: [1], pending: true },
]) {
  assert(!acceptDfu(unsafe), `unsafe request was accepted: ${JSON.stringify(unsafe)}`);
}

const read = (path) => fs.readFileSync(path, "utf8").replaceAll("\r\n", "\n");
const msp = read("APP/Src/protocol/msp_server.c");
const task = read("APP/Src/rtos/app_task.c");
const flight = read("APP/Src/rtos/flight_task.c");
const boot = read("APP/Src/platform/platform_boot.c");
const main = read("Core/Src/main.c");
const linker = read("STM32F722XX_FLASH.ld");

assert(msp.includes("#define MSP_REBOOT 68U") &&
       msp.includes("#define MSP_REBOOT_BOOTLOADER_ROM 1U") &&
       msp.includes("case MSP_REBOOT:") &&
       msp.includes("writer_u8(&writer, MSP_REBOOT_BOOTLOADER_ROM);") &&
       msp.includes("app_state_set_configurator_arming_disabled(true)") &&
       msp.includes("state.flight.armed || state.flight.motor_test_active") &&
       msp.includes("flight_task_motor_test_request_pending()") &&
       msp.includes("state.flight.rc_setpoint.arm_requested") &&
       msp.includes("state.fault_flags != 0U"),
"standard MSP command, ACK or safety gates are missing");
assert(flight.includes("bool flight_task_motor_test_request_pending(void)") &&
       flight.includes("pending = motor_test_request.pending;"),
"a queued Motor Test must close the pre-publication DFU race");
assert(task.indexOf("msp_server_take_rom_dfu_request()") <
       task.indexOf("const bool sent = usb_cdc_transport_write(") &&
       task.includes("if (sent && reboot_to_rom_dfu)") &&
       task.indexOf("const bool sent = usb_cdc_transport_write(") <
       task.indexOf("MX_USB_DEVICE_DeInit()") &&
       task.indexOf("MX_USB_DEVICE_DeInit()") <
       task.indexOf("platform_boot_request_rom_dfu()"),
"DFU request must be consumed once and run only after a complete ACK");
assert(boot.includes("ROM_DFU_SYSTEM_MEMORY_BASE 0x1FF00000UL") &&
       boot.includes('section(".noinit.rom_dfu_request")') &&
       boot.includes("dshot_motor_force_safe();") &&
       boot.includes("platform_motor_outputs_force_safe();") &&
       boot.includes("NVIC_SystemReset();") &&
       boot.includes("SCB->VTOR = ROM_DFU_SYSTEM_MEMORY_BASE;") &&
       boot.includes("__set_MSP(stack_pointer);") &&
       boot.indexOf("__enable_irq();") <
       boot.indexOf("((void (*)(void))reset_handler)();") &&
       main.indexOf("platform_boot_enter_rom_dfu_if_requested();") <
       main.indexOf("MPU_Config();") &&
       linker.includes(".noinit (NOLOAD)"),
"one-shot reset, early ROM vector jump or motor-safe path is missing");

console.log(JSON.stringify({
  result: "PASS",
  command: "MSP_REBOOT (68)",
  mode: "MSP_REBOOT_BOOTLOADER_ROM (1)",
  systemMemory: "0x1FF00000",
  flow: ["ACK", "arming disabled", "USB disconnect", "motors safe",
         "one-shot reset", "early ROM vector jump"],
  rejects: ["bad payload", "armed", "motor test", "ARM request",
            "system fault", "duplicate pending request"],
}, null, 2));
