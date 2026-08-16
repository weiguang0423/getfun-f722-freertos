/* Independent S3.7 CRSF framing, CRC and payload regression. */
import { readFileSync } from "node:fs";

const ROOT = new URL("../", import.meta.url);
const ADDRESS = 0xc8;
const TYPE_CHANNELS = 0x16;
const TYPE_LINK = 0x14;
const CRC_POLY = 0xd5;

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function crc8(bytes) {
  let crc = 0;
  for (const value of bytes) {
    crc ^= value;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = (crc & 0x80) !== 0
        ? ((crc << 1) ^ CRC_POLY) & 0xff
        : (crc << 1) & 0xff;
    }
  }
  return crc;
}

function packChannels(channels) {
  const payload = new Uint8Array(22);
  let bitOffset = 0;
  for (const channel of channels) {
    assert(channel >= 0 && channel <= 0x7ff, "channel out of range");
    for (let bit = 0; bit < 11; bit += 1) {
      if ((channel & (1 << bit)) !== 0) {
        const target = bitOffset + bit;
        payload[target >> 3] |= 1 << (target & 7);
      }
    }
    bitOffset += 11;
  }
  return payload;
}

function unpackChannels(payload) {
  const channels = [];
  let bitBuffer = 0;
  let bitsAvailable = 0;
  let inputIndex = 0;
  for (let channel = 0; channel < 16; channel += 1) {
    while (bitsAvailable < 11) {
      bitBuffer |= payload[inputIndex++] << bitsAvailable;
      bitsAvailable += 8;
    }
    channels.push(bitBuffer & 0x7ff);
    bitBuffer >>>= 11;
    bitsAvailable -= 11;
  }
  return channels;
}

function buildFrame(type, payload) {
  const body = Uint8Array.from([type, ...payload]);
  return Uint8Array.from([ADDRESS, body.length + 1, ...body, crc8(body)]);
}

function parseStream(bytes) {
  const frames = [];
  const stats = { crcErrors: 0, lengthErrors: 0, syncDrops: 0 };
  let frame = [];
  let expected = 0;
  for (const byte of bytes) {
    if (frame.length === 0) {
      if (byte === ADDRESS) frame.push(byte);
      else stats.syncDrops += 1;
      continue;
    }
    if (frame.length === 1) {
      if (byte < 2 || byte > 62) {
        stats.lengthErrors += 1;
        frame = [];
      } else {
        frame.push(byte);
        expected = byte + 2;
      }
      continue;
    }
    frame.push(byte);
    if (frame.length === expected) {
      const received = frame.at(-1);
      const calculated = crc8(frame.slice(2, -1));
      if (received === calculated) {
        frames.push({ type: frame[2], payload: frame.slice(3, -1) });
      } else {
        stats.crcErrors += 1;
      }
      frame = [];
      expected = 0;
    }
  }
  return { frames, stats };
}

const channelVector = [
  172, 992, 1811, 0, 2047, 600, 1200, 1600,
  300, 700, 1100, 1500, 1900, 400, 800, 1300,
];
const channelFrame = buildFrame(TYPE_CHANNELS, packChannels(channelVector));
assert(channelFrame.length === 26, "0x16 frame must be 26 bytes");
assert(channelFrame[1] === 24, "0x16 length field must be 24");

const corrupted = Uint8Array.from(channelFrame);
corrupted[10] ^= 0x01;
const linkPayload = Uint8Array.from([70, 72, 98, 0xfb, 1, 3, 4, 80, 75, 0xf8]);
const linkFrame = buildFrame(TYPE_LINK, linkPayload);
const stream = Uint8Array.from([
  0x55, 0xaa,
  ADDRESS, 1,
  ...corrupted,
  ...channelFrame,
  ...linkFrame,
]);
const parsed = parseStream(stream);
assert(parsed.frames.length === 2, "stream must recover two valid frames");
assert(parsed.stats.crcErrors === 1, "corrupt frame must fail CRC");
assert(parsed.stats.lengthErrors === 1, "invalid length must be rejected");
assert(parsed.stats.syncDrops === 2, "noise bytes must be counted");

const decodedChannels = unpackChannels(parsed.frames[0].payload);
assert(JSON.stringify(decodedChannels) === JSON.stringify(channelVector),
  "packed channel round-trip changed");
const pulseUs = decodedChannels.map((raw) =>
  1500 + Math.trunc(((raw - 992) * 5) / 8));
assert(pulseUs[0] === 988 && pulseUs[1] === 1500 && pulseUs[2] === 2011,
  "CRSF tick-to-microsecond mapping changed");

const link = parsed.frames[1].payload;
assert(-link[0] === -70 && link[2] === 98,
  "uplink RSSI/LQ decode changed");
assert((link[3] << 24) >> 24 === -5, "uplink SNR sign changed");
assert((link[9] << 24) >> 24 === -8, "downlink SNR sign changed");

const source = readFileSync(new URL("APP/Src/protocol/crsf.c", ROOT), "utf8");
const uart = readFileSync(new URL("APP/Src/bsp/crsf_uart.c", ROOT), "utf8");
const cmake = readFileSync(new URL("CMakeLists.txt", ROOT), "utf8");
assert(source.includes("CRSF_CRC_POLYNOMIAL 0xD5U"),
  "C CRC polynomial changed");
assert(source.includes("frame->payload_length < CRSF_RC_CHANNEL_PAYLOAD_SIZE"),
  "C channel payload bound missing");
assert(uart.includes("HAL_UARTEx_ReceiveToIdle_DMA"),
  "UART2 receive-to-idle DMA missing");
assert(uart.includes("DMA_BUFFER_SIZE 128U"),
  "DMA buffer contract changed");
assert(cmake.includes("APP/Src/protocol/crsf.c") &&
       cmake.includes("APP/Src/rtos/rc_task.c"),
  "S3.7 sources missing from target");

console.log(JSON.stringify({
  result: "PASS",
  frameAddress: `0x${ADDRESS.toString(16)}`,
  crcPolynomial: `0x${CRC_POLY.toString(16)}`,
  channelFrameBytes: channelFrame.length,
  channels: decodedChannels,
  pulseUs: pulseUs.slice(0, 4),
  link: {
    uplinkRssiDbm: [-link[0], -link[1]],
    uplinkLq: link[2],
    uplinkSnrDb: (link[3] << 24) >> 24,
    downlinkSnrDb: (link[9] << 24) >> 24,
  },
  recovery: parsed.stats,
}, null, 2));
