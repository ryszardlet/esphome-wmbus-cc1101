// Apator AT-WMBUS-16-2 telegram decoder.
//
// Apator's 162 family wraps a proprietary register stream inside a wM-Bus
// telegram. After the standard L|C|M|A|V|T|CI header, the payload starts
// with a 2/2F prelude (sometimes), then 0x0F (manufacturer-specific marker),
// then 8 bytes of date/faults, then a sequence of <register-id><data...>
// records. Register 0x10 carries the 4-byte little-endian total volume in
// liters.
//
// Register-size table mirrors wmbusmeters/driver_apator162.cc.

#pragma once

#include <cstdint>
#include <cstddef>

namespace esphome {
namespace wmbus_cc1101 {

struct Apator162Result {
  bool ok{false};
  double total_m3{0.0};
};

inline int apator162_register_size(uint8_t c) {
  switch (c) {
    case 0x00: return 4;  // date
    case 0x01: return 3;  // faults
    case 0xA1:
    case 0x10: return 4;  // total volume (liters, LE)
    case 0x11: return 2;  // flow
    case 0x40: return 6;
    case 0x41: return 2;
    case 0x42: return 4;
    case 0x43: return 2;
    case 0x44: return 3;
    case 0x71: return 1 + 2 * 4;
    case 0x72: return 1 + 3 * 4;
    case 0x73: return 1 + 4 * 4;
    case 0x74: return 1 + 5 * 4;
    case 0x75: return 1 + 6 * 4;
    case 0x76: return 1 + 7 * 4;
    case 0x77: return 1 + 8 * 4;
    case 0x78: return 1 + 9 * 4;
    case 0x79: return 1 + 10 * 4;
    case 0x7A: return 1 + 11 * 4;
    case 0x7B: return 1 + 12 * 4;
    case 0x80: case 0x81: case 0x82: case 0x83:
    case 0x84: case 0x86: case 0x87: return 10;
    case 0x85: case 0x88: case 0x8F: return 11;
    case 0x8A: return 9;
    case 0x8B: case 0x8C: return 6;
    case 0x8E: return 7;
    case 0xA0: return 4;
    case 0xA2: return 1;
    case 0xA3: return 7;
    case 0xA4: return 4;
    case 0xA5: case 0xA9: case 0xAF: return 1;
    case 0xA6: return 3;
    case 0xA7: case 0xA8: case 0xAA:
    case 0xAB: case 0xAC: case 0xAD: return 2;
    case 0xB0: return 5;
    case 0xB1: return 8;
    case 0xB2: return 16;
    case 0xB3: return 8;
    case 0xB4: return 2;
    case 0xB5: return 16;
    case 0xB6: case 0xB7: case 0xB8: case 0xB9:
    case 0xBA: case 0xBB: case 0xBC: case 0xBD:
    case 0xBE: case 0xBF: return 3;
    case 0xC0: case 0xC1: case 0xC2: case 0xC3:
    case 0xC4: case 0xC5: case 0xC6: case 0xC7: return 3;
    case 0xD0: case 0xD3: return 3;
    case 0xF0: return 4;
    default: return -1;
  }
}

// `payload` points at the byte right after CI. `len` is bytes available
// (already CRC-stripped).
inline Apator162Result decode_apator162(const uint8_t *payload, size_t len) {
  Apator162Result r;
  if (payload == nullptr || len < 12)
    return r;

  // Skip optional 2F2F filler then locate the manufacturer 0x0F marker.
  size_t i = 0;
  while (i < len && payload[i] == 0x2F)
    i++;
  // Some telegrams put one or more leading status bytes before 0x0F; tolerate
  // up to 8 bytes before bailing.
  size_t scan_limit = (i + 8 < len) ? i + 8 : len;
  while (i < scan_limit && payload[i] != 0x0F)
    i++;
  if (i >= len || payload[i] != 0x0F)
    return r;
  i++;  // consume 0x0F

  // Per wmbusmeters: skip 8 bytes of date/faults that always follow 0x0F.
  if (i + 8 > len)
    return r;
  i += 8;

  while (i < len) {
    uint8_t reg = payload[i];
    if (reg == 0xFF)
      break;  // padding / end
    int sz = apator162_register_size(reg);
    if (sz < 0)
      return r;  // unknown register — bail out instead of guessing
    i++;
    if (i + (size_t)sz > len)
      return r;

    if (reg == 0x10 && sz == 4) {
      uint32_t liters = (uint32_t)payload[i]
                       | ((uint32_t)payload[i + 1] << 8)
                       | ((uint32_t)payload[i + 2] << 16)
                       | ((uint32_t)payload[i + 3] << 24);
      r.total_m3 = liters / 1000.0;
      r.ok = true;
      return r;
    }
    i += (size_t)sz;
  }

  return r;
}

}  // namespace wmbus_cc1101
}  // namespace esphome
