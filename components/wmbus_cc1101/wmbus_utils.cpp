#include "wmbus_utils.h"

namespace esphome {
namespace wmbus_cc1101 {

namespace {

// EN13757-4 §6.2.4: 6 → 4 mapping for T1/C1 mode A. 64 entries, 0xFF = invalid.
constexpr uint8_t k3of6Invalid = 0xFF;
const uint8_t k3of6Lookup[64] = {
    /*0x00*/ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    /*0x08*/ 0xFF, 0xFF, 0xFF, 0x03, 0xFF, 0x01, 0x02, 0xFF,
    /*0x10*/ 0xFF, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x00, 0xFF,
    /*0x18*/ 0xFF, 0x05, 0x06, 0xFF, 0x04, 0xFF, 0xFF, 0xFF,
    /*0x20*/ 0xFF, 0xFF, 0xFF, 0x0B, 0xFF, 0x09, 0x0A, 0xFF,
    /*0x28*/ 0xFF, 0x0F, 0xFF, 0xFF, 0x08, 0xFF, 0xFF, 0xFF,
    /*0x30*/ 0xFF, 0x0D, 0x0E, 0xFF, 0x0C, 0xFF, 0xFF, 0xFF,
    /*0x38*/ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

}  // namespace

bool decode_3of6(const uint8_t *coded, size_t coded_len,
                 std::vector<uint8_t> &out) {
  out.clear();
  if (coded_len == 0)
    return false;

  size_t segments = coded_len * 8 / 6;  // each 6-bit segment → 1 nibble
  out.reserve((segments + 1) / 2);

  for (size_t i = 0; i < segments; i++) {
    size_t bit_idx = i * 6;
    size_t byte_idx = bit_idx / 8;
    size_t bit_off = bit_idx % 8;

    uint16_t shifted = static_cast<uint16_t>(coded[byte_idx]) << 8;
    if (byte_idx + 1 < coded_len)
      shifted |= coded[byte_idx + 1];

    uint8_t code = (shifted >> (10 - bit_off)) & 0x3F;
    uint8_t nibble = k3of6Lookup[code];
    if (nibble == k3of6Invalid)
      return false;

    if ((i & 1) == 0)
      out.push_back(static_cast<uint8_t>(nibble << 4));
    else
      out.back() |= nibble;
  }

  return true;
}

uint16_t wmbus_block_crc(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x3D65;
      else
        crc <<= 1;
    }
  }
  return ~crc;
}

bool strip_block_crcs(std::vector<uint8_t> &frame) {
  if (frame.empty())
    return false;
  uint8_t l = frame[0];
  size_t total_payload = static_cast<size_t>(l) + 1;  // includes the L byte itself
  if (frame.size() < total_payload + 2)
    return false;

  std::vector<uint8_t> out;
  out.reserve(total_payload);

  // Block 1: 10 bytes (L + 9) followed by CRC16
  size_t b1 = std::min<size_t>(10, total_payload);
  uint16_t crc = wmbus_block_crc(frame.data(), b1);
  uint16_t expected =
      (static_cast<uint16_t>(frame[b1]) << 8) | frame[b1 + 1];
  if (crc != expected)
    return false;
  out.insert(out.end(), frame.begin(), frame.begin() + b1);

  size_t pos = b1 + 2;          // start of block 2
  size_t consumed_payload = b1; // payload bytes consumed (sans CRCs)
  while (consumed_payload < total_payload) {
    size_t remaining = total_payload - consumed_payload;
    size_t bn = std::min<size_t>(16, remaining);
    if (pos + bn + 2 > frame.size())
      return false;
    uint16_t cn = wmbus_block_crc(frame.data() + pos, bn);
    uint16_t en =
        (static_cast<uint16_t>(frame[pos + bn]) << 8) | frame[pos + bn + 1];
    if (cn != en)
      return false;
    out.insert(out.end(), frame.begin() + pos, frame.begin() + pos + bn);
    pos += bn + 2;
    consumed_payload += bn;
  }

  frame = std::move(out);
  return true;
}

}  // namespace wmbus_cc1101
}  // namespace esphome
