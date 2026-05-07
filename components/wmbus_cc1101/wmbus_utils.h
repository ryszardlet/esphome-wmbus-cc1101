// wMBus low-level utilities: 3-of-6 decoding (T1) and EN13757-4 block CRC.
//
// 3-of-6 lookup table is from EN13757-4 §6.2.4. Each 6-bit code carries
// one 4-bit nibble; an invalid code aborts decoding and the frame is
// treated as a CRC failure.
//
// Block layout for "frame format A" (used by T1):
//   block 1: L | C | M0 M1 | A0 A1 A2 A3 | V | T   = 10 bytes + 2 CRC
//   block N (>= 2): up to 16 bytes payload + 2 CRC
//   final block:    remaining bytes + 2 CRC

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome {
namespace wmbus_cc1101 {

bool decode_3of6(const uint8_t *coded, size_t coded_len,
                 std::vector<uint8_t> &out);

// EN13757-4 polynomial 0x3D65, init 0x0000, output ones-complement.
uint16_t wmbus_block_crc(const uint8_t *data, size_t len);

// Strip per-block CRCs from a frame in format A, leaving raw L|C|M|A|...|payload.
// Returns false if any CRC is wrong.
bool strip_block_crcs(std::vector<uint8_t> &frame);

// Number of 3-of-6 encoded bytes for a given decoded length.
inline size_t encoded_size_3of6(size_t decoded) {
  return (3 * decoded + 1) / 2;
}

// Compute encoded payload size from L-field for T1 frame format A.
//   blocks: 1 block of 10 bytes (incl. L) + (L - 9 + 15) / 16 more blocks
inline size_t expected_decoded_size_from_l(uint8_t l_field) {
  size_t nr_blocks = (l_field < 26) ? 2 : (l_field - 26) / 16 + 3;
  return static_cast<size_t>(l_field) + 1 + 2 * nr_blocks;
}

}  // namespace wmbus_cc1101
}  // namespace esphome
