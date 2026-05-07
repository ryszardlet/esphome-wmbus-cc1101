// AES-128-CBC support for wM-Bus security mode 5 (EN 13757-3 §9.2.4).
//
// Mode 5 IV layout (16 bytes):
//   M-field (2 bytes, LE) | A-field (4 bytes, LE) | V (1) | T (1) | ACC × 8

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome {
namespace wmbus_cc1101 {

bool hex_to_aes_key(const std::string &hex, uint8_t key[16]);

void build_mode5_iv(uint16_t m_field, uint32_t a_field, uint8_t version,
                    uint8_t type, uint8_t acc, uint8_t iv[16]);

// Decrypts `len` bytes (must be a multiple of 16) of CBC-mode AES-128
// ciphertext into `plain`. The IV buffer is consumed but not modified.
// Returns false on any mbedtls error.
bool aes_cbc_decrypt(const uint8_t *cipher, size_t len, const uint8_t key[16],
                     const uint8_t iv[16], uint8_t *plain);

}  // namespace wmbus_cc1101
}  // namespace esphome
