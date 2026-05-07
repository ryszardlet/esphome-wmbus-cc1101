#include "wmbus_aes.h"

#include <cctype>
#include <cstring>

#include "mbedtls/aes.h"

namespace esphome {
namespace wmbus_cc1101 {

namespace {
inline int hex_nibble(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return 10 + (c - 'a');
  if (c >= 'A' && c <= 'F')
    return 10 + (c - 'A');
  return -1;
}
}  // namespace

bool hex_to_aes_key(const std::string &hex, uint8_t key[16]) {
  if (hex.size() != 32)
    return false;
  for (size_t i = 0; i < 16; i++) {
    int hi = hex_nibble(hex[2 * i]);
    int lo = hex_nibble(hex[2 * i + 1]);
    if (hi < 0 || lo < 0)
      return false;
    key[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return true;
}

void build_mode5_iv(uint16_t m_field, uint32_t a_field, uint8_t version,
                    uint8_t type, uint8_t acc, uint8_t iv[16]) {
  iv[0] = static_cast<uint8_t>(m_field & 0xFF);
  iv[1] = static_cast<uint8_t>((m_field >> 8) & 0xFF);
  iv[2] = static_cast<uint8_t>(a_field & 0xFF);
  iv[3] = static_cast<uint8_t>((a_field >> 8) & 0xFF);
  iv[4] = static_cast<uint8_t>((a_field >> 16) & 0xFF);
  iv[5] = static_cast<uint8_t>((a_field >> 24) & 0xFF);
  iv[6] = version;
  iv[7] = type;
  for (size_t i = 8; i < 16; i++)
    iv[i] = acc;
}

bool aes_cbc_decrypt(const uint8_t *cipher, size_t len, const uint8_t key[16],
                     const uint8_t iv[16], uint8_t *plain) {
  if ((len & 0x0F) != 0)
    return false;  // CBC requires 16-byte-aligned data

  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  if (mbedtls_aes_setkey_dec(&ctx, key, 128) != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }
  uint8_t iv_copy[16];
  std::memcpy(iv_copy, iv, 16);
  int rc = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_DECRYPT, len, iv_copy,
                                 cipher, plain);
  mbedtls_aes_free(&ctx);
  return rc == 0;
}

}  // namespace wmbus_cc1101
}  // namespace esphome
