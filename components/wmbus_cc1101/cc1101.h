// Minimal CC1101 driver for wM-Bus T1 reception.
// Uses Arduino-ESP32 3.x SPI HAL directly.

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace wmbus_cc1101 {

// SPI command strobes
constexpr uint8_t CC1101_SRES = 0x30;
constexpr uint8_t CC1101_SCAL = 0x33;
constexpr uint8_t CC1101_SRX = 0x34;
constexpr uint8_t CC1101_SIDLE = 0x36;
constexpr uint8_t CC1101_SFRX = 0x3A;
constexpr uint8_t CC1101_SNOP = 0x3D;

// Status registers (read with burst bit set)
constexpr uint8_t CC1101_PARTNUM = 0x30;
constexpr uint8_t CC1101_VERSION = 0x31;
constexpr uint8_t CC1101_RSSI = 0x34;
constexpr uint8_t CC1101_MARCSTATE = 0x35;
constexpr uint8_t CC1101_RXBYTES = 0x3B;

// FIFO
constexpr uint8_t CC1101_FIFO = 0x3F;

// SPI access masks
constexpr uint8_t CC1101_WRITE_BURST = 0x40;
constexpr uint8_t CC1101_READ_SINGLE = 0x80;
constexpr uint8_t CC1101_READ_BURST = 0xC0;

// MARCSTATE values we care about
constexpr uint8_t MARCSTATE_IDLE = 0x01;
constexpr uint8_t MARCSTATE_RX = 0x0D;
constexpr uint8_t MARCSTATE_RXFIFO_OVERFLOW = 0x11;

class CC1101Driver {
 public:
  void set_pins(int mosi, int miso, int clk, int cs, int gdo0, int gdo2) {
    mosi_ = mosi;
    miso_ = miso;
    clk_ = clk;
    cs_ = cs;
    gdo0_ = gdo0;
    gdo2_ = gdo2;
  }
  void set_frequency_hz(uint32_t hz) { frequency_hz_ = hz; }

  // Returns false if the chip didn't respond as expected.
  bool begin();

  // Programs registers for wM-Bus T1 (100 kbps, 2-FSK, 868.95 MHz default).
  void configure_for_wmbus_t1();

  // Strobe / register helpers (public so the component can poke status regs).
  uint8_t strobe(uint8_t cmd);
  uint8_t read_reg(uint8_t addr);
  uint8_t read_status(uint8_t addr);
  void write_reg(uint8_t addr, uint8_t value);
  void read_burst(uint8_t addr, uint8_t *buf, size_t len);

  uint8_t rx_bytes_available();
  bool is_rx_overflow();

  void flush_rx_and_listen();
  int8_t rssi_dbm() const { return last_rssi_dbm_; }
  void capture_rssi();

  int gdo0_pin() const { return gdo0_; }
  int gdo2_pin() const { return gdo2_; }

 private:
  int mosi_{-1}, miso_{-1}, clk_{-1}, cs_{-1}, gdo0_{-1}, gdo2_{-1};
  uint32_t frequency_hz_{868'950'000U};
  int8_t last_rssi_dbm_{0};

  void cs_low_();
  void cs_high_();
  uint8_t spi_xfer_(uint8_t b);
  void wait_miso_low_();
};

}  // namespace wmbus_cc1101
}  // namespace esphome
