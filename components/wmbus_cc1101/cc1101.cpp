#include "cc1101.h"

#include "esphome/core/log.h"

namespace esphome {
namespace wmbus_cc1101 {

static const char *const TAG = "wmbus_cc1101.radio";

static SPIClass *g_spi = nullptr;  // dedicated VSPI bus owned by this driver
static SPISettings g_spi_settings(4'000'000, MSBFIRST, SPI_MODE0);

// CC1101 standard 26 MHz reference oscillator
static constexpr uint32_t kFXtal = 26'000'000U;

void CC1101Driver::cs_low_() { digitalWrite(cs_, LOW); }
void CC1101Driver::cs_high_() { digitalWrite(cs_, HIGH); }

uint8_t CC1101Driver::spi_xfer_(uint8_t b) { return g_spi->transfer(b); }

// MISO goes low when the chip is ready (CHIP_RDYn).
void CC1101Driver::wait_miso_low_() {
  uint32_t deadline = millis() + 20;
  while (digitalRead(miso_) == HIGH) {
    if ((int32_t)(millis() - deadline) >= 0)
      break;
  }
}

bool CC1101Driver::begin() {
  pinMode(cs_, OUTPUT);
  cs_high_();
  pinMode(gdo0_, INPUT);
  if (gdo2_ >= 0)
    pinMode(gdo2_, INPUT);

  if (g_spi == nullptr) {
    // Use default SPI bus (VSPI on classic ESP32, FSPI on S2/S3/C3 etc.).
    // Arduino-ESP32 3.x picks the right one based on the SoC variant.
    g_spi = new SPIClass();
    g_spi->begin(clk_, miso_, mosi_, cs_);
    // Per-transaction settings are applied via beginTransaction() below;
    // no need to seed the bus defaults explicitly.
  }

  // Power-on reset sequence per CC1101 datasheet §19.1.2 (manual reset):
  cs_high_();
  delayMicroseconds(40);
  cs_low_();
  delayMicroseconds(40);
  cs_high_();
  delayMicroseconds(40);
  cs_low_();
  wait_miso_low_();
  g_spi->beginTransaction(g_spi_settings);
  spi_xfer_(CC1101_SRES);
  g_spi->endTransaction();
  wait_miso_low_();
  cs_high_();
  delay(10);

  uint8_t partnum = read_status(CC1101_PARTNUM);
  uint8_t version = read_status(CC1101_VERSION);
  ESP_LOGI(TAG, "CC1101 PARTNUM=0x%02X VERSION=0x%02X", partnum, version);
  if (version == 0x00 || version == 0xFF) {
    ESP_LOGE(TAG,
             "CC1101 not detected (VERSION=0x%02X). Check wiring of MOSI/MISO/SCK/CS.",
             version);
    return false;
  }
  return true;
}

uint8_t CC1101Driver::strobe(uint8_t cmd) {
  cs_low_();
  wait_miso_low_();
  g_spi->beginTransaction(g_spi_settings);
  uint8_t status = spi_xfer_(cmd);
  g_spi->endTransaction();
  cs_high_();
  return status;
}

uint8_t CC1101Driver::read_reg(uint8_t addr) {
  cs_low_();
  wait_miso_low_();
  g_spi->beginTransaction(g_spi_settings);
  spi_xfer_(addr | CC1101_READ_SINGLE);
  uint8_t value = spi_xfer_(0x00);
  g_spi->endTransaction();
  cs_high_();
  return value;
}

uint8_t CC1101Driver::read_status(uint8_t addr) {
  // Status registers must be read with the BURST bit set (datasheet §10.3).
  cs_low_();
  wait_miso_low_();
  g_spi->beginTransaction(g_spi_settings);
  spi_xfer_(addr | CC1101_READ_BURST);
  uint8_t value = spi_xfer_(0x00);
  g_spi->endTransaction();
  cs_high_();
  return value;
}

void CC1101Driver::write_reg(uint8_t addr, uint8_t value) {
  cs_low_();
  wait_miso_low_();
  g_spi->beginTransaction(g_spi_settings);
  spi_xfer_(addr);
  spi_xfer_(value);
  g_spi->endTransaction();
  cs_high_();
}

void CC1101Driver::read_burst(uint8_t addr, uint8_t *buf, size_t len) {
  if (len == 0)
    return;
  cs_low_();
  wait_miso_low_();
  g_spi->beginTransaction(g_spi_settings);
  spi_xfer_(addr | CC1101_READ_BURST);
  for (size_t i = 0; i < len; i++)
    buf[i] = spi_xfer_(0x00);
  g_spi->endTransaction();
  cs_high_();
}

uint8_t CC1101Driver::rx_bytes_available() {
  return read_status(CC1101_RXBYTES) & 0x7F;
}

bool CC1101Driver::is_rx_overflow() {
  return (read_status(CC1101_RXBYTES) & 0x80) != 0;
}

void CC1101Driver::capture_rssi() {
  // CC1101 RSSI register is two's-complement; offset is 74 dBm at 868 MHz
  // per TI DN505. Convert to dBm.
  uint8_t raw = read_status(CC1101_RSSI);
  int16_t rssi_dec = (raw >= 128) ? (int16_t)raw - 256 : (int16_t)raw;
  int16_t rssi_dbm = (rssi_dec / 2) - 74;
  last_rssi_dbm_ = (int8_t)rssi_dbm;
}

void CC1101Driver::flush_rx_and_listen() {
  strobe(CC1101_SIDLE);
  // Wait briefly for state machine to reach IDLE.
  for (int i = 0; i < 20; i++) {
    if ((read_status(CC1101_MARCSTATE) & 0x1F) == MARCSTATE_IDLE)
      break;
    delayMicroseconds(100);
  }
  strobe(CC1101_SFRX);
  strobe(CC1101_SRX);
}

void CC1101Driver::configure_for_wmbus_t1() {
  // Settings for wM-Bus T1, 100 kbps 2-FSK, 868.95 MHz, 50 kHz deviation.
  // Values cross-checked with TI Design Note DN022 and SzczepanLeon's driver.
  // IOCFG2 = 0x06: asserts on sync-word, deasserts at end of packet.
  //   GDO2 HIGH for the whole reception window → FALLING edge = packet end.
  write_reg(0x00, 0x06);  // IOCFG2  - sync-word indicator
  write_reg(0x01, 0x2E);  // IOCFG1  - high impedance (default)
  // IOCFG0 = 0x00: asserts when RX FIFO ≥ FIFOTHR threshold, deasserts when
  //   below. Active high → RISING edge = "drain me now".
  write_reg(0x02, 0x00);  // IOCFG0  - RX FIFO threshold
  // FIFOTHR = 0x07: ADC retention off, close-in RX 0dB, RX threshold 32 bytes.
  //   With 64-byte FIFO and 100 kbps T1, threshold→overflow window is ~5ms,
  //   so the IRQ has to be drained promptly.
  write_reg(0x03, 0x07);  // FIFOTHR
  write_reg(0x04, 0x54);  // SYNC1   - wM-Bus T1 sync high byte
  write_reg(0x05, 0x3D);  // SYNC0   - wM-Bus T1 sync low byte
  write_reg(0x06, 0xFF);  // PKTLEN  - unused in infinite-length mode
  write_reg(0x07, 0x00);  // PKTCTRL1 - no addr check, no append status
  // PKTCTRL0 = 0x02: infinite packet length, no whitening, no CRC.
  //   Chip stays in RX after sync; we strobe SIDLE+SFRX once a complete
  //   wMBus frame has been read out.
  write_reg(0x08, 0x02);  // PKTCTRL0
  write_reg(0x09, 0x00);  // ADDR
  write_reg(0x0A, 0x00);  // CHANNR
  write_reg(0x0B, 0x08);  // FSCTRL1 - IF freq ~152 kHz
  write_reg(0x0C, 0x00);  // FSCTRL0

  // FREQ = (f * 2^16) / f_xtal
  uint64_t freq_word = ((uint64_t)frequency_hz_ << 16) / kFXtal;
  write_reg(0x0D, (uint8_t)((freq_word >> 16) & 0xFF));  // FREQ2
  write_reg(0x0E, (uint8_t)((freq_word >> 8) & 0xFF));   // FREQ1
  write_reg(0x0F, (uint8_t)(freq_word & 0xFF));          // FREQ0

  write_reg(0x10, 0x5C);  // MDMCFG4 - chan BW ≈ 270 kHz
  write_reg(0x11, 0x04);  // MDMCFG3 - 100 kbps mantissa
  write_reg(0x12, 0x06);  // MDMCFG2 - 2-FSK, 16/16 sync word
  write_reg(0x13, 0x22);  // MDMCFG1 - FEC off, 4-byte preamble
  write_reg(0x14, 0xF8);  // MDMCFG0
  write_reg(0x15, 0x44);  // DEVIATN - ~50 kHz
  write_reg(0x16, 0x07);  // MCSM2
  write_reg(0x17, 0x00);  // MCSM1   - go to IDLE on packet (we restart RX in code)
  write_reg(0x18, 0x18);  // MCSM0   - autocal IDLE→RX
  write_reg(0x19, 0x2E);  // FOCCFG
  write_reg(0x1A, 0xBF);  // BSCFG
  write_reg(0x1B, 0x43);  // AGCCTRL2
  write_reg(0x1C, 0x09);  // AGCCTRL1
  write_reg(0x1D, 0xB5);  // AGCCTRL0
  write_reg(0x21, 0xB6);  // FREND1
  write_reg(0x22, 0x10);  // FREND0
  write_reg(0x23, 0xEA);  // FSCAL3
  write_reg(0x24, 0x2A);  // FSCAL2
  write_reg(0x25, 0x00);  // FSCAL1
  write_reg(0x26, 0x1F);  // FSCAL0
  write_reg(0x29, 0x59);  // FSTEST
  write_reg(0x2C, 0x81);  // TEST2
  write_reg(0x2D, 0x35);  // TEST1
  write_reg(0x2E, 0x09);  // TEST0

  strobe(CC1101_SCAL);
  delay(2);
  flush_rx_and_listen();
}

}  // namespace wmbus_cc1101
}  // namespace esphome
