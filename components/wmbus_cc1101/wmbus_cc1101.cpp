#include "wmbus_cc1101.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <Arduino.h>

#include "apator162.h"
#include "wmbus_utils.h"

namespace esphome {
namespace wmbus_cc1101 {

static const char *const TAG = "wmbus_cc1101";

// Format the address-block ID (M||A) as the wmbusmeters-style 8-hex string.
// We use the A field alone (4 bytes, little-endian) because that's the meter ID.
static std::string format_meter_id(uint32_t id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%08X", (unsigned)id);
  return std::string(buf);
}

void WMBusComponent::set_pins(int mosi, int miso, int clk, int cs, int gdo0,
                              int gdo2) {
  radio_.set_pins(mosi, miso, clk, cs, gdo0, gdo2);
}

// GDO0 fires on RX FIFO threshold (≥32 bytes) — set a flag so loop() drains.
// GDO2 fires on sync-word state changes — FALLING edge = end of packet.
// We keep the ISR trivial (only a flag store) so it's safe to mark IRAM_ATTR.
namespace {
volatile bool s_gdo0_flag = false;  // FIFO ≥ threshold
volatile bool s_gdo2_falling = false;  // packet ended

void IRAM_ATTR gdo0_isr_handler() { s_gdo0_flag = true; }
void IRAM_ATTR gdo2_isr_handler() { s_gdo2_falling = true; }
}  // namespace

void WMBusComponent::setup() {
  ESP_LOGI(TAG, "Initializing CC1101 (freq=%.3f MHz)", frequency_hz_ / 1.0e6);
  radio_ok_ = radio_.begin();
  if (!radio_ok_) {
    ESP_LOGE(TAG, "CC1101 init failed; component will be inactive");
    mark_failed();
    return;
  }
  radio_.configure_for_wmbus_t1();
  rx_buffer_.reserve(584);  // ample headroom for max T1 frame

  // Wire GDO0/GDO2 to hardware interrupts. Without this the ESPHome main
  // loop polls every ~16 ms which is far slower than the CC1101 64-byte
  // FIFO fills at 100 kbps (~5 ms threshold→overflow window).
  int gdo0 = radio_.gdo0_pin();
  if (gdo0 >= 0) {
    pinMode(gdo0, INPUT);
    attachInterrupt(digitalPinToInterrupt(gdo0), gdo0_isr_handler, RISING);
  }
  int gdo2 = radio_.gdo2_pin();
  if (gdo2 >= 0) {
    pinMode(gdo2, INPUT);
    attachInterrupt(digitalPinToInterrupt(gdo2), gdo2_isr_handler, FALLING);
  }
}

void WMBusComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "wmbus_cc1101 (CC1101 wM-Bus T1 receiver):");
  ESP_LOGCONFIG(TAG, "  frequency: %.3f MHz", frequency_hz_ / 1.0e6);
  ESP_LOGCONFIG(TAG, "  log_unknown: %s", YESNO(log_unknown_));
  ESP_LOGCONFIG(TAG, "  meters registered: %u", (unsigned)meters_.size());
}

void WMBusSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "  meter:");
  ESP_LOGCONFIG(TAG, "    id: %s", format_meter_id(meter_id_).c_str());
  ESP_LOGCONFIG(TAG, "    type: apator162");
  if (total_water_)
    LOG_SENSOR("    ", "total_water_m3", total_water_);
  if (rssi_)
    LOG_SENSOR("    ", "rssi", rssi_);
}

void WMBusComponent::loop() {
  if (!radio_ok_)
    return;

  // Quick gate: only spend time here when there's something to do. Either
  // the GDO0 ISR has flagged "FIFO past threshold" or we're already mid-frame.
  if (!s_gdo0_flag && !s_gdo2_falling && rx_buffer_.empty() &&
      !radio_.is_rx_overflow()) {
    // Cheap secondary check: maybe data trickled in below the 32-byte
    // threshold (e.g. final tail bytes after we drained). Probe RXBYTES
    // before exiting, but only this one SPI read.
    if (radio_.rx_bytes_available() == 0)
      return;
  }
  s_gdo0_flag = false;
  s_gdo2_falling = false;

  receive_frame_inline_();
}

void WMBusComponent::reset_rx_() {
  rx_buffer_.clear();
  have_length_ = false;
  expected_size_ = 0;
  s_gdo0_flag = false;
  s_gdo2_falling = false;
  radio_.flush_rx_and_listen();
}

bool WMBusComponent::determine_length_if_needed_() {
  if (have_length_)
    return true;
  if (rx_buffer_.size() < 2)
    return true;  // not yet — wait for more bytes
  std::vector<uint8_t> first;
  if (!decode_3of6(rx_buffer_.data(), 2, first) || first.empty()) {
    ESP_LOGV(TAG, "Bad 3-of-6 in L field; restarting RX");
    return false;
  }
  l_field_ = first[0];
  size_t decoded_total = expected_decoded_size_from_l(l_field_);
  expected_size_ = encoded_size_3of6(decoded_total);
  have_length_ = true;
  ESP_LOGV(TAG, "T1: L=%u, decoded=%u, encoded=%u",
           (unsigned)l_field_, (unsigned)decoded_total,
           (unsigned)expected_size_);
  if (expected_size_ < 12 || expected_size_ > 580) {
    ESP_LOGW(TAG, "Implausible expected size %u, dropping",
             (unsigned)expected_size_);
    return false;
  }
  return true;
}

// Drain a complete frame inline. Spins for up to 50 ms while bytes flow.
// At 100 kbps a worst-case T1 frame (~290 encoded bytes) takes ~23 ms on
// the air, so 50 ms covers the slow path (re-sync, partial reads) too.
//
// CC1101 RX-FIFO errata: a single-byte burst-read of the *last* byte while
// in RX corrupts the read. So while we're still expecting more data we
// always leave one byte in the FIFO; only the final chunk drains everything.
void WMBusComponent::receive_frame_inline_() {
  uint32_t deadline = millis() + 50;

  while ((int32_t)(millis() - deadline) < 0) {
    if (radio_.is_rx_overflow()) {
      ESP_LOGW(TAG, "RX FIFO overflow at %u/%u bytes (frame discarded)",
               (unsigned)rx_buffer_.size(), (unsigned)expected_size_);
      reset_rx_();
      return;
    }

    uint8_t avail = radio_.rx_bytes_available();

    // First chunk: capture RSSI now (valid right after sync word).
    if (rx_buffer_.empty() && avail > 0) {
      rx_started_ms_ = millis();
      radio_.capture_rssi();
      rx_rssi_ = radio_.rssi_dbm();
    }

    // Decide how many bytes to read this iteration.
    size_t to_read = 0;
    if (have_length_ && rx_buffer_.size() + avail >= expected_size_) {
      // Final chunk — drain exactly what's left.
      to_read = expected_size_ - rx_buffer_.size();
    } else if (avail > 1) {
      // Mid-frame — leave 1 byte in FIFO to avoid the errata.
      to_read = static_cast<size_t>(avail) - 1;
    }

    if (to_read > 0) {
      size_t old = rx_buffer_.size();
      rx_buffer_.resize(old + to_read);
      radio_.read_burst(CC1101_FIFO, rx_buffer_.data() + old, to_read);

      if (!determine_length_if_needed_()) {
        reset_rx_();
        return;
      }

      // Frame complete?
      if (have_length_ && rx_buffer_.size() >= expected_size_) {
        rx_buffer_.resize(expected_size_);  // trim any over-read
        std::vector<uint8_t> decoded;
        if (decode_t1_payload_(rx_buffer_, decoded)) {
          dispatch_decoded_(decoded, rx_rssi_);
        } else {
          ESP_LOGD(TAG, "Frame failed decode/CRC (len=%u)",
                   (unsigned)rx_buffer_.size());
        }
        reset_rx_();
        return;
      }
    } else {
      // Nothing to drain right now. Either we haven't synced yet
      // (rx_buffer_ empty) or we're waiting for the next 32-byte chunk.
      if (rx_buffer_.empty() && !s_gdo0_flag && !s_gdo2_falling)
        return;  // false alarm — let loop() come back later
      if (s_gdo2_falling) {
        s_gdo2_falling = false;  // packet ended; drain whatever remains
      }
      delayMicroseconds(200);
    }
  }

  ESP_LOGV(TAG, "RX inline timeout (%u/%u bytes)",
           (unsigned)rx_buffer_.size(), (unsigned)expected_size_);
  reset_rx_();
}

bool WMBusComponent::decode_t1_payload_(const std::vector<uint8_t> &raw_radio,
                                        std::vector<uint8_t> &decoded_frame) {
  if (!decode_3of6(raw_radio.data(), raw_radio.size(), decoded_frame))
    return false;
  // After 3-of-6 the buffer is L|C|M|A|...|CRC|... per-block. Strip CRCs.
  if (!strip_block_crcs(decoded_frame))
    return false;
  return true;
}

void WMBusComponent::dispatch_decoded_(const std::vector<uint8_t> &frame,
                                       int8_t rssi_dbm) {
  // Frame layout after CRC strip:
  // [0]=L  [1]=C  [2..3]=M  [4..7]=A (LE)  [8]=Ver  [9]=Type  [10]=CI  [11..]=payload
  if (frame.size() < 12) {
    ESP_LOGW(TAG, "Frame too short (%u bytes)", (unsigned)frame.size());
    return;
  }
  uint8_t l = frame[0];
  uint8_t c = frame[1];
  uint16_t m = frame[2] | (frame[3] << 8);
  uint32_t a = frame[4] | (frame[5] << 8) | (frame[6] << 16) | (frame[7] << 24);
  uint8_t ver = frame[8];
  uint8_t type = frame[9];
  uint8_t ci = frame[10];

  ESP_LOGD(TAG, "RX L=%u C=0x%02X M=0x%04X A=0x%08X V=0x%02X T=0x%02X CI=0x%02X RSSI=%d",
           (unsigned)l, c, m, a, ver, type, ci, rssi_dbm);

  WMBusSensor *match = nullptr;
  for (auto *s : meters_) {
    if (s->meter_id() == a) {
      match = s;
      break;
    }
  }
  if (!match) {
    if (log_unknown_) {
      ESP_LOGI(TAG, "Unknown meter A=0x%08X (M=0x%04X V=0x%02X T=0x%02X RSSI=%d)",
               a, m, ver, type, rssi_dbm);
    }
    return;
  }

  const uint8_t *payload = frame.data() + 11;
  size_t payload_len = frame.size() - 11;
  match->on_telegram(payload, payload_len, rssi_dbm);
}

void WMBusSensor::on_telegram(const uint8_t *payload, size_t payload_len,
                              int8_t rssi_dbm) {
  if (rssi_)
    rssi_->publish_state((float)rssi_dbm);

  switch (meter_type_) {
    case MeterType::APATOR162: {
      Apator162Result r = decode_apator162(payload, payload_len);
      if (r.ok && total_water_) {
        ESP_LOGI(TAG, "meter 0x%08X: total = %.3f m3 (RSSI %d dBm)",
                 (unsigned) meter_id_, r.total_m3, rssi_dbm);
        total_water_->publish_state((float)r.total_m3);
      } else if (!r.ok) {
        ESP_LOGW(TAG, "apator162: failed to decode payload (len=%u)",
                 (unsigned)payload_len);
      }
      break;
    }
  }
}

}  // namespace wmbus_cc1101
}  // namespace esphome
