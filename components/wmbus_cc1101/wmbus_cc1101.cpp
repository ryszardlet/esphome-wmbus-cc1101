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
  process_rx_();
}

void WMBusComponent::process_rx_() {
  // Bail out fast if there's nothing in the FIFO.
  if (radio_.is_rx_overflow()) {
    ESP_LOGW(TAG, "RX FIFO overflow, restarting RX");
    rx_buffer_.clear();
    have_length_ = false;
    expected_size_ = 0;
    radio_.flush_rx_and_listen();
    return;
  }

  uint8_t avail = radio_.rx_bytes_available();
  if (avail == 0) {
    // No data right now. If we were in the middle of a frame, time it out.
    if (!rx_buffer_.empty() && (millis() - rx_started_ms_) > 200) {
      ESP_LOGV(TAG, "RX timeout (got %u/%u bytes)",
               (unsigned)rx_buffer_.size(), (unsigned)expected_size_);
      rx_buffer_.clear();
      have_length_ = false;
      expected_size_ = 0;
      radio_.flush_rx_and_listen();
    }
    return;
  }

  // First read after sync: capture RSSI now (valid right after sync).
  if (rx_buffer_.empty()) {
    rx_started_ms_ = millis();
    radio_.capture_rssi();
    rx_rssi_ = radio_.rssi_dbm();
  }

  // CC1101 errata: when reading from FIFO during continuous RX, leave at
  // least 1 byte to avoid the well-known FIFO read corruption. Drain all
  // available bytes only when end-of-packet has been signalled.
  // We don't know expected_size_ yet for the first byte — we need the L
  // field decoded, which means we need 2 encoded bytes (16 bits → 10 bits
  // of payload, enough for the first nibble of L). The simplest safe
  // approach: keep at least 1 byte in the FIFO until we see expected_size_
  // is met.
  uint8_t to_read = avail;
  bool drain_all = have_length_ && (rx_buffer_.size() + avail >= expected_size_);
  if (!drain_all && to_read > 1)
    to_read -= 1;

  size_t old = rx_buffer_.size();
  rx_buffer_.resize(old + to_read);
  radio_.read_burst(CC1101_FIFO, rx_buffer_.data() + old, to_read);

  // Try to determine expected size as soon as we have 2 bytes (enough to
  // decode the first byte = L field).
  if (!have_length_ && rx_buffer_.size() >= 2) {
    std::vector<uint8_t> first;
    if (decode_3of6(rx_buffer_.data(), 2, first) && !first.empty()) {
      l_field_ = first[0];
      size_t decoded_total = expected_decoded_size_from_l(l_field_);
      expected_size_ = encoded_size_3of6(decoded_total);
      have_length_ = true;
      ESP_LOGV(TAG, "T1: L=%u, decoded=%u, encoded=%u",
               (unsigned)l_field_, (unsigned)decoded_total,
               (unsigned)expected_size_);
      if (expected_size_ > 600) {
        ESP_LOGW(TAG, "Implausible expected size %u, dropping",
                 (unsigned)expected_size_);
        rx_buffer_.clear();
        have_length_ = false;
        radio_.flush_rx_and_listen();
        return;
      }
    } else {
      // Bad L-field encoding — restart.
      ESP_LOGV(TAG, "Bad 3-of-6 in L field; restarting RX");
      rx_buffer_.clear();
      have_length_ = false;
      radio_.flush_rx_and_listen();
      return;
    }
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
    rx_buffer_.clear();
    have_length_ = false;
    expected_size_ = 0;
    radio_.flush_rx_and_listen();
  }
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
