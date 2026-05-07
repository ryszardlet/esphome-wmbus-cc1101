#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include <cstdint>
#include <string>
#include <vector>

#include "cc1101.h"

namespace esphome {
namespace wmbus_cc1101 {

enum class MeterType : uint8_t {
  APATOR162 = 0,
};

class WMBusComponent;  // forward

class WMBusSensor : public Component {
 public:
  void setup() override {}
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_meter_id(uint32_t id) { meter_id_ = id; }
  void set_meter_type(MeterType t) { meter_type_ = t; }
  void set_key_hex(const std::string &k) { key_hex_ = k; }
  void set_total_water_sensor(sensor::Sensor *s) { total_water_ = s; }
  void set_rssi_sensor(sensor::Sensor *s) { rssi_ = s; }

  uint32_t meter_id() const { return meter_id_; }
  MeterType meter_type() const { return meter_type_; }
  const std::string &key_hex() const { return key_hex_; }

  // Called by the hub when a fully-decoded telegram arrives whose A-field
  // matches our meter_id. `payload` starts right after the CI byte.
  void on_telegram(const uint8_t *payload, size_t payload_len, int8_t rssi_dbm);

 protected:
  uint32_t meter_id_{0};
  MeterType meter_type_{MeterType::APATOR162};
  std::string key_hex_;
  sensor::Sensor *total_water_{nullptr};
  sensor::Sensor *rssi_{nullptr};
};

class WMBusComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_pins(int mosi, int miso, int clk, int cs, int gdo0, int gdo2);
  void set_frequency_hz(uint32_t hz) { frequency_hz_ = hz; radio_.set_frequency_hz(hz); }
  void set_log_unknown(bool b) { log_unknown_ = b; }

  void register_meter(WMBusSensor *m) { meters_.push_back(m); }

 protected:
  // Called from main loop when CC1101 raised the FIFO threshold or sync IRQ.
  void process_rx_();
  // Decode a complete on-air block into an L|C|M|A|... raw frame.
  bool decode_t1_payload_(const std::vector<uint8_t> &raw_radio,
                          std::vector<uint8_t> &decoded_frame);
  void dispatch_decoded_(const std::vector<uint8_t> &frame, int8_t rssi_dbm);

  CC1101Driver radio_;
  uint32_t frequency_hz_{868'950'000U};
  bool log_unknown_{false};
  bool radio_ok_{false};

  std::vector<WMBusSensor *> meters_;

  // Reception state machine
  void receive_frame_inline_();
  void reset_rx_();
  bool determine_length_if_needed_();

  std::vector<uint8_t> rx_buffer_;
  size_t expected_size_{0};
  uint8_t l_field_{0};
  bool have_length_{false};
  uint32_t rx_started_ms_{0};
  int8_t rx_rssi_{0};
};

}  // namespace wmbus_cc1101
}  // namespace esphome
