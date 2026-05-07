#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

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
  void on_telegram(uint8_t ci, const uint8_t *payload, size_t payload_len,
                   int8_t rssi_dbm);

 protected:
  uint32_t meter_id_{0};
  MeterType meter_type_{MeterType::APATOR162};
  std::string key_hex_;
  sensor::Sensor *total_water_{nullptr};
  sensor::Sensor *rssi_{nullptr};
};

// Heap-allocated, ownership transferred via the FreeRTOS queue from the RX
// task to the main loop. The vector holds the raw 3-of-6 encoded payload as
// read from the FIFO; decoding happens on the main thread.
struct RxPacket {
  std::vector<uint8_t> raw;
  int8_t rssi_dbm{0};
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

  // Trampolines for FreeRTOS (need plain function pointers).
  static void rx_task_trampoline(void *arg);

 protected:
  void rx_task_loop_();
  // Drains a single complete on-air frame from the FIFO into `out`. Returns
  // false on overflow / timeout / decode error. Runs in the RX task.
  bool drain_one_frame_(std::vector<uint8_t> &out, int8_t &rssi_out);

  bool decode_t1_payload_(const std::vector<uint8_t> &raw_radio,
                          std::vector<uint8_t> &decoded_frame);
  void dispatch_decoded_(const std::vector<uint8_t> &frame, int8_t rssi_dbm);

  CC1101Driver radio_;
  uint32_t frequency_hz_{868'950'000U};
  bool log_unknown_{false};
  bool radio_ok_{false};

  std::vector<WMBusSensor *> meters_;

  // FreeRTOS handles. Static so the IRAM_ATTR ISRs can reach them without
  // dereferencing `this` (one component instance per ESP).
  TaskHandle_t rx_task_handle_{nullptr};
  QueueHandle_t rx_queue_{nullptr};
};

}  // namespace wmbus_cc1101
}  // namespace esphome
