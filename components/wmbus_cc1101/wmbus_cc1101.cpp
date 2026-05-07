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

// FIFO drain has to win the race against the radio: the CC1101 64-byte FIFO
// fills in ~5 ms once the 32-byte threshold is crossed at 100 kbps, and
// ESPHome's main loop() can stall for far longer than that whenever WiFi,
// SNTP or the API stack is active. The fix is to do the drain in a
// dedicated FreeRTOS task pinned to the Arduino core (core 1) at high
// priority, woken by the GDO ISRs so it preempts everything else.

namespace {
// Single component → one task handle. The ISRs need to reach this from
// IRAM, hence file-scope.
volatile TaskHandle_t s_rx_task_handle = nullptr;

void IRAM_ATTR gdo_isr_handler() {
  if (s_rx_task_handle == nullptr)
    return;
  BaseType_t hp_woken = pdFALSE;
  vTaskNotifyGiveFromISR(s_rx_task_handle, &hp_woken);
  portYIELD_FROM_ISR(hp_woken);
}
}  // namespace

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

  // 4-deep queue of completed packets, owned by the RX task and consumed
  // by loop(). Pointers are heap-allocated; receiver releases ownership
  // into the queue.
  rx_queue_ = xQueueCreate(4, sizeof(RxPacket *));
  if (rx_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create RX queue");
    mark_failed();
    return;
  }

  // Pin the receiver to core 1 on dual-core ESP32s — WiFi runs ISRs on
  // core 0 and would otherwise compete for the FIFO drain window.
  // Priority 24 sits just under the WiFi tasks but well above Arduino's
  // loopTask (priority 1).
  BaseType_t ok;
#if portNUM_PROCESSORS > 1
  ok = xTaskCreatePinnedToCore(WMBusComponent::rx_task_trampoline,
                               "wmbus_rx", 4096, this, 24,
                               &rx_task_handle_, 1);
#else
  ok = xTaskCreate(WMBusComponent::rx_task_trampoline, "wmbus_rx", 4096, this,
                   24, &rx_task_handle_);
#endif
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "Failed to create RX task");
    mark_failed();
    return;
  }
  s_rx_task_handle = rx_task_handle_;
  ESP_LOGI(TAG, "RX task created [%p]", rx_task_handle_);

  // Attach IRQs only after the task exists. Both edges feed the same
  // notification — the task drains until the frame is complete regardless
  // of which signal woke it.
  //   GDO0 (RISING) = RX FIFO ≥ 32 bytes ("come drain")
  //   GDO2 (RISING) = sync word detected ("a packet is starting")
  int gdo0 = radio_.gdo0_pin();
  if (gdo0 >= 0) {
    pinMode(gdo0, INPUT);
    attachInterrupt(digitalPinToInterrupt(gdo0), gdo_isr_handler, RISING);
  }
  int gdo2 = radio_.gdo2_pin();
  if (gdo2 >= 0) {
    pinMode(gdo2, INPUT);
    attachInterrupt(digitalPinToInterrupt(gdo2), gdo_isr_handler, RISING);
  }
}

void WMBusComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "wmbus_cc1101 (CC1101 wM-Bus T1 receiver):");
  ESP_LOGCONFIG(TAG, "  frequency: %.3f MHz", frequency_hz_ / 1.0e6);
  ESP_LOGCONFIG(TAG, "  log_unknown: %s", YESNO(log_unknown_));
  ESP_LOGCONFIG(TAG, "  meters registered: %u", (unsigned)meters_.size());
}

static std::string format_meter_id(uint32_t id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%08X", (unsigned)id);
  return std::string(buf);
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

// Main-thread side: pop completed packets from the queue, decode them, and
// publish to ESPHome sensors. Anything that touches sensor::Sensor must
// run here, not in the RX task.
void WMBusComponent::loop() {
  if (!radio_ok_ || rx_queue_ == nullptr)
    return;

  RxPacket *p_raw = nullptr;
  while (xQueueReceive(rx_queue_, &p_raw, 0) == pdPASS) {
    std::unique_ptr<RxPacket> pkt(p_raw);
    std::vector<uint8_t> decoded;
    if (decode_t1_payload_(pkt->raw, decoded)) {
      dispatch_decoded_(decoded, pkt->rssi_dbm);
    } else {
      ESP_LOGD(TAG, "Frame failed decode/CRC (encoded len=%u)",
               (unsigned)pkt->raw.size());
    }
  }
}

void WMBusComponent::rx_task_trampoline(void *arg) {
  static_cast<WMBusComponent *>(arg)->rx_task_loop_();
}

// RX task body: blocks on the notification, drains a frame, queues it for
// loop(), restarts RX, repeats. Runs forever.
void WMBusComponent::rx_task_loop_() {
  // Make sure we're listening when the task starts.
  radio_.flush_rx_and_listen();

  for (;;) {
    // Wait up to 60 s for an IRQ. The timeout is just a watchdog — if the
    // chip got stuck (e.g. silent overflow) we re-arm RX so we don't end up
    // permanently deaf.
    if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000))) {
      radio_.flush_rx_and_listen();
      continue;
    }

    std::vector<uint8_t> raw;
    int8_t rssi_dbm = 0;
    bool ok = drain_one_frame_(raw, rssi_dbm);
    radio_.flush_rx_and_listen();

    if (!ok)
      continue;

    auto *pkt = new RxPacket{std::move(raw), rssi_dbm};
    if (xQueueSend(rx_queue_, &pkt, 0) != pdTRUE) {
      // Queue full — drop the packet (don't block the RX task).
      delete pkt;
    }
  }
}

// Drains exactly one wMBus T1 frame from the CC1101 RX FIFO.
//
// Approach: spin reading RXBYTES via SPI; whenever bytes are available,
// burst them out (leaving 1 byte in the FIFO mid-frame to avoid the well-
// known CC1101 RX-FIFO errata). Keep going until either the L-field-derived
// expected size has been reached, or the radio goes IDLE / overflows / we
// time out.
bool WMBusComponent::drain_one_frame_(std::vector<uint8_t> &out,
                                      int8_t &rssi_out) {
  out.clear();
  out.reserve(584);

  uint32_t deadline = millis() + 100;  // generous: 100 ms covers any T1 frame
  size_t expected_size = 0;
  uint8_t l_field = 0;
  bool have_length = false;
  bool rssi_captured = false;

  while ((int32_t)(millis() - deadline) < 0) {
    if (radio_.is_rx_overflow()) {
      ESP_LOGW(TAG, "RX FIFO overflow at %u/%u bytes",
               (unsigned)out.size(), (unsigned)expected_size);
      return false;
    }

    uint8_t avail = radio_.rx_bytes_available();

    // First chunk: capture RSSI right after sync word (most accurate).
    if (avail > 0 && !rssi_captured) {
      radio_.capture_rssi();
      rssi_out = radio_.rssi_dbm();
      rssi_captured = true;
    }

    // How much we can / should pull this iteration.
    size_t to_read = 0;
    if (have_length && out.size() + avail >= expected_size) {
      to_read = expected_size - out.size();  // final chunk: drain exactly
    } else if (avail > 1) {
      to_read = static_cast<size_t>(avail) - 1;  // mid-frame: keep 1 byte
    }

    if (to_read > 0) {
      size_t old = out.size();
      out.resize(old + to_read);
      radio_.read_burst(CC1101_FIFO, out.data() + old, to_read);

      // Decode the L field as soon as we have its 2 encoded bytes.
      if (!have_length && out.size() >= 2) {
        std::vector<uint8_t> first;
        if (!decode_3of6(out.data(), 2, first) || first.empty()) {
          ESP_LOGV(TAG, "Bad 3-of-6 in L-field, dropping frame");
          return false;
        }
        l_field = first[0];
        size_t decoded_total = expected_decoded_size_from_l(l_field);
        expected_size = encoded_size_3of6(decoded_total);
        if (expected_size < 12 || expected_size > 580) {
          ESP_LOGW(TAG, "Implausible expected size %u (L=%u), dropping",
                   (unsigned)expected_size, (unsigned)l_field);
          return false;
        }
        have_length = true;
        ESP_LOGV(TAG, "T1 frame: L=%u → decoded=%u, encoded=%u",
                 (unsigned)l_field, (unsigned)decoded_total,
                 (unsigned)expected_size);
      }

      if (have_length && out.size() >= expected_size) {
        out.resize(expected_size);  // trim any over-read
        return true;
      }
    } else {
      // Nothing to drain at this exact instant. Yield briefly without
      // blocking the rest of the system.
      delayMicroseconds(150);
    }
  }

  ESP_LOGV(TAG, "RX timeout (%u/%u bytes)", (unsigned)out.size(),
           (unsigned)expected_size);
  return false;
}

bool WMBusComponent::decode_t1_payload_(const std::vector<uint8_t> &raw_radio,
                                        std::vector<uint8_t> &decoded_frame) {
  if (!decode_3of6(raw_radio.data(), raw_radio.size(), decoded_frame))
    return false;
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
  ESP_LOGD(TAG, "    frame: %s", format_hex_pretty(frame).c_str());

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
  match->on_telegram(ci, payload, payload_len, rssi_dbm);
}

void WMBusSensor::on_telegram(uint8_t ci, const uint8_t *payload,
                              size_t payload_len, int8_t rssi_dbm) {
  if (rssi_)
    rssi_->publish_state((float)rssi_dbm);

  switch (meter_type_) {
    case MeterType::APATOR162: {
      Apator162Result r = decode_apator162(payload, payload_len, ci);
      if (r.ok && total_water_) {
        ESP_LOGI(TAG, "meter 0x%08X: total = %.3f m3 (RSSI %d dBm)",
                 (unsigned) meter_id_, r.total_m3, rssi_dbm);
        total_water_->publish_state((float)r.total_m3);
      } else if (!r.ok) {
        // Dump the raw payload so the failure can be diagnosed offline
        // (e.g. fed to https://wmbusmeters.org/analyze/).
        std::vector<uint8_t> hex(payload, payload + payload_len);
        ESP_LOGW(TAG,
                 "apator162: failed to decode (CI=0x%02X, len=%u). Payload: %s",
                 ci, (unsigned) payload_len, format_hex_pretty(hex).c_str());
      }
      break;
    }
  }
}

}  // namespace wmbus_cc1101
}  // namespace esphome
