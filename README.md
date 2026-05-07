# esphome-wmbus-cc1101

ESPHome external component for reading Wireless M-Bus (wM-Bus) water-meter
telegrams with a **CC1101** radio module on an ESP32.

Built fresh for **ESPHome 2026.4.x** + **Arduino-ESP32 3.x**
(framework-arduinoespressif32 ≥ 3.3.8). Self-contained: it does **not**
override built-in ESPHome components and has no dependency on
`wmbus_common` / `wmbus_radio`.

## Supported

| What                | Status                                              |
| ------------------- | --------------------------------------------------- |
| Radio               | TI **CC1101** over SPI                              |
| Link mode           | wM-Bus **T1** at 868.950 MHz (3-of-6 encoded)       |
| Driver              | Apator **AT-WMBUS-16-2** (`apator162`)              |
| Encryption          | Mode 0 (unencrypted) — `key: "00…00"`               |
| Multiple meters     | Yes — one `sensor:` entry per meter                 |

Unencrypted apator162 meters are the original target. AES-128 (mode 5) is
intentionally not implemented — the decoder will emit a warning if it sees a
ciphertext frame it cannot read.

## Wiring (ESP32 ↔ CC1101)

| CC1101 pin | ESP32 GPIO | Note                            |
| ---------- | ---------- | ------------------------------- |
| MOSI       | GPIO23     | SPI MOSI                        |
| MISO       | GPIO19     | SPI MISO                        |
| SCK        | GPIO18     | SPI CLK                         |
| CSN        | GPIO5      | SPI CS (active low)             |
| GDO0       | GPIO4      | RX FIFO threshold IRQ (active low) |
| GDO2       | GPIO27     | Sync-word indicator (optional)  |
| VCC        | 3V3        | **Do not** wire to 5 V          |
| GND        | GND        |                                 |

GDO2 is wired but not strictly required; the component reads frames using
GDO0 (FIFO threshold) plus the chip's MARCSTATE.

## YAML

Add the component:

```yaml
external_components:
  - source: github://ryszardlet/esphome-wmbus-cc1101@main
    components: [ wmbus_cc1101 ]
```

Configure the radio:

```yaml
wmbus_cc1101:
  mosi_pin: GPIO23
  miso_pin: GPIO19
  clk_pin:  GPIO18
  cs_pin:   GPIO5
  gdo0_pin: GPIO4
  gdo2_pin: GPIO27
  frequency: 868.950   # MHz, optional, default 868.950
  log_unknown: true    # log telegrams that don't match any configured meter
```

Add one `sensor:` block per physical meter:

```yaml
sensor:
  - platform: wmbus_cc1101
    meter_id: 0x2541145
    type: apator162
    key: "00000000000000000000000000000000"
    total_water_m3:
      name: "Zimna woda dom"
      device_class: water
      state_class: total_increasing
      accuracy_decimals: 3
    rssi:
      name: "Zimna woda dom RSSI"
      entity_category: diagnostic
```

`meter_id` is the 4-byte ID printed on the meter, parsed little-endian
the same way wmbusmeters does it (i.e. the value you see in
`wmbusmeters --listenvs` output).

A full example with WiFi/API/OTA is in [`example.yaml`](example.yaml).

## How it works

1. CC1101 is configured for 100 kbps 2-FSK, sync word `0x543D`,
   variable packet length, no hardware CRC.
2. On every GDO0 IRQ the loop drains the RX FIFO byte by byte, working
   around the well-known CC1101 "≥1 byte must remain" errata.
3. Bytes are 3-of-6 decoded (T1 mode), block CRCs are validated using
   polynomial **0x3D65**, and the L/C/M/A/CI header is parsed.
4. The address field is matched against the configured `meter_id`s.
5. The matching meter's `apator162` decoder walks the manufacturer-specific
   register sequence, finds register `0x10` (total) and converts to m³.
6. RSSI captured at sync-word time is published alongside the volume.

## Compatibility notes

* Arduino-ESP32 3.x removed the `WiFiClient.h` umbrella header. This
  component does not include it and does not depend on it.
* No `components/esp32/` is shipped here — the built-in ESPHome platform
  is used as-is.
* The component does not depend on `wmbus_common` (which carries the full
  wmbusmeters decoder set and is the source of much of the breakage in
  other libraries on Arduino-ESP32 3.x).

## Acknowledgements

Algorithms (3-of-6 decoder, CC1101 register table, apator162 register
walk, wM-Bus block CRC) are derived from
[wmbusmeters](https://github.com/wmbusmeters/wmbusmeters) (GPL-3.0) and
[SzczepanLeon/esphome-components](https://github.com/SzczepanLeon/esphome-components)
(GPL-3.0). This repo is therefore distributed under **GPL-3.0-or-later**;
see [LICENSE](LICENSE).
