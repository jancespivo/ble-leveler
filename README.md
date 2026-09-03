# ble-leveler

[![License](https://img.shields.io/badge/License-MIT--or--Apache--2.0-blue.svg)](#license)
[![Language: Rust](https://img.shields.io/badge/Language-Rust-orange.svg)](https://www.rust-lang.org/)
[![Framework: Embassy](https://img.shields.io/badge/Framework-Embassy-purple.svg)](https://embassy.dev/)

Hassle-free Bluetooth leveling device and mobile interface. Powered by Rust, Bluetooth Low Energy (BLE), and the open-source Phyphox app.

**ble-leveler** is a wireless leveling solution for the Nordic nRF52840 microcontroller (Seeed Studio XIAO BLE Sense) and LSM6DS3 IMU. It operates as a self-describing sensor device that transmits an interactive dual-axis crosshair display directly to your smartphone with no custom app store installation.

---

## Use Cases

- **Campervans and Motorhomes (RVs):** Level the vehicle at campsites and position leveling ramps with live crosshair feedback from the driver seat.
- **Overland and Off-Road Vehicles:** Level 4x4 vehicles and rooftop tents on uneven wilderness terrain.
- **Trailers and Caravans:** Verify tongue level, hitch alignment, and uniform cargo weight distribution during loading and unhitching.
- **Mobile Workshops and Field Labs:** Align mobile workbenches, portable machine tools, and 3D printers in service vans.
- **Marine and Small Craft:** Measure static boat list and trim balance while docked or at anchor.
- **Portable Solar Setups:** Align portable ground panels to flat reference surfaces or targeted tilt angles.

---

## Key Features

- **Zero App Installation:** Works with the free [Phyphox app](https://phyphox.org/). The sensor transmits the full user interface over Bluetooth on first connection.
- **Universal 24-Orientation Mounting:** Mount the sensor in any direction. The system calculates vehicle pitch and roll automatically.
- **Persistent Calibration:** Saves mounting orientation, level reference points, and tolerance limits in non-volatile memory.
- **Visual Crosshair Display:** Live dual-axis crosshair display moves to the lower side of the vehicle for intuitive ramp positioning.
- **Low Power Consumption:** Uses Bluetooth Low Energy for long operational battery life.

---

## Documentation

For system architecture, hardware interface details, coordinate mathematics, BLE GATT specifications, and storage design, see [ARCHITECTURE.md](ARCHITECTURE.md).

---

## Hardware Requirements

- **Microcontroller Board:** Seeed Studio XIAO BLE Sense (Nordic nRF52840 with LSM6DS3 IMU).
- **Power Supply:** 5V USB-C or 3.7V lithium-polymer battery.
- **Mobile Device:** iOS or Android device with Bluetooth Low Energy and the free [Phyphox application](https://phyphox.org/).
- **Debug Probe:** SWD debug probe supported by probe-rs (such as J-Link, CMSIS-DAP, or ST-Link).

---

## Quick Start

### 1. Prerequisites

- Install the [Rust toolchain](https://rustup.rs/) with the `thumbv7em-none-eabi` target:
  ```sh
  rustup target add thumbv7em-none-eabi
  ```
- Install [probe-rs](https://probe.rs/) for flashing and runtime logs:
  ```sh
  cargo install probe-rs-tools --locked
  ```

### 2. Build and Flash Firmware

Connect your debug probe to the Seeed Studio XIAO BLE Sense SWD pins and run:

```sh
cargo run --release
```

The build script compresses `ble-leveler.phyphox` into the binary automatically, flashes the target, and starts `defmt` logging.

---

## Operation

1. **Open Phyphox:** Start the Phyphox application on your mobile device.
2. **Scan for Sensor:** Tap the **+** button, choose **Add experiment for Bluetooth device**, and select **Leveler**.
3. **Configure Mounting Orientation:**
   - Set **USB-C Port Points To** (Front, Rear, Left, Right, Up, Down).
   - Set **Top Label Points To** (Front, Rear, Left, Right, Up, Down).
4. **Set Zero Reference (Optional):** Park the vehicle on a known level surface and tap **Set Level (Zero Reference)** to store offset calibration in flash memory.
5. **Set Tolerance Thresholds:** Adjust front, rear, left, and right threshold limits for visual level indicators.

---

## License

This project is dual-licensed under either:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <https://www.apache.org/licenses/LICENSE-2.0>)
- MIT License ([LICENSE-MIT](LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.
