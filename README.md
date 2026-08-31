# Groover - Automated Irrigation System

Groover is an embedded firmware for a multi-channel automated plant watering system, built with Rust and the `embassy` asynchronous framework. It is designed to run on the Raspberry Pi Pico W.

The system monitors soil moisture and water levels, and is designed to control pumps or valves to water plants automatically.

## Core Functionality

-   **Multi-Channel Control:** The system manages three independent channels, allowing it to care for multiple plants or zones.
-   **Soil Moisture Sensing:** Reads data from three analog soil moisture sensors to determine when to water.
-   **Water Level Detection:** Monitors three digital level switches to detect if the water reservoir is low.
-   **Motor/Pump Control:** Provides PWM control for two motors (e.g., pumps or valves) using the RP2040's Programmable I/O (PIO) for precise control.
-   **Asynchronous Operation:** Built on the `embassy` framework, using separate asynchronous tasks for each channel to handle logic concurrently and efficiently.
-   **Real-Time Clock:** Includes a real-time clock (RTC) module that can be synchronized with network time, enabling scheduled watering routines.

## Hardware Requirements

-   **Microcontroller:** Raspberry Pi Pico W
-   **Motors:** 2x DC motors (for pumps or valves)
-   **Motor Driver:** A motor driver IC/board compatible with the motors and the Pico's 3.3V logic.
-   **Sensors:**
    -   3x Analog soil moisture sensors
    -   3x Digital level switches (e.g., float switches)

## Software Architecture

The firmware uses `embassy` to manage hardware and concurrency. The main loop periodically polls all sensors (3x moisture, 3x level switches). Sensor data is then passed to dedicated asynchronous tasks (`cerpadlo`, meaning "pump") for each of the three channels.

These tasks are intended to contain the logic for activating the motors based on the sensor readings. The `motors` module abstracts the PIO-based PWM control, providing a simple API to start, stop, and change the speed of the motors.

## Building and Flashing

### Prerequisites

1.  **Install Rust:** Follow the instructions at [rustup.rs](https://rustup.rs/).
2.  **Add Target:** Add the ARM Cortex-M0+ target required for the RP2040.
    ```sh
    rustup target add thumbv6m-none-eabi
    ```
3.  **Install Probe-RS:** This tool is used for flashing and debugging the microcontroller.
    ```sh
    cargo install probe-rs --features cli
    ```

### Build

Build the firmware in release mode:

```sh
cargo build --release
```

### Flash

1.  Connect the Raspberry Pi Pico W to your computer while holding the `BOOTSEL` button to put it in bootloader mode.
2.  Flash the firmware using `probe-rs`:
    ```sh
    probe-rs run --chip RP2040
    ```

## Project Structure

-   `src/main.rs`: Main application entry point. Handles initialization, sensor polling, and spawning of async tasks.
-   `src/motors.rs`: Module for motor control using PIO-based PWM.
-   `src/rtc.rs`: Module for real-time clock management.
-   `build.rs`: A build script, likely used to handle the wireless firmware.
-   `memory.x`: The linker script that defines the memory layout for the target hardware.
-   `Cargo.toml`: Defines the project's dependencies and metadata.
-   `rust-toolchain.toml`: Specifies the exact Rust toolchain to use for consistent builds.
-   `.cargo/config.toml`: Sets the default `probe-rs` runner for `cargo run`.
-   `docs/`: Contains project documentation, such as pinout diagrams.
-   `.gitignore`: Specifies files and directories for Git to ignore.