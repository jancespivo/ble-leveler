#![no_std]
#![no_main]

use bt_hci::uuid::appearance;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_nrf::rng;
use embassy_nrf::twim::Twim;
use embassy_time::Duration;
use static_cell::StaticCell;
use trouble_host::Address;

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::peripherals::TWISPI1;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use trouble_host::BleHostError;
use trouble_host::advertise::AdStructure;
use trouble_host::advertise::Advertisement;
use trouble_host::advertise::BR_EDR_NOT_SUPPORTED;
use trouble_host::advertise::LE_GENERAL_DISCOVERABLE;
use trouble_host::gap::GapConfig;
use trouble_host::gap::PeripheralConfig;
use trouble_host::gatt::GattConnection;
use trouble_host::peripheral::Peripheral;
use trouble_host::prelude::*;
use {defmt_rtt as _, panic_probe as _};

embassy_nrf::bind_interrupts!(struct Irqs {
    RNG => embassy_nrf::rng::InterruptHandler<embassy_nrf::peripherals::RNG>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;

    TWISPI1 => embassy_nrf::twim::InterruptHandler<TWISPI1>;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

/// How many outgoing L2CAP buffers per link
const L2CAP_TXQ: u8 = 3;

/// How many incoming L2CAP buffers per link
const L2CAP_RXQ: u8 = 3;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<embassy_nrf::mode::Async>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?
        .support_adv()
        .support_peripheral()
        .peripheral_count(1)?
        .buffer_cfg(
            DefaultPacketPool::MTU as u16,
            DefaultPacketPool::MTU as u16,
            L2CAP_TXQ,
            L2CAP_RXQ,
        )?
        .build(p, rng, mpsl, mem)
}

const PHYPHOX_EXPERIMENT: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/leveling.zip"));

const PHYPHOX_SERVICE_UUID: Uuid = uuid!("cddf0001-30f7-4671-8b43-5e40ba53514a");
#[gatt_service(uuid = PHYPHOX_SERVICE_UUID)]
struct PhyphoxService {
    #[characteristic(uuid = "cddf0002-30f7-4671-8b43-5e40ba53514a", read, notify)]
    experiment_data: [u8; 20],
    #[characteristic(
        uuid = "cddf0003-30f7-4671-8b43-5e40ba53514a",
        write,
        write_without_response
    )]
    experiment_ctrl: u8,
}

// --- IEEE 802.3 CRC32 Calculation ---
fn calculate_crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
}

async fn transfer_phyphox_experiment<P: PacketPool>(
    conn: &GattConnection<'_, '_, P>,
    server: &Server<'_>,
) {
    defmt::info!(
        "Starting Phyphox XML transfer ({} bytes)",
        PHYPHOX_EXPERIMENT.len()
    );
    let crc = calculate_crc32(PHYPHOX_EXPERIMENT);

    // 1. Build & Send 15-byte Header Message
    let mut header = [0u8; 20];
    header[0..7].copy_from_slice(b"phyphox");
    header[7..11].copy_from_slice(&(PHYPHOX_EXPERIMENT.len() as u32).to_be_bytes());
    header[11..15].copy_from_slice(&crc.to_be_bytes());

    let _ = server
        .phyphox
        .experiment_data
        .notify(conn, &header, false)
        .await;
    embassy_time::Timer::after(Duration::from_millis(20)).await;

    // 2. Transmit File in 20-byte chunks
    for chunk in PHYPHOX_EXPERIMENT.chunks(20) {
        let mut packet = [0u8; 20];
        packet[..chunk.len()].copy_from_slice(chunk);
        let _ = server
            .phyphox
            .experiment_data
            .notify(conn, &packet, false)
            .await;
        embassy_time::Timer::after(Duration::from_millis(15)).await;
    }
    defmt::info!("Phyphox XML transfer complete");
}

// --- GATT Table Definition ---
#[gatt_server]
struct Server {
    phyphox: PhyphoxService,
    leveling: LevelingService,
}

const LEVELING_SERVICE_UUID: Uuid = uuid!("12345678-1234-5678-1234-56789abcdef0");

#[gatt_service(uuid = LEVELING_SERVICE_UUID)]
struct LevelingService {
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef1", read, notify)]
    angles: [u8; 6],
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef2", write, read)]
    tare_cmd: u8,
}

const LSM6DS3_ADDRESS: u8 = 0x6A;

const LSM6DS3_WHO_AM_I_REG: u8 = 0x0F;
const LSM6DS3_CTRL1_XL: u8 = 0x10;
const LSM6DS3_CTRL2_G: u8 = 0x11;

const LSM6DS3_STATUS_REG: u8 = 0x1E;

const LSM6DS3_CTRL6_C: u8 = 0x15;
const LSM6DS3_CTRL7_G: u8 = 0x16;
const LSM6DS3_CTRL8_XL: u8 = 0x17;

const LSM6DS3_OUTX_L_G: u8 = 0x22;
const LSM6DS3_OUTX_H_G: u8 = 0x23;
const LSM6DS3_OUTY_L_G: u8 = 0x24;
const LSM6DS3_OUTY_H_G: u8 = 0x25;
const LSM6DS3_OUTZ_L_G: u8 = 0x26;
const LSM6DS3_OUTZ_H_G: u8 = 0x27;

const LSM6DS3_OUTX_L_XL: u8 = 0x28;
const LSM6DS3_OUTX_H_XL: u8 = 0x29;
const LSM6DS3_OUTY_L_XL: u8 = 0x2A;
const LSM6DS3_OUTY_H_XL: u8 = 0x2B;
const LSM6DS3_OUTZ_L_XL: u8 = 0x2C;
const LSM6DS3_OUTZ_H_XL: u8 = 0x2D;

// --- MPU-6500 Reading & Angle Calculations ---
async fn read_raw_accel(twi: &mut Twim<'static>) -> [u8; 6] {
    let mut raw_data = [0u8; 6];
    twi.write_read(LSM6DS3_ADDRESS, &[LSM6DS3_OUTX_L_XL], &mut raw_data)
        .await
        .unwrap();

    raw_data
}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // I2C
    defmt::info!("START");
    // Enable power to the IMU (P1.08 must be HIGH)
    let mut imu_pwr = Output::new(p.P1_08, Level::Low, OutputDrive::HighDrive);
    embassy_time::Timer::after(Duration::from_millis(50)).await;
    imu_pwr.set_high();
    embassy_time::Timer::after(Duration::from_millis(100)).await;

    defmt::info!("IMU powered");

    // Initialize I2C (TWIM)
    let mut config = embassy_nrf::twim::Config::default();
    config.sda_pullup = true;
    config.scl_pullup = true;

    static RAM_BUFFER: static_cell::ConstStaticCell<[u8; 16]> =
        static_cell::ConstStaticCell::new([0; 16]);
    let mut twi = Twim::new(p.TWISPI1, Irqs, p.P0_07, p.P0_27, config, RAM_BUFFER.take());
    defmt::info!("I2C prepared");

    let reg = [LSM6DS3_WHO_AM_I_REG];
    let mut raw_data = [0u8; 1];
    twi.write_read(LSM6DS3_ADDRESS, &reg, &mut raw_data)
        .await
        .unwrap();
    defmt::info!("IMU prepared {}", raw_data);

    // Configure LSM6DS3: 104 Hz (ODR), +/- 2g scale
    twi.write(LSM6DS3_ADDRESS, &[LSM6DS3_CTRL1_XL, 0x4A])
        .await
        .unwrap();
    twi.write(LSM6DS3_ADDRESS, &[LSM6DS3_CTRL8_XL, 0x09])
        .await
        .unwrap();

    defmt::info!("IMU configured");

    // BLE

    let mpsl_p =
        mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(defmt::unwrap!(mpsl::MultiprotocolServiceLayer::new(
        mpsl_p, Irqs, lfclk_cfg
    )));

    defmt::info!("BLE prepared");
    spawner.spawn(defmt::unwrap!(mpsl_task(&*mpsl)));
    defmt::info!("BLE started");

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut rng = rng::Rng::new(p.RNG, Irqs);

    let mut sdc_mem = sdc::Mem::<4720>::new();
    let sdc = defmt::unwrap!(build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem));

    // GENERIC

    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    defmt::info!("Our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources)
        .set_random_address(address)
        .build();
    let runner = stack.runner();
    let mut peripheral = stack.peripheral();

    defmt::info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::sensor::GENERIC_SENSOR,
    }))
    .unwrap();
    let _ = join(ble_task(runner), async {
        loop {
            let conn = advertise("Leveler", &mut peripheral, &server)
                .await
                .unwrap();

            // Run the GATT event processor and the sensor notification loop concurrently
            let _ = select(
                // Task A: Process incoming GATT discovery and read/write requests
                async {
                    loop {
                        match conn.next().await {
                            GattConnectionEvent::Disconnected { .. } => {
                                defmt::info!("Device disconnected");
                                break;
                            }
                            GattConnectionEvent::Gatt { event } => {
                                let mut should_transfer = false;

                                // Inspect the incoming write event
                                if let GattEvent::Write(write_event) = &event {
                                    if write_event.handle() == server.phyphox.experiment_ctrl.handle
                                    {
                                        write_event.with_data(|_offset, data| {
                                            if data.first() == Some(&1) {
                                                should_transfer = true;
                                            }
                                        });
                                    }
                                }

                                // Acknowledge and accept the ATT request
                                let _ = event.accept();

                                // Execute the async transfer outside the synchronous closure
                                if should_transfer {
                                    transfer_phyphox_experiment(&conn, &server).await;
                                }
                            }
                            _ => {}
                        }
                    }
                },
                // Task B: Sensor measurement and notify loop
                async {
                    loop {
                        embassy_time::Timer::after(Duration::from_millis(50)).await;
                        let raw_data = read_raw_accel(&mut twi).await;

                        if let Err(e) = server.leveling.angles.notify(&conn, &raw_data, false).await
                        {
                            defmt::warn!("Notify failed: {:?}", defmt::Debug2Format(&e));
                            break;
                        }
                    }
                },
            )
            .await;
        }
    })
    .await;
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteServiceUuids128(&[PHYPHOX_SERVICE_UUID
                .as_raw()
                .try_into()
                .unwrap()]),
            // AdStructure::IncompleteServiceUuids16(&[[0x0f, 0x18]]),
            // AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let mut scan_data = [0; 31];
    let scan_len = AdStructure::encode_slice(
        &[
            AdStructure::CompleteLocalName(name.as_bytes()),
            AdStructure::CompleteServiceUuids128(&[LEVELING_SERVICE_UUID
                .as_raw()
                .try_into()
                .unwrap()]),
        ],
        &mut scan_data[..],
    )?;

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..adv_len],
                scan_data: &scan_data[..scan_len],
            },
        )
        .await?;
    defmt::info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    defmt::info!("[adv] connection established");
    Ok(conn)
}
