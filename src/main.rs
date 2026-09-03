#![no_std]
#![no_main]

mod leveling;

use bt_hci::uuid::appearance;

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_nrf::rng;
use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Duration;
use leveling::{DeviceConfig, calculate_angles};
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

const L2CAP_TXQ: u8 = 3;
const L2CAP_RXQ: u8 = 3;
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2;

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

// CRC32 calculation for Phyphox experiment transfer verification
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

    // Build and send 15-byte Header Message
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

    //  Transmit file in 20-byte chunks
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

const LEVELING_SERVICE_UUID: Uuid = uuid!("a0000001-4493-41db-9609-eb926bd307c7");

#[gatt_service(uuid = LEVELING_SERVICE_UUID)]
struct LevelingService {
    #[characteristic(uuid = "a0000002-4493-41db-9609-eb926bd307c7", read, notify)]
    angles: [u8; 8],
    #[characteristic(
        uuid = "a0000003-4493-41db-9609-eb926bd307c7",
        write,
        write_without_response,
        read,
        notify,
        indicate
    )]
    config: [u8; 26],
}

#[gatt_server]
struct Server {
    phyphox: PhyphoxService,
    leveling: LevelingService,
}

// Dedicated 8 KB storage sector defined in memory.x (0x000FE000..0x00100000)
const STORAGE_RANGE: core::ops::Range<u32> = 0x000FE000..0x00100000;
const CONFIG_KEY: u8 = 1;

async fn load_config<'d>(flash: &mut nrf_mpsl::Flash<'d>) -> DeviceConfig {
    let mut buf = [0u8; 64];
    // Defensive Logic: fetch as slice &[u8] because sequential-storage panics on fixed array size mismatch.
    let res = sequential_storage::map::fetch_item::<u8, &[u8], _>(
        flash,
        STORAGE_RANGE,
        &mut sequential_storage::cache::NoCache::new(),
        &mut buf,
        &CONFIG_KEY,
    )
    .await;

    match res {
        Ok(Some(bytes)) => {
            if let Some(cfg) = DeviceConfig::from_bytes(bytes) {
                defmt::info!("Loaded config from flash -> {:?}", cfg);
                return cfg;
            }
        }
        Ok(None) => {
            defmt::info!("No stored config found in flash, using default config");
        }
        Err(e) => {
            defmt::warn!("Flash read error: {:?}", defmt::Debug2Format(&e));
        }
    }
    DeviceConfig::DEFAULT
}

async fn save_config<'d>(flash: &mut nrf_mpsl::Flash<'d>, config: &DeviceConfig) {
    let bytes = config.to_bytes();
    // Flash operations share radio timeslots via MPSL.
    // Retry with delay when ongoing BLE activity causes temporary ENOMEM errors.
    for attempt in 1..=5 {
        let mut buf = [0u8; 64];
        let res = sequential_storage::map::store_item::<u8, &[u8], _>(
            flash,
            STORAGE_RANGE,
            &mut sequential_storage::cache::NoCache::new(),
            &mut buf,
            &CONFIG_KEY,
            &&bytes[..],
        )
        .await;

        match res {
            Ok(()) => {
                defmt::info!("Saved config to flash (attempt {}): {:?}", attempt, config);
                return;
            }
            Err(e) => {
                defmt::warn!(
                    "Flash write attempt {} failed: {:?}",
                    attempt,
                    defmt::Debug2Format(&e)
                );
                embassy_time::Timer::after(Duration::from_millis(50)).await;
            }
        }
    }
    defmt::error!("Failed to save config to flash after 5 attempts");
}

const LSM6DS3_ADDRESS: u8 = 0x6A;
const LSM6DS3_WHO_AM_I_REG: u8 = 0x0F;
const LSM6DS3_CTRL1_XL: u8 = 0x10;
const LSM6DS3_CTRL8_XL: u8 = 0x17;
const LSM6DS3_OUTX_L_XL: u8 = 0x28;

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

    defmt::info!("START");
    // Enable power to the IMU (P1.08 on Seeed XIAO Sense must be HIGH)
    let mut imu_pwr = Output::new(p.P1_08, Level::Low, OutputDrive::HighDrive);
    embassy_time::Timer::after(Duration::from_millis(50)).await;
    imu_pwr.set_high();
    embassy_time::Timer::after(Duration::from_millis(100)).await;

    defmt::info!("IMU powered");

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

    let mpsl_p =
        mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    // The nrf_mpsl::Flash driver schedules flash operations inside timeslots to prevent
    // disrupting active BLE radio operations. Timeslot memory must be allocated on initialization.
    static SESSION_MEM: StaticCell<mpsl::SessionMem<1>> = StaticCell::new();
    let session_mem = SESSION_MEM.init(mpsl::SessionMem::new());

    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(defmt::unwrap!(
        mpsl::MultiprotocolServiceLayer::with_timeslots(mpsl_p, Irqs, lfclk_cfg, session_mem)
    ));

    let mut flash = nrf_mpsl::Flash::take(mpsl, p.NVMC);
    let dev_config =
        Mutex::<CriticalSectionRawMutex, _>::new(RefCell::new(load_config(&mut flash).await));

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

    let addr0 = embassy_nrf::pac::FICR.deviceaddr(0).read();
    let addr1 = embassy_nrf::pac::FICR.deviceaddr(1).read();
    let raw_addr = [
        addr0 as u8,
        (addr0 >> 8) as u8,
        (addr0 >> 16) as u8,
        (addr0 >> 24) as u8,
        addr1 as u8,
        (addr1 >> 8) as u8 | 0xC0,
    ];
    let address = Address::random(raw_addr);
    defmt::info!("Our address = {:?}", address);

    const HEX_CHARS: &[u8; 16] = b"0123456789ABCDEF";
    let dev_id = embassy_nrf::pac::FICR.deviceid(0).read();
    static DEVICE_NAME_BUF: static_cell::ConstStaticCell<[u8; 11]> =
        static_cell::ConstStaticCell::new([0; 11]);
    let name_buf = DEVICE_NAME_BUF.take();
    name_buf[0..8].copy_from_slice(b"Leveler ");
    name_buf[8] = HEX_CHARS[((dev_id >> 8) & 0xF) as usize];
    name_buf[9] = HEX_CHARS[((dev_id >> 4) & 0xF) as usize];
    name_buf[10] = HEX_CHARS[(dev_id & 0xF) as usize];
    let device_name: &'static str = core::str::from_utf8(name_buf).unwrap();
    defmt::info!("Broadcasting device name: {}", device_name);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources)
        .set_random_address(address)
        .build();
    let runner = stack.runner();
    let mut peripheral = stack.peripheral();

    defmt::info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: device_name,
        appearance: &appearance::sensor::GENERIC_SENSOR,
    }))
    .unwrap();

    let _ = server.set(
        &server.leveling.config,
        &dev_config.lock(|c| c.borrow().to_bytes()),
    );

    let _ = join(ble_task(runner), async {
        loop {
            let conn = advertise(device_name, &mut peripheral, &server)
                .await
                .unwrap();

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
                                let mut new_config: Option<DeviceConfig> = None;
                                let mut send_config = false;

                                if let GattEvent::Read(read_event) = &event {
                                    if read_event.handle() == server.leveling.config.handle {
                                        let cfg = dev_config.lock(|c| *c.borrow());
                                        defmt::info!("Phone read config -> {:?}", cfg);
                                    }
                                }

                                if let GattEvent::Write(write_event) = &event {
                                    let handle = write_event.handle();
                                    if handle == server.phyphox.experiment_ctrl.handle {
                                        write_event.with_data(|_offset, data| {
                                            if data.first() == Some(&1) {
                                                should_transfer = true;
                                            }
                                        });
                                    } else if handle == server.leveling.config.handle {
                                        write_event.with_data(|offset, data| {
                                            if let Some(cfg) = DeviceConfig::from_bytes(data) {
                                                let current_cfg = dev_config.lock(|c| *c.borrow());
                                                if cfg != current_cfg {
                                                    defmt::info!(
                                                        "Phone wrote new config (offset: {}, len: {}): {=[u8]}",
                                                        offset,
                                                        data.len(),
                                                        data
                                                    );
                                                    defmt::info!("New config applied -> {:?}", cfg);
                                                    new_config = Some(cfg);
                                                }
                                            } else {
                                                defmt::warn!(
                                                    "Failed to parse config payload from phone (offset: {}, len: {}): {=[u8]}",
                                                    offset,
                                                    data.len(),
                                                    data
                                                );
                                            }
                                        });
                                    } else if Some(handle) == server.phyphox.experiment_data.cccd_handle {
                                        write_event.with_data(|_offset, data| {
                                            let subscribed = data.iter().any(|&b| b != 0);
                                            defmt::info!("CCCD write on phyphox experiment_data (subscribed: {})", subscribed);
                                            if subscribed {
                                                should_transfer = true;
                                            }
                                        });
                                    } else if Some(handle) == server.leveling.config.cccd_handle {
                                        write_event.with_data(|_offset, data| {
                                            let subscribed = data.iter().any(|&b| b != 0);
                                            defmt::info!("CCCD write on leveling config (subscribed: {})", subscribed);
                                            if subscribed {
                                                send_config = true;
                                            }
                                        });
                                    } else if Some(handle) == server.leveling.angles.cccd_handle {
                                        write_event.with_data(|_offset, data| {
                                            let subscribed = data.iter().any(|&b| b != 0);
                                            defmt::info!("CCCD write on leveling angles (subscribed: {})", subscribed);
                                            if subscribed {
                                                send_config = true;
                                            }
                                        });
                                    }
                                }

                                let _ = event.accept();

                                if send_config {
                                    let cfg_bytes = dev_config.lock(|c| c.borrow().to_bytes());
                                    defmt::info!("Sending configuration on characteristic subscription...");
                                    let _ = server.leveling.config.indicate(&conn, &cfg_bytes, false).await;
                                    let _ = server.leveling.config.notify(&conn, &cfg_bytes, false).await;
                                }

                                if should_transfer {
                                    transfer_phyphox_experiment(&conn, &server).await;
                                }

                                if let Some(cfg) = new_config {
                                    // Save only when values change to prevent redundant flash operations and event loop blocking.
                                    let current_cfg = dev_config.lock(|c| *c.borrow());
                                    if cfg != current_cfg {
                                        dev_config.lock(|c| *c.borrow_mut() = cfg);
                                        let cfg_bytes = cfg.to_bytes();
                                        let _ = server.set(&server.leveling.config, &cfg_bytes);
                                        save_config(&mut flash, &cfg).await;
                                        defmt::info!("Sending updated config to client: {:?}", cfg);
                                        let _ = server.leveling.config.indicate(&conn, &cfg_bytes, false).await;
                                        let _ = server.leveling.config.notify(&conn, &cfg_bytes, false).await;
                                    }
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
                        let current_cfg = dev_config.lock(|c| *c.borrow());
                        if let Some(angles) = calculate_angles(&raw_data, &current_cfg) {
                            let angles_bytes = angles.to_bytes();
                            if let Err(e) =
                                server.leveling.angles.notify(&conn, &angles_bytes, false).await
                            {
                                defmt::warn!("Notify failed: {:?}", defmt::Debug2Format(&e));
                                break;
                            }
                        }
                    }
                },
            )
            .await;
        }
    })
    .await;
}

async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteServiceUuids128(&[LEVELING_SERVICE_UUID
                .as_raw()
                .try_into()
                .unwrap()]),
        ],
        &mut advertiser_data[..],
    )?;
    let mut scan_data = [0; 31];
    let scan_len = AdStructure::encode_slice(
        &[
            AdStructure::CompleteServiceUuids128(&[PHYPHOX_SERVICE_UUID
                .as_raw()
                .try_into()
                .unwrap()]),
            AdStructure::CompleteLocalName(name.as_bytes()),
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
