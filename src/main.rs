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
    angles: [u8; 6],
    #[characteristic(
        uuid = "a0000003-4493-41db-9609-eb926bd307c7",
        write,
        write_without_response,
        read,
        notify
    )]
    config: [u8; 18],
}

#[gatt_server]
struct Server {
    phyphox: PhyphoxService,
    leveling: LevelingService,
}

// Dedicated 8 KB storage sector defined in memory.x (0x000FE000..0x00100000)
const STORAGE_RANGE: core::ops::Range<u32> = 0x000FE000..0x00100000;
const CONFIG_KEY: u8 = 1;

#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
pub struct DeviceConfig {
    pub usbc_dir: u8,
    pub top_dir: u8,
    pub pitch_offset: f32,
    pub roll_offset: f32,
    pub th_front: f32,
    pub th_rear: f32,
    pub th_left: f32,
    pub th_right: f32,
}

impl DeviceConfig {
    pub const DEFAULT: Self = Self {
        usbc_dir: 2, // Rear
        top_dir: 5,  // Up
        pitch_offset: 0.0,
        roll_offset: 0.0,
        th_front: 1.5,
        th_rear: 1.5,
        th_left: 1.5,
        th_right: 1.5,
    };

    // The BLE write payload limit without MTU exchange is 20 bytes.
    // We scale thresholds to centidegrees (i16) to fit the whole structure into 18 bytes.
    pub fn to_bytes(&self) -> [u8; 18] {
        let mut bytes = [0u8; 18];
        bytes[0] = self.usbc_dir;
        bytes[1] = self.top_dir;
        bytes[2..6].copy_from_slice(&self.pitch_offset.to_le_bytes());
        bytes[6..10].copy_from_slice(&self.roll_offset.to_le_bytes());
        let f = (self.th_front * 100.0) as i16;
        let r = (self.th_rear * 100.0) as i16;
        let l = (self.th_left * 100.0) as i16;
        let rg = (self.th_right * 100.0) as i16;
        bytes[10..12].copy_from_slice(&f.to_le_bytes());
        bytes[12..14].copy_from_slice(&r.to_le_bytes());
        bytes[14..16].copy_from_slice(&l.to_le_bytes());
        bytes[16..18].copy_from_slice(&rg.to_le_bytes());
        bytes
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 10 {
            return None;
        }
        let usbc_dir = bytes[0];
        let top_dir = bytes[1];
        if !(1..=6).contains(&usbc_dir) || !(1..=6).contains(&top_dir) {
            return None;
        }
        let pitch_offset = f32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
        let roll_offset = f32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]);

        // Support legacy 10-byte flash records without thresholds from older firmware versions.
        let (th_front, th_rear, th_left, th_right) = if bytes.len() >= 18 {
            let f = i16::from_le_bytes([bytes[10], bytes[11]]) as f32 / 100.0;
            let r = i16::from_le_bytes([bytes[12], bytes[13]]) as f32 / 100.0;
            let l = i16::from_le_bytes([bytes[14], bytes[15]]) as f32 / 100.0;
            let rg = i16::from_le_bytes([bytes[16], bytes[17]]) as f32 / 100.0;
            (
                if f > 0.0 && !f.is_nan() { f } else { 1.5 },
                if r > 0.0 && !r.is_nan() { r } else { 1.5 },
                if l > 0.0 && !l.is_nan() { l } else { 1.5 },
                if rg > 0.0 && !rg.is_nan() { rg } else { 1.5 },
            )
        } else {
            (1.5, 1.5, 1.5, 1.5)
        };

        Some(Self {
            usbc_dir,
            top_dir,
            pitch_offset,
            roll_offset,
            th_front,
            th_rear,
            th_left,
            th_right,
        })
    }
}

async fn load_config<'d>(flash: &mut nrf_mpsl::Flash<'d>) -> DeviceConfig {
    let mut buf = [0u8; 64];
    let res = sequential_storage::map::fetch_item::<u8, [u8; 18], _>(
        flash,
        STORAGE_RANGE,
        &mut sequential_storage::cache::NoCache::new(),
        &mut buf,
        &CONFIG_KEY,
    )
    .await;

    match res {
        Ok(Some(bytes)) => {
            if let Some(cfg) = DeviceConfig::from_bytes(&bytes) {
                defmt::info!(
                    "Loaded config from flash -> USB-C: {}, Top: {}, Pitch Offset: {} deg, Roll Offset: {} deg, Thresholds [Front: {} deg, Rear: {} deg, Left: {} deg, Right: {} deg]",
                    cfg.usbc_dir,
                    cfg.top_dir,
                    cfg.pitch_offset,
                    cfg.roll_offset,
                    cfg.th_front,
                    cfg.th_rear,
                    cfg.th_left,
                    cfg.th_right
                );
                return cfg;
            }
        }
        Ok(None) => {
            // Check for a 10-byte legacy record before falling back to default values.
            let legacy_res = sequential_storage::map::fetch_item::<u8, [u8; 10], _>(
                flash,
                STORAGE_RANGE,
                &mut sequential_storage::cache::NoCache::new(),
                &mut buf,
                &CONFIG_KEY,
            )
            .await;
            if let Ok(Some(legacy_bytes)) = legacy_res {
                if let Some(cfg) = DeviceConfig::from_bytes(&legacy_bytes) {
                    defmt::info!(
                        "Loaded legacy config from flash -> USB-C: {}, Top: {}, Pitch Offset: {} deg, Roll Offset: {} deg, Thresholds [Front: {} deg, Rear: {} deg, Left: {} deg, Right: {} deg]",
                        cfg.usbc_dir,
                        cfg.top_dir,
                        cfg.pitch_offset,
                        cfg.roll_offset,
                        cfg.th_front,
                        cfg.th_rear,
                        cfg.th_left,
                        cfg.th_right
                    );
                    return cfg;
                }
            }
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
        let res = sequential_storage::map::store_item::<u8, [u8; 18], _>(
            flash,
            STORAGE_RANGE,
            &mut sequential_storage::cache::NoCache::new(),
            &mut buf,
            &CONFIG_KEY,
            &bytes,
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
    let mut dev_config = load_config(&mut flash).await;

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
    static DEVICE_NAME_BUF: static_cell::ConstStaticCell<[u8; 10]> =
        static_cell::ConstStaticCell::new([0; 10]);
    let name_buf = DEVICE_NAME_BUF.take();
    name_buf[0..6].copy_from_slice(b"Level ");
    name_buf[6] = HEX_CHARS[((dev_id >> 12) & 0xF) as usize];
    name_buf[7] = HEX_CHARS[((dev_id >> 8) & 0xF) as usize];
    name_buf[8] = HEX_CHARS[((dev_id >> 4) & 0xF) as usize];
    name_buf[9] = HEX_CHARS[(dev_id & 0xF) as usize];
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

    let _ = server.set(&server.leveling.config, &dev_config.to_bytes());

    let _ = join(ble_task(runner), async {
        loop {
            let conn = advertise(device_name, &mut peripheral, &server)
                .await
                .unwrap();

            // Transmit stored sensor configuration to smartphone upon connection
            let _ = server.leveling.config.notify(&conn, &dev_config.to_bytes(), false).await;

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

                                if let GattEvent::Read(read_event) = &event {
                                    if read_event.handle() == server.leveling.config.handle {
                                        defmt::info!(
                                            "Phone read config -> USB-C: {}, Top: {}, Pitch Offset: {} deg, Roll Offset: {} deg, Thresholds [Front: {} deg, Rear: {} deg, Left: {} deg, Right: {} deg]",
                                            dev_config.usbc_dir,
                                            dev_config.top_dir,
                                            dev_config.pitch_offset,
                                            dev_config.roll_offset,
                                            dev_config.th_front,
                                            dev_config.th_rear,
                                            dev_config.th_left,
                                            dev_config.th_right
                                        );
                                    }
                                }

                                if let GattEvent::Write(write_event) = &event {
                                    if write_event.handle() == server.phyphox.experiment_ctrl.handle
                                    {
                                        write_event.with_data(|_offset, data| {
                                            if data.first() == Some(&1) {
                                                should_transfer = true;
                                            }
                                        });
                                    } else if write_event.handle() == server.leveling.config.handle
                                    {
                                        write_event.with_data(|offset, data| {
                                            if let Some(cfg) = DeviceConfig::from_bytes(data) {
                                                if cfg != dev_config {
                                                    defmt::info!(
                                                        "Phone wrote new config (offset: {}, len: {}): {=[u8]}",
                                                        offset,
                                                        data.len(),
                                                        data
                                                    );
                                                    defmt::info!(
                                                        "New config applied -> USB-C: {}, Top: {}, Pitch Offset: {} deg, Roll Offset: {} deg, Thresholds [Front: {} deg, Rear: {} deg, Left: {} deg, Right: {} deg]",
                                                        cfg.usbc_dir,
                                                        cfg.top_dir,
                                                        cfg.pitch_offset,
                                                        cfg.roll_offset,
                                                        cfg.th_front,
                                                        cfg.th_rear,
                                                        cfg.th_left,
                                                        cfg.th_right
                                                    );
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
                                    }
                                }

                                let _ = event.accept();

                                if should_transfer {
                                    transfer_phyphox_experiment(&conn, &server).await;
                                }

                                if let Some(cfg) = new_config {
                                    // Save only when values change to prevent redundant flash operations and event loop blocking.
                                    if cfg != dev_config {
                                        dev_config = cfg;
                                        let _ = server.set(&server.leveling.config, &dev_config.to_bytes());
                                        save_config(&mut flash, &dev_config).await;
                                        let _ = server.leveling.config.notify(&conn, &dev_config.to_bytes(), false).await;
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
