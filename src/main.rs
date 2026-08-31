#![no_std]
#![no_main]

use bt_hci::uuid::appearance;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::rng;
use embassy_nrf::twim::Twim;
use embassy_time::Duration;
use static_cell::StaticCell;
use trouble_host::Address;

use defmt::*;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::peripherals::TWISPI0;
use micromath::F32Ext;
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

    TWISPI0 => embassy_nrf::twim::InterruptHandler<TWISPI0>;
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

// --- GATT Table Definition ---
#[gatt_server]
struct Server {
    leveling: LevelingService,
}

#[gatt_service(uuid = "12345678-1234-5678-1234-56789abcdef0")]
struct LevelingService {
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef1", read, notify)]
    angles: [u8; 4],
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef2", write, read)]
    tare_cmd: u8,
}

const IMU_ADDR: u8 = 0x6A;
const CTRL1_XL: u8 = 0x10;
const OUTX_L_XL: u8 = 0x28;

// --- MPU-6500 Reading & Angle Calculations ---
async fn read_mpu_angles(twi: &mut Twim<'static>) -> (f32, f32) {
    let rad_to_deg = 180.0 / 3.14159265;
    let mut raw_data = [0u8; 6];

    // 4. Read 6 raw bytes: X, Y, Z acceleration (2 bytes each, little-endian)
    twi.write_read(IMU_ADDR, &[OUTX_L_XL], &mut raw_data)
        .await
        .unwrap();

    let ax = i16::from_le_bytes([raw_data[0], raw_data[1]]) as f32;
    let ay = i16::from_le_bytes([raw_data[2], raw_data[3]]) as f32;
    let az = i16::from_le_bytes([raw_data[4], raw_data[5]]) as f32;

    // 5. Calculate Pitch and Roll angles
    let pitch = (ay).atan2((ax * ax + az * az).sqrt()) * rad_to_deg;
    let roll = (-ax).atan2(az) * rad_to_deg;
    (pitch, roll)
}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            #[cfg(feature = "defmt")]
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // I2C

    // 1. Enable power to the IMU (P1.08 must be HIGH)
    let _imu_pwr = Output::new(p.P1_08, Level::High, OutputDrive::Standard);
    embassy_time::Timer::after(Duration::from_millis(20)).await;

    // 2. Initialize I2C (TWIM)
    let config = embassy_nrf::twim::Config::default();
    static RAM_BUFFER: static_cell::ConstStaticCell<[u8; 16]> =
        static_cell::ConstStaticCell::new([0; 16]);
    let mut twi = Twim::new(p.TWISPI0, Irqs, p.P0_04, p.P0_05, config, RAM_BUFFER.take());

    // 3. Configure LSM6DS3: 104 Hz (ODR), +/- 2g scale
    let setup_buf = [CTRL1_XL, 0x40];
    twi.write(IMU_ADDR, &setup_buf).await.unwrap();

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
    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::new(
        mpsl_p, Irqs, lfclk_cfg
    )));
    spawner.spawn(unwrap!(mpsl_task(&*mpsl)));

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut rng = rng::Rng::new(p.RNG, Irqs);

    let mut sdc_mem = sdc::Mem::<4720>::new();
    let sdc = unwrap!(build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem));

    // GENERIC

    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("Our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources)
        .set_random_address(address)
        .build();
    let runner = stack.runner();
    let mut peripheral = stack.peripheral();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();
    let _ = join(ble_task(runner), async {
        loop {
            let conn = advertise("Trouble Example", &mut peripheral, &server)
                .await
                .unwrap();

            let pitch_offset = 0.0f32;
            let roll_offset = 0.0f32;
            loop {
                let (pitch, roll) = read_mpu_angles(&mut twi).await;

                let adj_pitch = ((pitch - pitch_offset) * 100.0) as i16;
                let adj_roll = ((roll - roll_offset) * 100.0) as i16;

                // Encode payload: [pitch_lo, pitch_hi, roll_lo, roll_hi]
                let payload: [u8; 4] = [
                    adj_pitch as u8,
                    (adj_pitch >> 8) as u8,
                    adj_roll as u8,
                    (adj_roll >> 8) as u8,
                ];
                server
                    .leveling
                    .angles
                    .notify(&conn, &payload, true)
                    .await
                    .unwrap(); // FIXME
            }
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
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::IncompleteServiceUuids16(&[[0x0f, 0x18]]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    Ok(conn)
}
