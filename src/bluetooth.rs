use embassy_nrf::rng;
use embassy_time::Duration;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc};
use trouble_host::BleHostError;
use trouble_host::advertise::AdStructure;
use trouble_host::advertise::Advertisement;
use trouble_host::advertise::BR_EDR_NOT_SUPPORTED;
use trouble_host::advertise::LE_GENERAL_DISCOVERABLE;
use trouble_host::gatt::GattConnection;
use trouble_host::peripheral::Peripheral;
use trouble_host::prelude::*;

pub const L2CAP_TXQ: u8 = 3;
pub const L2CAP_RXQ: u8 = 3;
pub const CONNECTIONS_MAX: usize = 1;
pub const L2CAP_CHANNELS_MAX: usize = 2;

pub fn build_sdc<'d, const N: usize>(
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

pub const PHYPHOX_EXPERIMENT: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/leveling.zip"));

pub const PHYPHOX_SERVICE_UUID: Uuid = uuid!("cddf0001-30f7-4671-8b43-5e40ba53514a");

#[gatt_service(uuid = PHYPHOX_SERVICE_UUID)]
pub struct PhyphoxService {
    #[characteristic(uuid = "cddf0002-30f7-4671-8b43-5e40ba53514a", read, notify)]
    pub experiment_data: [u8; 20],
    #[characteristic(
        uuid = "cddf0003-30f7-4671-8b43-5e40ba53514a",
        write,
        write_without_response
    )]
    pub experiment_ctrl: u8,
}

// CRC32 calculation for Phyphox experiment transfer verification
pub fn calculate_crc32(data: &[u8]) -> u32 {
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

pub async fn transfer_phyphox_experiment<P: PacketPool>(
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

    // Transmit file in 20-byte chunks
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

pub const LEVELING_SERVICE_UUID: Uuid = uuid!("a0000001-4493-41db-9609-eb926bd307c7");

#[gatt_service(uuid = LEVELING_SERVICE_UUID)]
pub struct LevelingService {
    #[characteristic(uuid = "a0000002-4493-41db-9609-eb926bd307c7", read, notify)]
    pub angles: [u8; 8],
    #[characteristic(
        uuid = "a0000003-4493-41db-9609-eb926bd307c7",
        write,
        write_without_response,
        read,
        notify,
        indicate
    )]
    pub config: [u8; 26],
}

#[gatt_server]
pub struct Server {
    pub phyphox: PhyphoxService,
    pub leveling: LevelingService,
}

pub async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

pub async fn advertise<'values, 'server, C: Controller>(
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
