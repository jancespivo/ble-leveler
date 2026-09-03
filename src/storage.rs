use crate::leveling::DeviceConfig;
use embassy_time::Duration;

// Dedicated 8 KB storage sector defined in memory.x (0x000FE000..0x00100000)
pub const STORAGE_RANGE: core::ops::Range<u32> = 0x000FE000..0x00100000;
pub const CONFIG_KEY: u8 = 1;

pub async fn load_config<'d>(flash: &mut nrf_mpsl::Flash<'d>) -> DeviceConfig {
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

pub async fn save_config<'d>(flash: &mut nrf_mpsl::Flash<'d>, config: &DeviceConfig) {
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
