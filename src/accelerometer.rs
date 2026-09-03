use embassy_nrf::twim::Twim;

pub const LSM6DS3_ADDRESS: u8 = 0x6A;
pub const LSM6DS3_WHO_AM_I_REG: u8 = 0x0F;
pub const LSM6DS3_CTRL1_XL: u8 = 0x10;
pub const LSM6DS3_CTRL8_XL: u8 = 0x17;
pub const LSM6DS3_OUTX_L_XL: u8 = 0x28;

/// Initialize LSM6DS3 Accelerometer:
/// - 104 Hz ODR, +/- 2g scale
/// - On-chip LPF2 digital filter enabled (ODR/9 bandwidth for mechanical vibration rejection)
pub async fn init_accelerometer(twi: &mut Twim<'static>) {
    let reg = [LSM6DS3_WHO_AM_I_REG];
    let mut who_am_i = [0u8; 1];
    twi.write_read(LSM6DS3_ADDRESS, &reg, &mut who_am_i)
        .await
        .unwrap();
    defmt::info!("LSM6DS3 WHO_AM_I: {:#04x}", who_am_i[0]);

    // CTRL1_XL (0x10): 104 Hz (ODR 0100), +/- 2g scale (FS 00), BW 50 Hz (10) -> 0x4A
    twi.write(LSM6DS3_ADDRESS, &[LSM6DS3_CTRL1_XL, 0x4A])
        .await
        .unwrap();

    // CTRL8_XL (0x17): Enable LPF2 digital low-pass filter on accelerometer -> 0x09
    twi.write(LSM6DS3_ADDRESS, &[LSM6DS3_CTRL8_XL, 0x09])
        .await
        .unwrap();

    defmt::info!("LSM6DS3 Accelerometer configured (104 Hz, +/- 2g, LPF2 active)");
}

/// Read 6 consecutive bytes of raw accelerometer data (OUTX_L_XL .. OUTZ_H_XL):
/// - Bytes 0..2: Accel X (int16 Little-Endian)
/// - Bytes 2..4: Accel Y (int16 Little-Endian)
/// - Bytes 4..6: Accel Z (int16 Little-Endian)
pub async fn read_raw_accel(twi: &mut Twim<'static>) -> [u8; 6] {
    let mut raw_data = [0u8; 6];
    twi.write_read(LSM6DS3_ADDRESS, &[LSM6DS3_OUTX_L_XL], &mut raw_data)
        .await
        .unwrap();
    raw_data
}
