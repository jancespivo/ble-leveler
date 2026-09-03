use embassy_nrf::twim::Twim;

pub const LSM6DS3_ADDRESS: u8 = 0x6A;
pub const LSM6DS3_WHO_AM_I_REG: u8 = 0x0F;
pub const LSM6DS3_CTRL1_XL: u8 = 0x10;
pub const LSM6DS3_CTRL8_XL: u8 = 0x17;
pub const LSM6DS3_OUTX_L_XL: u8 = 0x28;

pub async fn init_accelerometer(twi: &mut Twim<'static>) {
    let reg = [LSM6DS3_WHO_AM_I_REG];
    let mut who_am_i = [0u8; 1];
    twi.write_read(LSM6DS3_ADDRESS, &reg, &mut who_am_i)
        .await
        .unwrap();
    defmt::info!("IMU prepared WHO_AM_I: {}", who_am_i);

    // Configure LSM6DS3: 104 Hz (ODR), +/- 2g scale
    twi.write(LSM6DS3_ADDRESS, &[LSM6DS3_CTRL1_XL, 0x4A])
        .await
        .unwrap();
    twi.write(LSM6DS3_ADDRESS, &[LSM6DS3_CTRL8_XL, 0x09])
        .await
        .unwrap();

    defmt::info!("IMU configured");
}

pub async fn read_raw_accel(twi: &mut Twim<'static>) -> [u8; 6] {
    let mut raw_data = [0u8; 6];
    twi.write_read(LSM6DS3_ADDRESS, &[LSM6DS3_OUTX_L_XL], &mut raw_data)
        .await
        .unwrap();
    raw_data
}
