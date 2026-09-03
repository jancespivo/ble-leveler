use core::f32::consts::PI;

#[derive(Copy, Clone, Debug, PartialEq, defmt::Format)]
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

    pub fn to_bytes(&self) -> [u8; 26] {
        let mut bytes = [0u8; 26];
        bytes[0] = self.usbc_dir;
        bytes[1] = self.top_dir;
        bytes[2..6].copy_from_slice(&self.pitch_offset.to_le_bytes());
        bytes[6..10].copy_from_slice(&self.roll_offset.to_le_bytes());
        bytes[10..14].copy_from_slice(&self.th_front.to_le_bytes());
        bytes[14..18].copy_from_slice(&self.th_rear.to_le_bytes());
        bytes[18..22].copy_from_slice(&self.th_left.to_le_bytes());
        bytes[22..26].copy_from_slice(&self.th_right.to_le_bytes());
        bytes
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() >= 26 {
            let usbc_dir = bytes[0];
            let top_dir = bytes[1];
            if !(1..=6).contains(&usbc_dir) || !(1..=6).contains(&top_dir) {
                return None;
            }
            let pitch_offset = f32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
            let roll_offset = f32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]);

            let f = f32::from_le_bytes([bytes[10], bytes[11], bytes[12], bytes[13]]);
            let r = f32::from_le_bytes([bytes[14], bytes[15], bytes[16], bytes[17]]);
            let l = f32::from_le_bytes([bytes[18], bytes[19], bytes[20], bytes[21]]);
            let rg = f32::from_le_bytes([bytes[22], bytes[23], bytes[24], bytes[25]]);

            let th_front = if f > 0.0 && !f.is_nan() { f } else { 1.5 };
            let th_rear = if r > 0.0 && !r.is_nan() { r } else { 1.5 };
            let th_left = if l > 0.0 && !l.is_nan() { l } else { 1.5 };
            let th_right = if rg > 0.0 && !rg.is_nan() { rg } else { 1.5 };

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
        } else if bytes.len() == 18 {
            // Backward Compatibility: Decode legacy 18-byte format (thresholds scaled in centidegrees).
            let usbc_dir = bytes[0];
            let top_dir = bytes[1];
            if !(1..=6).contains(&usbc_dir) || !(1..=6).contains(&top_dir) {
                return None;
            }
            let pitch_offset = f32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
            let roll_offset = f32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]);

            let f = i16::from_le_bytes([bytes[10], bytes[11]]) as f32 / 100.0;
            let r = i16::from_le_bytes([bytes[12], bytes[13]]) as f32 / 100.0;
            let l = i16::from_le_bytes([bytes[14], bytes[15]]) as f32 / 100.0;
            let rg = i16::from_le_bytes([bytes[16], bytes[17]]) as f32 / 100.0;

            let th_front = if f > 0.0 && !f.is_nan() { f } else { 1.5 };
            let th_rear = if r > 0.0 && !r.is_nan() { r } else { 1.5 };
            let th_left = if l > 0.0 && !l.is_nan() { l } else { 1.5 };
            let th_right = if rg > 0.0 && !rg.is_nan() { rg } else { 1.5 };

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
        } else {
            None
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, defmt::Format)]
pub struct Angles {
    pub pitch: f32,
    pub roll: f32,
}

impl Angles {
    pub fn to_bytes(&self) -> [u8; 8] {
        let mut bytes = [0u8; 8];
        bytes[0..4].copy_from_slice(&self.pitch.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.roll.to_le_bytes());
        bytes
    }
}

/// 3-dimensional Cartesian vector for coordinate basis and acceleration operations.
#[derive(Copy, Clone, Debug, PartialEq, defmt::Format)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    #[inline]
    pub fn cross(self, rhs: Self) -> Self {
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }
}

/// Convert cardinal direction index (1..=6) to unit vector in vehicle coordinates.
/// 1=Front, 2=Rear, 3=Left, 4=Right, 5=Up, 6=Down.
pub fn cardinal_vector(dir: u8) -> Option<Vec3> {
    match dir {
        1 => Some(Vec3::new(0.0, 1.0, 0.0)),  // Front (+Y)
        2 => Some(Vec3::new(0.0, -1.0, 0.0)), // Rear (-Y)
        3 => Some(Vec3::new(-1.0, 0.0, 0.0)), // Left (-X)
        4 => Some(Vec3::new(1.0, 0.0, 0.0)),  // Right (+X)
        5 => Some(Vec3::new(0.0, 0.0, 1.0)),  // Up (+Z)
        6 => Some(Vec3::new(0.0, 0.0, -1.0)), // Down (-Z)
        _ => None,
    }
}

/// Compute the 3D sensor basis vectors (Xs, Ys, Zs) in vehicle frame coordinates.
///
/// Physical Sensor Alignment:
/// - USB-C port is along -X_s
/// - Top label is along +Z_s
/// - Lateral axis is along +Y_s = u_usbc x u_top
pub fn compute_basis_vectors(usbc_dir: u8, top_dir: u8) -> Option<(Vec3, Vec3, Vec3)> {
    let u_usbc = cardinal_vector(usbc_dir)?;
    let u_top = cardinal_vector(top_dir)?;

    // Strict Orthogonality Check: Mounting selections must be perpendicular (dot product == 0).
    if u_usbc.dot(u_top) != 0.0 {
        return None;
    }

    let x_s = Vec3::new(-u_usbc.x, -u_usbc.y, -u_usbc.z);
    let z_s = u_top;
    let y_s = u_usbc.cross(u_top);

    Some((x_s, y_s, z_s))
}

/// Transform raw accelerometer readings into pitch and roll angles with tare offset applied.
/// Returns `None` if the mounting configuration is invalid or non-orthogonal.
pub fn calculate_angles(raw_accel_bytes: &[u8; 6], config: &DeviceConfig) -> Option<Angles> {
    let (x_s, y_s, z_s) = compute_basis_vectors(config.usbc_dir, config.top_dir)?;

    let raw = Vec3::new(
        i16::from_le_bytes([raw_accel_bytes[0], raw_accel_bytes[1]]) as f32,
        i16::from_le_bytes([raw_accel_bytes[2], raw_accel_bytes[3]]) as f32,
        i16::from_le_bytes([raw_accel_bytes[4], raw_accel_bytes[5]]) as f32,
    );

    // Project raw acceleration into vehicle coordinates using vector dot products:
    // a_lat  = raw . X_s
    // a_long = raw . Y_s
    // a_vert = raw . Z_s
    let a_lat = raw.dot(x_s);
    let a_long = raw.dot(y_s);
    let a_vert = raw.dot(z_s);

    let rad_to_deg = 180.0 / PI;

    // Longitudinal acceleration is inverted so the crosshair moves toward the lower side (ball physics).
    let pitch_calc =
        libm::atan2f(-a_long, libm::sqrtf(a_lat * a_lat + a_vert * a_vert)) * rad_to_deg;
    let roll_calc = libm::atan2f(-a_lat, a_vert) * rad_to_deg;

    let pitch_now = pitch_calc - config.pitch_offset;
    let roll_now = roll_calc - config.roll_offset;

    Some(Angles {
        pitch: pitch_now,
        roll: roll_now,
    })
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn test_cardinal_vectors() {
        assert_eq!(cardinal_vector(1), Some(Vec3::new(0.0, 1.0, 0.0))); // Front
        assert_eq!(cardinal_vector(2), Some(Vec3::new(0.0, -1.0, 0.0))); // Rear
        assert_eq!(cardinal_vector(3), Some(Vec3::new(-1.0, 0.0, 0.0))); // Left
        assert_eq!(cardinal_vector(4), Some(Vec3::new(1.0, 0.0, 0.0))); // Right
        assert_eq!(cardinal_vector(5), Some(Vec3::new(0.0, 0.0, 1.0))); // Up
        assert_eq!(cardinal_vector(6), Some(Vec3::new(0.0, 0.0, -1.0))); // Down
        assert_eq!(cardinal_vector(0), None);
        assert_eq!(cardinal_vector(7), None);
    }

    #[test]
    fn test_basis_vectors_default() {
        // Default: USB-C is Rear (2), Top is Up (5)
        let (x_s, y_s, z_s) = compute_basis_vectors(2, 5).expect("valid basis vectors");
        assert_eq!(x_s, Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(y_s, Vec3::new(-1.0, 0.0, 0.0));
        assert_eq!(z_s, Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn test_invalid_parallel_configurations() {
        // Parallel vectors (Front, Front) -> None
        assert_eq!(compute_basis_vectors(1, 1), None);
        // Anti-parallel vectors (Front, Rear) -> None
        assert_eq!(compute_basis_vectors(1, 2), None);
        // Parallel vectors (Up, Up) -> None
        assert_eq!(compute_basis_vectors(5, 5), None);
        // Out-of-bounds -> None
        assert_eq!(compute_basis_vectors(0, 5), None);
    }

    #[test]
    fn test_all_24_cube_orientations_orthonormal() {
        let mut valid_count = 0;
        let mut invalid_count = 0;

        for usbc in 1..=6 {
            for top in 1..=6 {
                match compute_basis_vectors(usbc, top) {
                    Some((x_s, y_s, z_s)) => {
                        // Length of each basis vector must be 1.0
                        assert_eq!(x_s.dot(x_s), 1.0);
                        assert_eq!(y_s.dot(y_s), 1.0);
                        assert_eq!(z_s.dot(z_s), 1.0);

                        // Dot products must be 0.0 (mutually orthogonal)
                        assert_eq!(x_s.dot(y_s), 0.0);
                        assert_eq!(y_s.dot(z_s), 0.0);
                        assert_eq!(z_s.dot(x_s), 0.0);

                        // Right-handed determinant must be +1.0
                        let det = x_s.dot(y_s.cross(z_s));
                        assert_eq!(det, 1.0);

                        valid_count += 1;
                    }
                    None => {
                        invalid_count += 1;
                    }
                }
            }
        }
        // 6 faces * 4 valid perpendicular orientations per face = 24 valid orientations
        assert_eq!(valid_count, 24);
        // 36 total pairs - 24 valid = 12 invalid (6 parallel + 6 opposite)
        assert_eq!(invalid_count, 12);
    }

    #[test]
    fn test_calculate_angles_flat_level() {
        let cfg = DeviceConfig::DEFAULT;
        // Flat sensor: 1g on sensor +Z
        let mut raw = [0u8; 6];
        let az: i16 = 16384; // 1g in 2g scale
        raw[4..6].copy_from_slice(&az.to_le_bytes());

        let angles = calculate_angles(&raw, &cfg).expect("valid angles");
        assert!((angles.pitch - 0.0).abs() < 0.01);
        assert!((angles.roll - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_calculate_angles_invalid_config_returns_none() {
        let mut cfg = DeviceConfig::DEFAULT;
        cfg.usbc_dir = 1; // Front
        cfg.top_dir = 1; // Front (invalid collinear config)

        let mut raw = [0u8; 6];
        let az: i16 = 16384;
        raw[4..6].copy_from_slice(&az.to_le_bytes());

        assert_eq!(calculate_angles(&raw, &cfg), None);
    }

    #[test]
    fn test_calculate_angles_with_tare() {
        let mut cfg = DeviceConfig::DEFAULT;
        cfg.pitch_offset = 2.5;
        cfg.roll_offset = -1.0;

        let mut raw = [0u8; 6];
        let az: i16 = 16384;
        raw[4..6].copy_from_slice(&az.to_le_bytes());

        let angles = calculate_angles(&raw, &cfg).expect("valid angles");
        assert!((angles.pitch - (-2.5)).abs() < 0.01);
        assert!((angles.roll - 1.0).abs() < 0.01);
    }
}
