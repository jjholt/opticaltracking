use nalgebra as na;

#[derive(PartialEq, Debug, Clone, Copy)]
pub struct Position {
    translation: na::Vector3<f64>,
    rotation: na::UnitQuaternion<f64>,
}

impl Position {
}

impl std::fmt::Display for Position {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let e = self.rotation.euler_angles();
        write!(f, "{}Euler Angles: ({}, {}, {})", self.translation, e.0, e.1, e.2)
    }
}

#[derive(Clone, Copy)]
pub struct ProbeData {
    q0: f64,
    qx: f64,
    qy: f64,
    qz: f64,
    x: f64,
    y: f64,
    z: f64,
}

impl ProbeData {
    pub(crate) fn new( q0: f64, qx: f64, qy: f64, qz: f64, x: f64, y: f64, z: f64) -> ProbeData {
        Self {q0, qx, qy, qz, x, y, z}
    }
}
impl From<ProbeData> for Position {
    fn from(value: ProbeData) -> Self {
        let translation = na::Vector3::new(value.x, value.y, value.z);
        let rotation = na::Quaternion::new(value.q0, value.qx, value.qy, value.qz);
        let rotation = na::UnitQuaternion::new_normalize(rotation);
        Self {
            translation,
            rotation,
        }
    }
}

impl Position {
    pub fn translation(&self) -> &na::Vector3<f64>{
        &self.translation
    }
    pub fn rotation(&self) -> &na::UnitQuaternion<f64> {
        &self.rotation
    }
    pub fn new(probe_data: &ProbeData) -> Self {
        let translation = na::Vector3::new(probe_data.x, probe_data.y, probe_data.z);
        let rotation = na::UnitQuaternion::new_normalize(na::Quaternion::new(probe_data.q0, probe_data.qx, probe_data.qy, probe_data.qz));
        Self {
            translation,
            rotation,
        }
    }
    pub fn to_transform(self) -> na::Transform3<f64> {
        let rotation = self.rotation().to_homogeneous();
        let translation = na::Matrix4::new_translation(self.translation());
        let matrix = translation * rotation;
        na::Transform3::from_matrix_unchecked(matrix)
    }
}
