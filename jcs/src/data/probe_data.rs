use nalgebra as na;

#[derive(PartialEq, Debug, Clone, Copy)]
pub struct ProbeData {
    translation: na::Vector3<f32>,
    rotation: na::UnitQuaternion<f32>,
}

impl std::fmt::Display for ProbeData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let e = self.rotation.euler_angles();
        write!(f, "{}Euler Angles: ({}, {}, {})", self.translation, e.0, e.1, e.2)
    }
}

#[derive(Clone, Copy)]
pub struct ProbeRawData {
    q0: f32,
    qx: f32,
    qy: f32,
    qz: f32,
    x: f32,
    y: f32,
    z: f32,
}

impl ProbeRawData {
    pub(crate) fn new( q0: f32, qx: f32, qy: f32, qz: f32, x: f32, y: f32, z: f32) -> ProbeRawData {
        Self {q0, qx, qy, qz, x, y, z}
    }
}
impl From<ProbeRawData> for ProbeData {
    fn from(value: ProbeRawData) -> Self {
        let translation = na::Vector3::new(value.x, value.y, value.z);
        let quaternion = na::Quaternion::new(value.q0, value.qx, value.qy, value.qz);
        let rotation = na::UnitQuaternion::new_normalize(quaternion);
        Self {
            translation,
            rotation,
        }
    }
}

impl ProbeData {
    pub fn translation(&self) -> &na::Vector3<f32>{
        &self.translation
    }
    pub fn rotation(&self) -> &na::UnitQuaternion<f32> {
        &self.rotation
    }
    pub fn new(probe_data: &ProbeRawData) -> Self {
        let translation = na::Vector3::new(probe_data.x, probe_data.y, probe_data.z);
        let rotation = na::UnitQuaternion::new_normalize(na::Quaternion::new(probe_data.q0, probe_data.qx, probe_data.qy, probe_data.qz));
        Self {
            translation,
            rotation,
        }
    }
    pub fn to_transform(self) -> na::Transform3<f32> {
        let rotation = self.rotation().to_homogeneous();
        let translation = na::Matrix4::new_translation(self.translation());
        let matrix = translation * rotation;
        na::Transform3::from_matrix_unchecked(matrix)
    }
}
