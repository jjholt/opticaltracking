use std::marker::PhantomData;
use super::orientation::*;

use crate::IsFrameOfReference;
use nalgebra as na;

#[derive(PartialEq, Debug, Clone, Copy)]
pub struct Position {
    translation: na::Vector3<f64>,
    rotation: na::UnitQuaternion<f64>,
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

#[derive(Debug)]
pub struct Landmark<RB: IsFrameOfReference, O: Orientation> {
    probe_name: String,  // Comes from data
    probe_label: String, // Comes from config
    position: Position,
    bone: PhantomData<RB>,
    orientation: PhantomData<O>,
}

impl<RB: IsFrameOfReference, O: Orientation> std::fmt::Display for Landmark<RB, O> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}\nbone: {:?}\norientation: {:?}\n", self.position, self.bone, self.orientation)
    }
}

impl<RB: IsFrameOfReference, O: Orientation> Landmark<RB, O> {
    pub fn probe_name(&self) -> &str {
        &self.probe_name
    }
    pub fn probe_label(&self) -> &str {
        &self.probe_label
    }
    pub fn translations(&self) -> &na::Vector3<f64> {
        &self.position.translation
    }
    pub fn rotations(&self) -> &na::Quaternion<f64> {
        &self.position.rotation
    }
    #[cfg(test)]
    pub(crate) fn new(probe_name: &str, probe_label: &str, position: Position) -> Self {
        Self { probe_name: probe_name.to_string(), probe_label: probe_label.to_string(), position, bone: PhantomData, orientation: PhantomData }
    }
}


impl Position {
    pub fn new(probe_data: &ProbeData) -> Self {
        let translation = na::Vector3::new(probe_data.x, probe_data.y, probe_data.z);
        let rotation = na::UnitQuaternion::new_normalize(na::Quaternion::new(probe_data.q0, probe_data.qx, probe_data.qy, probe_data.qz));
        Self {
            translation,
            rotation,
        }
    }
    pub fn to_transform(self) -> na::Transform3<f64> {
        let rotation = self.rotation.to_homogeneous();
        let translation = na::Matrix4::new_translation(&self.translation);
        let matrix = translation * rotation;
        na::Transform3::from_matrix_unchecked(matrix)
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


#[cfg(test)]
mod valid_landmarks {

    use crate::bone_to_tracker::knee::RBTibia;

    use super::*;
    #[test]
    fn convert_probe_data_to_pos() {
        let datum = ProbeData {
            q0: 1.0,
            qy: 1.0,
            qx: 1.0,
            qz: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let probe_data: Position = datum.into();
        assert_eq!(probe_data, Position::new(&datum))
    }
    #[test]
    fn create_tibia() {
        let position = ProbeData { q0: 1.0, qy: 1.0, qx: 1.0, qz: 1.0, x: 0.0, y: 0.0, z: 0.0, }.into();
        let tibia_medial: Landmark<RBTibia, Medial> = Landmark {
            probe_name: "Black Probe".to_string(),
            probe_label: "Probe".to_string(),
            position,
            bone: PhantomData,
            orientation: PhantomData,
        };
        let tibia_lateral: Landmark<RBTibia, Lateral> = Landmark {
            probe_name: "Black Probe".to_string(),
            probe_label: "Probe".to_string(),
            position,
            bone: PhantomData,
            orientation: PhantomData,
        };
        let tibia_distal: Landmark<RBTibia, Distal> = Landmark {
            probe_name: "Black Probe".to_string(),
            probe_label: "Probe".to_string(),
            position,
            bone: PhantomData,
            orientation: PhantomData,
        };
    }

}
