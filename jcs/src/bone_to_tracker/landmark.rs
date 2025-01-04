use std::marker::PhantomData;
use super::orientation::*;

use crate::{IsFrameOfReference};
use crate::data::{Position, ProbeData};
use nalgebra as na;

#[derive(Debug)]
pub struct Landmark<RB: IsFrameOfReference, O: Orientation> {
    // Landmark should be a Datum for Probe  + bone stuff
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
        self.position.translation()
    }
    pub fn rotations(&self) -> &na::Quaternion<f64> {
        self.position.rotation()
    }
    #[cfg(test)]
    pub(crate) fn new(probe_name: &str, probe_label: &str, position: Position) -> Self {
        Self { probe_name: probe_name.to_string(), probe_label: probe_label.to_string(), position, bone: PhantomData, orientation: PhantomData }
    }
}


#[cfg(test)]
mod valid_landmarks {

    use crate::{bone_to_tracker::knee::RBTibia, data::ProbeData};

    use super::*;
    #[test]
    fn convert_probe_data_to_pos() {
        let datum = ProbeData::new(1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0) ;
        let probe_data: Position = datum.into();
        assert_eq!(probe_data, Position::new(&datum))
    }
    #[test]
    fn create_tibia() {
        let position = ProbeData::new(1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0).into();
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
