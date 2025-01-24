use std::marker::PhantomData;
use super::orientation::*;

use crate::IsFrameOfReference;
use crate::data::{ProbeData, ProbeRawData};
use nalgebra as na;

#[derive(Debug)]
pub struct Landmark<RB: IsFrameOfReference, O: Orientation> {
    // Landmark should be a Datum for Probe  + bone stuff
    probe_name: String,  // Comes from data
    probe_label: String, // Comes from config
    position: ProbeData,
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
    pub fn translations(&self) -> &na::Vector3<f32> {
        self.position.translation()
    }
    pub fn rotations(&self) -> &na::Quaternion<f32> {
        self.position.rotation()
    }
    // #[cfg(test)]
    pub(crate) fn new(probe_name: &str, probe_label: &str, position: ProbeData) -> Self {
        Self { probe_name: probe_name.to_string(), probe_label: probe_label.to_string(), position, bone: PhantomData, orientation: PhantomData }
    }
}
