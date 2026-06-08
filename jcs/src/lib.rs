mod digitisation;
mod landmark;
mod tracker;

#[cfg(feature = "knee")]
mod knee;

#[cfg(feature = "hip")]
mod hip;

use attitude_determination::FrameOfReference;
use input::{Camera, Marker};

#[derive(PartialEq)]
pub struct RigidBody<const N: usize> {}

impl<const N: usize> FrameOfReference for RigidBody<N> {}
