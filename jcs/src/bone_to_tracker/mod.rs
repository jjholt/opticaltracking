#[cfg(feature = "hip")]
pub mod hip;
#[cfg(feature = "knee")]
pub mod knee;
mod landmark;
mod orientation;
#[cfg(feature = "shoulder")]
pub mod shoulder;

pub use landmark::Landmark;
pub use orientation::*;

#[cfg(feature = "knee")]
pub use knee::{Femur, Patella, Tibia};

use crate::transform::gT;
use crate::transform::IsFrameOfReference;

#[derive(Debug)]
pub struct Global;
impl IsFrameOfReference for Global {}

#[derive(Clone, Copy, Debug)]
pub enum Side {
    Right,
    Left,
}
#[derive(Debug)]
pub struct Motion {
    flexion: f32,
    external: f32,
    varus: f32,
    anterior: f32,
    distal: f32,
    lateral: f32,
}

pub struct Kinematics(Vec<Motion>);
pub trait DefinedTracker
where
    Self: IsFrameOfReference + Sized,
{
    fn in_global(&self) -> gT<Self>;
}
