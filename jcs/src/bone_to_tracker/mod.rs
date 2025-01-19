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

use crate::transform::Mldivide;
use crate::{
    transform::{IsFrameOfReference, Transform},
    Tracker,
};

#[derive(Debug)]
pub struct Global;
impl IsFrameOfReference for Global {}

#[derive(Clone, Copy, Debug)]
pub enum Side {
    Right,
    Left,
}
pub trait DefinedTracker
where
    Self: IsFrameOfReference + Sized,
{
    fn fixed_frame(&self) -> Option<Transform<'_, Global, Self>>;
    fn in_tracker<'a>( &'a self, tracker: &'_ Transform<'_, Global, Tracker<Self>>) -> Option<Transform<'_, Tracker<Self>, Self>> {
        Some(tracker.mldivide(&self.fixed_frame()?))
    }
    fn in_global(&self) -> Option<Transform<'_, Global, Self>> {
        self.fixed_frame()
    }
}
