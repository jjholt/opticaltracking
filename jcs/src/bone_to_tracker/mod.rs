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
pub use knee::{Femur, Patella, RBFemur, RBPatella, RBTibia, Tibia};

use crate::transform::{IsFrameOfReference, Transform};

#[derive(Debug)]
pub(crate) struct Global;
impl IsFrameOfReference for Global {}

#[derive(Clone, Copy)]
pub enum Side {
    Right,
    Left,
}
trait Bone {
    type Body: IsFrameOfReference;
    fn fixed_frame(&self) -> Transform<'_, Global, Self::Body>;
}
