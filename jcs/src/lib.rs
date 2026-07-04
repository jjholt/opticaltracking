mod digitisation;

#[cfg(feature = "knee")]
mod knee;

#[cfg(feature = "hip")]
mod hip;

/// Attitude determination
use attitude_determination::FrameOfReference;
#[derive(PartialEq, Debug)]
pub struct RigidBody<const N: usize> {}
impl<const N: usize> FrameOfReference for RigidBody<N> {}
