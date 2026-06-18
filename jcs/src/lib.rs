mod digitisation;
mod tracker;

#[cfg(feature = "knee")]
mod knee;

#[cfg(feature = "hip")]
mod hip;

use std::marker::PhantomData;

use attitude_determination::FrameOfReference;
use input::{Camera, Marker};

#[derive(PartialEq, Debug)]
pub struct RigidBody<const N: usize> {}

pub trait Position {}
pub trait Bone {}

impl<const N: usize> FrameOfReference for RigidBody<N> {}

pub struct Landmark <B: Bone, P: Position>{
    bone: PhantomData<B>,
    position:  PhantomData<P>
}
