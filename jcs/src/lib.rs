#![allow(unused)]
#[macro_use]
extern crate approx;

mod bone_to_tracker;
mod data;
mod solvers;
mod transform;

use std::marker::PhantomData;

use bone_to_tracker::{Distal, Landmark, Lateral, Medial, Proximal, Side};
use transform::{gT, IsFrameOfReference};

use crate::bone_to_tracker::Orientation;

pub trait IsRigidBody {}
pub trait Marker {}

pub struct Probe;

#[derive(Debug)]
enum FarLandmark {
    Distal,
    Proximal,
}

#[derive(Debug)]
pub struct RigidBody<const ID: usize> {
    side: Side,
    medial: Landmark<RigidBody<ID>, Medial>,
    lateral: Landmark<RigidBody<ID>, Lateral>,
    far_landmark: Landmark<RigidBody<ID>, FarLandmark>,
    tracker: gT<Tracker<RigidBody<ID>>>,
}

#[derive(Debug)]
pub struct Tracker<RB: IsFrameOfReference>(PhantomData<RB>);

impl<const ID: usize> IsFrameOfReference for RigidBody<ID> {}
impl<const ID: usize> IsRigidBody for RigidBody<ID> {}

impl<RB: IsFrameOfReference> Marker for Tracker<RB> {}
impl<RB: IsFrameOfReference> IsFrameOfReference for Tracker<RB> {}

impl Marker for Probe {}
impl Orientation for FarLandmark {}
