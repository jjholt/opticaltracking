#![allow(unused)]
#[macro_use]
extern crate approx;

mod bone_to_tracker;
mod transform;
mod data;
mod solvers;

use std::marker::PhantomData;

use bone_to_tracker::{Distal, Landmark, Lateral, Medial, Proximal, Side};
use transform::{gT, IsFrameOfReference};

pub trait IsRigidBody {}
pub trait Marker {}

pub struct Probe;

#[derive(Debug, Default)]
pub struct RigidBody<const ID: usize> {
    side: Option<Side>,
    medial: Option<Landmark<RigidBody<ID>, Medial>>,
    lateral: Option<Landmark<RigidBody<ID>, Lateral>>,
    distal: Option<Landmark<RigidBody<ID>, Distal>>,
    proximal: Option<Landmark<RigidBody<ID>, Proximal>>,
    tracker: Option<gT<Tracker<RigidBody<ID>>>>,
}


#[derive(Debug)]
pub struct Tracker<RB: IsFrameOfReference>(PhantomData<RB>);

impl<const ID: usize> IsFrameOfReference for RigidBody<ID> {}
impl<const ID: usize> IsRigidBody for RigidBody<ID> {}

impl<RB: IsFrameOfReference> Marker for Tracker<RB> {}
impl<RB: IsFrameOfReference> IsFrameOfReference for Tracker<RB> {}

impl Marker for Probe {}
