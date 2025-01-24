#![allow(unused)]
#[macro_use]
extern crate approx;

mod bone_to_tracker;
mod transform;
mod data;
mod solvers;

use std::marker::PhantomData;

use bone_to_tracker::{Distal, Global, Landmark, Lateral, Medial, Proximal, Side};
use transform::{IsFrameOfReference, Transform};

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
    tracker: Option<Transform<Global, Tracker<RigidBody<ID>>>>,
}


#[derive(Debug)]
pub struct Tracker<RB: IsFrameOfReference>(PhantomData<RB>);

impl<const ID: usize> IsFrameOfReference for RigidBody<ID> {}
impl<const ID: usize> IsRigidBody for RigidBody<ID> {}

impl<RB: IsFrameOfReference> Marker for Tracker<RB> {}
impl<RB: IsFrameOfReference> IsFrameOfReference for Tracker<RB> {}

impl Marker for Probe {}


#[cfg(test)]
mod tests {
    use super::transform::Transform;
    use super::*;
    use bone_to_tracker::{knee::{Femur, Tibia}, Global};
    use nalgebra as na;
    use transform::Mldivide;

    #[test]
    fn new_transform() {
        let transf = na::Transform3::identity();
        let tibia = Transform::<Global, Tibia>::new(transf);
        let femur = Transform::<Global, Femur>::new(transf);

        let femoral_tracker = Transform::<Global, Tracker<Femur>>::new(transf);

        let tibia_in_femur = femur.mldivide(&tibia);
        let femur_in_femoral_tracker = femoral_tracker.mldivide(&femur);

        // Load data
        // Tie tracker data to bone
        // femoral tracker in global (data) * femur.in_tracker(&g_t_ft) (femur in femoral tracker) = femur in global (data)
        // tibial tracker in global (data) * tibia.in_tracker(&g_t_ft) (tibia in tibial tracker) = tibia in global (data)
        // femur in global mldivide tibia in global = tibia in femur
        //
        // JCS
        // need unit vectors for each landmark, e.g. x-axis in gTf (data) and their relative
        // position to each other (fTt)
    }
}
