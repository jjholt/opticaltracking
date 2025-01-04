#![allow(unused)]
#[macro_use]
extern crate approx;

mod bone_to_tracker;
mod transform;
mod data;

use std::marker::PhantomData;

use transform::IsFrameOfReference;

pub trait IsRigidBody {}
pub trait Marker {}

pub struct Probe;

#[derive(Debug)]
struct RigidBody<const ID: usize> {}

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
    use bone_to_tracker::{knee::{RBFemur, RBTibia}, Global};
    use nalgebra as na;
    use transform::Mldivide;

    #[test]
    fn new_transform() {
        let transf = na::Transform3::identity();
        let tibia = Transform::<Global, RBTibia>::new(transf);
        let femur = Transform::<Global, RBFemur>::new(transf);

        let femoral_tracker = Transform::<Global, Tracker<RBFemur>>::new(transf);

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
