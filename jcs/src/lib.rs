#[macro_use]
extern crate approx;

mod transform;
mod bone_to_tracker;

use std::marker::PhantomData;

use transform::IsFrameOfReference;

#[derive(Debug)]
struct RigidBody<const ID: usize> {}
impl<const ID: usize> IsFrameOfReference for RigidBody<ID> {}


#[derive(Debug)]
struct Tracker <RB: IsFrameOfReference> (PhantomData<RB>);

impl<RB: IsFrameOfReference> IsFrameOfReference for Tracker<RB> {}

#[derive(Debug)]
struct Global;
impl IsFrameOfReference for Global {}

#[cfg(test)]
mod tests {
    use super::*;
    use super::transform::Transform;
    use bone_to_tracker::knee::{RBFemur, RBTibia};
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

        // println!("{:#?}", tibia_in_femur);
        // println!("{:#?}", femur_in_femoral_tracker);
    }
}
