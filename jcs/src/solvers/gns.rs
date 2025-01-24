use std::marker::PhantomData;

use crate::{bone_to_tracker::{Femur, Global, Side, Tibia}, transform::{IsFrameOfReference, Transform}, RigidBody};

use super::{LocalFloat, Motion, Solver};

use nalgebra as na;

pub struct GroodAndSuntay<'a, T: IsFrameOfReference, V: IsFrameOfReference> {
    relative_motion: &'a Transform<T, V>,
    _from: PhantomData<T>,
    _to: PhantomData<V>,
}

impl<'a> GroodAndSuntay<'a, Femur, Tibia> {
    pub fn solve(&self, femur: Transform<Global, Femur>, tibia: Transform<Global,Tibia>, side: Side) -> Motion {
        let fTt = self.relative_motion;
        let e1_ = na::UnitVector3::new_normalize(femur.i());
        let e3_ = na::UnitVector3::new_normalize(tibia.k());
        let e2_ = na::UnitVector3::new_normalize(e3_.cross(&e1_).normalize());

        // Flexion
        let rx = (-e2_).dot(&femur.k()).asin();


        // External
        let ry = match side {
            Side::Right => (-e2_).dot(&tibia.i()).asin(),
            Side::Left => (e2_).dot(&tibia.i()).asin(),
        };
        // Varus
        let rz = {
            let beta = femur.i().dot(&tibia.k()).asin();
            match side {
            Side::Right => std::f32::consts::FRAC_PI_2 - beta,
            Side::Left => -(std::f32::consts::FRAC_PI_2 - beta),
            }
        };

        todo!("Include femur.origin()")

    }
}

impl<'a, T, V> GroodAndSuntay<'a, T, V> where T: IsFrameOfReference, V: IsFrameOfReference, {
    pub fn new(relative_motion: &'a Transform<T, V>) -> Self
    where
        T: IsFrameOfReference,
        V: IsFrameOfReference,
    {
        Self {
            relative_motion,
            _from: PhantomData,
            _to: PhantomData,
        }
    }


}



impl <const I: usize, const J: usize> Transform<RigidBody<I>,RigidBody<J>> {
    pub fn gns(&self) -> GroodAndSuntay<RigidBody<I>, RigidBody<J>> {
        GroodAndSuntay::new(self)
    }
}

