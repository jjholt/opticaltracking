use std::marker::PhantomData;
use nalgebra as na;

use crate::{
    bone_to_tracker::{DefinedTracker, Femur, Global, Side, Tibia},
    transform::{IsFrameOfReference, Transform},
};

pub struct GroodAndSuntay<T: IsFrameOfReference, V: IsFrameOfReference> {
    relative_motion: Transform<T, V>,
    _from: PhantomData<T>,
    _to: PhantomData<V>,
}

impl GroodAndSuntay<Femur, Tibia> {
    pub fn solve(femur: Transform<Global, Femur>, tibia: Transform<Global, Tibia>, side: Side) -> Motion {
        let femur_i = femur.i();
        let e2 = na::UnitVector3::new_normalize(tibia.k().cross(&femur_i));

        let flexion = (-e2).dot(&femur.k()).asin().to_degrees();
        let beta = femur_i.dot(&tibia.k()).acos().to_degrees();

        let (external, varus) = match side {
            Side::Right => ((-e2).dot(&tibia.i()).asin().to_degrees(), 90.0 - beta),
            Side::Left => (e2.dot(&tibia.i()).asin().to_degrees(), -(90.0 - beta)),
        };
        let h = tibia.origin() - femur.origin();
        let lateral = match side {
            Side::Right => h.dot(&femur_i),
            Side::Left => h.dot(&-femur_i),
        };
        let anterior = h.dot(&e2);
        let distal = -h.dot(&tibia.k());
        Motion {
            flexion,
            external,
            varus,
            anterior,
            lateral,
            distal,
        }
    }
}

impl<T, V> GroodAndSuntay<T, V>
where
    T: IsFrameOfReference,
    V: IsFrameOfReference,
{
    // pub fn new(relative_motion: Transform<T, V>) -> Self
    // where
    //     T: IsFrameOfReference,
    //     V: IsFrameOfReference,
    // {
    //     Self {
    //         relative_motion,
    //         _from: PhantomData,
    //         _to: PhantomData,
    //     }
    // }
}
#[derive(Debug)]
pub struct Motion {
    flexion: f32,
    external: f32,
    varus: f32,
    anterior: f32,
    distal: f32,
    lateral: f32,
}
