use std::marker::PhantomData;

use crate::{
    bone_to_tracker::{DefinedTracker, Femur, Side, Tibia},
    transform::{IsFrameOfReference, Transform},
};

use super::Solver;

pub struct GroodAndSuntay<T: IsFrameOfReference, V: IsFrameOfReference> {
    relative_motion: Transform<T, V>,
    _from: PhantomData<T>,
    _to: PhantomData<V>,
}

impl Solver<Femur, Tibia> for GroodAndSuntay<Femur, Tibia> where Tibia: DefinedTracker, Femur:DefinedTracker {
    fn solve(&self, rb2: Femur, rb1: Tibia) -> Motion {
        let tibia = rb1.in_global().unwrap();
        let femur = rb2.in_global().unwrap();

        let femur_i = femur.i();
        let e2 = tibia.k().cross(&femur_i).normalize();

        let flexion = (-e2).dot(&femur.k()).asin();
        let beta = femur_i.dot(&tibia.k()).acos();

        let (external, varus) = match rb2.side.unwrap() {
            Side::Right => ((-e2).dot(&tibia.i()), 90.0 - beta),
            Side::Left => (e2.dot(&tibia.i()).asin(), -(90.0 - beta)),
        };

        let h = rb1.origin() - rb2.origin();
        let lateral = match rb2.side.unwrap() {
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
    pub fn new(relative_motion: Transform<T, V>) -> Self
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
#[derive(Debug)]
pub struct Motion {
    flexion: f32,
    external: f32,
    varus: f32,
    anterior: f32,
    distal: f32,
    lateral: f32,
}
