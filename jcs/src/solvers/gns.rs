use std::marker::PhantomData;

use crate::transform::{IsFrameOfReference, Transform};

use super::Solver;

pub struct GroodAndSuntay<T: IsFrameOfReference, V: IsFrameOfReference> {
    relative_motion: Transform<T, V>,
    _from: PhantomData<T>,
    _to: PhantomData<V>,
}

impl<T, V> GroodAndSuntay<T, V> where T: IsFrameOfReference, V: IsFrameOfReference, {
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

    pub fn solve(&self, one: T, other: V) -> Motion {
        todo!()
    }
}
pub struct Motion {

}
