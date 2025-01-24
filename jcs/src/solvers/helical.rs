use std::marker::PhantomData;

use crate::{
    transform::{IsFrameOfReference, Transform},
    RigidBody,
};

pub struct HelicalAxis<'a, T: IsFrameOfReference, V: IsFrameOfReference> {
    relative_motion: &'a Transform<T, V>,
    _from: PhantomData<T>,
    _to: PhantomData<V>,
}

impl<'a, T, V> HelicalAxis<'a, T, V> where T: IsFrameOfReference, V: IsFrameOfReference {
    pub fn new(relative_motion: &'a Transform<T, V>) -> Self where T: IsFrameOfReference, V: IsFrameOfReference {
        Self {relative_motion, _from: PhantomData, _to: PhantomData}
    }
}

impl<const I: usize, const J: usize> Transform<RigidBody<I>, RigidBody<J>> {
    pub fn helical(&self) -> HelicalAxis<RigidBody<I>, RigidBody<J>> {
        HelicalAxis::new(self)
    }
}
