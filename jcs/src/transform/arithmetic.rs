use nalgebra as na;
use std::{marker::PhantomData, ops};

pub trait IsFrameOfReference {}

#[derive(Debug)]
pub struct Transform<'a, T, V>
where
    T: IsFrameOfReference,
    V: IsFrameOfReference,
{
    data: na::Transform3<f64>,
    _from: PhantomData<&'a T>,
    _to: PhantomData<&'a V>,
}

impl<'a, T, V> std::fmt::Display for Transform<'a, T, V>
where
    T: IsFrameOfReference,
    V: IsFrameOfReference,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.data.to_homogeneous())
    }
}
pub trait Mldivide<Rhs> {
    type Output;
    fn mldivide(&self, rhs: &Rhs) -> Self::Output;
}

impl<'b, A, G, B> ops::Mul<Transform<'b, G, B>> for Transform<'_, A, G>
where
    A: IsFrameOfReference + 'b,
    G: IsFrameOfReference,
    B: IsFrameOfReference,
{
    type Output = Transform<'b, A, B>;

    fn mul(self, rhs: Transform<'b, G, B>) -> Self::Output {
        let data = self.data * rhs.data; 
        Transform {
            data,
            _from: PhantomData,
            _to: PhantomData,
        }
    }
}


impl<'b, T, G, F> Mldivide<Transform<'b, G, T>> for Transform<'_, G, F>
where
    G: IsFrameOfReference,
    F: IsFrameOfReference + 'b,
    T: IsFrameOfReference,
{
    type Output = Transform<'b, F, T>;

    fn mldivide(&self, rhs: &Transform<'b, G, T>) -> Self::Output {
        let lu = na::LU::new(self.data.into());
        let matrix = lu.solve(&rhs.data.into()).unwrap();
        let data = na::Transform3::from_matrix_unchecked(matrix);
        Self::Output {
            data,
            _from: PhantomData,
            _to: PhantomData,
        }
    }
}

impl<'a, T, V> Transform<'a, T, V>
where
    T: IsFrameOfReference,
    V: IsFrameOfReference,
{
    pub fn new(data: na::Transform3<f64>) -> Transform<'a, T, V> {
        Transform {
            data,
            _from: PhantomData,
            _to: PhantomData,
        }
    }
    pub fn inner(&self) -> &na::Transform3<f64> {
        &self.data
    }
}
