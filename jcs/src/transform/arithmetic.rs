use nalgebra as na;
use std::{marker::PhantomData, ops};

pub trait IsFrameOfReference {}
#[derive(Debug)]
pub struct Transform<'a, T, V>
where
    T: IsFrameOfReference,
    V: IsFrameOfReference,
{
    // data: &'a[na::Transform3<f64>],
    data: Vec<na::Transform3<f64>>,
    _from: PhantomData<&'a T>,
    _to: PhantomData<&'a V>,
}

impl<'b, B, T, V> ops::Mul<Transform<'b, V, B>> for Transform<'_, T, V>
where
    T: IsFrameOfReference + 'b,
    V: IsFrameOfReference,
    B: IsFrameOfReference,
{
    type Output = Transform<'b, T, B>;

    fn mul(self, rhs: Transform<'b, V, B>) -> Self::Output {
        if self.data.len() != rhs.data.len() && self.data.len() != 1 {
            panic!(
                "Incompatible transform lengths. Lengths are {} and {}",
                self.data.len(),
                rhs.data.len()
            );
        }
        let data: Vec<_> = if self.data.len() == 1 {
            rhs.data.iter().map(|m| self.data[0] * m).collect()
        } else {
            self.data.iter().zip(rhs.data).map(|(l, r)| l * r).collect()
        };
        Transform {
            data,
            _from: PhantomData,
            _to: PhantomData,
        }
    }
}

pub trait Mldivide<Rhs> {
    type Output;
    fn mldivide(self, rhs: Rhs) -> Self::Output;
}

impl<'b, T, G, F> Mldivide<Transform<'b, G, T>> for Transform<'_, G, F>
where
    G: IsFrameOfReference,
    F: IsFrameOfReference + 'b,
    T: IsFrameOfReference,
{
    type Output = Transform<'b, F, T>;

    fn mldivide(self, rhs: Transform<'b, G, T>) -> Self::Output {
       // if different lengths panic 
        let data = self.data.iter().zip(rhs.data).map(|(l,r)| l.try_inverse().unwrap() * r).collect();
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
    pub fn new(data: &'a [na::Transform3<f64>]) -> Transform<'a, T, V> {
        Transform {
            data: data.to_owned(),
            _from: PhantomData,
            _to: PhantomData,
        }
    }
}
