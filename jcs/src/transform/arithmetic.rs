use nalgebra::{self as na, UnitQuaternion, VectorView4};
use std::{marker::PhantomData, ops};

pub trait IsFrameOfReference {}

#[derive(Debug)]
pub struct Transform<T, V>
where
    T: IsFrameOfReference,
    V: IsFrameOfReference,
{
    data: na::Transform3<f32>,
    _from: PhantomData<T>,
    _to: PhantomData<V>,
}

impl<T, V> std::fmt::Display for Transform<T, V>
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

impl<A, G, B> ops::Mul<Transform<G, B>> for Transform<A, G>
where
    A: IsFrameOfReference,
    G: IsFrameOfReference,
    B: IsFrameOfReference,
{
    type Output = Transform<A, B>;

    fn mul(self, rhs: Transform<G, B>) -> Self::Output {
        let data = self.data * rhs.data;
        Transform {
            data,
            _from: PhantomData,
            _to: PhantomData,
        }
    }
}

impl<T, G, F> Mldivide<Transform<G, T>> for Transform<G, F>
where
    G: IsFrameOfReference,
    F: IsFrameOfReference,
    T: IsFrameOfReference,
{
    type Output = Transform<F, T>;

    fn mldivide(&self, rhs: &Transform<G, T>) -> Self::Output {
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

impl<T, V> Transform<T, V>
where
    T: IsFrameOfReference,
    V: IsFrameOfReference,
{
    pub fn new(data: na::Transform3<f32>) -> Transform<T, V> {
        Transform {
            data,
            _from: PhantomData,
            _to: PhantomData,
        }
    }
    pub fn inner(&self) -> &na::Transform3<f32> {
        &self.data
    }
    pub fn translation(&self) -> na::Point3<f32> {
        na::Point3::from_homogeneous(self.inner().to_homogeneous().column(3).into()).unwrap()
    }
    pub fn rotation(&self) -> na::UnitQuaternion<f32> {
        let rotmat = self.inner().matrix().fixed_view::<3, 3>(0, 0).into_owned();
        UnitQuaternion::from_matrix(&rotmat)
    }
    pub fn i(&self) -> na::Vector3<f32> {
        self.inner().into_inner().fixed_view::<3,1>(0, 0).into_owned()
    }
    pub fn j(&self) -> na::Vector3<f32> {
        self.inner().into_inner().fixed_view::<3,1>(0, 1).into_owned()
    }
    pub fn k(&self) -> na::Vector3<f32> {
        self.inner().into_inner().fixed_view::<3,1>(0, 2).into_owned()
    }
    pub fn origin(&self) -> na::Point3<f32>{
        self.inner().into_inner().fixed_view::<3,1>(0, 3).into_owned().into()
    }
}
#[cfg(test)]
mod test {
    use core::f32;

    use crate::bone_to_tracker::{Femur, Global};

    use super::*;

    #[test]
    fn point_from_transform() {
        unimplemented!()
    }
    #[test]
    fn unitq_from_transform() {
        let rotation = na::Rotation3::<f32>::from_axis_angle(&na::Vector3::x_axis(), f32::consts::FRAC_PI_6);
        let t = na::Transform3::identity();
        let trans = Transform::<Global, Femur>::new(t * rotation);
        let unit_q = UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), f32::consts::FRAC_PI_6);

        assert_relative_eq!((t*rotation).to_homogeneous(), unit_q.to_homogeneous(), epsilon=1e-2)
    }
}
