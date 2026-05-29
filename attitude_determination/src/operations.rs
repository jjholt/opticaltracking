use nalgebra as na;
use std::{marker::PhantomData, ops};

use crate::{FrameOfReference, Transform};

pub trait Mldivide<Rhs> {
    type Output;
    fn mldivide(&self, rhs: &Rhs) -> Self::Output;
}

impl<R, A, B> ops::Div<Transform<B, A>> for Transform<R, A>
where
    R: FrameOfReference,
    A: FrameOfReference,
    B: FrameOfReference,
{
    type Output = Transform<R, B>;

    /// Solves `xA = B`, i.e., `A * inv(B)`
    ///
    /// if `A` is tibia in the global frame of reference (`gTt`), and `B` is tibia in the femoral frame of reference (`fTt`)
    /// then `A / B` describes femur in the global frame of reference (`gTf`)
    fn div(self, rhs: Transform<B, A>) -> Self::Output {
        Transform {
            reference: PhantomData,
            subject: PhantomData,
            matrix: self.matrix * rhs.matrix.inverse(),
        }
    }
}

impl<R, A, B> Mldivide<Transform<A, B>> for Transform<A, R>
where
    R: FrameOfReference,
    A: FrameOfReference,
    B: FrameOfReference,
{
    type Output = Transform<R, B>;

    /// Solves Ax = B. i.e., solves inv(A) * B
    fn mldivide(&self, rhs: &Transform<A, B>) -> Self::Output {
        let lu = na::LU::new(self.matrix.into());
        let matrix = lu.solve(&rhs.matrix.into()).unwrap();
        Transform {
            reference: PhantomData,
            subject: PhantomData,
            matrix: na::Affine3::from_matrix_unchecked(matrix),
        }
    }
}

impl<R, A, B> ops::Mul<Transform<A, B>> for Transform<R, A>
where
    R: FrameOfReference,
    A: FrameOfReference,
    B: FrameOfReference,
{
    type Output = Transform<R, B>;

    fn mul(self, rhs: Transform<A, B>) -> Self::Output {
        Transform {
            reference: PhantomData,
            subject: PhantomData,
            matrix: self.matrix * rhs.matrix,
        }
    }
}
