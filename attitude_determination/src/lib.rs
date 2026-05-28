use std::{fmt, marker::PhantomData, ops};

use nalgebra as na;

pub trait FrameOfReference {}

pub trait Mldivide<Rhs> {
    type Output;
    fn mldivide(&self, rhs: &Rhs) -> Self::Output;
}

pub struct Transform<R, A>
where
    A: FrameOfReference,
    R: FrameOfReference,
{
    reference: PhantomData<R>,
    subject: PhantomData<A>,
    matrix: na::Transform3<f64>,
}

impl<R, A> fmt::Display for Transform<R, A>
where
    A: FrameOfReference,
    R: FrameOfReference,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.matrix.to_homogeneous())
    }
}

impl<R, A, B> ops::Div<Transform<A, B>> for Transform<R, A>
where
    R: FrameOfReference,
    A: FrameOfReference,
    B: FrameOfReference,
{
    type Output = Transform<R, B>;

    fn div(self, rhs: Transform<A, B>) -> Self::Output {
        Transform {
            reference: PhantomData,
            subject: PhantomData,
            matrix: todo!(),
        }
    }
}

impl<R, A, B> Mldivide<Transform<A, B>> for Transform<R, A>
where
    R: FrameOfReference,
    A: FrameOfReference,
    B: FrameOfReference,
{
    type Output = Transform<R, B>;

    fn mldivide(&self, rhs: &Transform<A, B>) -> Self::Output {
        let lu = na::LU::new(self.matrix.into());
        let matrix = lu.solve(&rhs.matrix.into()).unwrap();
        Transform {
            reference: PhantomData,
            subject: PhantomData,
            matrix: na::Transform3::from_matrix_unchecked(matrix),
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

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn matrix_multiplication_works() {
        unimplemented!()
    }

    #[test]
    fn matrix_division_works() {
        unimplemented!()
    }

    #[test]
    fn mldivide_works() {
        unimplemented!()
    }
}
