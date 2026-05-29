use std::{fmt, marker::PhantomData, ops};

use nalgebra as na;

pub trait FrameOfReference: PartialEq {}

pub trait Mldivide<Rhs> {
    type Output;
    fn mldivide(&self, rhs: &Rhs) -> Self::Output;
}

#[derive(Debug, PartialEq)]
pub struct Transform<R, A>
where
    A: FrameOfReference,
    R: FrameOfReference,
{
    reference: PhantomData<R>,
    subject: PhantomData<A>,
    matrix: na::Affine3<f64>,
}

impl<R, A> approx::AbsDiffEq for Transform<R, A>
where
    A: FrameOfReference,
    R: FrameOfReference,
{
    type Epsilon = f64;

    fn default_epsilon() -> Self::Epsilon {
        std::f64::EPSILON
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.matrix.abs_diff_eq(&other.matrix, epsilon)
    }
}

impl<R, A> approx::RelativeEq for Transform<R, A>
where
    A: FrameOfReference,
    R: FrameOfReference,
{
    fn default_max_relative() -> Self::Epsilon {
        std::f64::EPSILON
    }

    fn relative_eq(&self, other: &Self, epsilon: Self::Epsilon, max_relative: Self::Epsilon)
        -> bool {
            self.matrix.relative_eq(&other.matrix, epsilon, max_relative)
    }
}

impl<R, A> Transform<R, A>
where
    A: FrameOfReference,
    R: FrameOfReference,
{
    pub fn new(matrix: na::Affine3<f64>) -> Self {
        Self {
            reference: PhantomData,
            subject: PhantomData,
            matrix,
        }
    }
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

#[cfg(test)]
mod test {

    use super::*;

    use approx::assert_relative_eq;

    #[derive(Debug, PartialEq)]
    struct A {}
    #[derive(Debug, PartialEq)]
    struct B {}
    #[derive(Debug, PartialEq)]
    struct C {}

    impl FrameOfReference for A {}
    impl FrameOfReference for B {}
    impl FrameOfReference for C {}

    #[test]
    fn can_compare_matrices_using_identities() {
        // B/A == (A'\B')'
        let identity = na::Affine3::from_matrix_unchecked(na::Matrix4::identity().into());
        let lhs = Transform::<B, A>::new(identity.clone());
        let rhs = Transform::<B, C>::new(identity.clone());
        let mldivide: Transform<A, C> = lhs.mldivide(&rhs);

        let lhs = Transform::<A, B>::new(identity.clone());
        let rhs = Transform::<C, B>::new(identity.clone());

        let mrdivide = lhs / rhs;

        assert_relative_eq!(mldivide, mrdivide);
    }

    #[test]
    fn mldivide_and_divide_give_similar_results() {
        let identity = na::Affine3::from_matrix_unchecked(na::Matrix4::identity().into());
        let lhs = Transform::<B, A>::new(identity.clone());

        let m = na::Translation3::new(3.0, 4.0, 5.0);
        let m= na::Affine3::from_matrix_unchecked(m.into());
        let rhs = Transform::<B,C>::new(m);

        let mldivide: Transform<A, C> = lhs.mldivide(&rhs);

        let lhs = Transform::<A, B>::new(identity.clone());
        let m = na::Translation3::new(-3.0, -4.0, -5.0);
        let m= na::Affine3::from_matrix_unchecked(m.into());
        let rhs = Transform::<C, B>::new(m.into());

        let mrdivide = lhs / rhs;

        assert_relative_eq!(mldivide, mrdivide);
    }
    
}
