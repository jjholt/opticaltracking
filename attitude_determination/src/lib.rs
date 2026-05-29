mod operations;
mod equality;
use std::{fmt, marker::PhantomData};

use nalgebra as na;

pub trait FrameOfReference: PartialEq {}


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


#[cfg(test)]
mod test {

    use super::*;
    use crate::operations::Mldivide;

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
