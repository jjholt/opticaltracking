use crate::{FrameOfReference, Transform};

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
