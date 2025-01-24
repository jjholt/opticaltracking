use crate::transform::{IsFrameOfReference, Transform};

mod gns;
mod sara;
mod helical;

pub use gns::GroodAndSuntay;

pub trait Solver {
    fn new(relative_motion: Transform<T,V>) -> Self where T: IsFrameOfReference, V: IsFrameOfReference;
    fn solve(&self);
}
