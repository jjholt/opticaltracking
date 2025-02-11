use crate::transform::{IsFrameOfReference, Transform};

mod gns;
mod sara;
mod helical;

pub use gns::GroodAndSuntay;
use gns::Motion;

pub trait Solver<V,T> where V: IsFrameOfReference, T: IsFrameOfReference {
    fn solve(&self, rb2: V, rb1: T) -> Motion;
}
