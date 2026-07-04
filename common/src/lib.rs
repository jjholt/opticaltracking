use std::marker::PhantomData;



/// Basic definitions of bones and locations
pub trait Position {}
pub trait Bone {}
pub struct Landmark<B: Bone, P: Position> {
    pub bone: PhantomData<B>,
    pub position: PhantomData<P>,
}

pub mod tracker;

mod error;
pub use error::{Error, Result};
