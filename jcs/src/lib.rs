mod tracker;
mod landmark;

#[cfg(feature = "knee")]
mod knee;

#[cfg(feature = "hip")]
mod hip;

use attitude_determination::FrameOfReference;

pub struct RigidBody<const N: usize> {}

impl <const N: usize> FrameOfReference for RigidBody<N> { }






pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
