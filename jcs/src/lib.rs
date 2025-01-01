mod transform;

use transform::IsFrameOfReference;
struct Landmark {}
struct Position {}

#[derive(Debug)]
struct RigidBody<const ID: usize> {}
impl<const ID: usize> IsFrameOfReference for RigidBody<ID> {}

type Tibia = RigidBody<1>;
type Femur = RigidBody<2>;

struct Tracker {}
impl IsFrameOfReference for Tracker {}

#[derive(Debug)]
struct Global;
impl IsFrameOfReference for Global {}


pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::transform::Transform;
    use nalgebra as na;
    use transform::Mldivide;

    #[test]
    fn new_transform() {
        let transf = vec![na::Transform3::identity()];
        let tibia = Transform::<Global, Tibia>::new(&transf);
        let femur = Transform::<Global, Femur>::new(&transf);

        let fTt = femur.mldivide(tibia);

        println!("{:#?}", fTt);
    }
}
