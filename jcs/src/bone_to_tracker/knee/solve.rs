use std::marker::PhantomData;

use crate::{
    bone_to_tracker::{Global, Motion, Side},
    solvers::{GroodAndSuntay, Solver},
    transform::{gT, IsFrameOfReference, Transform},
};

use super::{Femur, Patella, Tibia};

impl GroodAndSuntay {
    pub fn tibiofemoral() -> GroodAndSuntayKnee<Femur, Tibia> {
        GroodAndSuntayKnee {
            t: PhantomData,
            v: PhantomData,
        }
    }
    pub fn patellofemoral() -> GroodAndSuntayKnee<Femur, Patella> {
        GroodAndSuntayKnee {
            t: PhantomData,
            v: PhantomData,
        }
    }
}

pub struct GroodAndSuntayKnee<A: IsFrameOfReference, B: IsFrameOfReference> {
    t: PhantomData<A>,
    v: PhantomData<B>,
}

impl Solver for GroodAndSuntayKnee<Femur, Tibia> {
    type F = Femur;
    type T = Tibia;

    fn solve(&self, rb1: gT<Self::F>, rb2: gT<Self::T>, side: Side) -> Motion {
        let femur = rb1;
        let tibia = rb2;
        let e2 = tibia.k().cross(&femur.i()).normalize();

        let flexion = (-e2).dot(&femur.k()).asin().to_degrees();
        let beta = femur.i().dot(&tibia.k()).acos().to_degrees(); // epicondylar projected onto tibial superior-inferior

        let external = match side {
            Side::Right => (-e2).dot(&tibia.i()).asin().to_degrees(),
            Side::Left => e2.dot(&tibia.i()).asin().to_degrees(),
        };
        let varus = match side {
            Side::Right => 90.0 - beta,
            Side::Left => 90.0 - beta,
        };
        let h = tibia.origin() - femur.origin();
        let lateral = match side {
            Side::Right => h.dot(&femur.i()),
            Side::Left => h.dot(&-femur.i()),
        };
        let anterior = h.dot(&e2);
        let distal = -h.dot(&tibia.k());

        Motion {
            flexion,
            external,
            varus,
            anterior,
            lateral,
            distal,
        }
    }
}

impl Solver for GroodAndSuntayKnee<Femur, Patella> {
    type F = Femur;
    type T = Tibia;

    fn solve(&self, rb1: gT<Self::F>, rb2: gT<Self::T>, side: Side) -> Motion {
        todo!()
    }
}

#[cfg(test)]
mod test {
    use super::Solver;
    use super::*;
    use nalgebra as na;
    #[test]
    fn tibiofemoral_gns() {
        let side = Side::Left;
        let g_t_f = {
            let a = na::Matrix4::new(
                0.038036615, 0.3550867, -0.93405956, 23.877735,
                -0.86295205, -0.45963743, -0.2098743, -20.129425,
                -0.5038522, 0.81403124, 0.28893963, -2075.4763,
                0.0, 0.0, 0.0, 1.0,
            );
            let b = na::Transform3::from_matrix_unchecked(a);
            Transform::<Global, Femur>::new(b)
        };
        let g_t_t = {
            let a = na::Matrix4::new(
                -0.0122986585, -0.12232429, -0.99241406, 57.99079,
                -0.808449, -0.5828457, 0.08186005, -24.560608,
                -0.5884379, 0.8033229, -0.091724634, -2064.4182,
                0.0, 0.0, 0.0, 1.0,
            );
            let b = na::Transform3::from_matrix_unchecked(a);
            Transform::<Global, Tibia>::new(b)
        };
        let motion = GroodAndSuntay::tibiofemoral().solve(g_t_f, g_t_t, side);
    }
    #[test]
    fn patellofemoral_gns() {
        unimplemented!()
    }
}
