use std::marker::PhantomData;

use crate::{bone_to_tracker::Global, transform::{gT, IsFrameOfReference, Transform}, Marker};

use super::{ProbeData, ProbeRawData};

pub struct Datum<M: Marker> {
    data: ProbeData,
    marker: PhantomData<M>
}

impl <'a, M: Marker + IsFrameOfReference> From<ProbeRawData<'a>> for Datum<M> {
    fn from(value: ProbeRawData<'a>) -> Self {
        Self::new(value.into())
    }
}

impl<M: Marker + IsFrameOfReference> Datum<M> {
    // #[cfg(test)]
    pub fn new(data: ProbeData) -> Self {
        Self {
            data,
            marker: PhantomData,
        }
    }
    pub fn to_transform(&self) -> gT<M> {
        Transform::<Global, M>::new(self.data.to_transform())
    }
}

// #[cfg(feature = "knee")]
// struct Datum {
//     data: Position,
//     marker: Marker
// }
// #[cfg(feature = "knee")]
// enum Marker {
//     Probe,
//     Tibia,
//     Patella,
//     Femur
// }

#[cfg(test)]
mod datum_to_tracker {
    use crate::{
        bone_to_tracker::{Femur, Side, Tibia}, data::ProbeRawData, transform::Mldivide
    };

    #[test]
    fn stuff() {
        let side = Side::Left;
        // Femur Landmarks
        let name = "Black Probe";
        let label = "Probe";
        let fm = ProbeRawData::new(name, label, 0.8228, 0.1357, 0.4408, -0.3318, 15.3196, -54.9971, -2097.6023);
        let fl = ProbeRawData::new(name, label, 0.4031, 0.4746, 0.4195, -0.6603, 16.9156, 16.2064, -2059.3142);
        let fp = ProbeRawData::new(name, label, 0.4280, 0.4662, 0.4471, -0.6319, -8.5689, 15.8874, -2131.4353);

        // Tibia Landmarks
        let tm = ProbeRawData::new(name, label, 0.8156, 0.0787, 0.4628, -0.3381, 66.899, -61.4777, -2078.4102);
        let tl = ProbeRawData::new(name, label, 0.4197, 0.4198, 0.3991, -0.6987, 65.8513, -6.3346, -2031.8842);
        let td = ProbeRawData::new(name, label, 0.4268, 0.2327, 0.5690, -0.6632, 209.2022, -37.8499, -2040.4506);

        // Tracker positions
        let femur_tracker_data = ProbeRawData::new(name, label, 0.9573733, -0.0372205, -0.1895465, 0.2147628, -149.371, -19.411, -2148.287);
        let tibia_tracker_data = ProbeRawData::new(name, label, 0.0230, -0.1878, 0.0213, 0.9817, 128.0411, 196.8627, -2024.9063);

        // Data
        let f_data = ProbeRawData::new(name, label, 0.7169, 0.1776, 0.4286, 0.5204, -37.7920, -136.356, -1953.203);
        let t_data = ProbeRawData::new(name, label, 0.0347, -0.1902, 0.0599, 0.9793, 128.2, 205.321, -2050.397);


        let femur = Femur::new( side, fm.into(), fl.into(), fp.into(), femur_tracker_data.into());

        let tibia = Tibia::new( side, tm.into(), tl.into(), td.into(), tibia_tracker_data.into());


        let g_t_ti = tibia.take_datum(t_data.into());
        let g_t_fi = femur.take_datum(f_data.into());

        println!("femur in global {}", g_t_fi);
        println!("tibia in global {}", g_t_ti);

        let _f_t_t = g_t_fi.mldivide(&g_t_ti); // Tibia in femoral frame of reference

        // println!("Transform {}", f_t_t);
        // println!("Rotation: {}", f_t_t.rotation());
        // println!("Point {}", f_t_t.translation());

    }
    
}


