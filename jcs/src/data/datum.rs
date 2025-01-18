use std::marker::PhantomData;

use crate::{bone_to_tracker::Global, transform::{IsFrameOfReference, Transform}, Marker};

use super::ProbeData;

struct Datum<M: Marker> {
    data: ProbeData,
    marker: PhantomData<M>
}

impl<M: Marker + IsFrameOfReference> Datum<M> {
    #[cfg(test)]
    pub fn new(data: ProbeData) -> Self {
        Self {
            data,
            marker: PhantomData,
        }
    }
    pub fn to_transform(&self) -> Transform<'_, Global, M> {
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
        bone_to_tracker::{Distal, Landmark, Lateral, Medial, Proximal, Femur, Tibia, Side, DefinedTracker},
        data::ProbeRawData,
        transform::Mldivide, Tracker
    };

    use super::*;
    #[test]
    fn stuff() {
        let side = Side::Right;
        // Femur
        let fm = ProbeRawData::new(0.822817363793104, 0.135707186206897, 0.440885412068966, -0.331890289655172, 15.3196551724138, -54.9971034482759, -2097.60234482759);
        let fl = ProbeRawData::new(0.403151039655172, 0.474684148275862, 0.419551994827586, -0.66038950862069, 16.9156724137931, 16.2064655172414, -2059.31424137931);
        let fp = ProbeRawData::new(0.428051244067797, 0.466236162711864, 0.447185444067797, -0.631995389830509, -8.56891525423729, 15.8874915254237, -2131.43533898305);
        let femur = Femur::new()
            .set_side(side)
            .set_medial(fm.into())
            .set_lateral(fl.into())
            .set_proximal(fp.into());

        // Tibia
        let tm = ProbeRawData::new( 0.815640513793103, 0.0787777862068966, 0.462803577586207, -0.338106684482759, 66.899, -61.477775862069, -2078.41025862069);
        let tl = ProbeRawData::new( 0.41977958245614, 0.41981549122807, 0.399112952631579, -0.698737270175439, 65.8513684210526, -6.33466666666667, -2031.88421052632);
        let td = ProbeRawData::new( 0.426822132758621, 0.232742715517241, 0.569003496551724, -0.663226562068966, 209.20225862069, -37.8499310344828, -2040.45063793103);
        let tibia = Tibia::new()
            .set_side(side)
            .set_medial(tm.into())
            .set_lateral(tl.into())
            .set_distal(td.into());

        // Tracker
        let femur_probe_data = ProbeRawData::new(0.9573733, -0.0372205, -0.1895465, 0.2147628, -149.371, -19.411, -2148.287);
        let tibia_probe_data = ProbeRawData::new(0.957338862068966, -0.0375362206896552, -0.189658806896552, 0.214761898275862, -149.401965517241, -19.4073620689655, -2148.32782758621);

        let g_t_ft = Femur::tracker_in_global(femur_probe_data.into()); // femoral tracker in global
        let g_t_tt = Tibia::tracker_in_global(tibia_probe_data.into()); // tibia tracker in global

        
        let ft_t_f = femur.in_tracker(&g_t_ft);
        let tt_t_tc = tibia.in_tracker(&g_t_tt);

        // Data
        //
        let y_probe = ProbeRawData::new(0.7390869, 0.1446579, 0.3029275, 0.584003, 57.113, -9.573, -1830.174);
        let femur_datum = Datum::<Tracker<Femur>>::new(y_probe.into());
        let t_probe = ProbeRawData::new(0.0928529, -0.0505891, -0.2052756, 0.9729753, 279.027, 308.978, -1774.889);
        let tibia_datum = Datum::<Tracker<Tibia>>::new(t_probe.into());
        // let f_t_t = (femur_datum.to_transform() * ft_t_f).mldivide(&(tibia_datum.to_transform() * tt_t_tc)); // Tibia in femoral frame of reference
        let g_t_fi = femur_datum.to_transform() * ft_t_f.unwrap();
        let g_t_ti = tibia_datum.to_transform() * tt_t_tc.unwrap();

        let f_t_t = g_t_fi.mldivide(&g_t_ti); // Tibia in femoral frame of reference

    }
    
}
