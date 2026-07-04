use std::path::PathBuf;

use common::tracker::Tracker;

use crate::knee::{Distal, Femur, Lateral, Medial, Patella, Proximal, Tibia};

// use crate::{Bone, Position, tracker::Tracker};


pub struct DigitisationKnee {
    specimen: String,
    filepath: PathBuf,
    femur_medial: Option<Tracker<Femur, Medial>>,
    femur_lateral: Option<Tracker<Femur, Lateral>>,
    femur_proximal: Option<Tracker<Femur, Proximal>>,

    tibia_medial: Option<Tracker<Tibia, Medial>>,
    tibia_lateral: Option<Tracker<Tibia, Lateral>>,
    tibia_distal: Option<Tracker<Tibia, Distal>>,

    patella_medial: Option<Tracker<Patella, Medial>>,
    patella_lateral: Option<Tracker<Patella, Lateral>>,
    patella_distal: Option<Tracker<Patella, Distal>>,
}

impl DigitisationKnee {
    pub fn new(
        specimen: String,
        filepath: PathBuf,
        femur_medial: Option<Tracker<Femur, Medial>>,
        femur_lateral: Option<Tracker<Femur, Lateral>>,
        femur_proximal: Option<Tracker<Femur, Proximal>>,
        tibia_medial: Option<Tracker<Tibia, Medial>>,
        tibia_lateral: Option<Tracker<Tibia, Lateral>>,
        tibia_distal: Option<Tracker<Tibia, Distal>>,
        patella_medial: Option<Tracker<Patella, Medial>>,
        patella_lateral: Option<Tracker<Patella, Lateral>>,
        patella_distal: Option<Tracker<Patella, Distal>>,
    ) -> Self {
        Self {
            specimen,
            filepath,
            femur_medial,
            femur_lateral,
            femur_proximal,
            tibia_medial,
            tibia_lateral,
            tibia_distal,
            patella_medial,
            patella_lateral,
            patella_distal,
        }
    }
}

// pub struct DigitisationHip {
//     specimen: String,
//     filepath: PathBuf,
//     femur_medial: Tracker<Femur, Medial>,
//     femur_lateral: Tracker<Femur, Lateral>,
//     femur_proximal: Tracker<Femur, Proximal>,
//
//     tibia_medial: Tracker<Tibia, Medial>,
//     tibia_lateral: Tracker<Tibia, Lateral>,
//     tibia_distal: Tracker<Tibia, Distal>,
//
//     hip_medial: Tracker<Hip, Medial>,
//     hip_lateral: Tracker<Hip, Lateral>,
//     hip_distal: Tracker<Hip, Distal>,
// }
//


#[cfg(test)]
mod test {
    use super::*;
    
    #[test]
    fn create_digitisation() {
        use common::tracker::{Marker, Tracker};

        let femur_medial = Marker::default().assign(Femur::medial());
        let femur_lateral = Marker::default().assign(Femur::lateral());
        let femur_proximal = Marker::default().assign(Femur::proximal());

        let tibia_medial = Marker::default().assign(Tibia::medial());
        let tibia_lateral = Marker::default().assign(Tibia::lateral());
        let tibia_distal = Marker::default().assign(Tibia::distal());

        let patella_medial = Marker::default().assign(Patella::medial());
        let patella_lateral = Marker::default().assign(Patella::lateral());
        let patella_distal = Marker::default().assign(Patella::distal());


        let specimen = String::from("Spc1");
        let filepath = PathBuf::from("path/to/file");
        DigitisationKnee::new(specimen, filepath, femur_medial, femur_lateral, femur_proximal, tibia_medial, tibia_lateral, tibia_distal, patella_medial, patella_lateral, patella_distal);
    }
    
}
