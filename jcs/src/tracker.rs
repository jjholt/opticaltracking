use input::{Config, Marker};

use crate::{Bone, Landmark, Position};

pub struct Tracker<B: Bone, P: Position> {
    landmark: Landmark<B, P>,
    label: String,
    tx: Vec<f32>,
    ty: Vec<f32>,
    tz: Vec<f32>,
    q0: Vec<f32>,
    qx: Vec<f32>,
    qy: Vec<f32>,
    qz: Vec<f32>,
}

impl<B: Bone, P: Position> Tracker<B, P> {
    pub fn new(marker: Marker, config: &Config) -> Self {
        Self {
            landmark: todo!(),
            label: marker.label,
            tx: marker.tx,
            ty: marker.ty,
            tz: marker.tz,
            q0: marker.q0,
            qx: marker.qx,
            qy: marker.qy,
            qz: marker.qz,
        }
    }

    pub fn from_marker(marker: Marker, landmark: Landmark<B, P>) -> Tracker<B, P> {
        Self {
            landmark,
            label: marker.label,
            tx: marker.tx,
            ty: marker.ty,
            tz: marker.tz,
            q0: marker.q0,
            qx: marker.qx,
            qy: marker.qy,
            qz: marker.qz,
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    #[cfg(feature = "knee")]
    fn creates_objects() {
        use crate::knee::{Femur};

        let tracker = Tracker::from_marker(Marker::default(), Femur::medial());
    }
    
}
