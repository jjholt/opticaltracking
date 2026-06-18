#[derive(Debug, PartialEq)]
pub struct Marker {
    pub label: String,
    pub tx: Vec<f32>,
    pub ty: Vec<f32>,
    pub tz: Vec<f32>,
    pub q0: Vec<f32>,
    pub qx: Vec<f32>,
    pub qy: Vec<f32>,
    pub qz: Vec<f32>,
}

impl Default for Marker {
    fn default() -> Self {
        Self { label: Default::default(), tx: Default::default(), ty: Default::default(), tz: Default::default(), q0: Default::default(), qx: Default::default(), qy: Default::default(), qz: Default::default() }
    }
}

impl Marker {
    pub fn new(label: String, tx: Vec<f32>, ty: Vec<f32>, tz: Vec<f32>, q0: Vec<f32>, qx: Vec<f32>, qy: Vec<f32>, qz: Vec<f32>) -> Self {
        Self { label, tx, ty, tz, q0, qx, qy, qz }
    }


    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            label: String::new(),
            tx: Vec::<f32>::with_capacity(capacity),
            ty: Vec::<f32>::with_capacity(capacity),
            tz: Vec::<f32>::with_capacity(capacity),
            q0: Vec::<f32>::with_capacity(capacity),
            qx: Vec::<f32>::with_capacity(capacity),
            qy: Vec::<f32>::with_capacity(capacity),
            qz: Vec::<f32>::with_capacity(capacity),
        }
    }

    pub fn with_label(mut self, label: &str) -> Self {
        self.label =  label.into();
        self
    }
}

#[derive(Debug)]
pub struct MarkerOffsets {
    pub tx: usize,
    pub ty: usize,
    pub tz: usize,
    pub q0: usize,
    pub qx: usize,
    pub qy: usize,
    pub qz: usize,
}

impl MarkerOffsets {
    pub fn from_headers(headers: &[&str]) -> Self {
        let mut tx: Option<usize> = None;
        let mut ty: Option<usize> = None;
        let mut tz: Option<usize> = None;
        let mut q0: Option<usize> = None;
        let mut qx: Option<usize> = None;
        let mut qy: Option<usize> = None;
        let mut qz: Option<usize> = None;

        for (i, &h) in headers.iter().enumerate() {
            match h.trim() {
                "Q0" => { q0.get_or_insert(i); },
                "Qx" => { qx.get_or_insert(i); },
                "Qy" => { qy.get_or_insert(i); },
                "Qz" => { qz.get_or_insert(i); },
                "Tx" => { tx.get_or_insert(i); },
                "Ty" => { ty.get_or_insert(i); },
                "Tz" => { tz.get_or_insert(i); },
                _ => {}
            }
        };
        Self {
            tx: tx.expect("Missing field tx"),
            ty: ty.expect("Missing field ty"),
            tz: tz.expect("Missing field tz"),
            q0: q0.expect("Missing field q0"),
            qx: qx.expect("Missing field qx"),
            qy: qy.expect("Missing field qy"),
            qz: qz.expect("Missing field qz"),
        }
    }
}
