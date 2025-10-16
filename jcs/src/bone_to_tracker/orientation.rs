pub trait Orientation {}
#[derive(Debug)]
pub struct Medial {}
impl std::fmt::Display for Medial {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Medial")
    }
}
#[derive(Debug)]
pub struct Lateral {}
#[derive(Debug)]
pub struct Anterior {}
#[derive(Debug)]
pub struct Posterior {}
#[derive(Debug)]
pub struct ProximalDistal {}

impl Orientation for Medial {}
impl Orientation for Lateral {}
impl Orientation for Anterior {}
impl Orientation for Posterior {}
impl Orientation for ProximalDistal {}
