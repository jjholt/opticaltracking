use serde::Deserialize;

mod certus;
mod polaris;
mod generic;

#[derive(Deserialize, Debug, Hash, PartialEq, Eq)]
pub enum Camera {
    Certus,
    Polaris,
    Generic,
    Unknown,
}

impl Camera {
    pub fn determine_camera(headers: &str) -> Self {
        headers.split(",").filter_map(|s| {
            let s = s.to_lowercase();
            match s {
                _ if s.contains("tool") => Some(Camera::Polaris),
                _ if s.contains("generic") => Some(Camera::Generic),
                _ if s.contains("frame") => Some(Camera::Certus),
                _ => None,
            }
        })
        .next()
        .unwrap_or(Camera::Unknown)
    }
}
