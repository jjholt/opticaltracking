use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};

use serde::Deserialize;

use crate::Camera;

#[derive(Deserialize, Debug)]
struct TrackerLabels {
    tibia: Option<String>,
    femur: Option<String>,
    patella: Option<String>,
    hip: Option<String>,
    probe: Option<String>,
}

#[derive(Deserialize, Debug)]
#[serde(default)]
struct ConfigDigitisation {
    /// Folders to look for digitisation files
    /// Defaults to `["digit", "calib"]`.
    folders: Vec<PathBuf>,
    /// Angle of digitisation
    /// Defaults to `0.0`
    angle: f32,
    /// Whether to apply a correction to kinematics based on the digitisation angle
    /// Defaults to `false`
    correction: bool,
}

impl Default for ConfigDigitisation {
    fn default() -> Self {
        Self {
            folders: vec![PathBuf::from("digit"), PathBuf::from("calib")],
            angle: 0.0,
            correction: false,
        }
    }
}

#[derive(Deserialize, Debug)]
pub struct Config {
    camera_labels: HashMap<Camera, TrackerLabels>,
    #[serde(default)]
    digitisation: ConfigDigitisation,
}

impl Config {
    pub fn from_path(path: &Path) -> crate::Result<Self> {
        let file = std::fs::read_to_string(path)?;
        Ok(toml::from_str(&file)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CONFIG_STR: &str = r#"
[camera_labels.Polaris]
tibia = "T"
femur = "Y"
probe = "Probe"

[camera_labels.Certus]
tibia = "tibia"
femur = "femur"
patella = "patella"
probe = "Probe"
"#;

    #[test]
    fn parse_config() {
        let config: Config = toml::from_str(CONFIG_STR).unwrap();
        let polaris = &config.camera_labels[&Camera::Polaris];
        let certus = &config.camera_labels[&Camera::Certus];

        assert_eq!(polaris.tibia.as_deref(), Some("T"));
        assert_eq!(polaris.femur.as_deref(), Some("Y"));
        assert_eq!(polaris.patella, None);
        assert_eq!(polaris.probe.as_deref(), Some("Probe"));

        assert_eq!(certus.tibia.as_deref(), Some("tibia"));
        assert_eq!(certus.patella.as_deref(), Some("patella"));
    }
    #[test]
    fn config_from_path() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("data")
            .join("config.toml");
        let config = Config::from_path(&path).unwrap();
        let polaris = &config.camera_labels[&Camera::Polaris];
        let certus = &config.camera_labels[&Camera::Certus];

        assert_eq!(polaris.tibia.as_deref(), Some("T"));
        assert_eq!(polaris.femur.as_deref(), Some("Y"));
        assert_eq!(polaris.patella, None);
        assert_eq!(polaris.probe.as_deref(), Some("Probe"));

        assert_eq!(certus.tibia.as_deref(), Some("tibia"));
        assert_eq!(certus.patella.as_deref(), Some("patella"));
    }
}
