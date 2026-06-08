#[derive(thiserror::Error, Debug)]
pub enum Error {
   #[error("failed to read file: {0}")]
   Io(#[from] std::io::Error),
   #[error("failed to parse: {0}")]
   Parse(#[from] toml::de::Error)
}

pub type Result<T> = std::result::Result<T, Error>;
