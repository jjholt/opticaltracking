use std::fs::File;
use std::io::{self, BufRead};

pub fn read(path: &str) -> io::Result<()> {
    // let file = File::open(path)?;
    let mut reader = csv::Reader::from_path(path)?;
    let headers = reader.headers()?.iter();
    let mask: Vec<bool> = headers.clone().map(|c| !c.contains("Port")).collect();
    let filtered: Vec<_> = headers.zip(mask).filter(|(_, b)| *b).collect();
    println!("{:#?}", filtered);
    // let mut reader = io::BufReader::new(file);
    // let test = reader.lines().take(2).filter_map(Result::ok);
    // let test = test.map(|line| {
    //     line.split("Port ").skip(1).enumerate()
    // });
    // println!("{:#?}", test);
    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn reads_file() {
        crate::polaris::read("data.csv").unwrap();
    }
}
