use common::tracker::{Marker, MarkerOffsets};

pub fn read(input: &str) -> Vec<Marker> {
    let mut lines = input.lines();
    let headers: Vec<&str> = lines.next().expect("Missing header").split(",").collect();
    let idx_tracker: Vec<_> = headers
        .iter()
        .by_ref()
        .enumerate()
        .filter_map(|(i, c)| c.contains("Port").then_some(i))
        .collect();
    if idx_tracker.len() < 2 {
        panic!("Too few trackers in file. Check all trackers are active");
    }

    let o = MarkerOffsets::from_headers(&headers);

    let num_lines = input.lines().count() - 1;
    let mut markers: Vec<Marker> = idx_tracker
        .iter()
        .map(|i| Marker::with_capacity(num_lines).with_label(headers[*i]))
        .collect();

    for line in lines {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let line: Vec<&str> = line.split(",").collect();
        for (idx_marker, idx) in idx_tracker.iter().enumerate() {
            let marker = &mut markers[idx_marker];
            marker.q0.push(parse_f32(line[idx - 1 + o.q0]));
            marker.qx.push(parse_f32(line[idx - 1 + o.qx]));
            marker.qy.push(parse_f32(line[idx - 1 + o.qy]));
            marker.qz.push(parse_f32(line[idx - 1 + o.qz]));
            marker.tx.push(parse_f32(line[idx - 1 + o.tx]));
            marker.ty.push(parse_f32(line[idx - 1 + o.ty]));
            marker.tz.push(parse_f32(line[idx - 1 + o.tz]));
        }
    }
    markers
}

#[inline]
fn parse_f32(s: &str) -> f32 {
    lexical::parse(s.trim())
        .ok()
        .filter(|f: &f32| f.abs() < 1e20)
        .unwrap_or(f32::NAN)
}

#[cfg(test)]
mod test {
    use super::*;

    const INPUT: &str = r"Tools,Port 0x01: JJH Probe  s/n:3E4C880C,Frame,Time [sec],Face,State,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,Markers,State,Tx,Ty,Tz,State,Tx,Ty,Tz,State,Tx,Ty,Tz,Port 0x02: JJH T tibia tracker  s/n:3E4C8800,Frame,Time [sec],Face,State,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,Markers,State,Tx,Ty,Tz,State,Tx,Ty,Tz,State,Tx,Ty,Tz,Port 0x03: JJH Y femur tracker  s/n:3E4C8801,Frame,Time [sec],Face,State,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,Markers,State,Tx,Ty,Tz,State,Tx,Ty,Tz,State,Tx,Ty,Tz
3,Port 0x01: JJH Probe  s/n:3E4C880C,650512279,1778067471.318767388,1,OK,0.5973400,0.4357213,0.5083389,0.4415014,-42.126,-236.320,-1655.290,0.2372032,3,OK,-54.047,-355.147,-1630.068,OK,-63.615,-454.763,-1607.345,OK,-65.116,-497.277,-1622.411,Port 0x02: JJH T tibia tracker  s/n:3E4C8800,650512279,1778067471.318767388,1,OK,0.4553634,0.2025390,0.1262113,-0.8577254,82.578,-64.021,-1575.863,0.2618398,3,OK,82.377,-64.192,-1575.993,OK,25.053,10.764,-1557.116,OK,127.934,1.703,-1534.221,Port 0x03: JJH Y femur tracker  s/n:3E4C8801,650512279,1778067471.318767388,1,OK,0.9642453,0.2464048,0.0975374,0.0014287,-236.679,-123.412,-1628.129,0.4350872,3,OK,-236.181,-123.598,-1628.341,OK,-290.424,-55.476,-1578.788,OK,-328.452,-128.206,-1610.619
3,Port 0x01: JJH Probe  s/n:3E4C880C,650512282,1778067471.368767388,1,OK,0.5956001,0.4373517,0.5092747,0.4411613,-42.052,-236.351,-1655.106,0.2153924,3,OK,-53.821,-355.234,-1630.110,OK,-63.259,-454.892,-1607.531,OK,-64.815,-497.413,-1622.743,Port 0x02: JJH T tibia tracker  s/n:3E4C8800,650512282,1778067471.368767388,1,OK,0.4553981,0.2022687,0.1260506,-0.8577943,82.615,-64.096,-1575.851,0.2525886,3,OK,82.418,-64.257,-1575.975,OK,25.102,10.697,-1557.125,OK,127.977,1.644,-1534.266,Port 0x03: JJH Y femur tracker  s/n:3E4C8801,650512282,1778067471.368767388,1,OK,0.9642249,0.2467881,0.0967675,0.0014112,-236.621,-123.482,-1628.116,0.4386713,3,OK,-236.127,-123.655,-1628.319,OK,-290.380,-55.538,-1578.798,OK,-328.442,-128.274,-1610.756
3,Port 0x01: JJH Probe  s/n:3E4C880C,650537443,1778067890.718813102,1,Too Few Markers,-3.697314E28,-3.697314E28,-3.697314E28,-3.697314E28,-3.697314E28,-3.697314E28,-3.697314E28,0.0000000,3,Off Angle,-41.508,-298.640,-1545.255,Off Angle,-68.538,-345.721,-1458.348,Off Angle,-84.766,-383.634,-1439.392,Port 0x02: JJH T tibia tracker  s/n:3E4C8800,650537443,1778067890.718813102,1,OK,0.4551402,0.1988662,0.1192681,-0.8596946,83.131,-63.515,-1577.213,0.2485822,3,OK,82.937,-63.672,-1577.332,OK,25.921,11.786,-1559.596,OK,128.785,2.653,-1536.650,Port 0x03: JJH Y femur tracker  s/n:3E4C8801,650537443,1778067890.718813102,1,OK,0.9641443,0.2468603,0.0973864,0.0012952,-236.614,-123.753,-1628.163,0.4438388,3,OK,-236.115,-123.945,-1628.379,OK,-290.336,-55.816,-1578.760,OK,-328.396,-128.540,-1610.684
        ";
    #[test]
    fn handles_nan() {
        let markers = vec![
            Marker {
                label: "Port 0x01: JJH Probe  s/n:3E4C880C".into(),
                tx: vec![-42.126, -42.052, f32::NAN],
                ty: vec![-236.32, -236.351, f32::NAN],
                tz: vec![-1655.29, -1655.106, f32::NAN],
                q0: vec![0.59734, 0.5956001, f32::NAN],
                qx: vec![0.4357213, 0.4373517, f32::NAN],
                qy: vec![0.5083389, 0.5092747, f32::NAN],
                qz: vec![0.4415014, 0.4411613, f32::NAN],
            },
            Marker {
                label: "Port 0x02: JJH T tibia tracker  s/n:3E4C8800".into(),
                tx: vec![82.578, 82.615, 83.131],
                ty: vec![-64.021, -64.096, -63.515],
                tz: vec![-1575.863, -1575.851, -1577.213],
                q0: vec![0.4553634, 0.4553981, 0.4551402],
                qx: vec![0.202539, 0.2022687, 0.1988662],
                qy: vec![0.1262113, 0.1260506, 0.1192681],
                qz: vec![-0.8577254, -0.8577943, -0.8596946],
            },
            Marker {
                label: "Port 0x03: JJH Y femur tracker  s/n:3E4C8801".into(),
                tx: vec![-236.679, -236.621, -236.614],
                ty: vec![-123.412, -123.482, -123.753],
                tz: vec![-1628.129, -1628.116, -1628.163],
                q0: vec![0.9642453, 0.9642249, 0.9641443],
                qx: vec![0.2464048, 0.2467881, 0.2468603],
                qy: vec![0.0975374, 0.0967675, 0.0973864],
                qz: vec![0.0014287, 0.0014112, 0.0012952],
            },
        ];

        let from_text = &read(INPUT)[0];

        assert_eq!(markers[0].tx[2].is_nan(), from_text.tx[2].is_nan())
    }

    #[test]
    fn splits_headers() {
        let markers = read(INPUT);
        // println!("{:?}", markers);
    }
}
