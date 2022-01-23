use std::f64::consts::PI;

use osmpbfreader::Node;


pub fn bearing(node_a: &Node, node_b: &Node) -> f64 {
    let lat_a = node_a.lat() * PI / 180.0;
    let lat_b = node_b.lat() * PI / 180.0;
    let lon_a = node_a.lon() * PI / 180.0;
    let lon_b = node_b.lon() * PI / 180.0;

    (lat_b - lat_a).atan2(lon_b - lon_a)
}

#[allow(non_snake_case)]
#[allow(unused)]
pub fn geo_distance_meters(lat1_deg: f64, lon1_deg: f64, lat2_deg: f64, lon2_deg: f64) -> f64 {
    let lat1_rad = lat1_deg * PI / 180.0;
    let lat2_rad = lat2_deg * PI / 180.0;
    let lon1_rad = lon1_deg * PI / 180.0;
    let lon2_rad = lon2_deg * PI / 180.0;

    // Vincenty's formulae
    let a = 6378137.0; // WGS84
    let f = 1.0/298.257223563; // WGS84
    let b = (1.0 - f) * a;

    let U1 = ((1.0 - f) * lat1_rad.tan()).atan();
    let U2 = ((1.0 - f) * lat2_rad.tan()).atan();
    let L = lon2_rad - lon1_rad;

    let mut lambda = L;
    let mut cos2_alpha;
    let mut sin_sigma;
    let mut cos_sigma;
    let mut sigma;
    let mut cos_2sigmam;
    loop {
        let prev_lambda = lambda;
        sin_sigma = (
            (U2.cos() * lambda.sin()).powi(2)
            + (U1.cos() * U2.sin() - U1.sin() * U2.cos() * lambda.cos()).powi(2)
        ).sqrt();
        cos_sigma = U1.sin() * U2.sin() + U1.cos() * U2.cos() * lambda.cos();
        sigma = sin_sigma.atan2(cos_sigma);
        let sin_alpha = (U1.cos() * U2.cos() * lambda.sin()) / sigma.sin();
        cos2_alpha = 1.0 - sin_alpha.powi(2);
        cos_2sigmam = sigma.cos() - (2.0 * U1.sin() * U2.sin()) / cos2_alpha;
        let C = f / 16.0 * cos2_alpha * (4.0 + f * (4.0 - 3.0 * cos2_alpha));
        lambda = L + (1.0 - C) * f * sin_alpha * (
            sigma + C * sin_sigma * (
                cos_2sigmam + C * cos_sigma * (
                    -1.0 + 2.0 * cos_2sigmam.powi(2)
                )
            )
        );
        if (lambda - prev_lambda).abs() < 1e-6 {
            break;
        }
    }

    let u2 = cos2_alpha * (a.powi(2) - b.powi(2)) / b.powi(2);
    let A = 1.0 + u2 / 16384.0 * (4096.0 + u2 * (-768.0 + u2 * (320.0 - 175.0 * u2)));
    let B = u2 / 1024.0 * (256.0 + u2 * (128.0 + u2 * (74.0 - 47.0 * u2)));
    let delta_sigma = B * sin_sigma * (
        cos_2sigmam + 1.0/4.0 * B * (
            cos_sigma * (
                -1.0 + 2.0 * cos_2sigmam.powi(2)
            )
            - B/6.0 * cos_2sigmam * (-3.0 + 4.0 * sin_sigma.powi(2)) * (-3.0 + 4.0 * cos_2sigmam.powi(2))
        )
    );
    let s = b * A * (sigma - delta_sigma);

    s
}

pub fn sphere_distance_meters(lat1_deg: f64, lon1_deg: f64, lat2_deg: f64, lon2_deg: f64) -> f64 {
    let lat1_rad = lat1_deg * PI / 180.0;
    let lat2_rad = lat2_deg * PI / 180.0;
    let lon1_rad = lon1_deg * PI / 180.0;
    let lon2_rad = lon2_deg * PI / 180.0;

    // haversine formula
    let a = 6378137.0; // WGS84
    let f = 1.0/298.257223563; // WGS84
    let b = (1.0 - f) * a;
    let r = (a + b) / 2.0;

    fn haversine(theta: f64) -> f64 {
        (1.0 - theta.cos()) / 2.0
    }

    let hav = haversine(lat2_rad - lat1_rad) + lat1_rad.cos() * lat2_rad.cos() * haversine(lon2_rad - lon1_rad);
    let d = 2.0 * r * hav.sqrt().asin();

    d
}

pub fn bearing_diff_rad(bearing1_rad: f64, bearing2_rad: f64) -> f64 {
    let mut left = bearing1_rad - bearing2_rad;
    let mut right = bearing2_rad - bearing1_rad;

    if left < 0.0 {
        left += 2.0 * PI;
    }
    if right < 0.0 {
        right += 2.0 * PI;
    }

    if left < right {
        -left
    } else {
        right
    }
}
