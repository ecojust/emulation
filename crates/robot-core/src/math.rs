//! Small, allocation-free f64 operations. Public quaternions are scalar-first.
use crate::types::{Mat4, Quaternion, Vec3};

pub const IDENTITY: Mat4 = [
    1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
];

#[inline]
pub fn dot(a: &Vec3, b: &Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
pub fn sub(a: &Vec3, b: &Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] - b[i])
}
#[inline]
pub fn cross(a: &Vec3, b: &Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
#[inline]
pub fn norm(a: &Vec3) -> f64 {
    a[0].hypot(a[1]).hypot(a[2])
}
#[inline]
pub fn unit(a: &Vec3) -> Option<Vec3> {
    let length = norm(a);
    (length.is_finite() && length >= 1e-10).then(|| a.map(|v| v / length))
}
#[inline]
pub fn translation(m: &Mat4) -> Vec3 {
    [m[3], m[7], m[11]]
}

pub fn multiply_matrices(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut out = [0.; 16];
    for row in 0..4 {
        for col in 0..4 {
            out[row * 4 + col] = (0..4).map(|k| a[row * 4 + k] * b[k * 4 + col]).sum();
        }
    }
    out
}

/// Normalize a mathematical quaternion, scaling first to avoid overflow.
pub fn normalize_quaternion(value: &Quaternion) -> Result<Quaternion, String> {
    if value.iter().any(|v| !v.is_finite()) {
        return Err("四元数需要四个有限数值".into());
    }
    let scale = value.iter().fold(0_f64, |a, b| a.max(b.abs()));
    if scale < 1e-12 {
        return Err("四元数不能为零".into());
    }
    let scaled = value.map(|v| v / scale);
    let length = scaled.iter().map(|v| v * v).sum::<f64>().sqrt();
    let result = scaled.map(|v| v / length);
    let sign = result
        .iter()
        .find(|v| v.abs() > 1e-12)
        .copied()
        .unwrap_or(1.)
        .signum();
    Ok(result.map(|v| v * sign))
}

/// Strict public orientation validation matching ABB NOrient's correction range.
pub fn normalize_orientation(value: &Quaternion) -> Result<Quaternion, String> {
    let sum = value.iter().map(|v| v * v).sum::<f64>();
    if !sum.is_finite() || (sum - 1.).abs() > 0.1 + 1e-12 {
        return Err("四元数不是有效的单位姿态，请检查 [q1,q2,q3,q4]，其中 q1 为实部".into());
    }
    normalize_quaternion(value)
}

pub fn quaternion_to_matrix(value: &Quaternion) -> Result<Mat4, String> {
    let [w, x, y, z] = normalize_quaternion(value)?;
    Ok([
        1. - 2. * (y * y + z * z),
        2. * (x * y - w * z),
        2. * (x * z + w * y),
        0.,
        2. * (x * y + w * z),
        1. - 2. * (x * x + z * z),
        2. * (y * z - w * x),
        0.,
        2. * (x * z - w * y),
        2. * (y * z + w * x),
        1. - 2. * (x * x + y * y),
        0.,
        0.,
        0.,
        0.,
        1.,
    ])
}

/// Input is a valid rigid rotation; invalid matrices yield NaN, never an invented pose.
pub fn matrix_to_quaternion(m: &Mat4) -> Quaternion {
    let trace = m[0] + m[5] + m[10];
    let q = if trace > 0. {
        let s = (trace + 1.).max(0.).sqrt() * 2.;
        [
            s / 4.,
            (m[9] - m[6]) / s,
            (m[2] - m[8]) / s,
            (m[4] - m[1]) / s,
        ]
    } else if m[0] > m[5] && m[0] > m[10] {
        let s = (1. + m[0] - m[5] - m[10]).max(0.).sqrt() * 2.;
        [
            (m[9] - m[6]) / s,
            s / 4.,
            (m[1] + m[4]) / s,
            (m[2] + m[8]) / s,
        ]
    } else if m[5] > m[10] {
        let s = (1. + m[5] - m[0] - m[10]).max(0.).sqrt() * 2.;
        [
            (m[2] - m[8]) / s,
            (m[1] + m[4]) / s,
            s / 4.,
            (m[6] + m[9]) / s,
        ]
    } else {
        let s = (1. + m[10] - m[0] - m[5]).max(0.).sqrt() * 2.;
        [
            (m[4] - m[1]) / s,
            (m[2] + m[8]) / s,
            (m[6] + m[9]) / s,
            s / 4.,
        ]
    };
    normalize_quaternion(&q).unwrap_or([f64::NAN; 4])
}

pub fn quaternion_distance_deg(a: &Quaternion, b: &Quaternion) -> f64 {
    let (Ok(a), Ok(b)) = (normalize_quaternion(a), normalize_quaternion(b)) else {
        return f64::NAN;
    };
    let dot = a
        .iter()
        .zip(b)
        .map(|(a, b)| a * b)
        .sum::<f64>()
        .abs()
        .clamp(-1., 1.);
    (2. * dot.acos()).to_degrees()
}

pub fn slerp_quaternion(a: &Quaternion, b: &Quaternion, t: f64) -> Quaternion {
    let (Ok(a), Ok(mut b)) = (normalize_quaternion(a), normalize_quaternion(b)) else {
        return [f64::NAN; 4];
    };
    let mut dot = a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>();
    if dot < 0. {
        b = b.map(|v| -v);
        dot = -dot;
    }
    dot = dot.clamp(-1., 1.);
    let q = if dot > 0.9995 {
        std::array::from_fn(|i| a[i] + t * (b[i] - a[i]))
    } else {
        let angle = dot.acos();
        let inv_sine = 1. / angle.sin();
        let sa = ((1. - t) * angle).sin() * inv_sine;
        let sb = (t * angle).sin() * inv_sine;
        std::array::from_fn(|i| sa * a[i] + sb * b[i])
    };
    normalize_quaternion(&q).unwrap_or([f64::NAN; 4])
}
