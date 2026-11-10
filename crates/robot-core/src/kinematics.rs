//! Standard-DH FK and analytic spherical-wrist IK for the two registered IRB 1100 models.
//! Matrices, points, axes and bounded angle variants use stack arrays.
use crate::configuration::get_configuration;
use crate::math::{
    matrix_to_quaternion, multiply_matrices, norm, quaternion_distance_deg, quaternion_to_matrix,
    sub, translation, IDENTITY,
};
use crate::types::{DhRow, FkResult, IkSolution, JointAngles, Mat4, Pose, RobConf, RobotModel};
use std::f64::consts::{FRAC_PI_2, PI};
use std::fmt::Write;

pub const IK_POSITION_TOLERANCE_MM: f64 = 0.01;
pub const IK_ORIENTATION_TOLERANCE_DEG: f64 = 0.001;

#[inline]
fn transform(row: &DhRow, joint: f64) -> Mat4 {
    let (s, c) = (joint * row.theta_sign + row.theta_offset_deg)
        .to_radians()
        .sin_cos();
    // All supported alpha angles are exact multiples of 90 degrees.
    let (sa, ca) = match row.alpha_deg {
        0. => (0., 1.),
        90. => (1., 0.),
        -90. => (-1., 0.),
        value => value.to_radians().sin_cos(),
    };
    [
        c,
        -s * ca,
        s * sa,
        row.a_mm * c,
        s,
        c * ca,
        -c * sa,
        row.a_mm * s,
        0.,
        sa,
        ca,
        row.d_mm,
        0.,
        0.,
        0.,
        1.,
    ]
}

fn forward_finite(model: &RobotModel, joints: &JointAngles) -> FkResult {
    let mut frames = [IDENTITY; 7];
    let mut joint_origins = [[0.; 3]; 6];
    let mut joint_axes = [[0.; 3]; 6];
    for i in 0..6 {
        let previous = frames[i];
        joint_origins[i] = translation(&previous);
        let sign = model.dh[i].theta_sign;
        joint_axes[i] = [previous[2] * sign, previous[6] * sign, previous[10] * sign];
        frames[i + 1] = multiply_matrices(&previous, &transform(&model.dh[i], joints[i]));
    }
    FkResult {
        pose: Pose {
            position: translation(&frames[6]),
            orientation: matrix_to_quaternion(&frames[6]),
        },
        frames,
        joint_origins,
        joint_axes,
        shoulder: translation(&frames[1]),
        elbow: translation(&frames[2]),
        wrist_center: translation(&frames[4]),
    }
}

pub fn forward_kinematics(model: &RobotModel, joints: &JointAngles) -> Result<FkResult, String> {
    if joints.iter().any(|v| !v.is_finite()) {
        return Err("关节角需要六个有限数值".into());
    }
    Ok(forward_finite(model, joints))
}

pub fn joints_within_limits(model: &RobotModel, joints: &JointAngles) -> bool {
    joints
        .iter()
        .zip(&model.limits)
        .all(|(q, limit)| q.is_finite() && *q >= limit.min_deg - 1e-8 && *q <= limit.max_deg + 1e-8)
}

/// Full turns remain physical motions, never modulo-360 zero distances.
pub fn joint_distance(a: &JointAngles, b: &JointAngles) -> f64 {
    const WEIGHTS: [f64; 6] = [1., 1., 1., 0.7, 0.8, 0.45];
    (0..6)
        .map(|i| WEIGHTS[i] * (a[i] - b[i]).powi(2))
        .sum::<f64>()
        .sqrt()
}

struct AngleVariants {
    values: [f64; 4],
    count: usize,
}
impl AngleVariants {
    fn iter(&self) -> std::slice::Iter<'_, f64> {
        self.values[..self.count].iter()
    }
}

/// Current model limits span at most 800 degrees, hence at most three turns.
fn equivalent_angles(degrees: f64, min: f64, max: f64) -> AngleVariants {
    let mut result = AngleVariants {
        values: [0.; 4],
        count: 0,
    };
    if !degrees.is_finite()
        || !min.is_finite()
        || !max.is_finite()
        || max < min
        || max - min > 1080.
    {
        return result;
    }
    let first = ((min - degrees - 1e-8) / 360.).ceil() as i32;
    let last = ((max - degrees + 1e-8) / 360.).floor() as i32;
    for turn in first..=last {
        if result.count == result.values.len() {
            break;
        }
        let q = (degrees + f64::from(turn) * 360.).clamp(min, max);
        result.values[result.count] = if q.abs() < 1e-10 { 0. } else { q };
        result.count += 1;
    }
    result
}

#[derive(Clone, Debug)]
pub struct RawCandidate {
    pub joints: JointAngles,
    pub pose: Pose,
    pub configuration: Option<RobConf>,
    pub singular: bool,
    pub position_error_mm: f64,
    pub orientation_error_deg: f64,
    pub score: f64,
}

fn lexical_joints(a: &JointAngles, b: &JointAngles) -> std::cmp::Ordering {
    for i in 0..6 {
        let order = a[i].total_cmp(&b[i]);
        if !order.is_eq() {
            return order;
        }
    }
    std::cmp::Ordering::Equal
}

/// Eight physical branches, expanded to every complete-turn representation inside limits.
/// This analytic decomposition is specific to the registered IRB1100 DH chain.
pub fn solve_ik_candidates(
    model: &RobotModel,
    pose: &Pose,
    seed: &JointAngles,
) -> Vec<RawCandidate> {
    solve_ik_candidates_cancellable(model, pose, seed, &mut || false)
        .expect("a non-cancellable IK query cannot be cancelled")
}

fn check_ik_cancelled(cancelled: &mut impl FnMut() -> bool) -> Result<(), String> {
    if cancelled() {
        Err("端点逆解检查已取消".into())
    } else {
        Ok(())
    }
}

fn solve_ik_candidates_cancellable(
    model: &RobotModel,
    pose: &Pose,
    seed: &JointAngles,
    cancelled: &mut impl FnMut() -> bool,
) -> Result<Vec<RawCandidate>, String> {
    check_ik_cancelled(cancelled)?;
    if pose.position.iter().chain(seed).any(|v| !v.is_finite()) {
        return Ok(Vec::new());
    }
    let Ok(target_rotation) = quaternion_to_matrix(&pose.orientation) else {
        return Ok(Vec::new());
    };
    let g = &model.geometry;
    let wrist = [
        pose.position[0] - g.flange_mm * target_rotation[2],
        pose.position[1] - g.flange_mm * target_rotation[6],
        pose.position[2] - g.flange_mm * target_rotation[10],
    ];
    let rho = wrist[0].hypot(wrist[1]);
    let vertical = wrist[2] - g.shoulder_height_mm;
    let lower = g.elbow_offset_mm.hypot(g.forearm_mm);
    let offset_angle = g.elbow_offset_mm.atan2(g.forearm_mm);
    let cosine = (rho * rho + vertical * vertical - g.upper_arm_mm.powi(2) - lower * lower)
        / (2. * g.upper_arm_mm * lower);
    if !cosine.is_finite() || !(-1. - 1e-10..=1. + 1e-10).contains(&cosine) {
        return Ok(Vec::new());
    }
    let elbow_angle = cosine.clamp(-1., 1.).acos();
    let shoulder_base = if rho < 1e-8 {
        seed[0].to_radians()
    } else {
        wrist[1].atan2(wrist[0])
    };
    let mut candidates: Vec<RawCandidate> = Vec::with_capacity(48);
    // Integer coordinate keys avoid formatting six strings for every candidate.
    let mut seen: Vec<[i64; 6]> = Vec::with_capacity(48);
    let values = |angle: f64, i: usize| {
        equivalent_angles(angle, model.limits[i].min_deg, model.limits[i].max_deg)
    };
    for shoulder in [shoulder_base, shoulder_base + PI] {
        check_ik_cancelled(cancelled)?;
        let horizontal = wrist[0] * shoulder.cos() + wrist[1] * shoulder.sin();
        for bend in [elbow_angle, -elbow_angle] {
            let shoulder_angle = FRAC_PI_2 - vertical.atan2(horizontal)
                + (lower * bend.sin()).atan2(g.upper_arm_mm + lower * bend.cos());
            let q3 = (offset_angle - FRAC_PI_2 - bend).to_degrees();
            for &q1 in values(shoulder.to_degrees(), 0).iter() {
                for &q2 in values(shoulder_angle.to_degrees(), 1).iter() {
                    for &elbow in values(q3, 2).iter() {
                        let mut arm_frame = IDENTITY;
                        for (i, q) in [q1, q2, elbow].iter().enumerate() {
                            arm_frame = multiply_matrices(&arm_frame, &transform(&model.dh[i], *q));
                        }
                        let mut relative = IDENTITY;
                        for row in 0..3 {
                            for col in 0..3 {
                                relative[row * 4 + col] = (0..3)
                                    .map(|k| arm_frame[k * 4 + row] * target_rotation[k * 4 + col])
                                    .sum();
                            }
                        }
                        // R36 = Rz(q4) Ry(-q5) Rz(q6 + 180 degrees).
                        let middle = relative[10].clamp(-1., 1.).acos();
                        let (wrists, count) = if middle.sin().abs() < 1e-9 {
                            if relative[10] < 0. {
                                continue;
                            } // A5=180 lies outside both models.
                            let sum = relative[4].atan2(relative[0]).to_degrees();
                            ([[seed[3], 0., sum - seed[3] - 180.], [0.; 3]], 1)
                        } else {
                            let a = relative[6].atan2(relative[2]).to_degrees();
                            let c = relative[9].atan2(-relative[8]).to_degrees();
                            (
                                [
                                    [a, -middle.to_degrees(), c - 180.],
                                    [a + 180., middle.to_degrees(), c],
                                ],
                                2,
                            )
                        };
                        for [a4, a5, a6] in &wrists[..count] {
                            for &q4 in values(*a4, 3).iter() {
                                for &q5 in values(*a5, 4).iter() {
                                    for &q6 in values(*a6, 5).iter() {
                                        check_ik_cancelled(cancelled)?;
                                        let joints = [q1, q2, elbow, q4, q5, q6];
                                        let key = joints.map(|v| (v * 1e6).round() as i64);
                                        if seen.contains(&key) {
                                            continue;
                                        }
                                        seen.push(key);
                                        let fk = forward_finite(model, &joints);
                                        let position_error_mm =
                                            norm(&sub(&fk.pose.position, &pose.position));
                                        let orientation_error_deg = quaternion_distance_deg(
                                            &fk.pose.orientation,
                                            &pose.orientation,
                                        );
                                        if !position_error_mm.is_finite()
                                            || !orientation_error_deg.is_finite()
                                            || position_error_mm > IK_POSITION_TOLERANCE_MM
                                            || orientation_error_deg > IK_ORIENTATION_TOLERANCE_DEG
                                        {
                                            continue;
                                        }
                                        let configuration = get_configuration(model, &joints, &fk);
                                        candidates.push(RawCandidate {
                                            joints,
                                            pose: fk.pose,
                                            configuration: configuration.configuration,
                                            singular: configuration.singular,
                                            position_error_mm,
                                            orientation_error_deg,
                                            score: joint_distance(&joints, seed),
                                        });
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    check_ik_cancelled(cancelled)?;
    candidates.sort_unstable_by(|a, b| {
        a.score
            .total_cmp(&b.score)
            .then_with(|| lexical_joints(&a.joints, &b.joints))
    });
    check_ik_cancelled(cancelled)?;
    Ok(candidates)
}

pub fn inverse_kinematics(model: &RobotModel, pose: &Pose, seed: &JointAngles) -> Vec<IkSolution> {
    candidates_to_solutions(solve_ik_candidates(model, pose, seed))
}

/// Endpoint-only inspection can be interrupted without returning a partial set of branches.
pub fn inverse_kinematics_cancellable(
    model: &RobotModel,
    pose: &Pose,
    seed: &JointAngles,
    mut cancelled: impl FnMut() -> bool,
) -> Result<Vec<IkSolution>, String> {
    let candidates = solve_ik_candidates_cancellable(model, pose, seed, &mut cancelled)?;
    let result = candidates_to_solutions(candidates);
    check_ik_cancelled(&mut cancelled)?;
    Ok(result)
}

fn candidates_to_solutions(candidates: Vec<RawCandidate>) -> Vec<IkSolution> {
    candidates
        .into_iter()
        .filter_map(|candidate| {
            let configuration = candidate.configuration?;
            if candidate.singular {
                return None;
            }
            let mut id = String::with_capacity(72);
            for (index, q) in candidate.joints.iter().enumerate() {
                if index > 0 {
                    id.push('/');
                }
                // Writing into a String is infallible; one allocation per id.
                let _ = write!(id, "{q:.4}");
            }
            Some(IkSolution {
                id,
                joints: candidate.joints,
                pose: candidate.pose,
                configuration,
                position_error_mm: candidate.position_error_mm,
                orientation_error_deg: candidate.orientation_error_deg,
                score: candidate.score,
            })
        })
        .collect()
}
