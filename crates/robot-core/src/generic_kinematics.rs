//! Bounded six-axis URDF kinematics, independent of ABB DH and robconf rules.
use crate::math::{
    cross, dot, matrix_to_quaternion, multiply_matrices, norm, sub, translation, IDENTITY,
};
use crate::types::{JointAngles, JointLimit, Mat4, Pose, Quaternion, Vec3};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};

pub const MAX_CHAIN_STEPS: usize = 32;
pub const IK_POSITION_TOLERANCE_MM: f64 = 0.001;
pub const IK_ORIENTATION_TOLERANCE_RAD: f64 = 1e-5;
pub const IK_MAX_ITERATIONS: usize = 80;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GenericChainStep {
    /// Parent-to-joint fixed transform, row-major, translation in millimetres.
    pub origin: Mat4,
    /// Unit rotation axis in the joint frame; ignored for fixed steps.
    pub axis: Vec3,
    #[serde(default)]
    pub joint_index: Option<usize>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GenericChain {
    pub model_id: String,
    /// Only root-to-tool ancestors, including all fixed tool transforms.
    pub steps: Vec<GenericChainStep>,
    pub limits: [JointLimit; 6],
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GenericFkResult {
    pub pose: Pose,
    pub transform: Mat4,
    pub joint_origins: [Vec3; 6],
    pub joint_axes: [Vec3; 6],
}

pub fn validate_generic_chain(chain: &GenericChain) -> Result<(), String> {
    if chain.model_id.trim().is_empty()
        || chain.model_id.len() > 128
        || !(6..=MAX_CHAIN_STEPS).contains(&chain.steps.len())
    {
        return Err("URDF 运动链需要有效型号与 6 至 32 个步骤。".into());
    }
    let mut seen = [false; 6];
    let mut reach = 0.;
    for step in &chain.steps {
        let m = &step.origin;
        if m.iter().any(|v| !v.is_finite())
            || m[12].abs() > 1e-10
            || m[13].abs() > 1e-10
            || m[14].abs() > 1e-10
            || (m[15] - 1.).abs() > 1e-10
            || [m[3], m[7], m[11]].iter().any(|v| v.abs() > 100_000.)
        {
            return Err("URDF 固定变换不是有界刚体矩阵。".into());
        }
        let columns = [[m[0], m[4], m[8]], [m[1], m[5], m[9]], [m[2], m[6], m[10]]];
        if (0..3).any(|i| {
            (dot(&columns[i], &columns[i]) - 1.).abs() > 1e-8
                || (0..i).any(|j| dot(&columns[i], &columns[j]).abs() > 1e-8)
        }) || (dot(&columns[0], &cross(&columns[1], &columns[2])) - 1.).abs() > 1e-8
            || step.axis.iter().any(|v| !v.is_finite())
            || (norm(&step.axis) - 1.).abs() > 1e-8
        {
            return Err("URDF 旋转与关节轴必须正交、右手且归一化。".into());
        }
        reach += norm(&translation(m));
        if let Some(index) = step.joint_index {
            if index >= 6 || seen[index] {
                return Err("URDF 必须恰好包含六个不重复的旋转关节。".into());
            }
            seen[index] = true;
        }
    }
    if seen.iter().any(|value| !value) || reach > 1_000_000. {
        return Err("URDF 运动链关节缺失或长度超出范围。".into());
    }
    for limit in &chain.limits {
        if !limit.min_deg.is_finite()
            || !limit.max_deg.is_finite()
            || !limit.max_speed_deg.is_finite()
            || limit.min_deg >= limit.max_deg
            || limit.min_deg.abs() > 3600.
            || limit.max_deg.abs() > 3600.
            || limit.max_speed_deg <= 0.
            || limit.max_speed_deg > 10_000.
        {
            return Err("URDF 关节范围或速度无效。".into());
        }
    }
    Ok(())
}

pub fn generic_joints_within_limits(chain: &GenericChain, joints: &JointAngles) -> bool {
    joints.iter().zip(&chain.limits).all(|(angle, limit)| {
        angle.is_finite() && *angle >= limit.min_deg - 1e-9 && *angle <= limit.max_deg + 1e-9
    })
}

fn rotation(axis: &Vec3, angle: f64) -> Mat4 {
    let [x, y, z] = *axis;
    let (s, c) = angle.sin_cos();
    let d = 1. - c;
    [
        c + x * x * d,
        x * y * d - z * s,
        x * z * d + y * s,
        0.,
        y * x * d + z * s,
        c + y * y * d,
        y * z * d - x * s,
        0.,
        z * x * d - y * s,
        z * y * d + x * s,
        c + z * z * d,
        0.,
        0.,
        0.,
        0.,
        1.,
    ]
}

pub(crate) fn forward_unchecked<F: FnMut() -> bool>(
    chain: &GenericChain,
    joints: &JointAngles,
    cancel: &mut F,
) -> Result<GenericFkResult, ()> {
    let mut transform = IDENTITY;
    let mut joint_origins = [[0.; 3]; 6];
    let mut joint_axes = [[0.; 3]; 6];
    for step in &chain.steps {
        if cancel() {
            return Err(());
        }
        transform = multiply_matrices(&transform, &step.origin);
        if let Some(index) = step.joint_index {
            joint_origins[index] = translation(&transform);
            joint_axes[index] = std::array::from_fn(|row| {
                (0..3)
                    .map(|column| transform[row * 4 + column] * step.axis[column])
                    .sum()
            });
            transform = multiply_matrices(
                &transform,
                &rotation(&step.axis, joints[index].to_radians()),
            );
        }
    }
    Ok(GenericFkResult {
        pose: Pose {
            position: translation(&transform),
            orientation: matrix_to_quaternion(&transform),
        },
        transform,
        joint_origins,
        joint_axes,
    })
}

pub fn forward_generic_kinematics(
    chain: &GenericChain,
    joints: &JointAngles,
) -> Result<GenericFkResult, String> {
    validate_generic_chain(chain)?;
    if !generic_joints_within_limits(chain, joints) {
        return Err("关节角无效或超出 URDF 限位。".into());
    }
    forward_unchecked(chain, joints, &mut || false).map_err(|_| "运动计算已取消。".into())
}

/// Spatial shortest rotation vector, log(target * inverse(current)), in radians.
pub(crate) fn orientation_error(target: &Quaternion, current: &Quaternion) -> Vec3 {
    let [a, b, c, d] = *target;
    let [w, x, y, z] = *current;
    let mut relative = [
        a * w + b * x + c * y + d * z,
        -a * x + b * w - c * z + d * y,
        -a * y + b * z + c * w - d * x,
        -a * z - b * y + c * x + d * w,
    ];
    if relative[0] < 0. {
        relative = relative.map(|value| -value);
    }
    let sine = norm(&[relative[1], relative[2], relative[3]]);
    let factor = if sine < 1e-12 {
        2.
    } else {
        2. * sine.atan2(relative[0].max(0.)) / sine
    };
    [
        relative[1] * factor,
        relative[2] * factor,
        relative[3] * factor,
    ]
}

fn residual(target: &Pose, current: &Pose, scale: f64) -> ([f64; 6], bool) {
    let position = sub(&target.position, &current.position);
    let orientation = orientation_error(&target.orientation, &current.orientation);
    (
        [
            position[0] / scale,
            position[1] / scale,
            position[2] / scale,
            orientation[0],
            orientation[1],
            orientation[2],
        ],
        norm(&position) <= IK_POSITION_TOLERANCE_MM
            && norm(&orientation) <= IK_ORIENTATION_TOLERANCE_RAD,
    )
}

/// Small fixed 6x6 partial-pivot elimination, without a numerical-library allocation.
fn solve_linear(mut matrix: [[f64; 6]; 6], mut rhs: [f64; 6]) -> Option<[f64; 6]> {
    for column in 0..6 {
        let pivot = (column..6).max_by(|a, b| {
            matrix[*a][column]
                .abs()
                .total_cmp(&matrix[*b][column].abs())
        })?;
        if matrix[pivot][column].abs() < 1e-16 {
            return None;
        }
        matrix.swap(column, pivot);
        rhs.swap(column, pivot);
        for row in column + 1..6 {
            let factor = matrix[row][column] / matrix[column][column];
            matrix[row][column] = 0.;
            for j in column + 1..6 {
                matrix[row][j] -= factor * matrix[column][j];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    let mut result = [0.; 6];
    for row in (0..6).rev() {
        result[row] = (rhs[row]
            - (row + 1..6)
                .map(|column| matrix[row][column] * result[column])
                .sum::<f64>())
            / matrix[row][row];
    }
    result
        .iter()
        .all(|value| value.is_finite())
        .then_some(result)
}

pub(crate) fn solve_unchecked<F: FnMut() -> bool>(
    chain: &GenericChain,
    target: &Pose,
    seed: &JointAngles,
    cancel: &mut F,
) -> Result<Option<JointAngles>, ()> {
    let scale = chain
        .steps
        .iter()
        .map(|step| norm(&translation(&step.origin)))
        .sum::<f64>()
        .max(100.);
    let mut joints = *seed;
    let mut damping: f64 = 0.001;
    for _ in 0..IK_MAX_ITERATIONS {
        if cancel() {
            return Err(());
        }
        let fk = forward_unchecked(chain, &joints, cancel)?;
        let (error, converged) = residual(target, &fk.pose, scale);
        if converged {
            return Ok(Some(joints));
        }
        let cost = error.iter().map(|v| v * v).sum::<f64>();
        let mut jacobian = [[0.; 6]; 6];
        for column in 0..6 {
            let linear = cross(
                &fk.joint_axes[column],
                &sub(&fk.pose.position, &fk.joint_origins[column]),
            );
            for row in 0..3 {
                jacobian[row][column] = linear[row] / scale;
                jacobian[row + 3][column] = fk.joint_axes[column][row];
            }
        }
        let normal: [[f64; 6]; 6] = std::array::from_fn(|row| {
            std::array::from_fn(|column| {
                (0..6)
                    .map(|k| jacobian[k][row] * jacobian[k][column])
                    .sum::<f64>()
                    + if row == column { damping * damping } else { 0. }
            })
        });
        let rhs = std::array::from_fn(|column| {
            (0..6).map(|row| jacobian[row][column] * error[row]).sum()
        });
        let Some(mut delta) = solve_linear(normal, rhs) else {
            return Ok(None);
        };
        let largest = delta
            .iter()
            .fold(0_f64, |value, next| value.max(next.abs()));
        let cap = 4_f64.to_radians();
        if largest > cap {
            delta = delta.map(|value| value * cap / largest);
        }
        let mut accepted = false;
        for attempt in 0..8 {
            if cancel() {
                return Err(());
            }
            let alpha = 0.5_f64.powi(attempt);
            let trial = std::array::from_fn(|i| {
                (joints[i] + (delta[i] * alpha).to_degrees())
                    .clamp(chain.limits[i].min_deg, chain.limits[i].max_deg)
            });
            let trial_fk = forward_unchecked(chain, &trial, cancel)?;
            let (trial_error, done) = residual(target, &trial_fk.pose, scale);
            let trial_cost = trial_error.iter().map(|value| value * value).sum::<f64>();
            if done || trial_cost < cost * (1. - 1e-10) {
                joints = trial;
                accepted = true;
                damping = (damping * 0.5).max(1e-6);
                if done {
                    return Ok(Some(joints));
                }
                break;
            }
        }
        if !accepted {
            if damping >= 1. {
                return Ok(None);
            }
            damping = (damping * 10.).min(1.);
        }
    }
    Ok(None)
}

pub fn inverse_generic_kinematics_cancellable<F: FnMut() -> bool>(
    chain: &GenericChain,
    target: &Pose,
    seed: &JointAngles,
    mut cancel: F,
) -> Result<Option<JointAngles>, String> {
    if cancel() {
        return Err("运动计算已取消。".into());
    }
    validate_generic_chain(chain)?;
    if !generic_joints_within_limits(chain, seed)
        || target
            .position
            .iter()
            .any(|v| !v.is_finite() || v.abs() > 1_000_000.)
        || target.orientation.iter().any(|v| !v.is_finite())
        || (target.orientation.iter().map(|v| v * v).sum::<f64>().sqrt() - 1.).abs() > 1e-6
    {
        return Err("数值逆解的姿态或起始关节无效。".into());
    }
    solve_unchecked(chain, target, seed, &mut cancel).map_err(|_| "运动计算已取消。".into())
}

pub fn inverse_generic_kinematics(
    chain: &GenericChain,
    target: &Pose,
    seed: &JointAngles,
    cancel: &AtomicBool,
) -> Result<Option<JointAngles>, String> {
    inverse_generic_kinematics_cancellable(chain, target, seed, || cancel.load(Ordering::Relaxed))
}
