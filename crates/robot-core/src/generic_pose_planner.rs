//! Seed-continuous Cartesian motion. Interpolation is always in world TCP coordinates.
use crate::generic_endpoint_inspection::{solution, valid_pose, GenericIkSolution};
use crate::generic_kinematics::{
    forward_unchecked, generic_joints_within_limits, solve_task_unchecked, validate_generic_chain,
    GenericChain,
};
use crate::generic_planner::{
    GenericPlanFailure, MAX_JOINT_STEP_DEG, MAX_SAMPLES, ORIENTATION_STEP_DEG, TRANSLATION_STEP_MM,
};
use crate::math::{
    matrix_to_quaternion, multiply_matrices, norm, quaternion_distance_deg, quaternion_to_matrix,
    slerp_quaternion, sub, translation, IDENTITY,
};
use crate::types::{FailureCode, JointAngles, Mat4, MotionSample, Pose, Target};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};

pub fn identity_pose() -> Pose {
    Pose {
        position: [0.; 3],
        orientation: [1., 0., 0., 0.],
    }
}
pub type GenericPoseTarget = Target;
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GenericPosePlanRequest {
    pub chain: GenericChain,
    pub start_joints: JointAngles,
    pub target: GenericPoseTarget,
    #[serde(default = "identity_pose")]
    pub base_pose: Pose,
    #[serde(default = "identity_pose")]
    pub tool_pose: Pose,
    pub speed_mm_per_second: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GenericPosePlanSuccess {
    pub path: Vec<MotionSample>,
    pub length_mm: f64,
    pub orientation_travel_deg: f64,
    pub duration_seconds: f64,
    pub warnings: Vec<String>,
    pub selected: GenericIkSolution,
    pub automatic_orientation: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum GenericPosePlanResult {
    Success(GenericPosePlanSuccess),
    Failure(GenericPlanFailure),
}
fn failure(code: FailureCode, message: &str, failed_at: Option<f64>) -> GenericPosePlanResult {
    GenericPosePlanResult::Failure(GenericPlanFailure {
        code,
        message: message.into(),
        failed_at,
    })
}
fn cancelled() -> GenericPosePlanResult {
    failure(
        FailureCode::Cancelled,
        "运动计算已取消，未返回部分路径。",
        None,
    )
}
fn matrix(pose: &Pose) -> Mat4 {
    let mut result = quaternion_to_matrix(&pose.orientation).expect("validated quaternion");
    for axis in 0..3 {
        result[axis * 4 + 3] = pose.position[axis];
    }
    result
}
fn pose(matrix: &Mat4) -> Pose {
    Pose {
        position: translation(matrix),
        orientation: matrix_to_quaternion(matrix),
    }
}
fn inverse(matrix: &Mat4) -> Mat4 {
    let mut result = IDENTITY;
    for row in 0..3 {
        for column in 0..3 {
            result[row * 4 + column] = matrix[column * 4 + row];
        }
        result[row * 4 + 3] = -(0..3)
            .map(|column| result[row * 4 + column] * matrix[column * 4 + 3])
            .sum::<f64>();
    }
    result
}
fn world_tcp(base: &Mat4, tool: &Mat4, flange: &Mat4) -> Pose {
    pose(&multiply_matrices(&multiply_matrices(base, flange), tool))
}

fn plan_target<F: FnMut() -> bool>(
    request: &GenericPosePlanRequest,
    start: Pose,
    target: Pose,
    base: &Mat4,
    tool: &Mat4,
    cancel: &mut F,
) -> GenericPosePlanResult {
    let base_inverse = inverse(base);
    let length_mm = norm(&sub(&target.position, &start.position));
    let orientation_travel_deg = quaternion_distance_deg(&start.orientation, &target.orientation);
    let segments = (length_mm / TRANSLATION_STEP_MM)
        .ceil()
        .max((orientation_travel_deg / ORIENTATION_STEP_DEG).ceil())
        .max(1.) as usize;
    if segments >= MAX_SAMPLES {
        return failure(
            FailureCode::InvalidInput,
            "运动长度或姿态变化超出 1601 个采样点的单段上限。",
            None,
        );
    }
    let mut path = vec![MotionSample {
        t: 0.,
        pose: start,
        joints: request.start_joints,
    }];
    let mut previous = request.start_joints;
    let mut duration_seconds = length_mm / request.speed_mm_per_second;
    for index in 1..=segments {
        if cancel() {
            return cancelled();
        }
        let t = index as f64 / segments as f64;
        let desired = Pose {
            position: std::array::from_fn(|axis| {
                start.position[axis] + (target.position[axis] - start.position[axis]) * t
            }),
            orientation: slerp_quaternion(&start.orientation, &target.orientation, t),
        };
        let base_tcp_target = pose(&multiply_matrices(&base_inverse, &matrix(&desired)));
        let joints = match solve_task_unchecked(
            &request.chain,
            &base_tcp_target,
            &previous,
            Some(tool),
            false,
            cancel,
        ) {
            Err(()) => return cancelled(),
            Ok(None) => {
                return failure(
                    FailureCode::PathUnreachable,
                    "TCP 直线上的数值逆解未收敛；没有返回部分运动。",
                    Some(t),
                )
            }
            Ok(Some(joints)) => joints,
        };
        if joints
            .iter()
            .zip(previous)
            .any(|(a, b)| (a - b).abs() > MAX_JOINT_STEP_DEG + 1e-8)
        {
            return failure(
                FailureCode::ConfigurationChange,
                "相邻采样的关节变化超过 15°，未通过连续性检查。",
                Some(t),
            );
        }
        let Ok(fk) = forward_unchecked(&request.chain, &joints, cancel) else {
            return cancelled();
        };
        let actual = world_tcp(base, tool, &fk.transform);
        // A flange orientation error can amplify TCP position error for an offset tool.
        if norm(&sub(&actual.position, &desired.position)) > 0.001001
            || quaternion_distance_deg(&actual.orientation, &desired.orientation) > 0.00058
        {
            return failure(
                FailureCode::PathUnreachable,
                "带工具偏置的 TCP 采样未达到位置精度要求。",
                Some(t),
            );
        }
        for axis in 0..6 {
            duration_seconds = duration_seconds.max(
                (joints[axis] - previous[axis]).abs() / request.chain.limits[axis].max_speed_deg
                    * segments as f64,
            );
        }
        path.push(MotionSample {
            t,
            pose: actual,
            joints,
        });
        previous = joints;
    }
    if cancel() {
        return cancelled();
    }
    let last = path.last().expect("complete path");
    let selected = solution(last.joints, last.pose, &target, &request.start_joints);
    GenericPosePlanResult::Success(GenericPosePlanSuccess {
        path,
        length_mm,
        orientation_travel_deg,
        duration_seconds,
        selected,
        automatic_orientation: request.target.orientation.is_none(),
        warnings: vec![
            "URDF 数值逆解与离散 TCP 直线采样；未验证完整路径碰撞、控制器配置、加速度或动力学。"
                .into(),
        ],
    })
}

pub fn plan_generic_pose_move_l_cancellable<F: FnMut() -> bool>(
    request: &GenericPosePlanRequest,
    mut cancel: F,
) -> GenericPosePlanResult {
    if cancel() {
        return cancelled();
    }
    if let Err(message) = validate_generic_chain(&request.chain) {
        return failure(FailureCode::InvalidInput, &message, None);
    }
    if !generic_joints_within_limits(&request.chain, &request.start_joints) {
        return failure(FailureCode::JointLimit, "起始关节超出 URDF 限位。", None);
    }
    let specified = Pose {
        position: request.target.position,
        orientation: request.target.orientation.unwrap_or([1., 0., 0., 0.]),
    };
    if !valid_pose(&specified)
        || !valid_pose(&request.base_pose)
        || !valid_pose(&request.tool_pose)
        || !request.speed_mm_per_second.is_finite()
        || !(1. ..=2000.).contains(&request.speed_mm_per_second)
    {
        return failure(
            FailureCode::InvalidInput,
            "TCP 目标、坐标变换或运动速度无效。",
            None,
        );
    }
    let base = matrix(&request.base_pose);
    let tool = matrix(&request.tool_pose);
    let Ok(fk) = forward_unchecked(&request.chain, &request.start_joints, &mut cancel) else {
        return cancelled();
    };
    let start = world_tcp(&base, &tool, &fk.transform);
    let target = Pose {
        position: request.target.position,
        orientation: request.target.orientation.unwrap_or(start.orientation),
    };
    let first = plan_target(request, start, target, &base, &tool, &mut cancel);
    if request.target.orientation.is_some() || matches!(first, GenericPosePlanResult::Success(_)) {
        return first;
    }
    if cancel() {
        return cancelled();
    }
    // Position mode first preserves direction, then tries one bounded seed-continuous relaxed direction.
    let base_target = pose(&multiply_matrices(&inverse(&base), &matrix(&target)));
    let joints = match solve_task_unchecked(
        &request.chain,
        &base_target,
        &request.start_joints,
        Some(&tool),
        true,
        &mut cancel,
    ) {
        Err(()) => return cancelled(),
        Ok(None) => return first,
        Ok(Some(joints)) => joints,
    };
    let Ok(end_fk) = forward_unchecked(&request.chain, &joints, &mut cancel) else {
        return cancelled();
    };
    let relaxed = world_tcp(&base, &tool, &end_fk.transform);
    let target = Pose {
        position: request.target.position,
        orientation: relaxed.orientation,
    };
    plan_target(request, start, target, &base, &tool, &mut cancel)
}
pub fn plan_generic_pose_move_l(
    request: &GenericPosePlanRequest,
    cancel: &AtomicBool,
) -> GenericPosePlanResult {
    plan_generic_pose_move_l_cancellable(request, || cancel.load(Ordering::Relaxed))
}
