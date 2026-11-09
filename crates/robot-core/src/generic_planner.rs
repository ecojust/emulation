//! Sampled Cartesian planning for a validated six-axis URDF chain.
use crate::generic_kinematics::{
    forward_unchecked, generic_joints_within_limits, solve_unchecked, validate_generic_chain,
    GenericChain,
};
use crate::math::{norm, quaternion_distance_deg, slerp_quaternion, sub};
use crate::types::{FailureCode, JointAngles, MotionSample, Pose};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};

pub const TRANSLATION_STEP_MM: f64 = 5.;
pub const ORIENTATION_STEP_DEG: f64 = 2.;
pub const MAX_SAMPLES: usize = 1601;
pub const MAX_JOINT_STEP_DEG: f64 = 15.;
pub const END_JOINT_TOLERANCE_DEG: f64 = 0.001;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GenericPlanRequest {
    pub chain: GenericChain,
    pub start_joints: JointAngles,
    pub target_joints: JointAngles,
    pub speed_mm_per_second: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GenericPlanSuccess {
    pub path: Vec<MotionSample>,
    pub length_mm: f64,
    pub orientation_travel_deg: f64,
    pub duration_seconds: f64,
    pub warnings: Vec<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GenericPlanFailure {
    pub code: FailureCode,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failed_at: Option<f64>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum GenericPlanResult {
    Success(GenericPlanSuccess),
    Failure(GenericPlanFailure),
}

fn failure(code: FailureCode, message: &str, failed_at: Option<f64>) -> GenericPlanResult {
    GenericPlanResult::Failure(GenericPlanFailure {
        code,
        message: message.into(),
        failed_at,
    })
}
fn cancelled() -> GenericPlanResult {
    failure(
        FailureCode::Cancelled,
        "运动计算已取消，未返回部分路径。",
        None,
    )
}

pub fn plan_generic_move_l_cancellable<F: FnMut() -> bool>(
    request: &GenericPlanRequest,
    mut cancel: F,
) -> GenericPlanResult {
    if cancel() {
        return cancelled();
    }
    if let Err(message) = validate_generic_chain(&request.chain) {
        return failure(FailureCode::InvalidInput, &message, None);
    }
    if !request.speed_mm_per_second.is_finite()
        || !(1. ..=2000.).contains(&request.speed_mm_per_second)
    {
        return failure(
            FailureCode::InvalidInput,
            "运动速度须在 1 至 2000 mm/s 之间。",
            None,
        );
    }
    let chain = &request.chain;
    if !generic_joints_within_limits(chain, &request.start_joints)
        || !generic_joints_within_limits(chain, &request.target_joints)
    {
        return failure(
            FailureCode::JointLimit,
            "起点或记录的目标关节超出 URDF 限位。",
            None,
        );
    }
    let Ok(start) = forward_unchecked(chain, &request.start_joints, &mut cancel) else {
        return cancelled();
    };
    let Ok(target) = forward_unchecked(chain, &request.target_joints, &mut cancel) else {
        return cancelled();
    };
    let length_mm = norm(&sub(&target.pose.position, &start.pose.position));
    let orientation_travel_deg =
        quaternion_distance_deg(&start.pose.orientation, &target.pose.orientation);
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
    let mut path = Vec::with_capacity(segments + 1);
    path.push(MotionSample {
        t: 0.,
        pose: start.pose,
        joints: request.start_joints,
    });
    let mut previous = request.start_joints;
    let mut duration_seconds = length_mm / request.speed_mm_per_second;
    for index in 1..=segments {
        if cancel() {
            return cancelled();
        }
        let t = index as f64 / segments as f64;
        let pose = Pose {
            position: std::array::from_fn(|axis| {
                start.pose.position[axis]
                    + (target.pose.position[axis] - start.pose.position[axis]) * t
            }),
            orientation: slerp_quaternion(&start.pose.orientation, &target.pose.orientation, t),
        };
        let mut joints = match solve_unchecked(chain, &pose, &previous, &mut cancel) {
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
        if index == segments {
            if joints
                .iter()
                .zip(request.target_joints)
                .any(|(actual, expected)| (actual - expected).abs() > END_JOINT_TOLERANCE_DEG)
            {
                return failure(
                    FailureCode::ConfigurationChange,
                    "直线终点无法连续到达记录的关节姿态，不能替换为另一逆解分支。",
                    Some(t),
                );
            }
            joints = request.target_joints;
        }
        if joints
            .iter()
            .zip(previous)
            .any(|(current, last)| (current - last).abs() > MAX_JOINT_STEP_DEG + 1e-8)
        {
            return failure(
                FailureCode::ConfigurationChange,
                "相邻采样的关节变化超过 15°，未通过连续性检查。",
                Some(t),
            );
        }
        let Ok(fk) = forward_unchecked(chain, &joints, &mut cancel) else {
            return cancelled();
        };
        for axis in 0..6 {
            duration_seconds = duration_seconds.max(
                (joints[axis] - previous[axis]).abs() / chain.limits[axis].max_speed_deg
                    * segments as f64,
            );
        }
        path.push(MotionSample {
            t,
            pose: fk.pose,
            joints,
        });
        previous = joints;
    }
    if cancel() {
        return cancelled();
    }
    GenericPlanResult::Success(GenericPlanSuccess {
        path,
        length_mm,
        orientation_travel_deg,
        duration_seconds,
        warnings: vec![
            "URDF 数值逆解与离散 TCP 直线采样；未验证完整路径碰撞、控制器配置、加速度或动力学。"
                .into(),
        ],
    })
}

pub fn plan_generic_move_l(request: &GenericPlanRequest, cancel: &AtomicBool) -> GenericPlanResult {
    plan_generic_move_l_cancellable(request, || cancel.load(Ordering::Relaxed))
}
