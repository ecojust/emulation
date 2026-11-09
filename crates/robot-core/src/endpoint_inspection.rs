//! Independently verified endpoint IK alternatives. No trajectory or motion-program claim.
use crate::configuration::get_configuration;
use crate::kinematics::{
    forward_kinematics, inverse_kinematics_cancellable, joint_distance, joints_within_limits,
    IK_ORIENTATION_TOLERANCE_DEG, IK_POSITION_TOLERANCE_MM,
};
use crate::math::{norm, normalize_orientation, quaternion_distance_deg, sub};
use crate::model::get_robot_model;
use crate::types::{IkSolution, JointAngles, Pose};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};

pub const MAX_ENDPOINT_SOLUTIONS: usize = 256;
pub const MAX_ENDPOINT_COORDINATE_MM: f64 = 1_000_000.;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct EndpointInspectionRequest {
    pub model_id: String,
    pub pose: Pose,
    pub seed_joints: JointAngles,
}

pub fn inspect_endpoint(request: &EndpointInspectionRequest) -> Result<Vec<IkSolution>, String> {
    inspect_endpoint_cancellable(request, || false)
}

pub fn inspect_endpoint_with_cancel(
    request: &EndpointInspectionRequest,
    cancel: &AtomicBool,
) -> Result<Vec<IkSolution>, String> {
    inspect_endpoint_cancellable(request, || cancel.load(Ordering::Relaxed))
}

pub fn inspect_endpoint_cancellable<F: FnMut() -> bool>(
    request: &EndpointInspectionRequest,
    mut is_cancelled: F,
) -> Result<Vec<IkSolution>, String> {
    let cancelled = || "端点检查已取消。".to_string();
    if is_cancelled() {
        return Err(cancelled());
    }
    let model = get_robot_model(&request.model_id)
        .ok_or_else(|| "该机器人尚未适配端点逆解检查。".to_string())?;
    if request
        .pose
        .position
        .iter()
        .any(|coordinate| !coordinate.is_finite() || coordinate.abs() > MAX_ENDPOINT_COORDINATE_MM)
    {
        return Err("端点位置须为 ±1000000 mm 内的有限坐标。".into());
    }
    let pose = Pose {
        position: request.pose.position,
        orientation: normalize_orientation(&request.pose.orientation)?,
    };
    if !joints_within_limits(&model, &request.seed_joints) {
        return Err("端点参考关节无效或超出机器人关节范围。".into());
    }
    let solutions =
        inverse_kinematics_cancellable(&model, &pose, &request.seed_joints, &mut is_cancelled)?;
    if solutions.len() > MAX_ENDPOINT_SOLUTIONS {
        return Err("端点逆解返回数量超出支持范围。".into());
    }
    let mut ids = HashSet::with_capacity(solutions.len());
    let mut joints = HashSet::with_capacity(solutions.len());
    for solution in &solutions {
        if is_cancelled() {
            return Err(cancelled());
        }
        if solution.id.trim().is_empty()
            || solution.id.len() > 128
            || !ids.insert(solution.id.as_str())
            || !joints.insert(solution.joints.map(|angle| (angle * 1e6).round() as i64))
            || !joints_within_limits(&model, &solution.joints)
            || !solution.position_error_mm.is_finite()
            || solution.position_error_mm < 0.
            || !solution.orientation_error_deg.is_finite()
            || solution.orientation_error_deg < 0.
            || !solution.score.is_finite()
            || solution.score < 0.
        {
            return Err("端点逆解返回了无效或重复的候选姿态。".into());
        }
        let fk = forward_kinematics(&model, &solution.joints)?;
        let configuration = get_configuration(&model, &solution.joints, &fk);
        let position_error = norm(&sub(&fk.pose.position, &pose.position));
        let orientation_error = quaternion_distance_deg(&fk.pose.orientation, &pose.orientation);
        let score = joint_distance(&solution.joints, &request.seed_joints);
        if configuration.singular
            || configuration.configuration != Some(solution.configuration)
            || position_error > IK_POSITION_TOLERANCE_MM
            || orientation_error > IK_ORIENTATION_TOLERANCE_DEG
            || norm(&sub(&fk.pose.position, &solution.pose.position)) > 1e-6
            || quaternion_distance_deg(&fk.pose.orientation, &solution.pose.orientation) > 1e-5
            || (solution.position_error_mm - position_error).abs() > 1e-6
            || (solution.orientation_error_deg - orientation_error).abs() > 1e-5
            || (solution.score - score).abs() > 1e-6 * score.max(1.)
        {
            return Err("端点逆解未通过正运动学、配置或目标姿态复核。".into());
        }
    }
    if is_cancelled() {
        return Err(cancelled());
    }
    Ok(solutions)
}
