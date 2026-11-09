//! ABB confdata: 3HAC050917-001 Rev AC, pp.1649–1655, serial six-axis rules.
use crate::math::{cross, dot, norm, sub};
use crate::types::{ConfigurationResult, FkResult, JointAngles, RobotModel};

pub const SHOULDER_TOLERANCE_MM: f64 = 0.1;
pub const ELBOW_TOLERANCE_MM: f64 = 0.1;
pub const WRIST_TOLERANCE_DEG: f64 = 0.1;

pub fn joint_quadrant(angle_deg: f64) -> Option<i32> {
    if !angle_deg.is_finite() {
        return None;
    }
    let boundary = (angle_deg / 90.).round() * 90.;
    let value = if (angle_deg - boundary).abs() <= 1e-10 {
        boundary
    } else {
        angle_deg
    };
    let quadrant = (value / 90.).floor();
    if quadrant < i32::MIN as f64 || quadrant > i32::MAX as f64 {
        return None;
    }
    Some(quadrant as i32)
}

fn undefined(reason: &str, singular: bool) -> ConfigurationResult {
    ConfigurationResult {
        configuration: None,
        singular,
        reason: Some(reason.into()),
    }
}

pub fn get_configuration(
    model: &RobotModel,
    joints: &JointAngles,
    fk: &FkResult,
) -> ConfigurationResult {
    let points = [
        &fk.shoulder,
        &fk.elbow,
        &fk.wrist_center,
        &fk.joint_origins[0],
        &fk.joint_axes[1],
    ];
    if joints.iter().any(|v| !v.is_finite())
        || points.iter().any(|p| p.iter().any(|v| !v.is_finite()))
    {
        return undefined("关节或正运动学数据无效", false);
    }
    let angle = (joints[0] * model.dh[0].theta_sign + model.dh[0].theta_offset_deg).to_radians();
    let forward = [angle.cos(), angle.sin(), 0.];
    let shoulder_side = dot(&sub(&fk.wrist_center, &fk.joint_origins[0]), &forward);
    if shoulder_side.abs() <= SHOULDER_TOLERANCE_MM {
        return undefined("腕心接近轴 1，肩部配置不确定", true);
    }
    let front = cross(&fk.joint_axes[1], &sub(&fk.elbow, &fk.shoulder));
    let scale = norm(&front);
    if scale < 1e-10 {
        return undefined("下臂方向无效，无法确定肘部配置", true);
    }
    let elbow_side = dot(&sub(&fk.wrist_center, &fk.elbow), &front) / scale;
    if elbow_side.abs() <= ELBOW_TOLERANCE_MM {
        return undefined("腕心接近下臂延长线，肘部配置不确定", true);
    }
    if joints[4].to_radians().sin().abs() <= WRIST_TOLERANCE_DEG.to_radians().sin() {
        return undefined("轴 5 接近腕部奇异位形，无法稳定确定 robconf", true);
    }
    let cfx = if shoulder_side < 0. { 4 } else { 0 }
        + if elbow_side < 0. { 2 } else { 0 }
        + if joints[4] < 0. { 1 } else { 0 };
    let (Some(cf1), Some(cf4), Some(cf6)) = (
        joint_quadrant(joints[0]),
        joint_quadrant(joints[3]),
        joint_quadrant(joints[5]),
    ) else {
        return undefined("关节转数超出配置表示范围", false);
    };
    ConfigurationResult {
        configuration: Some([cf1, cf4, cf6, cfx]),
        singular: false,
        reason: None,
    }
}
