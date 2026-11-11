//! Bounded incremental Jog for the simulated robot. Network/session state stays
//! outside this numerical module; every accepted step is independently checked.
use crate::{collision::CollisionZone, grinding::GrindingRobot, types::{JointAngles, Pose}};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum JogMode { Linear, Joint }
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum JogSpace { World, Tool }
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct JogRequest {
    pub robot: GrindingRobot,
    pub base_pose: Pose,
    pub tool_pose: Pose,
    pub tool_radius_mm: f64,
    pub zones: Vec<CollisionZone>,
    pub start_joints: JointAngles,
    pub mode: JogMode,
    pub space: JogSpace,
    pub axes: [f64; 6],
    pub speed_scale: f64,
    pub step_seconds: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct JogSuccess { pub joints: JointAngles, pub pose: Pose }
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct JogFailure { pub code: String, pub message: String }
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum JogResult { Success(JogSuccess), Failure(JogFailure) }

pub fn jog_step(request: &JogRequest, cancel: &AtomicBool) -> JogResult {
    jog_step_cancellable(request, || cancel.load(Ordering::Acquire))
}
pub fn jog_step_cancellable<F: FnMut() -> bool>(request: &JogRequest, mut cancel: F) -> JogResult {
    if cancel() { return JogResult::Failure(JogFailure { code: "cancelled".into(), message: "示教移动已停止。".into() }); }
    match crate::grinding::pendant_step(request, &mut cancel) {
        Ok(result) => JogResult::Success(result),
        Err(error) => JogResult::Failure(JogFailure { code: error.code, message: error.message }),
    }
}
