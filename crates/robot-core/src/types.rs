use serde::{Deserialize, Serialize};

pub type Vec3 = [f64; 3];
pub type Quaternion = [f64; 4];
pub type JointAngles = [f64; 6];
/// Row-major transform; millimetre translation at 3, 7 and 11.
pub type Mat4 = [f64; 16];
/// ABB confdata. Other controllers have their own posture payload in program adapters.
pub type RobConf = [i32; 4];

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct Pose {
    pub position: Vec3,
    pub orientation: Quaternion,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct DhRow {
    pub a_mm: f64,
    pub d_mm: f64,
    pub alpha_deg: f64,
    pub theta_offset_deg: f64,
    pub theta_sign: f64,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct JointLimit {
    pub min_deg: f64,
    pub max_deg: f64,
    pub max_speed_deg: f64,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct Geometry {
    pub shoulder_height_mm: f64,
    pub upper_arm_mm: f64,
    pub elbow_offset_mm: f64,
    pub forearm_mm: f64,
    pub flange_mm: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct RobotModel {
    pub id: String,
    pub name: String,
    pub reach_mm: f64,
    pub payload_kg: f64,
    pub dh: [DhRow; 6],
    pub limits: [JointLimit; 6],
    pub geometry: Geometry,
    pub home_joints: JointAngles,
    pub ready_joints: JointAngles,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct FkResult {
    pub pose: Pose,
    pub frames: [Mat4; 7],
    pub joint_origins: [Vec3; 6],
    pub joint_axes: [Vec3; 6],
    pub shoulder: Vec3,
    pub elbow: Vec3,
    pub wrist_center: Vec3,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct ConfigurationResult {
    pub configuration: Option<RobConf>,
    pub singular: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct IkSolution {
    pub id: String,
    pub joints: JointAngles,
    pub pose: Pose,
    pub configuration: RobConf,
    pub position_error_mm: f64,
    pub orientation_error_deg: f64,
    pub score: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct MotionSample {
    pub t: f64,
    pub pose: Pose,
    pub joints: JointAngles,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct Target {
    pub position: Vec3,
    pub orientation: Option<Quaternion>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct PlanRequest {
    pub model_id: String,
    pub start_joints: JointAngles,
    pub target: Target,
    pub speed_mm_per_second: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct PlanSuccess {
    pub selected: IkSolution,
    pub solutions: Vec<IkSolution>,
    pub path: Vec<MotionSample>,
    pub length_mm: f64,
    pub orientation_travel_deg: f64,
    pub duration_seconds: f64,
    pub warnings: Vec<String>,
    pub automatic_orientation: bool,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FailureCode {
    InvalidInput,
    JointLimit,
    Unreachable,
    SingularStart,
    SingularPath,
    ConfigurationChange,
    PathUnreachable,
    Cancelled,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct PlanFailure {
    pub code: FailureCode,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failed_at: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub solutions: Option<Vec<IkSolution>>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum PlanResult {
    Success(PlanSuccess),
    Failure(PlanFailure),
}
