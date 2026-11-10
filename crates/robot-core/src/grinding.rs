//! Bounded TCP path construction with conservative swept-envelope certification.
//!
//! The certificate concerns the returned piecewise-linear joint trajectory and
//! the declared robot/tool envelopes against world-axis-aligned exclusion boxes.
//! It is not a self-collision, controller, calibration or dynamics certificate.
use crate::collision::{
    link_bounding_boxes, obb_intersects_danger_zone, validate_danger_zone, CollisionZone,
    OrientedBox,
};
use crate::configuration::get_configuration;
use crate::generic_kinematics::{
    forward_unchecked, generic_joints_within_limits, solve_unchecked, validate_generic_chain,
    GenericChain,
};
use crate::kinematics::{forward_kinematics, inverse_kinematics_cancellable, joints_within_limits};
use crate::math::{
    matrix_to_quaternion, multiply_matrices, norm, normalize_orientation, quaternion_distance_deg,
    quaternion_to_matrix, slerp_quaternion, sub, translation, IDENTITY,
};
use crate::model::get_robot_model;
use crate::types::{JointAngles, Mat4, MotionSample, Pose, Quaternion, RobConf, Vec3};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};

pub const MAX_TARGETS: usize = 64;
pub const MAX_RETURN_SAMPLES: usize = 8192;
pub const MAX_IK_CALLS: usize = 32768;
pub const MAX_INTERVAL_CHECKS: usize = 100000;
pub const MAX_CERT_DEPTH: usize = 12;
pub const TRANSLATION_STEP_MM: f64 = 2.;
pub const ORIENTATION_STEP_DEG: f64 = 1.;
pub const MAX_JOINT_STEP_DEG: f64 = 15.;
pub const FIXED_ORIENTATION_TOLERANCE_DEG: f64 = 0.1;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GrindingBody {
    /// Number of completed root-to-flange chain steps (0 is the root frame).
    pub step_index: usize,
    pub center_mm: Vec3,
    pub half_size_mm: Vec3,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum GrindingRobot {
    Abb {
        #[serde(rename = "modelId")]
        model_id: String,
    },
    Generic {
        chain: GenericChain,
        bodies: Vec<GrindingBody>,
    },
}
fn default_detours() -> bool {
    true
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GrindingRequest {
    pub robot: GrindingRobot,
    pub base_pose: Pose,
    pub tool_pose: Pose,
    pub tool_radius_mm: f64,
    pub start_joints: JointAngles,
    pub targets: Vec<Pose>,
    #[serde(default)]
    pub orientation: Option<Quaternion>,
    pub zones: Vec<CollisionZone>,
    pub clearance_mm: f64,
    pub speed_mm_per_second: f64,
    #[serde(default = "default_detours")]
    pub allow_detours: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GrindingWaypoint {
    pub pose: Pose,
    pub joints: JointAngles,
    pub configuration: Option<RobConf>,
    pub target_index: Option<usize>,
    pub kind: String,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GrindingSuccess {
    pub path: Vec<MotionSample>,
    pub waypoints: Vec<GrindingWaypoint>,
    pub duration_seconds: f64,
    pub length_mm: f64,
    pub orientation: Option<Quaternion>,
    pub checked_intervals: usize,
    pub warnings: Vec<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GrindingFailure {
    pub code: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failed_target: Option<usize>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum GrindingResult {
    Success(GrindingSuccess),
    Failure(GrindingFailure),
}
fn fail(code: &str, message: &str) -> GrindingFailure {
    GrindingFailure {
        code: code.into(),
        message: message.into(),
        failed_target: None,
    }
}
type Checked<T> = Result<T, GrindingFailure>;
fn cancelled() -> GrindingFailure {
    fail("cancelled", "路径规划已取消，未返回部分结果。")
}

fn pose_matrix(pose: &Pose) -> Checked<Mat4> {
    if pose
        .position
        .iter()
        .any(|x| !x.is_finite() || x.abs() > 1e6)
    {
        return Err(fail(
            "invalid_input",
            "位置必须是 ±1000000 mm 范围内的有限数值。",
        ));
    }
    let q = normalize_orientation(&pose.orientation).map_err(|m| fail("invalid_input", &m))?;
    let mut m = quaternion_to_matrix(&q).map_err(|m| fail("invalid_input", &m))?;
    for i in 0..3 {
        m[i * 4 + 3] = pose.position[i];
    }
    Ok(m)
}
fn matrix_pose(m: &Mat4) -> Pose {
    Pose {
        position: translation(m),
        orientation: matrix_to_quaternion(m),
    }
}
fn inverse(m: &Mat4) -> Mat4 {
    let mut out = IDENTITY;
    for r in 0..3 {
        for c in 0..3 {
            out[r * 4 + c] = m[c * 4 + r];
        }
        out[r * 4 + 3] = -(0..3).map(|k| out[r * 4 + k] * m[k * 4 + 3]).sum::<f64>();
    }
    out
}
fn point(m: &Mat4, p: &Vec3) -> Vec3 {
    std::array::from_fn(|r| m[r * 4 + 3] + (0..3).map(|k| m[r * 4 + k] * p[k]).sum::<f64>())
}
fn rotation(axis: &Vec3, degrees: f64) -> Mat4 {
    let [x, y, z] = *axis;
    let (s, c) = degrees.to_radians().sin_cos();
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
fn transformed_box(m: &Mat4, center: Vec3, half: Vec3) -> Checked<OrientedBox> {
    OrientedBox::new(
        point(m, &center),
        std::array::from_fn(|i| [m[i], m[4 + i], m[8 + i]]),
        half,
    )
    .map_err(|m| fail("invalid_input", &m))
}

struct Context<'a> {
    request: &'a GrindingRequest,
    base: Mat4,
    base_inverse: Mat4,
    tool: Mat4,
    tool_inverse: Mat4,
    radius_bound: f64,
    speed_limits: [f64; 6],
    ik_calls: usize,
    checked_intervals: usize,
}
impl<'a> Context<'a> {
    fn new(request: &'a GrindingRequest) -> Checked<Self> {
        if !(1..=MAX_TARGETS).contains(&request.targets.len())
            || request.zones.len() > 64
            || !request.clearance_mm.is_finite()
            || !(0. ..=1000.).contains(&request.clearance_mm)
            || !request.tool_radius_mm.is_finite()
            || !(0. ..=1000.).contains(&request.tool_radius_mm)
            || !request.speed_mm_per_second.is_finite()
            || !(1. ..=2000.).contains(&request.speed_mm_per_second)
        {
            return Err(fail(
                "invalid_input",
                "目标数量、禁入区数量、工具半径、安全间距或速度超出范围。",
            ));
        }
        let base = pose_matrix(&request.base_pose)?;
        let tool = pose_matrix(&request.tool_pose)?;
        for target in &request.targets {
            pose_matrix(target)?;
        }
        if let Some(q) = request.orientation {
            normalize_orientation(&q).map_err(|m| fail("invalid_input", &m))?;
        }
        let mut ids = std::collections::HashSet::new();
        for zone in &request.zones {
            if zone.id.trim().is_empty() || zone.id.len() > 128 || !ids.insert(&zone.id) {
                return Err(fail("invalid_input", "禁入区标识无效或重复。"));
            }
            validate_danger_zone(&zone.danger_zone()).map_err(|m| fail("invalid_input", &m))?;
        }
        let (reach, limits) = match &request.robot {
            GrindingRobot::Abb { model_id } => {
                let model = get_robot_model(model_id)
                    .ok_or_else(|| fail("invalid_input", "不支持的机器人型号。"))?;
                if !joints_within_limits(&model, &request.start_joints) {
                    return Err(fail("joint_limit", "起点关节超出限位。"));
                }
                let boxes = link_bounding_boxes(model_id, &[0.; 6])
                    .map_err(|m| fail("invalid_input", &m))?;
                let reach = 2.
                    * model
                        .dh
                        .iter()
                        .map(|row| row.a_mm.hypot(row.d_mm))
                        .sum::<f64>()
                    + boxes
                        .iter()
                        .map(|b| norm(&b.center_mm()) + norm(&b.half_size_mm()))
                        .fold(0., f64::max);
                (reach, model.limits.map(|l| l.max_speed_deg))
            }
            GrindingRobot::Generic { chain, bodies } => {
                validate_generic_chain(chain).map_err(|m| fail("invalid_input", &m))?;
                if !generic_joints_within_limits(chain, &request.start_joints) {
                    return Err(fail("joint_limit", "起点关节超出限位。"));
                }
                if bodies.is_empty()
                    || bodies.len() > 128
                    || bodies.iter().any(|b| {
                        b.step_index > chain.steps.len()
                            || b.center_mm
                                .iter()
                                .any(|x| !x.is_finite() || x.abs() > 100000.)
                            || b.half_size_mm
                                .iter()
                                .any(|x| !x.is_finite() || *x <= 0. || *x > 100000.)
                    })
                {
                    return Err(fail(
                        "invalid_input",
                        "需要完整有效的已加载机器人几何包络。",
                    ));
                }
                let mut stages = [false; 7];
                for body in bodies {
                    let stage = chain.steps[..body.step_index]
                        .iter()
                        .filter(|s| s.joint_index.is_some())
                        .count();
                    stages[stage] = true;
                }
                if stages.iter().any(|present| !*present) {
                    return Err(fail(
                        "invalid_input",
                        "机器人包络必须覆盖基座和全部六个运动轴。",
                    ));
                }
                let reach = chain
                    .steps
                    .iter()
                    .map(|s| norm(&translation(&s.origin)))
                    .sum::<f64>()
                    + bodies
                        .iter()
                        .map(|b| norm(&b.center_mm) + norm(&b.half_size_mm))
                        .fold(0., f64::max);
                (reach, chain.limits.map(|l| l.max_speed_deg))
            }
        };
        Ok(Self {
            request,
            base,
            base_inverse: inverse(&base),
            tool,
            tool_inverse: inverse(&tool),
            radius_bound: reach
                + norm(&request.tool_pose.position)
                + 3_f64.sqrt() * request.tool_radius_mm.max(1e-6),
            speed_limits: limits,
            ik_calls: 0,
            checked_intervals: 0,
        })
    }
    fn tcp<F: FnMut() -> bool>(&self, q: &JointAngles, cancel: &mut F) -> Checked<Pose> {
        if cancel() {
            return Err(cancelled());
        }
        let flange = match &self.request.robot {
            GrindingRobot::Abb { model_id } => {
                let model = get_robot_model(model_id).unwrap();
                forward_kinematics(&model, q)
                    .map_err(|m| fail("invalid_input", &m))?
                    .frames[6]
            }
            GrindingRobot::Generic { chain, .. } => {
                forward_unchecked(chain, q, cancel)
                    .map_err(|_| cancelled())?
                    .transform
            }
        };
        Ok(matrix_pose(&multiply_matrices(
            &multiply_matrices(&self.base, &flange),
            &self.tool,
        )))
    }
    fn config(&self, q: &JointAngles) -> Checked<Option<RobConf>> {
        match &self.request.robot {
            GrindingRobot::Abb { model_id } => {
                let model = get_robot_model(model_id).unwrap();
                let fk = forward_kinematics(&model, q).map_err(|m| fail("invalid_input", &m))?;
                let result = get_configuration(&model, q, &fk);
                if result.singular {
                    Err(fail("singular_path", "路径进入奇异区域，未生成连续路径。"))
                } else {
                    Ok(result.configuration)
                }
            }
            _ => Ok(None),
        }
    }
    fn ik<F: FnMut() -> bool>(
        &mut self,
        pose: &Pose,
        seed: &JointAngles,
        cancel: &mut F,
    ) -> Checked<JointAngles> {
        if cancel() {
            return Err(cancelled());
        }
        self.ik_calls += 1;
        if self.ik_calls > MAX_IK_CALLS {
            return Err(fail(
                "resource_limit",
                "路径搜索达到逆解预算，未返回部分路径。",
            ));
        }
        let target = matrix_pose(&multiply_matrices(
            &multiply_matrices(&self.base_inverse, &pose_matrix(pose)?),
            &self.tool_inverse,
        ));
        let joints = match &self.request.robot {
            GrindingRobot::Abb { model_id } => {
                let model = get_robot_model(model_id).unwrap();
                let conf = self
                    .config(seed)?
                    .ok_or_else(|| fail("singular_path", "起点没有有效机器人配置。"))?;
                inverse_kinematics_cancellable(&model, &target, seed, &mut *cancel)
                    .map_err(|_| cancelled())?
                    .into_iter()
                    .find(|s| s.configuration[3] == conf[3])
                    .map(|s| s.joints)
            }
            GrindingRobot::Generic { chain, .. } => {
                solve_unchecked(chain, &target, seed, cancel).map_err(|_| cancelled())?
            }
        }
        .ok_or_else(|| {
            fail(
                "unreachable",
                "目标方向或路径上的逆解不可达，不能跳换关节分支。",
            )
        })?;
        if joints
            .iter()
            .zip(seed)
            .any(|(a, b)| (a - b).abs() > MAX_JOINT_STEP_DEG + 1e-8)
        {
            return Err(fail(
                "configuration_change",
                "相邻采样关节变化超过15°，未生成跳跃姿态。",
            ));
        }
        Ok(joints)
    }
    fn boxes<F: FnMut() -> bool>(
        &self,
        q: &JointAngles,
        cancel: &mut F,
    ) -> Checked<Vec<OrientedBox>> {
        if cancel() {
            return Err(cancelled());
        }
        let (mut boxes, flange) = match &self.request.robot {
            GrindingRobot::Abb { model_id } => {
                let source =
                    link_bounding_boxes(model_id, q).map_err(|m| fail("invalid_input", &m))?;
                let boxes = source
                    .iter()
                    .map(|b| {
                        let axes = b.axes();
                        let m = [
                            axes[0][0],
                            axes[1][0],
                            axes[2][0],
                            b.center_mm()[0],
                            axes[0][1],
                            axes[1][1],
                            axes[2][1],
                            b.center_mm()[1],
                            axes[0][2],
                            axes[1][2],
                            axes[2][2],
                            b.center_mm()[2],
                            0.,
                            0.,
                            0.,
                            1.,
                        ];
                        transformed_box(
                            &multiply_matrices(&self.base, &m),
                            [0.; 3],
                            b.half_size_mm(),
                        )
                    })
                    .collect::<Checked<Vec<_>>>()?;
                (
                    boxes,
                    forward_kinematics(&get_robot_model(model_id).unwrap(), q)
                        .map_err(|m| fail("invalid_input", &m))?
                        .frames[6],
                )
            }
            GrindingRobot::Generic { chain, bodies } => {
                let mut frames = vec![IDENTITY];
                let mut m = IDENTITY;
                for step in &chain.steps {
                    if cancel() {
                        return Err(cancelled());
                    }
                    m = multiply_matrices(&m, &step.origin);
                    if let Some(j) = step.joint_index {
                        m = multiply_matrices(&m, &rotation(&step.axis, q[j]));
                    }
                    frames.push(m);
                }
                let boxes = bodies
                    .iter()
                    .map(|b| {
                        transformed_box(
                            &multiply_matrices(&self.base, &frames[b.step_index]),
                            b.center_mm,
                            b.half_size_mm,
                        )
                    })
                    .collect::<Checked<Vec<_>>>()?;
                (boxes, m)
            }
        };
        let offset = self.request.tool_pose.position;
        let radius = self.request.tool_radius_mm.max(1e-6);
        boxes.push(transformed_box(
            &multiply_matrices(&self.base, &flange),
            offset.map(|x| x / 2.),
            offset.map(|x| x.abs() / 2. + radius),
        )?);
        Ok(boxes)
    }
    fn clear(&self, boxes: &[OrientedBox], padding: f64) -> Checked<bool> {
        for b in boxes {
            let expanded = OrientedBox::new(
                b.center_mm(),
                b.axes(),
                b.half_size_mm().map(|v| v + padding),
            )
            .map_err(|m| fail("invalid_input", &m))?;
            for z in self.request.zones.iter().filter(|z| z.enabled) {
                if obb_intersects_danger_zone(&expanded, &z.danger_zone())
                    .map_err(|m| fail("invalid_input", &m))?
                {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }
    /// L*sum(|dq|)/2 bounds every point's displacement from the midpoint of a
    /// joint-linear interval. Inflating each OBB by this radius contains its
    /// entire swept body, including rotation; endpoint-only tests cannot do so.
    fn certify<F: FnMut() -> bool>(
        &mut self,
        a: &JointAngles,
        b: &JointAngles,
        depth: usize,
        fixed: Option<&Quaternion>,
        cancel: &mut F,
    ) -> Checked<()> {
        if cancel() {
            return Err(cancelled());
        }
        self.checked_intervals += 1;
        if self.checked_intervals > MAX_INTERVAL_CHECKS {
            return Err(fail(
                "resource_limit",
                "连续包络检查达到预算，不能确认路径安全。",
            ));
        }
        let mid = std::array::from_fn(|i| (a[i] + b[i]) / 2.);
        let boxes = self.boxes(&mid, cancel)?;
        let sweep = self.radius_bound
            * a.iter()
                .zip(b)
                .map(|(x, y)| (x - y).to_radians().abs())
                .sum::<f64>()
            / 2.;
        let orientation_clear = if let Some(expected) = fixed {
            let error = quaternion_distance_deg(&self.tcp(&mid, cancel)?.orientation, expected);
            if error > FIXED_ORIENTATION_TOLERANCE_DEG + 1e-8 {
                return Err(fail(
                    "orientation_tolerance",
                    "关节插值的工具方向偏差超过0.1°，未生成路径。",
                ));
            }
            error + a.iter().zip(b).map(|(x, y)| (x - y).abs()).sum::<f64>() / 2.
                <= FIXED_ORIENTATION_TOLERANCE_DEG
        } else {
            true
        };
        if orientation_clear && self.clear(&boxes, self.request.clearance_mm + sweep + 1e-7)? {
            return Ok(());
        }
        if !self.clear(&boxes, self.request.clearance_mm)? {
            return Err(fail("collision", "机器人或工具的保守包络与禁入区相交。"));
        }
        if depth >= MAX_CERT_DEPTH {
            return Err(fail(
                "uncertain",
                "连续运动包络或固定方向无法在检查精度内完成验证，未生成路径。",
            ));
        }
        self.certify(a, &mid, depth + 1, fixed, cancel)?;
        self.certify(&mid, b, depth + 1, fixed, cancel)
    }
    fn leg<F: FnMut() -> bool>(
        &mut self,
        start: JointAngles,
        target: Pose,
        cancel: &mut F,
    ) -> Checked<Leg> {
        let origin = self.tcp(&start, cancel)?;
        let length = norm(&sub(&target.position, &origin.position));
        let angle = quaternion_distance_deg(&origin.orientation, &target.orientation);
        let count = (length / TRANSLATION_STEP_MM)
            .ceil()
            .max((angle / ORIENTATION_STEP_DEG).ceil())
            .max(1.) as usize;
        if count >= MAX_RETURN_SAMPLES {
            return Err(fail("resource_limit", "单段路径超过采样上限。"));
        }
        let mut samples = Vec::with_capacity(count);
        let mut previous = start;
        let mut duration = (length / self.request.speed_mm_per_second).max(angle / 500.);
        for index in 1..=count {
            if cancel() {
                return Err(cancelled());
            }
            let t = index as f64 / count as f64;
            let wanted = Pose {
                position: std::array::from_fn(|i| {
                    origin.position[i] + t * (target.position[i] - origin.position[i])
                }),
                orientation: slerp_quaternion(&origin.orientation, &target.orientation, t),
            };
            let q = self.ik(&wanted, &previous, cancel)?;
            self.certify(
                &previous,
                &q,
                0,
                if angle <= 0.01 {
                    Some(&target.orientation)
                } else {
                    None
                },
                cancel,
            )?;
            for j in 0..6 {
                duration =
                    duration.max((q[j] - previous[j]).abs() / self.speed_limits[j] * count as f64);
            }
            samples.push(MotionSample {
                t,
                pose: self.tcp(&q, cancel)?,
                joints: q,
            });
            previous = q;
        }
        if !duration.is_finite() {
            return Err(fail(
                "invalid_input",
                "关节速度产生非有限运动时间，未生成路径。",
            ));
        }
        Ok(Leg {
            samples,
            duration: duration.max(0.001),
            length,
        })
    }
}
struct Leg {
    samples: Vec<MotionSample>,
    duration: f64,
    length: f64,
}
fn retryable(error: &GrindingFailure) -> bool {
    matches!(
        error.code.as_str(),
        "collision" | "uncertain" | "unreachable" | "configuration_change" | "singular_path"
    )
}

pub fn plan_grinding_cancellable<F: FnMut() -> bool>(
    request: &GrindingRequest,
    mut cancel: F,
) -> GrindingResult {
    match run(request, &mut cancel) {
        Ok(value) => GrindingResult::Success(value),
        Err(error) => GrindingResult::Failure(error),
    }
}
pub fn plan_grinding(request: &GrindingRequest, cancel: &AtomicBool) -> GrindingResult {
    plan_grinding_cancellable(request, || cancel.load(Ordering::Relaxed))
}
fn run<F: FnMut() -> bool>(request: &GrindingRequest, cancel: &mut F) -> Checked<GrindingSuccess> {
    if cancel() {
        return Err(cancelled());
    }
    let mut context = Context::new(request)?;
    let orientation = request
        .orientation
        .as_ref()
        .map(normalize_orientation)
        .transpose()
        .map_err(|m| fail("invalid_input", &m))?;
    context.config(&request.start_joints)?;
    context.certify(
        &request.start_joints,
        &request.start_joints,
        0,
        None,
        cancel,
    )?;
    let mut current = request.start_joints;
    let mut path = vec![MotionSample {
        t: 0.,
        pose: context.tcp(&current, cancel)?,
        joints: current,
    }];
    let mut waypoints = Vec::new();
    let mut duration = 0.;
    let mut length = 0.;
    let mut used_detour = false;
    for (target_index, input) in request.targets.iter().enumerate() {
        let target = Pose {
            position: input.position,
            orientation: orientation.unwrap_or(
                normalize_orientation(&input.orientation).map_err(|m| fail("invalid_input", &m))?,
            ),
        };
        let direct = context.leg(current, target, cancel);
        let mut route = match direct {
            Ok(leg) => vec![leg],
            Err(mut error) => {
                error.failed_target = Some(target_index);
                if !request.allow_detours
                    || !retryable(&error)
                    || !request.zones.iter().any(|z| z.enabled)
                {
                    return Err(error);
                }
                let start = context.tcp(&current, cancel)?;
                let mut found = None;
                'search: for extra in [25., 75.] {
                    for axis in 0..3 {
                        for sign in [-1., 1.] {
                            if cancel() {
                                return Err(cancelled());
                            }
                            let edge = request
                                .zones
                                .iter()
                                .filter(|z| z.enabled)
                                .map(|z| z.center_mm[axis] + sign * z.size_mm[axis] / 2.)
                                .fold(
                                    if sign < 0. {
                                        f64::INFINITY
                                    } else {
                                        f64::NEG_INFINITY
                                    },
                                    |a, b| if sign < 0. { a.min(b) } else { a.max(b) },
                                );
                            let plane = edge
                                + sign * (request.tool_radius_mm + request.clearance_mm + extra);
                            let mut p1 = start;
                            p1.position[axis] = plane;
                            p1.orientation = target.orientation;
                            let mut p2 = target;
                            p2.position[axis] = plane;
                            if [p1, p2]
                                .iter()
                                .any(|p| p.position.iter().any(|x| !x.is_finite() || x.abs() > 1e6))
                            {
                                continue;
                            }
                            let candidate_poses = [start, p1, p2, target];
                            let estimate = candidate_poses
                                .windows(2)
                                .map(|p| {
                                    (norm(&sub(&p[1].position, &p[0].position))
                                        / TRANSLATION_STEP_MM)
                                        .ceil()
                                        .max(
                                            (quaternion_distance_deg(
                                                &p[0].orientation,
                                                &p[1].orientation,
                                            ) / ORIENTATION_STEP_DEG)
                                                .ceil(),
                                        )
                                        .max(1.) as usize
                                })
                                .sum::<usize>();
                            if estimate + path.len() > MAX_RETURN_SAMPLES {
                                continue;
                            }
                            let mut q = current;
                            let mut candidate = Vec::new();
                            let mut valid = true;
                            for p in [p1, p2, target] {
                                match context.leg(q, p, cancel) {
                                    Ok(leg) => {
                                        q = leg.samples.last().unwrap().joints;
                                        candidate.push(leg);
                                    }
                                    Err(next) => {
                                        if !retryable(&next) {
                                            return Err(next);
                                        }
                                        valid = false;
                                        break;
                                    }
                                }
                            }
                            if valid {
                                found = Some(candidate);
                                break 'search;
                            }
                        }
                    }
                }
                match found {
                    Some(route) => {
                        used_detour = true;
                        route
                    }
                    None => {
                        error.message =
                            format!("{} 有界绕行搜索未找到完整可验证路径。", error.message);
                        return Err(error);
                    }
                }
            }
        };
        let last = route.len() - 1;
        for (i, leg) in route.drain(..).enumerate() {
            if path.len() + leg.samples.len() > MAX_RETURN_SAMPLES {
                return Err(fail(
                    "resource_limit",
                    "整条路径超过采样上限，未返回部分结果。",
                ));
            }
            let endpoint = leg.samples.last().unwrap();
            current = endpoint.joints;
            waypoints.push(GrindingWaypoint {
                pose: endpoint.pose,
                joints: current,
                configuration: context.config(&current)?,
                target_index: if i == last { Some(target_index) } else { None },
                // The role describes the incoming motion, not the identity of its endpoint.
                // Preserve target_index even when reaching it via an approach/detour.
                kind: if last == 0 && target_index > 0 {
                    "process"
                } else {
                    "transfer"
                }
                .into(),
            });
            for mut sample in leg.samples {
                sample.t = duration + sample.t * leg.duration;
                path.push(sample);
            }
            duration += leg.duration;
            length += leg.length;
            if !duration.is_finite() || !length.is_finite() {
                return Err(fail("resource_limit", "累计路径长度或时间超过数值范围。"));
            }
        }
    }
    if cancel() {
        return Err(cancelled());
    }
    let mut previous_time = -1.;
    for sample in &mut path {
        sample.t /= duration;
        if !sample.t.is_finite() || !(0. ..=1.).contains(&sample.t) || sample.t <= previous_time {
            return Err(fail(
                "resource_limit",
                "路径采样时间超出数值精度，未返回部分路径。",
            ));
        }
        previous_time = sample.t;
    }
    let mut warnings=vec!["已验证声明的机器人／工具保守包络与禁入区在关节插值全程分离；未包含自碰撞、实际工件接触、控制器或动力学验收。".into()];
    if used_detour {
        warnings.push(
            "绕行点为转移运动，须在加工工艺中关闭打磨接触；不得将绕行当作表面加工轨迹。".into(),
        );
    }
    if orientation.is_some()
        && request
            .targets
            .iter()
            .any(|p| quaternion_distance_deg(&p.orientation, &orientation.unwrap()) > 0.1)
    {
        warnings.push("输入点的方向已按统一工具方向调整；原输入数据未修改。".into());
    }
    Ok(GrindingSuccess {
        path,
        waypoints,
        duration_seconds: duration,
        length_mm: length,
        orientation,
        checked_intervals: context.checked_intervals,
        warnings,
    })
}
