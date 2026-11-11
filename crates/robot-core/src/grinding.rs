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
    forward_unchecked, generic_joints_within_limits, solve_task_unchecked, validate_generic_chain,
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

pub const MAX_TARGETS: usize = 256;
pub const MAX_RETURN_SAMPLES: usize = 8192;
pub const MAX_IK_CALLS: usize = 32768;
pub const MAX_INTERVAL_CHECKS: usize = 100000;
pub const MAX_CERT_DEPTH: usize = 12;
pub const TRANSLATION_STEP_MM: f64 = 2.;
pub const ORIENTATION_STEP_DEG: f64 = 1.;
pub const MAX_JOINT_STEP_DEG: f64 = 15.;
pub const FIXED_ORIENTATION_TOLERANCE_DEG: f64 = 0.1;
pub const SEARCH_BEAM_WIDTH: usize = 4;
pub const MAX_LEG_REFINEMENTS: usize = 3;

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
            let expected = normalize_orientation(&q).map_err(|m| fail("invalid_input", &m))?;
            for target in &request.targets {
                let actual = normalize_orientation(&target.orientation)
                    .map_err(|m| fail("invalid_input", &m))?;
                if quaternion_distance_deg(&expected, &actual) > 1e-5 {
                    return Err(fail(
                        "invalid_input",
                        "旧版统一方向参数与目标姿态不一致；不会覆盖输入目标。",
                    ));
                }
            }
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
    fn ik_candidates<F: FnMut() -> bool>(
        &mut self,
        pose: &Pose,
        seed: &JointAngles,
        explore: bool,
        cancel: &mut F,
    ) -> Checked<Vec<JointAngles>> {
        if cancel() {
            return Err(cancelled());
        }
        let base_tcp = matrix_pose(&multiply_matrices(&self.base_inverse, &pose_matrix(pose)?));
        let target = matrix_pose(&multiply_matrices(
            &pose_matrix(&base_tcp)?,
            &self.tool_inverse,
        ));
        let mut candidates = Vec::new();
        match &self.request.robot {
            GrindingRobot::Abb { model_id } => {
                self.ik_calls += 1;
                if self.ik_calls > MAX_IK_CALLS {
                    return Err(fail(
                        "resource_limit",
                        "路径搜索达到逆解预算，未返回部分路径。",
                    ));
                }
                let model = get_robot_model(model_id).unwrap();
                let conf = self
                    .config(seed)?
                    .ok_or_else(|| fail("singular_path", "起点没有有效机器人配置。"))?;
                // cf1/cf4/cf6 are quadrant counters, not branch identities. Crossing
                // 0/90 degrees smoothly must not be mistaken for a wrist flip.
                candidates = inverse_kinematics_cancellable(&model, &target, seed, &mut *cancel)
                    .map_err(|_| cancelled())?
                    .into_iter()
                    .filter(|s| s.configuration[3] == conf[3])
                    .map(|s| s.joints)
                    .collect();
            }
            GrindingRobot::Generic { chain, .. } => {
                let mut seeds = vec![*seed];
                if explore {
                    for axis in 0..6 {
                        for offset in [-10., 10.] {
                            let mut next = *seed;
                            next[axis] = (next[axis] + offset)
                                .clamp(chain.limits[axis].min_deg, chain.limits[axis].max_deg);
                            seeds.push(next);
                        }
                    }
                }
                for trial in seeds {
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
                    if let Some(q) = solve_task_unchecked(
                        chain,
                        &base_tcp,
                        &trial,
                        Some(&self.tool),
                        false,
                        cancel,
                    )
                    .map_err(|_| cancelled())?
                    {
                        if !candidates.iter().any(|old| same_joints(old, &q)) {
                            candidates.push(q);
                        }
                    }
                }
            }
        }
        let had_candidates = !candidates.is_empty();
        candidates.retain(|q| {
            q.iter()
                .zip(seed)
                .all(|(a, b)| (a - b).abs() <= MAX_JOINT_STEP_DEG + 1e-8)
        });
        if had_candidates && candidates.is_empty() {
            return Err(fail(
                "configuration_change",
                "相邻采样关节变化超过15°，需要细化路径。",
            ));
        }
        candidates.sort_by(|a, b| {
            joint_cost(seed, a)
                .total_cmp(&joint_cost(seed, b))
                .then_with(|| compare_joints(a, b))
        });
        if candidates.is_empty() {
            return Err(fail(
                "unreachable",
                "未找到保持目标姿态且连续的逆解，不能跳换关节分支。",
            ));
        }
        Ok(candidates)
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
        expected: Option<(&Quaternion, &Quaternion)>,
        cancel: &mut F,
    ) -> Checked<()> {
        self.certify_with_configuration(a, b, depth, expected, true, cancel)
    }
    // Direct joint jogging can cross a Cartesian singularity/configuration
    // boundary. Its swept bodies still require the same collision certificate.
    fn certify_with_configuration<F: FnMut() -> bool>(
        &mut self,
        a: &JointAngles,
        b: &JointAngles,
        depth: usize,
        expected: Option<(&Quaternion, &Quaternion)>,
        check_configuration: bool,
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
        let expected_mid = expected.map(|(a, b)| slerp_quaternion(a, b, 0.5));
        if check_configuration && matches!(self.request.robot, GrindingRobot::Abb { .. }) {
            if self.config(&mid)?.map(|q| q[3]) != self.config(a)?.map(|q| q[3]) {
                return Err(fail(
                    "configuration_change",
                    "关节插值跨越机器人分支，未生成路径。",
                ));
            }
        }
        let orientation_clear = if let Some((expected_a, expected_b)) = expected {
            let error = quaternion_distance_deg(
                &self.tcp(&mid, cancel)?.orientation,
                &expected_mid.unwrap(),
            );
            if error > FIXED_ORIENTATION_TOLERANCE_DEG + 1e-8 {
                return Err(fail(
                    "orientation_tolerance",
                    "关节插值偏离目标姿态插值超过0.1°，需要细化路径。",
                ));
            }
            error
                + a.iter().zip(b).map(|(x, y)| (x - y).abs()).sum::<f64>() / 2.
                + quaternion_distance_deg(expected_a, expected_b) / 2.
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
                "连续运动包络或姿态插值无法在检查精度内完成验证，未生成路径。",
            ));
        }
        self.certify_with_configuration(
            a,
            &mid,
            depth + 1,
            expected.map(|(qa, _)| (qa, expected_mid.as_ref().unwrap())),
            check_configuration,
            cancel,
        )?;
        self.certify_with_configuration(
            &mid,
            b,
            depth + 1,
            expected.map(|(_, qb)| (expected_mid.as_ref().unwrap(), qb)),
            check_configuration,
            cancel,
        )
    }
    fn legs<F: FnMut() -> bool>(
        &mut self,
        start: JointAngles,
        target: Pose,
        explore: bool,
        cancel: &mut F,
    ) -> Checked<Vec<Leg>> {
        let origin = self.tcp(&start, cancel)?;
        let length = norm(&sub(&target.position, &origin.position));
        let angle = quaternion_distance_deg(&origin.orientation, &target.orientation);
        let initial = (length / TRANSLATION_STEP_MM)
            .ceil()
            .max((angle / ORIENTATION_STEP_DEG).ceil())
            .max(1.) as usize;
        let mut last_error = fail("unreachable", "未找到连续运动。");
        for refinement in 0..=MAX_LEG_REFINEMENTS {
            let count = initial.saturating_mul(1 << refinement);
            if count >= MAX_RETURN_SAMPLES {
                return Err(fail("sample_limit", "单段路径超过采样上限。"));
            }
            match self
                .legs_at_resolution(start, origin, target, length, angle, count, explore, cancel)
            {
                Ok(legs) => return Ok(legs),
                Err(error)
                    if matches!(
                        error.code.as_str(),
                        "orientation_tolerance" | "configuration_change"
                    ) =>
                {
                    last_error = error
                }
                Err(error) => return Err(error),
            }
        }
        Err(last_error)
    }
    #[allow(clippy::too_many_arguments)]
    fn legs_at_resolution<F: FnMut() -> bool>(
        &mut self,
        start: JointAngles,
        origin: Pose,
        target: Pose,
        length: f64,
        angle: f64,
        count: usize,
        explore: bool,
        cancel: &mut F,
    ) -> Checked<Vec<Leg>> {
        let mut beam = vec![Leg {
            samples: Vec::with_capacity(count),
            duration: (length / self.request.speed_mm_per_second)
                .max(angle / 500.)
                .max(0.001),
            length,
            cost: length * 0.01,
        }];
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
            let previous_orientation = slerp_quaternion(
                &origin.orientation,
                &target.orientation,
                (index - 1) as f64 / count as f64,
            );
            let mut next = Vec::new();
            let mut error = fail("unreachable", "未找到连续运动。");
            for state in &beam {
                let previous = state.samples.last().map(|s| s.joints).unwrap_or(start);
                let candidates = match self.ik_candidates(&wanted, &previous, explore, cancel) {
                    Ok(candidates) => candidates,
                    Err(e) if retryable(&e) => {
                        error = e;
                        continue;
                    }
                    Err(e) => return Err(e),
                };
                let mut accepted = 0;
                for q in candidates {
                    match self.certify(
                        &previous,
                        &q,
                        0,
                        Some((&previous_orientation, &wanted.orientation)),
                        cancel,
                    ) {
                        Ok(()) => (),
                        Err(e) if retryable(&e) => {
                            error = e;
                            continue;
                        }
                        Err(e) => return Err(e),
                    }
                    let actual = self.tcp(&q, cancel)?;
                    if norm(&sub(&actual.position, &wanted.position)) > 0.002
                        || quaternion_distance_deg(&actual.orientation, &wanted.orientation) > 0.002
                    {
                        error = fail("unreachable", "实际TCP未达到输入位姿精度，未生成路径。");
                        continue;
                    }
                    let mut child = state.clone();
                    for j in 0..6 {
                        child.duration = child
                            .duration
                            .max((q[j] - previous[j]).abs() / self.speed_limits[j] * count as f64);
                    }
                    if !child.duration.is_finite() {
                        return Err(fail(
                            "invalid_input",
                            "关节速度产生非有限运动时间，未生成路径。",
                        ));
                    }
                    child.cost += joint_cost(&previous, &q);
                    child.samples.push(MotionSample {
                        t,
                        pose: actual,
                        joints: q,
                    });
                    next.push(child);
                    accepted += 1;
                    if accepted == SEARCH_BEAM_WIDTH {
                        break;
                    }
                }
            }
            next.sort_by(|a, b| {
                a.cost.total_cmp(&b.cost).then_with(|| {
                    compare_joints(
                        &a.samples.last().unwrap().joints,
                        &b.samples.last().unwrap().joints,
                    )
                })
            });
            let mut distinct: Vec<Leg> = Vec::new();
            for state in next {
                if !distinct.iter().any(|s| {
                    same_joints(
                        &s.samples.last().unwrap().joints,
                        &state.samples.last().unwrap().joints,
                    )
                }) {
                    distinct.push(state);
                    if distinct.len() == SEARCH_BEAM_WIDTH {
                        break;
                    }
                }
            }
            if distinct.is_empty() {
                return Err(error);
            }
            beam = distinct;
        }
        Ok(beam)
    }
}
#[derive(Clone)]
struct Leg {
    samples: Vec<MotionSample>,
    duration: f64,
    length: f64,
    cost: f64,
}
fn same_joints(a: &JointAngles, b: &JointAngles) -> bool {
    a.iter().zip(b).all(|(a, b)| (a - b).abs() <= 0.001)
}
fn compare_joints(a: &JointAngles, b: &JointAngles) -> std::cmp::Ordering {
    a.iter()
        .zip(b)
        .map(|(a, b)| a.total_cmp(b))
        .find(|order| *order != std::cmp::Ordering::Equal)
        .unwrap_or(std::cmp::Ordering::Equal)
}
fn joint_cost(a: &JointAngles, b: &JointAngles) -> f64 {
    a.iter()
        .zip(b)
        .enumerate()
        .map(|(i, (a, b))| (a - b).abs() * if i >= 3 { 1.5 } else { 1. })
        .sum()
}
fn retryable(error: &GrindingFailure) -> bool {
    matches!(
        error.code.as_str(),
        "collision"
            | "uncertain"
            | "unreachable"
            | "configuration_change"
            | "singular_path"
            | "orientation_tolerance"
            | "sample_limit"
    )
}

pub fn plan_grinding_cancellable<F: FnMut() -> bool>(
    request: &GrindingRequest,
    mut cancel: F,
) -> GrindingResult {
    match run(request, &mut cancel) {
        Ok(value) => GrindingResult::Success(value),
        Err(mut error) => {
            if error.code == "sample_limit" {
                error.code = "resource_limit".into();
            }
            GrindingResult::Failure(error)
        }
    }
}
pub fn plan_grinding(request: &GrindingRequest, cancel: &AtomicBool) -> GrindingResult {
    plan_grinding_cancellable(request, || cancel.load(Ordering::Relaxed))
}
#[derive(Clone)]
struct SearchState {
    current: JointAngles,
    path: Vec<MotionSample>,
    waypoints: Vec<GrindingWaypoint>,
    duration: f64,
    length: f64,
    cost: f64,
    used_detour: bool,
}
fn prune_states(mut states: Vec<SearchState>) -> Vec<SearchState> {
    states.sort_by(|a, b| {
        a.cost
            .total_cmp(&b.cost)
            .then_with(|| compare_joints(&a.current, &b.current))
    });
    let mut kept: Vec<SearchState> = Vec::new();
    for state in states {
        if !kept
            .iter()
            .any(|other| same_joints(&other.current, &state.current))
        {
            kept.push(state);
            if kept.len() == SEARCH_BEAM_WIDTH {
                break;
            }
        }
    }
    kept
}
fn routes(request: &GrindingRequest, start: Pose, target: Pose, explore: bool) -> Vec<Vec<Pose>> {
    let mut routes = vec![vec![target]];
    if !explore || !request.allow_detours || !request.zones.iter().any(|z| z.enabled) {
        return routes;
    }
    for extra in [25., 75.] {
        for axis in 0..3 {
            for sign in [-1., 1.] {
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
                let plane = edge + sign * (request.tool_radius_mm + request.clearance_mm + extra);
                let mut p1 = start;
                p1.position[axis] = plane;
                // Transfer orientations follow the endpoint interpolation; input target is untouched.
                p1.orientation = slerp_quaternion(&start.orientation, &target.orientation, 1. / 3.);
                let mut p2 = target;
                p2.position[axis] = plane;
                p2.orientation = slerp_quaternion(&start.orientation, &target.orientation, 2. / 3.);
                if [p1, p2]
                    .iter()
                    .all(|p| p.position.iter().all(|x| x.is_finite() && x.abs() <= 1e6))
                {
                    routes.push(vec![p1, p2, target]);
                }
            }
        }
    }
    routes
}
fn search<F: FnMut() -> bool>(
    context: &mut Context,
    initial: &SearchState,
    explore: bool,
    cancel: &mut F,
) -> Checked<SearchState> {
    let mut beam = vec![initial.clone()];
    for target_index in 0..context.request.targets.len() {
        let input = context.request.targets[target_index];
        let target = Pose {
            position: input.position,
            orientation: normalize_orientation(&input.orientation)
                .map_err(|m| fail("invalid_input", &m))?,
        };
        let mut next = Vec::new();
        let mut failure = fail("unreachable", "有界搜索未找到保留全部目标位姿的连续路径。");
        for state in &beam {
            let start = context.tcp(&state.current, cancel)?;
            for route in routes(context.request, start, target, explore) {
                let mut origin = start;
                let mut estimate = state.path.len();
                for pose in &route {
                    estimate += (norm(&sub(&pose.position, &origin.position)) / TRANSLATION_STEP_MM)
                        .ceil()
                        .max(
                            (quaternion_distance_deg(&origin.orientation, &pose.orientation)
                                / ORIENTATION_STEP_DEG)
                                .ceil(),
                        )
                        .max(1.) as usize;
                    origin = *pose;
                }
                if estimate > MAX_RETURN_SAMPLES {
                    failure = fail("sample_limit", "整条路径超过采样上限，未返回部分结果。");
                    continue;
                }
                let detour = route.len() > 1;
                let mut partial = vec![state.clone()];
                for (route_index, pose) in route.iter().enumerate() {
                    let mut children = Vec::new();
                    for prefix in &partial {
                        let legs = match context.legs(prefix.current, *pose, explore, cancel) {
                            Ok(legs) => legs,
                            Err(error) if retryable(&error) => {
                                failure = error;
                                continue;
                            }
                            Err(error) => return Err(error),
                        };
                        for leg in legs {
                            if prefix.path.len() + leg.samples.len() > MAX_RETURN_SAMPLES {
                                failure =
                                    fail("sample_limit", "整条路径超过采样上限，未返回部分结果。");
                                continue;
                            }
                            let mut child = prefix.clone();
                            let endpoint = leg.samples.last().unwrap();
                            child.current = endpoint.joints;
                            child.waypoints.push(GrindingWaypoint {
                                pose: *pose,
                                joints: endpoint.joints,
                                configuration: context.config(&endpoint.joints)?,
                                target_index: if route_index == route.len() - 1 {
                                    Some(target_index)
                                } else {
                                    None
                                },
                                kind: if !detour && target_index > 0 {
                                    "process"
                                } else {
                                    "transfer"
                                }
                                .into(),
                            });
                            for mut sample in leg.samples {
                                sample.t = child.duration + sample.t * leg.duration;
                                child.path.push(sample);
                            }
                            child.duration += leg.duration;
                            child.length += leg.length;
                            child.cost += leg.cost;
                            child.used_detour |= detour;
                            if !child.duration.is_finite()
                                || !child.length.is_finite()
                                || !child.cost.is_finite()
                            {
                                return Err(fail(
                                    "resource_limit",
                                    "累计路径长度或时间超过数值范围。",
                                ));
                            }
                            children.push(child);
                        }
                    }
                    partial = prune_states(children);
                    if partial.is_empty() {
                        break;
                    }
                }
                next.extend(partial);
                // Keep memory bounded while still comparing every deterministic route.
                next = prune_states(next);
                if target_index + 1 == context.request.targets.len() && !next.is_empty() {
                    return Ok(next.remove(0));
                }
            }
        }
        if next.is_empty() {
            failure.failed_target = Some(target_index);
            return Err(failure);
        }
        beam = prune_states(next);
    }
    beam.into_iter()
        .next()
        .ok_or_else(|| fail("unreachable", "未找到完整路径。"))
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
    let initial = SearchState {
        current: request.start_joints,
        path: vec![MotionSample {
            t: 0.,
            pose: context.tcp(&request.start_joints, cancel)?,
            joints: request.start_joints,
        }],
        waypoints: Vec::new(),
        duration: 0.,
        length: 0.,
        cost: 0.,
        used_detour: false,
    };
    let mut found = match search(&mut context, &initial, false, cancel) {
        Ok(found) => found,
        Err(error) if retryable(&error) => search(&mut context, &initial, true, cancel)?,
        Err(error) => return Err(error),
    };
    if cancel() {
        return Err(cancelled());
    }
    let mut previous = -1.;
    for sample in &mut found.path {
        sample.t /= found.duration;
        if !sample.t.is_finite() || !(0. ..=1.).contains(&sample.t) || sample.t <= previous {
            return Err(fail(
                "resource_limit",
                "路径采样时间超出数值精度，未返回部分路径。",
            ));
        }
        previous = sample.t;
    }
    let mut warnings = vec!["输入目标的位置和姿态保持不变；使用有界候选搜索选择连续关节路径，不保证全局最优或穷举全部解。".into(), "已验证声明的机器人／工具保守包络与禁入区在关节插值全程分离；姿态跟随目标间的四元数插值；未包含自碰撞、实际工件接触、控制器或动力学验收。".into()];
    if found.used_detour {
        warnings.push(
            "绕行点为转移运动，须在加工工艺中关闭打磨接触；不得将绕行当作表面加工轨迹。".into(),
        );
    }
    Ok(GrindingSuccess {
        path: found.path,
        waypoints: found.waypoints,
        duration_seconds: found.duration,
        length_mm: found.length,
        orientation,
        checked_intervals: context.checked_intervals,
        warnings,
    })
}

/// Shared bounded geometry checks for the separate pendant numerical API.
/// Reuses the exact robot/tool swept envelopes used by path planning. Joint
/// mode deliberately has no Cartesian branch/singularity restriction.
pub(crate) fn pendant_step<F: FnMut() -> bool>(
    request: &crate::pendant_jog::JogRequest,
    cancel: &mut F,
) -> Checked<crate::pendant_jog::JogSuccess> {
    use crate::pendant_jog::{JogMode, JogSpace, JogSuccess};
    if !request.step_seconds.is_finite() || request.step_seconds <= 0. || request.step_seconds > 0.05
        || !request.speed_scale.is_finite() || !(0. ..=1.).contains(&request.speed_scale)
        || request.axes.iter().any(|axis| !axis.is_finite() || axis.abs() > 1.)
        || request.start_joints.iter().any(|joint| !joint.is_finite())
        || request.mode == JogMode::Linear && request.axes[3..].iter().any(|axis| *axis != 0.)
    { return Err(fail("invalid_input", "示教方向、速度或步长无效。")); }
    let context_request = GrindingRequest {
        robot: request.robot.clone(), base_pose: request.base_pose, tool_pose: request.tool_pose,
        tool_radius_mm: request.tool_radius_mm, start_joints: request.start_joints,
        targets: vec![Pose { position: [0.; 3], orientation: [1., 0., 0., 0.] }],
        orientation: None, zones: request.zones.clone(), clearance_mm: 0.,
        speed_mm_per_second: (100. * request.speed_scale).max(1.), allow_detours: false,
    };
    let mut context = Context::new(&context_request)?;
    let origin = context.tcp(&request.start_joints, cancel)?;
    let magnitude = request.axes.iter().map(|axis| axis * axis).sum::<f64>().sqrt();
    if magnitude < 1e-12 || request.speed_scale == 0. {
        return Ok(JogSuccess { joints: request.start_joints, pose: origin });
    }
    let unit = request.axes.map(|axis| axis / magnitude.max(1.));
    let start_boxes = context.boxes(&request.start_joints, cancel)?;
    if !context.clear(&start_boxes, 0.)? { return Err(fail("collision", "机器人或工具的保守包络与禁入区相交，请先调整场景。")); }
    let (joints, pose) = match request.mode {
        JogMode::Joint => {
            let candidate = std::array::from_fn(|index| request.start_joints[index]
                + unit[index] * request.speed_scale * request.step_seconds * 30_f64.min(context.speed_limits[index]));
            let valid = match &request.robot {
                GrindingRobot::Abb { model_id } => joints_within_limits(&get_robot_model(model_id).unwrap(), &candidate),
                GrindingRobot::Generic { chain, .. } => generic_joints_within_limits(chain, &candidate),
            };
            if !valid { return Err(fail("joint_limit", "该轴已到达关节限位，已停止示教移动。")); }
            context.certify_with_configuration(&request.start_joints, &candidate, 0, None, false, cancel)?;
            (candidate, context.tcp(&candidate, cancel)?)
        }
        JogMode::Linear => {
            context.config(&request.start_joints).map_err(|mut error| { error.message = "当前姿态接近奇异位置，请先使用轴模式调整关节。".into(); error })?;
            let mut direction: Vec3 = [unit[0], unit[1], unit[2]];
            if request.space == JogSpace::Tool {
                let matrix = quaternion_to_matrix(&origin.orientation).map_err(|m| fail("invalid_input", &m))?;
                direction = std::array::from_fn(|row| (0..3).map(|column| matrix[row * 4 + column] * direction[column]).sum());
            }
            let target = Pose { position: std::array::from_fn(|index| origin.position[index]
                + direction[index] * 100. * request.speed_scale * request.step_seconds), orientation: origin.orientation };
            let legs = context.legs(request.start_joints, target, false, cancel)?;
            let selected = legs.first().and_then(|leg| leg.samples.last()).ok_or_else(|| fail("unreachable", "未找到连续示教位姿。"))?;
            let candidate = selected.joints;
            for index in 0..6 {
                if (candidate[index] - request.start_joints[index]).abs() > context.speed_limits[index] * request.step_seconds + 1e-8 {
                    return Err(fail("joint_speed", "当前直线方向超过关节速度，请降低示教速度或使用轴模式。"));
                }
            }
            // The UI applies one endpoint per tick, so also certify the direct
            // joint segment instead of relying only on internal IK samples.
            context.certify(&request.start_joints, &candidate, 0, Some((&origin.orientation, &target.orientation)), cancel)?;
            (candidate, context.tcp(&candidate, cancel)?)
        }
    };
    if cancel() { return Err(cancelled()); }
    Ok(JogSuccess { joints, pose })
}
