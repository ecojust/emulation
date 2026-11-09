//! Cancellable, sampled Cartesian motion planning for registered robot models.
//!
//! This validates nominal geometry and configuration continuity. It does not
//! simulate collisions, calibration, acceleration, or a controller's trajectory.
use std::sync::atomic::{AtomicBool, Ordering};

use crate::configuration::get_configuration;
use crate::kinematics::{
    forward_kinematics, inverse_kinematics, joint_distance, joints_within_limits,
    solve_ik_candidates,
};
use crate::math::{
    cross, dot, matrix_to_quaternion, norm, normalize_orientation, quaternion_distance_deg,
    quaternion_to_matrix, slerp_quaternion, sub, unit,
};
use crate::model::get_robot_model;
use crate::types::{
    FailureCode, FkResult, IkSolution, JointAngles, MotionSample, PlanFailure, PlanRequest,
    PlanResult, PlanSuccess, Pose, Quaternion, RobotModel, Vec3,
};

pub const TRANSLATION_STEP_MM: f64 = 5.;
pub const ORIENTATION_STEP_DEG: f64 = 2.;
pub const MAX_SAMPLES: usize = 1601;
pub const MAX_JOINT_STEP_DEG: f64 = 15.;
pub const MIN_SPEED_MM_PER_SECOND: f64 = 1.;
pub const MAX_SPEED_MM_PER_SECOND: f64 = 2000.;
/// Matches the orientation speed in the ABB postprocessor's explicit speeddata.
pub const ORIENTATION_SPEED_DEG_PER_SECOND: f64 = 500.;

fn failure(code: FailureCode, message: &str) -> PlanResult {
    PlanResult::Failure(PlanFailure {
        code,
        message: message.into(),
        failed_at: None,
        solutions: None,
    })
}

fn path_failure(
    code: FailureCode,
    message: &str,
    failed_at: Option<f64>,
    solutions: Vec<IkSolution>,
) -> PlanResult {
    PlanResult::Failure(PlanFailure {
        code,
        message: message.into(),
        failed_at,
        solutions: Some(solutions),
    })
}

fn cancelled() -> PlanResult {
    failure(FailureCode::Cancelled, "运动计算已取消，未生成运动指令。")
}

/// The current direction is tried before this function, so successful common
/// cases never allocate, construct, normalize or sort the search grid.
fn automatic_orientations(model: &RobotModel, start: &Pose, position: &Vec3) -> Vec<Quaternion> {
    let start_r = quaternion_to_matrix(&start.orientation)
        .expect("forward kinematics produces a normalized orientation");
    let initial_x = [start_r[0], start_r[4], start_r[8]];
    let radial = unit(&[position[0], position[1], 0.]).unwrap_or([1., 0., 0.]);
    let shoulder_direction = unit(&[
        position[0],
        position[1],
        position[2] - model.geometry.shoulder_height_mm,
    ]);
    let mut directions = Vec::with_capacity(9);
    if let Some(direction) = shoulder_direction {
        directions.push(direction);
    }
    directions.extend([
        radial,
        [-radial[0], -radial[1], 0.],
        [0., 0., -1.],
        [0., 0., 1.],
        [radial[0], radial[1], -1.],
        [radial[0], radial[1], 1.],
        [-radial[0], -radial[1], -1.],
        [-radial[0], -radial[1], 1.],
    ]);
    let mut result = Vec::<(Quaternion, f64)>::with_capacity(54);
    for direction in directions {
        let Some(z) = unit(&direction) else { continue };
        let projection = dot(&initial_x, &z);
        let projected = std::array::from_fn(|i| initial_x[i] - projection * z[i]);
        let fallback_axis = if z[2].abs() < 0.9 {
            [0., 0., 1.]
        } else {
            [0., 1., 0.]
        };
        let Some(x) = unit(&projected).or_else(|| unit(&cross(&fallback_axis, &z))) else {
            continue;
        };
        let y = cross(&z, &x);
        for angle_deg in [0_f64, 45., -45., 90., -90., 180.] {
            let (sin, cos) = angle_deg.to_radians().sin_cos();
            let xx: Vec3 = std::array::from_fn(|i| x[i] * cos + y[i] * sin);
            let yy: Vec3 = std::array::from_fn(|i| y[i] * cos - x[i] * sin);
            let orientation = matrix_to_quaternion(&[
                xx[0], yy[0], z[0], 0., xx[1], yy[1], z[1], 0., xx[2], yy[2], z[2], 0., 0., 0., 0.,
                1.,
            ]);
            let travel = quaternion_distance_deg(&orientation, &start.orientation);
            if travel < 0.01
                || result
                    .iter()
                    .any(|(q, _)| quaternion_distance_deg(q, &orientation) < 0.01)
            {
                continue;
            }
            result.push((orientation, travel));
        }
    }
    // Stable sorting preserves direction/roll order for equal orientation costs.
    result.sort_by(|a, b| a.1.total_cmp(&b.1));
    result.into_iter().map(|(q, _)| q).collect()
}

fn turn_is_continuous(candidate: &JointAngles, start: &JointAngles) -> bool {
    [0, 3, 5]
        .iter()
        .all(|&axis| (candidate[axis] - start[axis]).abs() <= 180. + 1e-8)
}

struct PathContext<'a> {
    model: &'a RobotModel,
    start_joints: &'a JointAngles,
    start_fk: &'a FkResult,
    start_cfx: i32,
    speed: f64,
}

fn evaluate_path<F: FnMut() -> bool>(
    context: &PathContext<'_>,
    target: &Pose,
    automatic_orientation: bool,
    should_cancel: &mut F,
) -> PlanResult {
    let PathContext {
        model,
        start_joints,
        start_fk,
        start_cfx,
        speed,
    } = *context;
    if should_cancel() {
        return cancelled();
    }
    let solutions = inverse_kinematics(model, target, start_joints);
    if should_cancel() {
        return cancelled();
    }
    if solutions.is_empty() {
        let singular = solve_ik_candidates(model, target, start_joints)
            .iter()
            .any(|c| c.singular);
        return if singular {
            path_failure(
                FailureCode::SingularPath,
                "目标姿态处于奇异区域，无法确定连续的机器人配置。",
                Some(1.),
                solutions,
            )
        } else {
            path_failure(
                FailureCode::Unreachable,
                "目标位置和方向没有满足六轴关节限位的逆解。",
                None,
                solutions,
            )
        };
    }
    if !solutions.iter().any(|c| c.configuration[3] == start_cfx) {
        return path_failure(
            FailureCode::ConfigurationChange,
            "目标姿态有逆解，但需切换 cfx 分支；不能从当前姿态直接执行保持配置的 MoveL。",
            None,
            solutions,
        );
    }
    let length_mm = norm(&sub(&target.position, &start_fk.pose.position));
    let orientation_travel_deg =
        quaternion_distance_deg(&start_fk.pose.orientation, &target.orientation);
    let intervals_f64 = (length_mm / TRANSLATION_STEP_MM)
        .ceil()
        .max((orientation_travel_deg / ORIENTATION_STEP_DEG).ceil())
        .max(1.);
    if !intervals_f64.is_finite() || intervals_f64 >= MAX_SAMPLES as f64 {
        return path_failure(
            FailureCode::InvalidInput,
            "路径超过本次仿真的采样范围，请缩小目标距离。",
            None,
            solutions,
        );
    }
    let intervals = intervals_f64 as usize;
    let mut path = Vec::with_capacity(intervals + 1);
    path.push(MotionSample {
        t: 0.,
        pose: start_fk.pose,
        joints: *start_joints,
    });
    let mut previous = *start_joints;
    let mut selected_index = 0;
    let mut joint_duration = 0_f64;
    for step in 1..=intervals {
        if should_cancel() {
            return cancelled();
        }
        let t = step as f64 / intervals as f64;
        let pose = Pose {
            position: std::array::from_fn(|axis| {
                start_fk.pose.position[axis]
                    + t * (target.position[axis] - start_fk.pose.position[axis])
            }),
            orientation: slerp_quaternion(&start_fk.pose.orientation, &target.orientation, t),
        };
        let mut has_nonsingular = false;
        let mut has_matching = false;
        let mut has_singular = false;
        let mut best: Option<(JointAngles, Pose, f64)> = None;
        if step == intervals {
            for (index, candidate) in solutions.iter().enumerate() {
                has_nonsingular = true;
                if candidate.configuration[3] != start_cfx {
                    continue;
                }
                has_matching = true;
                if !turn_is_continuous(&candidate.joints, start_joints) {
                    continue;
                }
                let score = joint_distance(&candidate.joints, &previous);
                if best
                    .as_ref()
                    .is_none_or(|(_, _, previous_score)| score < *previous_score)
                {
                    best = Some((candidate.joints, candidate.pose, score));
                    selected_index = index;
                }
            }
        } else {
            // Keep raw fixed-size numeric candidates at interior points. IDs and
            // all public endpoint metadata are allocated only for the endpoint.
            let candidates = solve_ik_candidates(model, &pose, &previous);
            for candidate in candidates {
                has_singular |= candidate.singular;
                let Some(configuration) = candidate.configuration else {
                    continue;
                };
                if candidate.singular {
                    continue;
                }
                has_nonsingular = true;
                if configuration[3] != start_cfx {
                    continue;
                }
                has_matching = true;
                if !turn_is_continuous(&candidate.joints, start_joints) {
                    continue;
                }
                if best
                    .as_ref()
                    .is_none_or(|(_, _, score)| candidate.score < *score)
                {
                    best = Some((candidate.joints, candidate.pose, candidate.score));
                }
            }
        }
        if should_cancel() {
            return cancelled();
        }
        if !has_matching {
            let (code, message) = if has_singular {
                (
                    FailureCode::SingularPath,
                    "直线路径进入奇异区域，无法保持连续的机器人配置。",
                )
            } else if has_nonsingular {
                (
                    FailureCode::ConfigurationChange,
                    "直线路径需要切换机器人分支，已停止生成 MoveL。",
                )
            } else {
                (
                    FailureCode::PathUnreachable,
                    "目标端点有逆解，但直线路径中间无法满足关节限位或可达范围。",
                )
            };
            return path_failure(code, message, Some(t), solutions);
        }
        let Some((joints, actual_pose, _)) = best else {
            return path_failure(
                FailureCode::ConfigurationChange,
                "直线路径要求 A1、A4 或 A6 相对起点转动超过 180°，不符合当前 ConfL 配置约束。",
                Some(t),
                solutions,
            );
        };
        if joints
            .iter()
            .zip(previous)
            .any(|(q, p)| (q - p).abs() > MAX_JOINT_STEP_DEG)
        {
            return path_failure(
                FailureCode::SingularPath,
                "相邻路径采样的关节变化过大，无法确认连续可达；请调整目标方向或起始姿态。",
                Some(t),
                solutions,
            );
        }
        for axis in 0..6 {
            joint_duration = joint_duration.max(
                (joints[axis] - previous[axis]).abs() / model.limits[axis].max_speed_deg
                    * intervals as f64,
            );
        }
        path.push(MotionSample {
            t,
            pose: actual_pose,
            joints,
        });
        previous = joints;
    }
    if should_cancel() {
        return cancelled();
    }
    let ideal_duration = length_mm / speed;
    let duration_seconds = 0.05_f64
        .max(ideal_duration)
        .max(orientation_travel_deg / ORIENTATION_SPEED_DEG_PER_SECOND)
        .max(joint_duration);
    let warnings = if joint_duration > ideal_duration + 0.01 {
        vec!["预览时长已按关节额定速度延长；实际控制器还会考虑加速度和负载。".into()]
    } else {
        Vec::new()
    };
    PlanResult::Success(PlanSuccess {
        selected: solutions[selected_index].clone(),
        solutions,
        path,
        length_mm,
        orientation_travel_deg,
        duration_seconds,
        warnings,
        automatic_orientation,
    })
}

pub fn plan_move_l(request: &PlanRequest) -> PlanResult {
    plan_move_l_cancellable(request, || false)
}

/// Cooperative cancellation is checked between every direction and path sample.
/// The flag only carries a boolean, so Relaxed ordering is sufficient.
pub fn plan_move_l_with_cancel(request: &PlanRequest, cancel: &AtomicBool) -> PlanResult {
    plan_move_l_cancellable(request, || cancel.load(Ordering::Relaxed))
}

/// Allows callers to combine cancellation with a deadline without spawning a
/// polling thread. Returning true never exposes an incomplete successful path.
pub fn plan_move_l_cancellable<F: FnMut() -> bool>(
    request: &PlanRequest,
    mut should_cancel: F,
) -> PlanResult {
    if should_cancel() {
        return cancelled();
    }
    if request
        .start_joints
        .iter()
        .chain(request.target.position.iter())
        .any(|v| !v.is_finite())
        || !request.speed_mm_per_second.is_finite()
        || !(MIN_SPEED_MM_PER_SECOND..=MAX_SPEED_MM_PER_SECOND)
            .contains(&request.speed_mm_per_second)
    {
        return failure(
            FailureCode::InvalidInput,
            "请输入有限的 XYZ、六个关节角，以及 1–2000 mm/s 的速度。",
        );
    }
    let Some(model) = get_robot_model(&request.model_id) else {
        return failure(FailureCode::InvalidInput, "不支持的机器人型号。");
    };
    if !joints_within_limits(&model, &request.start_joints) {
        return failure(
            FailureCode::JointLimit,
            "起始关节角超出该型号的实际关节限位。",
        );
    }
    let Ok(start) = forward_kinematics(&model, &request.start_joints) else {
        return failure(FailureCode::InvalidInput, "起始关节角无效。");
    };
    let configuration = get_configuration(&model, &request.start_joints, &start);
    let Some(start_conf) = configuration
        .configuration
        .filter(|_| !configuration.singular)
    else {
        return failure(FailureCode::SingularStart, "起点处于奇异区域，请先调整关节或选择示例起始姿态；机械零位不能直接作为这条 MoveL 的起点。");
    };
    let orientation = match request.target.orientation {
        Some(q) => match normalize_orientation(&q) {
            Ok(q) => Some(q),
            Err(_) => {
                return failure(
                    FailureCode::InvalidInput,
                    "目标四元数不是有效的单位姿态，请检查 [q1,q2,q3,q4]，其中 q1 为实部。",
                )
            }
        },
        None => None,
    };
    // Triangle inequality around the physical shoulder, not the misleading
    // nominal reach around the world origin. This also bounds finite huge input
    // before orientation-grid arithmetic or any f64-to-usize conversion.
    let g = &model.geometry;
    let maximum_distance = g.upper_arm_mm + g.elbow_offset_mm.abs() + g.forearm_mm + g.flange_mm;
    if norm(&sub(&request.target.position, &start.shoulder)) > maximum_distance + 1e-6 {
        return failure(
            FailureCode::Unreachable,
            "目标位置不可达，超出该机器人标称几何的可达范围。",
        );
    }
    let context = PathContext {
        model: &model,
        start_joints: &request.start_joints,
        start_fk: &start,
        start_cfx: start_conf[3],
        speed: request.speed_mm_per_second,
    };
    let first_target = Pose {
        position: request.target.position,
        orientation: orientation.unwrap_or(start.pose.orientation),
    };
    let first = evaluate_path(
        &context,
        &first_target,
        orientation.is_none(),
        &mut should_cancel,
    );
    if orientation.is_some() {
        return first;
    }
    let mut best_failure = match first {
        success @ PlanResult::Success(_) => return success,
        PlanResult::Failure(error) if error.code == FailureCode::Cancelled => {
            return PlanResult::Failure(error)
        }
        PlanResult::Failure(error) => error,
    };
    if should_cancel() {
        return cancelled();
    }
    for orientation in automatic_orientations(&model, &start.pose, &request.target.position) {
        if should_cancel() {
            return cancelled();
        }
        let target = Pose {
            position: request.target.position,
            orientation,
        };
        let outcome = evaluate_path(&context, &target, true, &mut should_cancel);
        match outcome {
            success @ PlanResult::Success(_) => return success,
            PlanResult::Failure(error) if error.code == FailureCode::Cancelled => {
                return PlanResult::Failure(error)
            }
            PlanResult::Failure(error) => {
                let count = |failure: &PlanFailure| failure.solutions.as_ref().map_or(0, Vec::len);
                if count(&error) > count(&best_failure) {
                    best_failure = error;
                }
            }
        }
    }
    best_failure.message = format!(
        "自动姿态搜索未找到满足当前起点与直线路径的方向。可调整目标、指定方向或更换起始姿态。{}",
        best_failure.message
    );
    PlanResult::Failure(best_failure)
}
