use std::sync::atomic::AtomicBool;

use emulation_core::configuration::get_configuration;
use emulation_core::kinematics::{forward_kinematics, inverse_kinematics, joints_within_limits};
use emulation_core::math::{norm, quaternion_distance_deg, slerp_quaternion, sub};
use emulation_core::model::get_robot_model;
use emulation_core::planner::{
    plan_move_l, plan_move_l_cancellable, plan_move_l_with_cancel, MAX_SAMPLES,
    ORIENTATION_STEP_DEG, TRANSLATION_STEP_MM,
};
use emulation_core::{FailureCode, PlanRequest, PlanResult, PlanSuccess, RobotModel, Target};

const MODELS: [&str; 2] = ["irb1100-4-0.58", "irb1100-4-0.475"];

fn request(model: &RobotModel) -> PlanRequest {
    let start = forward_kinematics(model, &model.ready_joints).unwrap().pose;
    PlanRequest {
        model_id: model.id.clone(),
        start_joints: model.ready_joints,
        target: Target {
            position: [start.position[0] - 15., 30., start.position[2] - 10.],
            orientation: Some(start.orientation),
        },
        speed_mm_per_second: 100.,
    }
}

fn success(result: PlanResult) -> PlanSuccess {
    match result {
        PlanResult::Success(result) => result,
        PlanResult::Failure(error) => panic!("Expected valid sampled motion: {error:?}"),
    }
}

fn failure_code(result: PlanResult) -> FailureCode {
    match result {
        PlanResult::Success(result) => panic!("Unexpected valid motion: {:?}", result.selected),
        PlanResult::Failure(error) => error.code,
    }
}

#[test]
fn both_models_follow_the_entire_straight_path_with_constant_configuration() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        let request = request(&model);
        let initial = forward_kinematics(&model, &request.start_joints).unwrap();
        let start_conf = get_configuration(&model, &request.start_joints, &initial)
            .configuration
            .unwrap();
        let result = success(plan_move_l(&request));
        assert!(result.path.len() > 2 && result.path.len() <= MAX_SAMPLES);
        assert_eq!(result.path[0].joints, request.start_joints);
        assert_eq!(result.path.last().unwrap().joints, result.selected.joints);
        for (i, sample) in result.path.iter().enumerate() {
            let fk = forward_kinematics(&model, &sample.joints).unwrap();
            assert!(joints_within_limits(&model, &sample.joints));
            for axis in 0..3 {
                let expected = initial.pose.position[axis]
                    + sample.t * (request.target.position[axis] - initial.pose.position[axis]);
                assert!((sample.pose.position[axis] - expected).abs() < 1e-6);
                assert!((sample.pose.position[axis] - fk.pose.position[axis]).abs() < 1e-6);
            }
            assert!(
                quaternion_distance_deg(
                    &sample.pose.orientation,
                    &slerp_quaternion(
                        &initial.pose.orientation,
                        &request.target.orientation.unwrap(),
                        sample.t
                    )
                ) < 1e-4
            );
            assert_eq!(
                get_configuration(&model, &sample.joints, &fk)
                    .configuration
                    .unwrap()[3],
                start_conf[3]
            );
            if i > 0 {
                let previous = &result.path[i - 1];
                assert!(
                    norm(&sub(&sample.pose.position, &previous.pose.position))
                        <= TRANSLATION_STEP_MM + 1e-6
                );
                assert!(
                    quaternion_distance_deg(&sample.pose.orientation, &previous.pose.orientation)
                        <= ORIENTATION_STEP_DEG + 1e-6
                );
            }
        }
        assert!((result.length_mm - 35.).abs() < 1e-6);
        assert!(result.duration_seconds >= result.length_mm / request.speed_mm_per_second);
        assert!(!result.automatic_orientation);
    }
}

#[test]
fn automatic_orientation_preserves_the_starting_direction_when_reachable() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        let mut request = request(&model);
        let fixed = success(plan_move_l(&request));
        request.target.orientation = None;
        let automatic = success(plan_move_l(&request));
        assert!(automatic.automatic_orientation);
        assert!(automatic.orientation_travel_deg < 1e-4);
        assert_eq!(automatic.selected.joints, fixed.selected.joints);
        assert_eq!(automatic.path, fixed.path);
    }
}

#[test]
fn automatic_search_can_find_a_different_continuous_direction_after_first_attempt_fails() {
    // Verified against the previous TypeScript planner: keeping the ready
    // direction fails, while a 105-degree orientation change admits a path.
    let model = get_robot_model(MODELS[0]).unwrap();
    let mut request = request(&model);
    request.target.position = [200., 0., 100.];
    assert!(matches!(plan_move_l(&request), PlanResult::Failure(_)));
    request.target.orientation = None;
    let result = success(plan_move_l(&request));
    assert!(result.automatic_orientation);
    assert!((result.orientation_travel_deg - 105.).abs() < 1e-5);
    assert!(
        norm(&sub(
            &result.selected.pose.position,
            &request.target.position
        )) < 1e-6
    );
    assert!(result.path.len() > 100);
    assert_eq!(result.selected.configuration[3], 0);
}

#[test]
fn pure_tool_rotation_keeps_tcp_fixed_and_respects_angular_speed() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        let mut request = request(&model);
        let mut end_joints = model.ready_joints;
        end_joints[5] += 60.;
        let target = forward_kinematics(&model, &end_joints).unwrap().pose;
        request.target = Target {
            position: target.position,
            orientation: Some(target.orientation),
        };
        let result = success(plan_move_l(&request));
        let start = forward_kinematics(&model, &request.start_joints)
            .unwrap()
            .pose;
        assert!(result.length_mm < 1e-8);
        assert!((result.orientation_travel_deg - 60.).abs() < 1e-6);
        assert!(result.path.len() >= 31);
        assert!(result.duration_seconds >= 60. / 500. - 1e-9);
        for sample in &result.path {
            assert!(norm(&sub(&sample.pose.position, &start.position)) < 1e-6);
            assert!(
                (quaternion_distance_deg(&start.orientation, &sample.pose.orientation)
                    - 60. * sample.t)
                    .abs()
                    < 1e-5
            );
        }
        assert!((result.selected.joints[5] - 60.).abs() < 1e-6);
    }
}

#[test]
fn mechanical_zero_and_out_of_limit_starts_never_produce_a_success() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        let mut request = request(&model);
        request.start_joints = [0.; 6];
        assert_eq!(
            failure_code(plan_move_l(&request)),
            FailureCode::SingularStart
        );
        request.start_joints = [0., 120., 0., 0., 40., 0.];
        assert_eq!(failure_code(plan_move_l(&request)), FailureCode::JointLimit);
    }
}

#[test]
fn rejects_non_finite_coordinates_bad_quaternions_and_invalid_speeds() {
    let model = get_robot_model(MODELS[0]).unwrap();
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut request = request(&model);
        request.target.position[0] = value;
        assert_eq!(
            failure_code(plan_move_l(&request)),
            FailureCode::InvalidInput
        );
    }
    for q in [[0.; 4], [2., 0., 0., 0.], [f64::NAN, 0., 0., 0.]] {
        let mut request = request(&model);
        request.target.orientation = Some(q);
        assert_eq!(
            failure_code(plan_move_l(&request)),
            FailureCode::InvalidInput
        );
    }
    for speed in [0., 0.5, -1., 2001., f64::INFINITY, f64::NAN] {
        let mut request = request(&model);
        request.speed_mm_per_second = speed;
        assert_eq!(
            failure_code(plan_move_l(&request)),
            FailureCode::InvalidInput
        );
    }
    let mut request = request(&model);
    request.model_id = "not-a-registered-robot".into();
    assert_eq!(
        failure_code(plan_move_l(&request)),
        FailureCode::InvalidInput
    );
}

#[test]
fn extreme_finite_positions_are_bounded_before_automatic_orientation_arithmetic() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        for position in [
            [2000., 0., 500.],
            [f64::MAX, f64::MAX, f64::MAX],
            [-f64::MAX; 3],
        ] {
            let mut request = request(&model);
            request.target.position = position;
            request.target.orientation = None;
            assert_eq!(
                failure_code(plan_move_l(&request)),
                FailureCode::Unreachable
            );
        }
        // The original nominal 580/475mm radius is not a valid sphere around
        // the world origin: this valid TCP is farther away than that radius.
        let result = success(plan_move_l(&request(&model)));
        assert!(norm(&result.selected.pose.position) > model.reach_mm);
    }
}

#[test]
fn complete_axis_six_turns_are_not_silently_unwound() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        let mut request = request(&model);
        request.start_joints[5] = 360.;
        let start = forward_kinematics(&model, &request.start_joints)
            .unwrap()
            .pose;
        request.target = Target {
            position: [start.position[0] - 10., 10., start.position[2]],
            orientation: Some(start.orientation),
        };
        let result = success(plan_move_l(&request));
        assert!(result.selected.joints[5] > 300.);
        assert!(result.selected.configuration[2] >= 3);
    }
}

#[test]
fn reachable_endpoints_do_not_imply_reachable_linear_motion() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        let mut request = request(&model);
        let target = forward_kinematics(&model, &[180., 20., -30., 0., 40., 0.])
            .unwrap()
            .pose;
        assert!(!inverse_kinematics(&model, &target, &request.start_joints).is_empty());
        request.target = Target {
            position: target.position,
            orientation: Some(target.orientation),
        };
        assert!(matches!(
            failure_code(plan_move_l(&request)),
            FailureCode::PathUnreachable
                | FailureCode::SingularPath
                | FailureCode::ConfigurationChange
        ));
    }
}

#[test]
fn cancellation_before_start_and_during_sampling_never_returns_partial_success() {
    let model = get_robot_model(MODELS[0]).unwrap();
    let request = request(&model);
    assert_eq!(
        failure_code(plan_move_l_with_cancel(&request, &AtomicBool::new(true))),
        FailureCode::Cancelled
    );
    let mut checks = 0;
    let cancelled = plan_move_l_cancellable(&request, || {
        checks += 1;
        checks >= 8
    });
    assert_eq!(failure_code(cancelled), FailureCode::Cancelled);
    assert_eq!(checks, 8);
    let resumed = success(plan_move_l_with_cancel(&request, &AtomicBool::new(false)));
    assert!(!resumed.path.is_empty());
}

#[test]
fn native_result_json_matches_the_existing_frontend_contract() {
    let model = get_robot_model(MODELS[0]).unwrap();
    let json = serde_json::to_value(plan_move_l(&request(&model))).unwrap();
    assert_eq!(json["status"], "success");
    assert!(json["lengthMm"].is_number());
    assert!(json["automaticOrientation"].is_boolean());
    assert!(json["selected"]["configuration"].is_array());
    assert!(json["path"][0]["pose"]["orientation"].is_array());
    let failed = serde_json::to_value(plan_move_l_with_cancel(
        &request(&model),
        &AtomicBool::new(true),
    ))
    .unwrap();
    assert_eq!(failed["status"], "failure");
    assert_eq!(failed["code"], "cancelled");
    assert!(failed.get("path").is_none());
    assert!(failed.get("selected").is_none());
}
