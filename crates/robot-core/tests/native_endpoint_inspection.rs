use emulation_core::configuration::get_configuration;
use emulation_core::endpoint_inspection::{
    inspect_endpoint, inspect_endpoint_cancellable, inspect_endpoint_with_cancel,
    EndpointInspectionRequest,
};
use emulation_core::kinematics::{forward_kinematics, inverse_kinematics, joints_within_limits};
use emulation_core::model::get_robot_model;
use std::sync::atomic::AtomicBool;

fn request(model_id: &str) -> EndpointInspectionRequest {
    let model = get_robot_model(model_id).unwrap();
    EndpointInspectionRequest {
        model_id: model_id.into(),
        pose: forward_kinematics(&model, &model.ready_joints)
            .unwrap()
            .pose,
        seed_joints: model.ready_joints,
    }
}

#[test]
fn endpoint_inspection_keeps_all_stable_ik_branches_and_turns_without_a_path_claim() {
    for id in ["irb1100-4-0.475", "irb1100-4-0.58"] {
        let input = request(id);
        let model = get_robot_model(id).unwrap();
        let solutions = inspect_endpoint(&input).unwrap();
        assert!(solutions.len() > 8);
        assert_eq!(
            solutions,
            inverse_kinematics(&model, &input.pose, &input.seed_joints)
        );
        for solution in &solutions {
            assert!(joints_within_limits(&model, &solution.joints));
            let fk = forward_kinematics(&model, &solution.joints).unwrap();
            assert_eq!(
                get_configuration(&model, &solution.joints, &fk).configuration,
                Some(solution.configuration)
            );
        }
        let json = serde_json::to_value(solutions).unwrap();
        assert!(json.is_array());
        assert!(json[0].get("path").is_none());
        assert!(json[0].get("status").is_none());
    }
}

#[test]
fn valid_unreachable_pose_returns_no_alternatives_and_scaled_unit_input_is_not_mutated() {
    let mut input = request("irb1100-4-0.58");
    input.pose.orientation = input.pose.orientation.map(|component| component * 1.01);
    let original = input.clone();
    assert!(!inspect_endpoint(&input).unwrap().is_empty());
    assert_eq!(input, original);
    input.pose.position = [99999.; 3];
    assert!(inspect_endpoint(&input).unwrap().is_empty());
}

#[test]
fn malformed_or_unsupported_endpoint_requests_are_errors_not_empty_successes() {
    let original = request("irb1100-4-0.58");
    let mut input = original.clone();
    input.model_id = "yaskawa-gp7".into();
    assert!(inspect_endpoint(&input).is_err());
    for coordinate in [f64::NAN, f64::INFINITY, 1_000_000.0001] {
        let mut input = original.clone();
        input.pose.position[0] = coordinate;
        assert!(inspect_endpoint(&input).is_err());
    }
    for orientation in [[0.; 4], [2., 0., 0., 0.], [f64::NAN, 0., 0., 1.]] {
        let mut input = original.clone();
        input.pose.orientation = orientation;
        assert!(inspect_endpoint(&input).is_err());
    }
    for angle in [231., f64::NAN, f64::INFINITY] {
        let mut input = original.clone();
        input.seed_joints[0] = angle;
        assert!(inspect_endpoint(&input).is_err());
    }
    let mut json = serde_json::to_value(&original).unwrap();
    json["success"] = true.into();
    assert!(serde_json::from_value::<EndpointInspectionRequest>(json).is_err());
}

#[test]
fn cancelling_during_candidate_generation_never_returns_partial_alternatives() {
    let input = request("irb1100-4-0.58");
    assert!(inspect_endpoint_with_cancel(&input, &AtomicBool::new(true)).is_err());
    let mut polls = 0;
    let result = inspect_endpoint_cancellable(&input, || {
        polls += 1;
        polls >= 12
    });
    assert!(result.is_err());
    assert!(polls >= 12);
    assert!(!inspect_endpoint(&input).unwrap().is_empty());
}
