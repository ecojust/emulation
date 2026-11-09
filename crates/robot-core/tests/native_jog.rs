//! Jogging is a chain of accepted Cartesian segments, not repeated IK from
//! mechanical zero or from a target the planner has already rejected.
use emulation_core::configuration::get_configuration;
use emulation_core::kinematics::{forward_kinematics, joints_within_limits};
use emulation_core::math::{norm, quaternion_distance_deg, sub};
use emulation_core::model::get_robot_model;
use emulation_core::planner::plan_move_l;
use emulation_core::{
    FailureCode, JointAngles, PlanRequest, PlanResult, PlanSuccess, Pose, Quaternion, RobotModel,
    Target,
};

const MODELS: [&str; 2] = ["irb1100-4-0.58", "irb1100-4-0.475"];

fn segment(model: &RobotModel, accepted: JointAngles, target: Pose) -> PlanResult {
    plan_move_l(&PlanRequest {
        model_id: model.id.clone(),
        start_joints: accepted,
        target: Target {
            position: target.position,
            orientation: Some(target.orientation),
        },
        speed_mm_per_second: 100.,
    })
}

fn accepted(result: PlanResult) -> PlanSuccess {
    match result {
        PlanResult::Success(result) => result,
        PlanResult::Failure(error) => panic!("Incremental jog failed: {error:?}"),
    }
}

// World Z rotation is pre-multiplied; public quaternion order is [w,x,y,z].
fn rotate_world_z(q: Quaternion, degrees: f64) -> Quaternion {
    let (s, c) = (degrees.to_radians() * 0.5).sin_cos();
    [
        c * q[0] - s * q[3],
        c * q[1] - s * q[2],
        c * q[2] + s * q[1],
        c * q[3] + s * q[0],
    ]
}

#[test]
fn one_mm_world_x_jog_requires_a_nonsingular_start_on_both_models() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        let mut zero_target = forward_kinematics(&model, &model.home_joints).unwrap().pose;
        zero_target.position[0] += 1.;
        let PlanResult::Failure(error) = segment(&model, model.home_joints, zero_target) else {
            panic!("A singular mechanical zero must not become a valid jog start");
        };
        assert_eq!(error.code, FailureCode::SingularStart);

        let mut ready_target = forward_kinematics(&model, &model.ready_joints)
            .unwrap()
            .pose;
        ready_target.position[0] += 1.;
        let result = accepted(segment(&model, model.ready_joints, ready_target));
        assert!(norm(&sub(&result.selected.pose.position, &ready_target.position)) < 1e-6);
        assert!(
            quaternion_distance_deg(&result.selected.pose.orientation, &ready_target.orientation)
                < 1e-4
        );
        assert!(!result.automatic_orientation);
    }
}

#[test]
fn repeated_small_translations_use_the_previous_accepted_joints_and_keep_configuration() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        let initial = forward_kinematics(&model, &model.ready_joints).unwrap();
        let cfx = get_configuration(&model, &model.ready_joints, &initial)
            .configuration
            .unwrap()[3];
        let mut joints = model.ready_joints;
        let mut target = initial.pose;
        // A closed 5mm square, delivered in 1mm samples as a gizmo might do.
        for (axis, direction) in [(0, 1.), (1, 1.), (0, -1.), (1, -1.)] {
            for _ in 0..5 {
                target.position[axis] += direction;
                let result = accepted(segment(&model, joints, target));
                assert_eq!(result.path[0].joints, joints);
                assert_eq!(result.selected.configuration[3], cfx);
                assert!(joints_within_limits(&model, &result.selected.joints));
                assert!(norm(&sub(&result.selected.pose.position, &target.position)) < 1e-6);
                assert!(
                    quaternion_distance_deg(
                        &result.selected.pose.orientation,
                        &initial.pose.orientation
                    ) < 1e-4
                );
                joints = result.selected.joints;
            }
        }
        let returned = forward_kinematics(&model, &joints).unwrap();
        assert!(norm(&sub(&returned.pose.position, &initial.pose.position)) < 1e-6);
        for (actual, expected) in joints.iter().zip(model.ready_joints) {
            assert!((actual - expected).abs() < 1e-6);
        }
    }
}

#[test]
fn repeated_world_axis_rotations_keep_tcp_fixed_and_return_without_a_branch_jump() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        let initial = forward_kinematics(&model, &model.ready_joints).unwrap();
        let cfx = get_configuration(&model, &model.ready_joints, &initial)
            .configuration
            .unwrap()[3];
        let mut joints = model.ready_joints;
        for angle in (1..=10).chain((0..10).rev()) {
            let target = Pose {
                position: initial.pose.position,
                orientation: rotate_world_z(initial.pose.orientation, angle as f64),
            };
            let result = accepted(segment(&model, joints, target));
            assert_eq!(result.path[0].joints, joints);
            assert_eq!(result.selected.configuration[3], cfx);
            assert!(result.length_mm < 1e-6);
            assert!(result.orientation_travel_deg < 1.0001);
            assert!(norm(&sub(&result.selected.pose.position, &initial.pose.position)) < 1e-6);
            assert!(
                quaternion_distance_deg(&result.selected.pose.orientation, &target.orientation)
                    < 1e-4
            );
            assert!(result
                .selected
                .joints
                .iter()
                .zip(joints)
                .all(|(next, previous)| (next - previous).abs() < 5.));
            joints = result.selected.joints;
        }
        for (actual, expected) in joints.iter().zip(model.ready_joints) {
            assert!((actual - expected).abs() < 1e-6);
        }
    }
}

#[test]
fn repeated_unchanged_target_at_ready_does_not_drift_or_choose_another_branch() {
    for id in MODELS {
        let model = get_robot_model(id).unwrap();
        let pose = forward_kinematics(&model, &model.ready_joints)
            .unwrap()
            .pose;
        let mut joints = model.ready_joints;
        for _ in 0..5 {
            let result = accepted(segment(&model, joints, pose));
            assert!(result.length_mm < 1e-6);
            assert!(result.orientation_travel_deg < 1e-4);
            assert!(norm(&sub(&result.selected.pose.position, &pose.position)) < 1e-6);
            for (actual, expected) in result.selected.joints.iter().zip(model.ready_joints) {
                assert!((actual - expected).abs() < 1e-6);
            }
            joints = result.selected.joints;
        }
    }
}
