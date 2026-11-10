use emulation_core::{
    generic_endpoint_inspection::{
        inspect_generic_endpoint, inspect_generic_endpoint_cancellable, GenericEndpointRequest,
    },
    generic_kinematics::{forward_generic_kinematics, GenericChain},
    generic_pose_planner::{
        identity_pose, plan_generic_pose_move_l, plan_generic_pose_move_l_cancellable,
        GenericPosePlanRequest, GenericPosePlanResult,
    },
    math::{
        matrix_to_quaternion, multiply_matrices, norm, quaternion_distance_deg,
        quaternion_to_matrix, sub, translation,
    },
    types::{FailureCode, JointAngles, Pose, Target},
};
use std::sync::atomic::AtomicBool;
fn chains() -> Vec<GenericChain> {
    serde_json::from_str(include_str!(
        "../../../tests/fixtures/catalog/generic-chains.json"
    ))
    .unwrap()
}
fn seed() -> JointAngles {
    [20., 15., -25., 25., 35., 10.]
}
fn compose(a: Pose, b: Pose) -> Pose {
    let mut left = quaternion_to_matrix(&a.orientation).unwrap();
    let mut right = quaternion_to_matrix(&b.orientation).unwrap();
    for i in 0..3 {
        left[i * 4 + 3] = a.position[i];
        right[i * 4 + 3] = b.position[i];
    }
    let m = multiply_matrices(&left, &right);
    Pose {
        position: translation(&m),
        orientation: matrix_to_quaternion(&m),
    }
}
fn request(chain: GenericChain) -> GenericPosePlanRequest {
    let base = Pose {
        position: [350., -180., 220.],
        orientation: [0.4_f64.cos(), 0., 0., 0.4_f64.sin()],
    };
    let tool = Pose {
        position: [120., 35., 170.],
        orientation: [0.15_f64.cos(), 0.15_f64.sin(), 0., 0.],
    };
    let target_joints = std::array::from_fn(|i| seed()[i] + [0.4, -0.3, 0.5, 0.4, -0.2, 0.6][i]);
    let target = compose(
        compose(
            base,
            forward_generic_kinematics(&chain, &target_joints)
                .unwrap()
                .pose,
        ),
        tool,
    );
    GenericPosePlanRequest {
        chain,
        start_joints: seed(),
        target: Target {
            position: target.position,
            orientation: Some(target.orientation),
        },
        base_pose: base,
        tool_pose: tool,
        speed_mm_per_second: 100.,
    }
}
#[test]
fn real_model_tcp_pose_plans_and_fixed_tcp_rotation_include_base_and_tool() {
    for chain in chains() {
        let mut input = request(chain);
        for rotation in [false, true] {
            let start = compose(
                compose(
                    input.base_pose,
                    forward_generic_kinematics(&input.chain, &seed())
                        .unwrap()
                        .pose,
                ),
                input.tool_pose,
            );
            if rotation {
                input.target = Target {
                    position: start.position,
                    orientation: Some(
                        compose(
                            start,
                            Pose {
                                position: [0.; 3],
                                orientation: [0.025_f64.cos(), 0., 0., 0.025_f64.sin()],
                            },
                        )
                        .orientation,
                    ),
                };
            }
            let result = plan_generic_pose_move_l(&input, &AtomicBool::new(false));
            let GenericPosePlanResult::Success(result) = result else {
                panic!("{} {result:?}", input.chain.model_id)
            };
            assert_eq!(result.path[0].joints, seed());
            for pair in result.path.windows(2) {
                let sample = &pair[1];
                let expected: [f64; 3] = std::array::from_fn(|i| {
                    start.position[i] + (input.target.position[i] - start.position[i]) * sample.t
                });
                assert!(norm(&sub(&sample.pose.position, &expected)) <= 0.001001);
                assert!(sample
                    .joints
                    .iter()
                    .zip(pair[0].joints)
                    .all(|(a, b)| (a - b).abs() <= 15.00000001));
                let world = compose(
                    compose(
                        input.base_pose,
                        forward_generic_kinematics(&input.chain, &sample.joints)
                            .unwrap()
                            .pose,
                    ),
                    input.tool_pose,
                );
                assert!(norm(&sub(&world.position, &sample.pose.position)) < 1e-7);
            }
            assert_eq!(result.selected.pose, result.path.last().unwrap().pose);
            assert!(
                quaternion_distance_deg(
                    &result.selected.pose.orientation,
                    &input.target.orientation.unwrap()
                ) <= 0.00058
            );
        }
    }
}
#[test]
fn endpoint_candidates_are_independent_bounded_deterministic_and_configuration_free() {
    for chain in chains() {
        let request = GenericEndpointRequest {
            pose: forward_generic_kinematics(&chain, &seed()).unwrap().pose,
            chain,
            seed_joints: seed(),
        };
        let solutions = inspect_generic_endpoint(&request).unwrap();
        assert!(
            solutions.len() > 1 && solutions.len() <= 32,
            "{} {}",
            request.chain.model_id,
            solutions.len()
        );
        assert_eq!(solutions[0].joints, seed());
        assert_eq!(inspect_generic_endpoint(&request).unwrap(), solutions);
        for (index, candidate) in solutions.iter().enumerate() {
            assert!(candidate.position_error_mm <= 0.001001);
            assert!(candidate.orientation_error_deg <= 0.00058);
            assert!(!solutions[..index].iter().any(|other| other
                .joints
                .iter()
                .zip(candidate.joints)
                .all(|(a, b)| (a - b).abs() <= 0.001)));
        }
    }
}
#[test]
fn position_mode_defaults_frames_and_cancellation_never_returns_partial_results() {
    for chain in chains() {
        let start = forward_generic_kinematics(&chain, &seed()).unwrap().pose;
        let mut input = request(chain);
        input.base_pose = identity_pose();
        input.tool_pose = identity_pose();
        input.target = Target {
            position: [start.position[0] + 1., start.position[1], start.position[2]],
            orientation: None,
        };
        let GenericPosePlanResult::Success(result) =
            plan_generic_pose_move_l(&input, &AtomicBool::new(false))
        else {
            panic!("position mode {}", input.chain.model_id)
        };
        assert!(result.automatic_orientation);
        let mut polls = 0;
        assert!(
            matches!(plan_generic_pose_move_l_cancellable(&input, || { polls += 1; polls > 20 }), GenericPosePlanResult::Failure(failure) if failure.code == FailureCode::Cancelled)
        );
        let endpoint = GenericEndpointRequest {
            chain: input.chain,
            pose: start,
            seed_joints: seed(),
        };
        polls = 0;
        assert!(inspect_generic_endpoint_cancellable(&endpoint, || {
            polls += 1;
            polls > 100
        })
        .unwrap_err()
        .contains("取消"));
    }
}
