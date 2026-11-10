use emulation_core::generic_kinematics::{
    forward_generic_kinematics, inverse_generic_kinematics, inverse_generic_kinematics_cancellable,
    validate_generic_chain, GenericChain,
};
use emulation_core::generic_planner::{
    plan_generic_move_l, plan_generic_move_l_cancellable, GenericPlanRequest, GenericPlanResult,
};
use emulation_core::math::{
    cross, multiply_matrices, norm, quaternion_distance_deg, quaternion_to_matrix,
    slerp_quaternion, sub, IDENTITY,
};
use emulation_core::types::{FailureCode, JointAngles, Pose};
use std::sync::atomic::AtomicBool;

fn chains() -> Vec<GenericChain> {
    serde_json::from_str(include_str!(
        "../../../tests/fixtures/catalog/generic-chains.json"
    ))
    .unwrap()
}
fn ready() -> JointAngles {
    [10., 20., -25., 15., 30., -10.]
}
fn near_target() -> JointAngles {
    [13., 18., -23., 17., 31., -12.]
}
fn request(chain: GenericChain) -> GenericPlanRequest {
    GenericPlanRequest {
        chain,
        start_joints: ready(),
        target_joints: near_target(),
        speed_mm_per_second: 100.,
    }
}

#[test]
fn actual_catalog_zero_frames_and_joint_one_signs_match_urdf_anchors() {
    let fixtures = chains();
    assert_eq!(fixtures.len(), 4);
    for chain in fixtures {
        validate_generic_chain(&chain).unwrap();
        let (expected, sign) = match chain.model_id.as_str() {
            "fanuc-crx10ia-l" => ([719.5, -150., 955.], 1.),
            "fanuc-r2000ib-165f" => ([1847.5, 0., 1970.], -1.),
            "yaskawa-gp7" => ([560., 0., 815.], 1.),
            "yaskawa-gp110" => ([1540., 0., 1645.], 1.),
            other => panic!("unexpected fixture {other}"),
        };
        let zero = forward_generic_kinematics(&chain, &[0.; 6]).unwrap();
        assert!(
            norm(&sub(&zero.pose.position, &expected)) < 1e-7,
            "{}",
            chain.model_id
        );
        let rotated = forward_generic_kinematics(&chain, &[90., 0., 0., 0., 0., 0.]).unwrap();
        assert!(
            norm(&sub(
                &rotated.pose.position,
                &[-sign * expected[1], sign * expected[0], expected[2]]
            )) < 1e-7
        );
        let orientation = if chain.model_id.starts_with("fanuc") {
            [(1.57_f64 / 2.).cos(), 0., (1.57_f64 / 2.).sin(), 0.]
        } else {
            [
                0.,
                std::f64::consts::FRAC_1_SQRT_2,
                0.,
                std::f64::consts::FRAC_1_SQRT_2,
            ]
        };
        assert!(quaternion_distance_deg(&zero.pose.orientation, &orientation) < 1e-5);
    }
}

#[test]
fn geometric_jacobian_matches_independent_finite_differences_on_all_real_chains() {
    let h = 1e-4_f64;
    for chain in chains() {
        let q = ready();
        let center = forward_generic_kinematics(&chain, &q).unwrap();
        for axis in 0..6 {
            let mut low = q;
            low[axis] -= h;
            let mut high = q;
            high[axis] += h;
            let a = forward_generic_kinematics(&chain, &low).unwrap();
            let b = forward_generic_kinematics(&chain, &high).unwrap();
            let numerical =
                sub(&b.pose.position, &a.pose.position).map(|v| v / (2. * h.to_radians()));
            let analytic = cross(
                &center.joint_axes[axis],
                &sub(&center.pose.position, &center.joint_origins[axis]),
            );
            assert!(
                norm(&sub(&numerical, &analytic)) < 1e-5,
                "{} axis{} translation",
                chain.model_id,
                axis
            );
            let ra = quaternion_to_matrix(&a.pose.orientation).unwrap();
            let rb = quaternion_to_matrix(&b.pose.orientation).unwrap();
            let mut inverse = IDENTITY;
            for row in 0..3 {
                for col in 0..3 {
                    inverse[row * 4 + col] = ra[col * 4 + row];
                }
            }
            let delta = multiply_matrices(&rb, &inverse);
            let angular = [
                delta[9] - delta[6],
                delta[2] - delta[8],
                delta[4] - delta[1],
            ]
            .map(|v| v / (4. * h.to_radians()));
            assert!(
                norm(&sub(&angular, &center.joint_axes[axis])) < 1e-7,
                "{} axis{} rotation",
                chain.model_id,
                axis
            );
        }
    }
}

#[test]
fn real_catalog_paths_follow_cartesian_line_and_shortest_rotation_with_actual_fk() {
    let idle = AtomicBool::new(false);
    for chain in chains() {
        let input = request(chain);
        let GenericPlanResult::Success(plan) = plan_generic_move_l(&input, &idle) else {
            panic!(
                "{} nearby path failed: {:?}",
                input.chain.model_id,
                plan_generic_move_l(&input, &idle)
            );
        };
        let start = forward_generic_kinematics(&input.chain, &input.start_joints)
            .unwrap()
            .pose;
        let end = forward_generic_kinematics(&input.chain, &input.target_joints)
            .unwrap()
            .pose;
        assert_eq!(plan.path.first().unwrap().joints, input.start_joints);
        assert_eq!(plan.path.last().unwrap().joints, input.target_joints);
        assert!(plan.path.len() >= 3 && plan.path.len() <= 1601);
        let mut differs_from_joint_lerp = false;
        for sample in &plan.path {
            let fk = forward_generic_kinematics(&input.chain, &sample.joints).unwrap();
            assert!(norm(&sub(&fk.pose.position, &sample.pose.position)) < 1e-9);
            let expected = std::array::from_fn(|axis| {
                start.position[axis] + (end.position[axis] - start.position[axis]) * sample.t
            });
            assert!(
                norm(&sub(&sample.pose.position, &expected)) <= 0.001001,
                "{} t{}",
                input.chain.model_id,
                sample.t
            );
            let orientation = slerp_quaternion(&start.orientation, &end.orientation, sample.t);
            assert!(quaternion_distance_deg(&sample.pose.orientation, &orientation) < 0.00058);
            differs_from_joint_lerp |= (0..6).any(|axis| {
                (sample.joints[axis]
                    - (input.start_joints[axis]
                        + (input.target_joints[axis] - input.start_joints[axis]) * sample.t))
                    .abs()
                    > 1e-3
            });
        }
        assert!(
            differs_from_joint_lerp,
            "{} must solve the Cartesian line, not lerp joint angles",
            input.chain.model_id
        );
        for pair in plan.path.windows(2) {
            for axis in 0..6 {
                let delta = (pair[1].joints[axis] - pair[0].joints[axis]).abs();
                assert!(delta <= 15.00000001);
                assert!(
                    delta / ((pair[1].t - pair[0].t) * plan.duration_seconds)
                        <= input.chain.limits[axis].max_speed_deg + 1e-8
                );
            }
        }
    }
}

#[test]
fn real_inverse_rejects_unreachable_targets_and_preserves_valid_nearby_pose() {
    let idle = AtomicBool::new(false);
    for chain in chains() {
        let target = forward_generic_kinematics(&chain, &near_target())
            .unwrap()
            .pose;
        let solved = inverse_generic_kinematics(&chain, &target, &ready(), &idle)
            .unwrap()
            .unwrap();
        let actual = forward_generic_kinematics(&chain, &solved).unwrap().pose;
        assert!(norm(&sub(&actual.position, &target.position)) <= 0.001001);
        assert!(quaternion_distance_deg(&actual.orientation, &target.orientation) < 0.00058);
        let far = Pose {
            position: [999_000.; 3],
            orientation: target.orientation,
        };
        assert!(inverse_generic_kinematics(&chain, &far, &ready(), &idle)
            .unwrap()
            .is_none());
    }
}

#[test]
fn same_tcp_on_another_full_turn_cannot_silently_replace_the_recorded_branch() {
    let idle = AtomicBool::new(false);
    for chain in chains() {
        let mut input = request(chain);
        input.start_joints[5] = -170.;
        input.target_joints = input.start_joints;
        input.target_joints[5] = 190.;
        let GenericPlanResult::Failure(failure) = plan_generic_move_l(&input, &idle) else {
            panic!("full-turn branch should be rejected");
        };
        assert_eq!(failure.code, FailureCode::ConfigurationChange);
    }
}

#[test]
fn malformed_chains_joint_limits_and_sampling_bounds_fail_without_partial_paths() {
    let idle = AtomicBool::new(false);
    let original = chains().remove(0);
    let mut invalid = original.clone();
    invalid.steps[1].joint_index = invalid.steps[0].joint_index;
    assert!(validate_generic_chain(&invalid).is_err());
    let mut invalid = original.clone();
    invalid.steps[0].origin[0] = -1.;
    assert!(validate_generic_chain(&invalid).is_err());
    let mut invalid = original.clone();
    invalid.steps[0].axis = [0.; 3];
    assert!(validate_generic_chain(&invalid).is_err());
    let mut invalid = original.clone();
    invalid.limits[0].max_speed_deg = 0.;
    assert!(validate_generic_chain(&invalid).is_err());
    let mut input = request(original.clone());
    input.target_joints[0] = input.chain.limits[0].max_deg + 1.;
    assert!(
        matches!(plan_generic_move_l(&input, &idle), GenericPlanResult::Failure(f) if f.code == FailureCode::JointLimit)
    );
    let mut input = request(original);
    input.speed_mm_per_second = 2001.;
    assert!(
        matches!(plan_generic_move_l(&input, &idle), GenericPlanResult::Failure(f) if f.code == FailureCode::InvalidInput)
    );
    let mut long = request(
        chains()
            .into_iter()
            .find(|c| c.model_id == "yaskawa-gp110")
            .unwrap(),
    );
    for step in &mut long.chain.steps {
        for axis in [3, 7, 11] {
            step.origin[axis] *= 50.;
        }
    }
    long.start_joints[0] = -60.;
    long.target_joints = long.start_joints;
    long.target_joints[0] = 60.;
    assert!(
        matches!(plan_generic_move_l(&long, &idle), GenericPlanResult::Failure(f) if f.code == FailureCode::InvalidInput)
    );
}

#[test]
fn cancellation_inside_fk_and_iteration_never_returns_a_partial_plan_or_inverse() {
    let chain = chains().remove(0);
    let input = request(chain.clone());
    let mut calls = 0;
    let result = plan_generic_move_l_cancellable(&input, || {
        calls += 1;
        calls >= 60
    });
    assert!(matches!(result, GenericPlanResult::Failure(f) if f.code == FailureCode::Cancelled));
    assert_eq!(calls, 60);
    let far = Pose {
        position: [999_000.; 3],
        orientation: [1., 0., 0., 0.],
    };
    let mut calls = 0;
    let result = inverse_generic_kinematics_cancellable(&chain, &far, &ready(), || {
        calls += 1;
        calls >= 20
    });
    assert!(result.unwrap_err().contains("取消"));
    assert_eq!(calls, 20);
}
