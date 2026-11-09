use emulation_core::configuration::get_configuration;
use emulation_core::kinematics::{forward_kinematics, inverse_kinematics, joints_within_limits};
use emulation_core::math::{
    normalize_orientation, normalize_quaternion, quaternion_distance_deg, quaternion_to_matrix,
    slerp_quaternion,
};
use emulation_core::model::robot_models;
use emulation_core::{JointAngles, Pose};

#[test]
fn recovers_two_hundred_deterministic_full_range_joint_samples() {
    for model in robot_models() {
        let mut random = 287431_u32;
        let mut checked = 0;
        for _ in 0..100 {
            let joints: JointAngles = std::array::from_fn(|i| {
                random = random.wrapping_mul(1664525).wrapping_add(1013904223);
                let fraction = f64::from(random) / 4294967296.;
                let l = model.limits[i];
                l.min_deg + 2. + fraction * (l.max_deg - l.min_deg - 4.)
            });
            let fk = forward_kinematics(&model, &joints).unwrap();
            if get_configuration(&model, &joints, &fk).singular {
                continue;
            }
            let solutions = inverse_kinematics(&model, &fk.pose, &joints);
            assert!(!solutions.is_empty(), "missing {:?}", joints);
            for (a, b) in solutions[0].joints.iter().zip(joints) {
                assert!(
                    (a - b).abs() < 1e-5,
                    "wrong branch {joints:?}: {:?}",
                    solutions[0].joints
                );
            }
            for solution in &solutions {
                assert!(joints_within_limits(&model, &solution.joints));
                assert!(solution.position_error_mm < 1e-6);
                assert!(solution.orientation_error_deg < 1e-4);
                let verified = forward_kinematics(&model, &solution.joints).unwrap();
                assert_eq!(
                    get_configuration(&model, &solution.joints, &verified).configuration,
                    Some(solution.configuration)
                );
            }
            checked += 1;
        }
        assert!(checked > 95);
    }
}

#[test]
fn retains_wrist_elbow_alternatives_and_full_a6_turns() {
    for model in robot_models() {
        let seed = [20., 20., -40., 25., 40., 370.];
        let pose = forward_kinematics(&model, &seed).unwrap().pose;
        let solutions = inverse_kinematics(&model, &pose, &seed);
        assert!((solutions[0].joints[5] - 370.).abs() < 1e-8);
        assert!(solutions.iter().any(|s| (s.joints[5] - 10.).abs() < 1e-7));
        let branches: std::collections::HashSet<_> =
            solutions.iter().map(|s| s.configuration[3]).collect();
        assert!(branches.len() >= 2);
        let keys: std::collections::HashSet<_> = solutions.iter().map(|s| s.id.as_str()).collect();
        assert_eq!(keys.len(), solutions.len());
    }
}

#[test]
fn rejects_non_finite_inputs_and_unreachable_poses_without_fabricated_joints() {
    for model in robot_models() {
        for pose in [
            Pose {
                position: [5000., 0., 300.],
                orientation: [1., 0., 0., 0.],
            },
            Pose {
                position: [400., 0., 500.],
                orientation: [0.; 4],
            },
            Pose {
                position: [f64::NAN, 0., 500.],
                orientation: [1., 0., 0., 0.],
            },
            Pose {
                position: [f64::MAX; 3],
                orientation: [1., 0., 0., 0.],
            },
        ] {
            assert!(inverse_kinematics(&model, &pose, &model.ready_joints).is_empty());
        }
        assert!(forward_kinematics(&model, &[f64::INFINITY; 6]).is_err());
        assert!(!joints_within_limits(&model, &[f64::NAN; 6]));
        assert!(!joints_within_limits(&model, &[0., 120., 0., 0., 0., 0.]));
    }
}

#[test]
fn equivalent_quaternion_signs_and_scales_give_identical_ik() {
    for model in robot_models() {
        let pose = forward_kinematics(&model, &model.ready_joints)
            .unwrap()
            .pose;
        let scaled = Pose {
            position: pose.position,
            orientation: pose.orientation.map(|q| q * -8.),
        };
        let a = inverse_kinematics(&model, &pose, &model.ready_joints);
        let b = inverse_kinematics(&model, &scaled, &model.ready_joints);
        assert_eq!(a.len(), b.len());
        for (qa, qb) in a[0].joints.iter().zip(b[0].joints) {
            assert!((qa - qb).abs() < 1e-8);
        }
    }
}

#[test]
fn scalar_first_math_follows_shortest_orientation_arc() {
    let sqrt = std::f64::consts::FRAC_1_SQRT_2;
    let huge = normalize_quaternion(&[1e308, 0., 1e308, 0.]).unwrap();
    assert!((huge[0] - sqrt).abs() < 1e-14);
    assert!((huge[2] - sqrt).abs() < 1e-14);
    let a = [1., 0., 0., 0.];
    let b = [-sqrt, 0., 0., -sqrt];
    let mid = slerp_quaternion(&a, &b, 0.5);
    assert!((quaternion_distance_deg(&a, &mid) - 45.).abs() < 1e-8);
    assert!((quaternion_distance_deg(&mid, &b) - 45.).abs() < 1e-8);
    assert_eq!(
        quaternion_distance_deg(&a, &slerp_quaternion(&a, &[-1., 0., 0., 0.], 0.5)),
        0.
    );
    let rot = quaternion_to_matrix(&[sqrt, 0., sqrt, 0.]).unwrap();
    for (i, v) in [(0, 0.), (2, 1.), (5, 1.), (8, -1.), (10, 0.)] {
        assert!((rot[i] - v).abs() < 1e-14);
    }
}

#[test]
fn public_orientation_validation_only_normalizes_abb_permitted_error() {
    assert!(normalize_orientation(&[0.707, 0., 0.707, 0.]).is_ok());
    for q in [
        [0.; 4],
        [1., 1., 0., 0.],
        [2., 0., 0., 0.],
        [f64::INFINITY, 0., 0., 0.],
        [f64::NAN, 0., 0., 0.],
    ] {
        assert!(normalize_orientation(&q).is_err());
    }
}
