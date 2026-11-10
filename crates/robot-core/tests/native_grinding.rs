use emulation_core::collision::CollisionZone;
use emulation_core::generic_kinematics::{forward_generic_kinematics, GenericChain};
use emulation_core::grinding::*;
use emulation_core::kinematics::forward_kinematics;
use emulation_core::math::{
    matrix_to_quaternion, multiply_matrices, norm, quaternion_distance_deg, quaternion_to_matrix,
    sub,
};
use emulation_core::model::get_robot_model;
use emulation_core::types::{JointAngles, Pose};
use std::sync::atomic::AtomicBool;

const ID: Pose = Pose {
    position: [0.; 3],
    orientation: [1., 0., 0., 0.],
};
const READY: JointAngles = [0., 20., -30., 0., 40., 0.];
fn multiply(a: Pose, b: Pose) -> Pose {
    let mut am = quaternion_to_matrix(&a.orientation).unwrap();
    let mut bm = quaternion_to_matrix(&b.orientation).unwrap();
    for i in 0..3 {
        am[i * 4 + 3] = a.position[i];
        bm[i * 4 + 3] = b.position[i];
    }
    let m = multiply_matrices(&am, &bm);
    Pose {
        position: [m[3], m[7], m[11]],
        orientation: matrix_to_quaternion(&m),
    }
}
fn request() -> GrindingRequest {
    let model = get_robot_model("irb1100-4-0.58").unwrap();
    let tcp = forward_kinematics(&model, &READY).unwrap().pose;
    GrindingRequest {
        robot: GrindingRobot::Abb { model_id: model.id },
        base_pose: ID,
        tool_pose: ID,
        tool_radius_mm: 1.,
        start_joints: READY,
        targets: vec![
            Pose {
                position: [tcp.position[0], 4., tcp.position[2]],
                ..tcp
            },
            Pose {
                position: [tcp.position[0], 8., tcp.position[2]],
                ..tcp
            },
        ],
        orientation: Some(tcp.orientation),
        zones: vec![],
        clearance_mm: 1.,
        speed_mm_per_second: 100.,
        allow_detours: true,
    }
}
fn success(request: &GrindingRequest) -> GrindingSuccess {
    match plan_grinding(request, &AtomicBool::new(false)) {
        GrindingResult::Success(r) => r,
        other => panic!("expected success, got {other:?}"),
    }
}
fn failure(request: &GrindingRequest) -> GrindingFailure {
    match plan_grinding(request, &AtomicBool::new(false)) {
        GrindingResult::Failure(r) => r,
        other => panic!("expected failure, got {other:?}"),
    }
}
#[test]
fn fixed_world_tcp_orientation_and_original_targets_are_preserved() {
    let mut r = request();
    r.targets[1].orientation = r.targets[1].orientation.map(|x| -x);
    let before = r.clone();
    let plan = success(&r);
    assert_eq!(r, before);
    assert_eq!(plan.waypoints.len(), 2);
    assert_eq!(plan.path.first().unwrap().t, 0.);
    assert_eq!(plan.path.last().unwrap().t, 1.);
    for (i, w) in plan.waypoints.iter().enumerate() {
        assert_eq!(w.target_index, Some(i));
        assert_eq!(w.kind, if i == 0 { "transfer" } else { "process" });
        assert!(norm(&sub(&w.pose.position, &r.targets[i].position)) < 0.01);
        assert!(quaternion_distance_deg(&w.pose.orientation, &r.orientation.unwrap()) < 0.001);
    }
    for pair in plan.path.windows(2) {
        assert!(pair[0].t < pair[1].t);
        for j in 0..6 {
            assert!((pair[0].joints[j] - pair[1].joints[j]).abs() <= 15.);
        }
    }
}
#[test]
fn rotated_translated_base_and_rotated_offset_tool_use_tcp_not_flange_line() {
    let mut r = request();
    let half = std::f64::consts::FRAC_1_SQRT_2;
    r.base_pose = Pose {
        position: [700., -300., 120.],
        orientation: [half, 0., 0., half],
    };
    r.tool_pose = Pose {
        position: [45., 10., 70.],
        orientation: [half, half, 0., 0.],
    };
    let flange = forward_kinematics(&get_robot_model("irb1100-4-0.58").unwrap(), &READY)
        .unwrap()
        .pose;
    let start = multiply(multiply(r.base_pose, flange), r.tool_pose);
    r.orientation = Some(start.orientation);
    r.targets = vec![
        Pose {
            position: [start.position[0] + 4., start.position[1], start.position[2]],
            ..start
        },
        Pose {
            position: [start.position[0] + 8., start.position[1], start.position[2]],
            ..start
        },
    ];
    let plan = success(&r);
    for sample in &plan.path {
        let actual = multiply(
            multiply(
                r.base_pose,
                forward_kinematics(&get_robot_model("irb1100-4-0.58").unwrap(), &sample.joints)
                    .unwrap()
                    .pose,
            ),
            r.tool_pose,
        );
        assert!(norm(&sub(&actual.position, &sample.pose.position)) < 1e-8);
        assert!(quaternion_distance_deg(&actual.orientation, &start.orientation) < 0.001);
        assert!((sample.pose.position[1] - start.position[1]).abs() < 0.01);
        assert!((sample.pose.position[2] - start.position[2]).abs() < 0.01);
    }
}
#[test]
fn thin_obstacle_between_clear_endpoints_is_rejected_including_tool_body() {
    let mut r = request();
    r.tool_pose.position = [0., 0., 200.];
    r.tool_radius_mm = 0.02;
    r.clearance_mm = 0.;
    r.allow_detours = false;
    let start = multiply(
        forward_kinematics(&get_robot_model("irb1100-4-0.58").unwrap(), &READY)
            .unwrap()
            .pose,
        r.tool_pose,
    );
    r.targets = vec![Pose {
        position: [start.position[0], 8., start.position[2]],
        ..start
    }];
    r.orientation = Some(start.orientation);
    let free = success(&r);
    r.zones = vec![CollisionZone {
        id: "thin".into(),
        enabled: true,
        center_mm: [start.position[0], 1., start.position[2]],
        size_mm: [0.01, 0.01, 0.01],
    }];
    let mut stationary = r.clone();
    stationary.targets = vec![start];
    success(&stationary);
    stationary.start_joints = free.path.last().unwrap().joints;
    stationary.targets = r.targets.clone();
    success(&stationary);
    assert_eq!(failure(&r).code, "collision");
}
#[test]
fn fixed_direction_route_can_add_explicit_transfer_waypoints() {
    let mut r = request();
    r.tool_pose.position = [0., 0., 200.];
    r.tool_radius_mm = 0.02;
    r.clearance_mm = 0.;
    let start = multiply(
        forward_kinematics(&get_robot_model("irb1100-4-0.58").unwrap(), &READY)
            .unwrap()
            .pose,
        r.tool_pose,
    );
    r.targets = vec![Pose {
        position: [start.position[0], 8., start.position[2]],
        ..start
    }];
    r.orientation = Some(start.orientation);
    r.zones = vec![CollisionZone {
        id: "thin".into(),
        enabled: true,
        center_mm: [start.position[0], 1., start.position[2]],
        size_mm: [0.01, 0.01, 0.01],
    }];
    let plan = success(&r);
    assert!(plan
        .waypoints
        .iter()
        .any(|w| w.kind == "transfer" && w.target_index.is_none()));
    assert_eq!(plan.waypoints.last().unwrap().target_index, Some(0));
    assert!(plan.waypoints.iter().all(|w| w.kind == "transfer"));
    for w in &plan.waypoints {
        assert!(quaternion_distance_deg(&w.pose.orientation, &start.orientation) < 0.001);
    }
}
#[test]
fn approach_is_checked_and_world_obstacles_follow_base_transforms() {
    let mut r = request();
    r.base_pose.position = [2000., 0., 0.];
    for p in &mut r.targets {
        p.position[0] += 2000.;
    }
    r.zones = vec![CollisionZone {
        id: "base".into(),
        enabled: true,
        center_mm: [2000., 0., 50.],
        size_mm: [100., 100., 100.],
    }];
    assert_eq!(failure(&r).code, "collision");
    r.zones[0].center_mm = [0., 0., 50.];
    success(&r);
}
#[test]
fn invalid_inputs_singular_start_and_mid_work_cancellation_fail_closed() {
    let mut r = request();
    r.targets.clear();
    assert_eq!(failure(&r).code, "invalid_input");
    r = request();
    r.clearance_mm = f64::NAN;
    assert_eq!(failure(&r).code, "invalid_input");
    r = request();
    r.start_joints = [0.; 6];
    assert_eq!(failure(&r).code, "singular_path");
    r = request();
    r.targets[1].position = [10000.; 3];
    assert_eq!(failure(&r).code, "resource_limit");
    r = request();
    let mut calls = 0;
    let result = plan_grinding_cancellable(&r, || {
        calls += 1;
        calls > 20
    });
    assert!(
        matches!(result,GrindingResult::Failure(GrindingFailure{code,..}) if code=="cancelled")
    );
}
#[test]
fn generic_model_and_declared_geometry_do_not_use_abb_kinematics() {
    let chains: Vec<GenericChain> = serde_json::from_str(include_str!(
        "../../../tests/fixtures/catalog/generic-chains.json"
    ))
    .unwrap();
    for chain in chains {
        let q = [10., 20., -25., 15., 30., -10.];
        let start = forward_generic_kinematics(&chain, &q).unwrap().pose;
        let mut r = request();
        r.start_joints = q;
        r.orientation = Some(start.orientation);
        r.targets = vec![Pose {
            position: [start.position[0], start.position[1] + 2., start.position[2]],
            ..start
        }];
        r.robot = GrindingRobot::Generic {
            bodies: (0..=chain.steps.len())
                .map(|step_index| GrindingBody {
                    step_index,
                    center_mm: [0.; 3],
                    half_size_mm: [1.; 3],
                })
                .collect(),
            chain,
        };
        let plan = success(&r);
        assert!(plan.waypoints[0].configuration.is_none());
        assert!(
            norm(&sub(
                &plan.waypoints[0].pose.position,
                &r.targets[0].position
            )) < 0.01
        );
    }
}
#[test]
fn null_orientation_retains_per_target_direction_with_offset_tcp() {
    let mut r = request();
    r.orientation = None;
    r.allow_detours = false;
    r.tool_pose.position = [20., 0., 30.];
    let mut end = READY;
    end[0] = 2.;
    end[3] = 1.;
    let target = multiply(
        forward_kinematics(&get_robot_model("irb1100-4-0.58").unwrap(), &end)
            .unwrap()
            .pose,
        r.tool_pose,
    );
    r.targets = vec![target];
    let plan = success(&r);
    assert_eq!(plan.orientation, None);
    assert!(
        quaternion_distance_deg(&plan.waypoints[0].pose.orientation, &target.orientation) < 0.001
    );
}

#[test]
fn fixed_direction_is_preserved_between_returned_joint_samples() {
    let r = request();
    let plan = success(&r);
    let model = get_robot_model("irb1100-4-0.58").unwrap();
    for interval in plan.path.windows(2) {
        for i in 0..=20 {
            let t = i as f64 / 20.;
            let q = std::array::from_fn(|j| {
                interval[0].joints[j] + t * (interval[1].joints[j] - interval[0].joints[j])
            });
            let actual = forward_kinematics(&model, &q).unwrap().pose;
            assert!(
                quaternion_distance_deg(&actual.orientation, &r.orientation.unwrap())
                    <= FIXED_ORIENTATION_TOLERANCE_DEG
            );
        }
    }
}

#[test]
fn missing_generic_axis_geometry_and_invalid_body_frames_are_rejected() {
    let chains: Vec<GenericChain> = serde_json::from_str(include_str!(
        "../../../tests/fixtures/catalog/generic-chains.json"
    ))
    .unwrap();
    let chain = chains[0].clone();
    let mut r = request();
    r.start_joints = [10., 20., -25., 15., 30., -10.];
    let pose = forward_generic_kinematics(&chain, &r.start_joints)
        .unwrap()
        .pose;
    r.targets = vec![pose];
    r.orientation = None;
    let mut bodies: Vec<_> = (0..=chain.steps.len())
        .map(|step_index| GrindingBody {
            step_index,
            center_mm: [0.; 3],
            half_size_mm: [1.; 3],
        })
        .collect();
    r.robot = GrindingRobot::Generic {
        chain: chain.clone(),
        bodies: bodies.clone(),
    };
    success(&r);
    bodies.retain(|b| {
        chain.steps[..b.step_index]
            .iter()
            .filter(|s| s.joint_index.is_some())
            .count()
            != 3
    });
    r.robot = GrindingRobot::Generic {
        chain: chain.clone(),
        bodies,
    };
    assert_eq!(failure(&r).code, "invalid_input");
    r.robot = GrindingRobot::Generic {
        chain: chain.clone(),
        bodies: vec![GrindingBody {
            step_index: chain.steps.len() + 1,
            center_mm: [0.; 3],
            half_size_mm: [1.; 3],
        }],
    };
    assert_eq!(failure(&r).code, "invalid_input");
}

#[test]
fn enabled_clearance_is_checked_and_disabled_zones_are_not_an_obstacle() {
    let mut r = request();
    let base = CollisionZone {
        id: "margin".into(),
        enabled: true,
        center_mm: [0., 0., -2.],
        size_mm: [1., 1., 1.],
    };
    r.zones = vec![base];
    r.clearance_mm = 5.;
    assert_eq!(failure(&r).code, "collision");
    r.zones[0].enabled = false;
    success(&r);
    r.clearance_mm = -1.;
    assert_eq!(failure(&r).code, "invalid_input");
}

#[test]
fn non_finite_timing_is_never_returned_as_success() {
    let chains: Vec<GenericChain> = serde_json::from_str(include_str!(
        "../../../tests/fixtures/catalog/generic-chains.json"
    ))
    .unwrap();
    let mut chain = chains[0].clone();
    for limit in &mut chain.limits {
        limit.max_speed_deg = 1e-320;
    }
    let mut r = request();
    r.start_joints = [10., 20., -25., 15., 30., -10.];
    let start = forward_generic_kinematics(&chain, &r.start_joints)
        .unwrap()
        .pose;
    r.orientation = Some(start.orientation);
    r.targets = vec![Pose {
        position: [start.position[0], start.position[1] + 2., start.position[2]],
        ..start
    }];
    r.robot = GrindingRobot::Generic {
        bodies: (0..=chain.steps.len())
            .map(|step_index| GrindingBody {
                step_index,
                center_mm: [0.; 3],
                half_size_mm: [1.; 3],
            })
            .collect(),
        chain,
    };
    assert_eq!(failure(&r).code, "invalid_input");
}

#[test]
fn out_of_range_generated_detours_are_skipped_without_losing_other_routes() {
    let mut r = request();
    r.tool_pose.position = [0., 0., 200.];
    r.tool_radius_mm = 0.02;
    r.clearance_mm = 0.;
    let start = multiply(
        forward_kinematics(&get_robot_model("irb1100-4-0.58").unwrap(), &READY)
            .unwrap()
            .pose,
        r.tool_pose,
    );
    r.targets = vec![Pose {
        position: [start.position[0], 8., start.position[2]],
        ..start
    }];
    r.orientation = Some(start.orientation);
    r.zones = vec![
        CollisionZone {
            id: "thin".into(),
            enabled: true,
            center_mm: [start.position[0], 1., start.position[2]],
            size_mm: [0.01, 0.01, 0.01],
        },
        CollisionZone {
            id: "remote-boundary".into(),
            enabled: true,
            center_mm: [-999990., 0., 0.],
            size_mm: [100., 10., 10.],
        },
    ];
    let plan = success(&r);
    assert!(plan.waypoints.iter().all(|p| p.kind == "transfer"));
    assert!(plan.waypoints.iter().any(|p| p.target_index.is_none()));
    assert_eq!(plan.waypoints.last().unwrap().target_index, Some(0));
}

#[test]
fn distinct_input_target_poses_are_kept_exactly_with_rotated_base_and_offset_tool() {
    let mut r = request();
    r.orientation = None;
    r.allow_detours = false;
    r.base_pose = Pose {
        position: [450., -230., 120.],
        orientation: [0.3_f64.cos(), 0., 0., 0.3_f64.sin()],
    };
    r.tool_pose = Pose {
        position: [30., 15., 70.],
        orientation: [0.15_f64.cos(), 0.15_f64.sin(), 0., 0.],
    };
    let model = get_robot_model("irb1100-4-0.58").unwrap();
    r.targets = [[1., 20., -30., 0., 40., 2.], [2., 20., -30., 0., 40., 4.]]
        .iter()
        .map(|q| {
            multiply(
                r.base_pose,
                multiply(forward_kinematics(&model, q).unwrap().pose, r.tool_pose),
            )
        })
        .collect();
    let before = r.clone();
    let plan = success(&r);
    assert_eq!(r, before);
    for (index, point) in plan.waypoints.iter().enumerate() {
        assert_eq!(point.pose.position, r.targets[index].position);
        assert!(
            quaternion_distance_deg(&point.pose.orientation, &r.targets[index].orientation) < 1e-5
        );
    }
    assert!(
        quaternion_distance_deg(
            &plan.waypoints[0].pose.orientation,
            &plan.waypoints[1].pose.orientation
        ) > 1.
    );
    r.orientation = Some(r.targets[0].orientation);
    assert_eq!(failure(&r).code, "invalid_input");
}
#[test]
fn smooth_abb_quadrant_crossings_are_allowed_but_wrist_branch_flips_are_not() {
    let mut r = request();
    r.orientation = None;
    r.allow_detours = false;
    r.start_joints = [89.6, 20., -30., 89.6, 40., 89.6];
    let model = get_robot_model("irb1100-4-0.58").unwrap();
    r.targets = [
        [90.1, 20., -30., 90.1, 40., 90.1],
        [90.4, 20., -30., 90.4, 40., 90.4],
    ]
    .iter()
    .map(|q| forward_kinematics(&model, q).unwrap().pose)
    .collect();
    let plan = success(&r);
    assert_eq!(
        &plan.waypoints.last().unwrap().configuration.unwrap()[..3],
        &[1, 1, 1]
    );
    let initial = emulation_core::configuration::get_configuration(
        &model,
        &r.start_joints,
        &forward_kinematics(&model, &r.start_joints).unwrap(),
    )
    .configuration
    .unwrap();
    assert!(plan
        .waypoints
        .iter()
        .all(|p| p.configuration.unwrap()[3] == initial[3]));
    r.start_joints = [0., 20., -30., 0., -0.2, 0.];
    r.targets = vec![
        forward_kinematics(&model, &[0., 20., -30., 0., 0.2, 0.])
            .unwrap()
            .pose,
    ];
    assert!(matches!(
        plan_grinding(&r, &AtomicBool::new(false)),
        GrindingResult::Failure(_)
    ));
}
#[test]
fn redundant_chain_uses_bounded_candidates_after_greedy_dead_end() {
    let r: GrindingRequest = serde_json::from_str(include_str!(
        "../../../tests/fixtures/grinding/redundant-lookahead.json"
    ))
    .unwrap();
    let plan = success(&r);
    assert_eq!(plan.waypoints.len(), 2);
    assert!(plan.checked_intervals <= 100000);
    for (index, point) in plan.waypoints.iter().enumerate() {
        assert_eq!(point.pose.position, r.targets[index].position);
        assert_eq!(point.target_index, Some(index));
        assert_eq!(point.configuration, None);
    }
    let mut calls = 0;
    assert!(
        matches!(plan_grinding_cancellable(&r,||{calls+=1;calls>1500}),GrindingResult::Failure(error) if error.code=="cancelled")
    );
}
