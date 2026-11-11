//! Native incremental pendant motion: actual FK/IK and declared swept envelopes.
use emulation_core::collision::CollisionZone;
use emulation_core::generic_kinematics::{forward_generic_kinematics, GenericChain};
use emulation_core::grinding::{GrindingBody, GrindingRobot};
use emulation_core::kinematics::forward_kinematics;
use emulation_core::math::{matrix_to_quaternion, multiply_matrices, norm, quaternion_distance_deg, quaternion_to_matrix, sub};
use emulation_core::model::get_robot_model;
use emulation_core::pendant_jog::{jog_step, jog_step_cancellable, JogMode, JogRequest, JogResult, JogSpace, JogSuccess};
use emulation_core::types::{JointAngles, Pose};
use std::sync::atomic::AtomicBool;

const IDENTITY: Pose = Pose { position: [0.; 3], orientation: [1., 0., 0., 0.] };
const READY: JointAngles = [0., 20., -30., 0., 40., 0.];

fn request() -> JogRequest {
    JogRequest {
        robot: GrindingRobot::Abb { model_id: "irb1100-4-0.58".into() },
        base_pose: IDENTITY,
        tool_pose: IDENTITY,
        tool_radius_mm: 0.,
        zones: vec![],
        start_joints: READY,
        mode: JogMode::Joint,
        space: JogSpace::World,
        axes: [1., 0., 0., 0., 0., 0.],
        speed_scale: 1.,
        step_seconds: 0.05,
    }
}
fn success(input: &JogRequest) -> JogSuccess {
    match jog_step(input, &AtomicBool::new(false)) {
        JogResult::Success(result) => result,
        other => panic!("expected successful jog, received {other:?}"),
    }
}
fn failure_code(input: &JogRequest) -> String {
    match jog_step(input, &AtomicBool::new(false)) {
        JogResult::Failure(result) => result.code,
        other => panic!("expected rejected jog, received {other:?}"),
    }
}
fn multiply(a: Pose, b: Pose) -> Pose {
    let mut am = quaternion_to_matrix(&a.orientation).unwrap();
    let mut bm = quaternion_to_matrix(&b.orientation).unwrap();
    for axis in 0..3 { am[axis * 4 + 3] = a.position[axis]; bm[axis * 4 + 3] = b.position[axis]; }
    let matrix = multiply_matrices(&am, &bm);
    Pose { position: [matrix[3], matrix[7], matrix[11]], orientation: matrix_to_quaternion(&matrix) }
}
fn world_tcp(input: &JogRequest, joints: &JointAngles) -> Pose {
    let flange = match &input.robot {
        GrindingRobot::Abb { model_id } => forward_kinematics(&get_robot_model(model_id).unwrap(), joints).unwrap().pose,
        GrindingRobot::Generic { chain, .. } => forward_generic_kinematics(chain, joints).unwrap().pose,
    };
    multiply(multiply(input.base_pose, flange), input.tool_pose)
}
fn close_pose(actual: &Pose, expected: &Pose) {
    assert!(norm(&sub(&actual.position, &expected.position)) < 0.003, "position {actual:?} != {expected:?}");
    assert!(quaternion_distance_deg(&actual.orientation, &expected.orientation) < 0.003);
}
fn generic_requests() -> Vec<JogRequest> {
    let chains: Vec<GenericChain> = serde_json::from_str(include_str!("../../../tests/fixtures/catalog/generic-chains.json")).unwrap();
    assert_eq!(chains.len(), 4);
    chains.into_iter().map(|chain| {
        let mut input = request();
        input.start_joints = [10., 20., -25., 15., 30., -10.];
        // These are explicitly supplied test envelopes at every chain frame;
        // real catalog body bounds are separately checked by mesh fixtures.
        input.robot = GrindingRobot::Generic {
            bodies: (0..=chain.steps.len()).map(|step_index| GrindingBody { step_index, center_mm: [0.; 3], half_size_mm: [1.; 3] }).collect(),
            chain,
        };
        input
    }).collect()
}

#[test]
fn joint_jog_preserves_unselected_axes_and_uses_exact_seed_without_ik() {
    for model_id in ["irb1100-4-0.58", "irb1100-4-0.475"] {
        let mut input = request(); input.robot = GrindingRobot::Abb { model_id: model_id.into() };
        input.start_joints[0] = 0.123456789123;
        input.axes = [0., 0., 0., -1., 0., 0.];
        input.speed_scale = 0.4;
        let before = input.clone();
        let actual = success(&input);
        assert_eq!(input, before, "jog mutated its request");
        let mut expected = input.start_joints; expected[3] -= 30. * 0.4 * 0.05;
        for axis in 0..6 { assert!((actual.joints[axis] - expected[axis]).abs() < 1e-12); }
        close_pose(&actual.pose, &world_tcp(&input, &actual.joints));
    }
}
#[test]
fn joint_jog_is_available_at_abb_mechanical_zero() {
    let mut input = request(); input.start_joints = [0.; 6]; input.axes = [0., 1., 0., 0., 0., 0.];
    let actual = success(&input);
    assert_eq!(actual.joints, [0., 1.5, 0., 0., 0., 0.]);
    close_pose(&actual.pose, &world_tcp(&input, &actual.joints));
}
#[test]
fn combined_stick_input_has_bounded_velocity() {
    let mut input = request(); input.axes = [1., 1., 0., 0., 0., 0.];
    let actual = success(&input);
    let delta: Vec<f64> = actual.joints.iter().zip(input.start_joints).map(|(a,b)| a-b).collect();
    assert!((delta.iter().map(|value| value * value).sum::<f64>().sqrt() - 1.5).abs() < 1e-10);
    assert!((delta[0] - delta[1]).abs() < 1e-10);
    assert_eq!(&actual.joints[2..], &input.start_joints[2..]);
}
#[test]
fn declared_joint_speed_limits_are_respected_for_generic_chains() {
    let mut input = generic_requests().remove(0);
    if let GrindingRobot::Generic { chain, .. } = &mut input.robot { chain.limits[0].max_speed_deg = 2.; }
    let actual = success(&input);
    assert!((actual.joints[0] - input.start_joints[0] - 0.1).abs() < 1e-12);
}
#[test]
fn joint_limit_crossing_fails_without_clamping_or_changing_branch() {
    let mut input = request();
    input.start_joints[0] = get_robot_model("irb1100-4-0.58").unwrap().limits[0].max_deg - 0.1;
    assert_eq!(failure_code(&input), "joint_limit");
    input.axes[0] = -1.; success(&input);
}
#[test]
fn world_linear_jog_retains_tcp_orientation_and_requested_distance() {
    let mut input = request(); input.mode = JogMode::Linear; input.axes = [0., 1., 0., 0., 0., 0.]; input.speed_scale = 0.2;
    let start = world_tcp(&input, &input.start_joints);
    let mut expected = start; expected.position[1] += 100. * 0.2 * 0.05;
    let actual = success(&input);
    close_pose(&actual.pose, &expected);
    close_pose(&world_tcp(&input, &actual.joints), &expected);
    for axis in 0..6 { assert!((actual.joints[axis] - input.start_joints[axis]).abs() < 5.); }
}
#[test]
fn tool_linear_jog_uses_world_tcp_rotation_with_base_and_tool_offsets() {
    let mut input = request(); input.mode = JogMode::Linear; input.space = JogSpace::Tool;
    input.axes = [0., 1., 0., 0., 0., 0.]; input.speed_scale = 0.05;
    let half = std::f64::consts::FRAC_1_SQRT_2;
    input.base_pose = Pose { position: [700., -300., 120.], orientation: [half, 0., 0., half] };
    input.tool_pose = Pose { position: [10., 0., 35.], orientation: [half, 0., 0., half] };
    let start = world_tcp(&input, &input.start_joints);
    let rotation = quaternion_to_matrix(&start.orientation).unwrap();
    let mut expected = start;
    for axis in 0..3 { expected.position[axis] += rotation[axis * 4 + 1] * 0.25; }
    let actual = success(&input);
    close_pose(&actual.pose, &expected);
    close_pose(&world_tcp(&input, &actual.joints), &expected);
}
#[test]
fn all_catalog_generic_models_support_joint_and_fixed_orientation_linear_steps() {
    for mut input in generic_requests() {
        let actual = success(&input);
        assert!((actual.joints[0] - input.start_joints[0] - 1.5).abs() < 1e-12);
        assert_eq!(&actual.joints[1..], &input.start_joints[1..]);
        close_pose(&actual.pose, &world_tcp(&input, &actual.joints));
        input.mode = JogMode::Linear; input.axes = [0., 1., 0., 0., 0., 0.]; input.speed_scale = 0.1;
        let mut expected = world_tcp(&input, &input.start_joints); expected.position[1] += 0.5;
        let actual = success(&input);
        close_pose(&actual.pose, &expected);
        close_pose(&world_tcp(&input, &actual.joints), &expected);
    }
}
#[test]
fn both_modes_check_transformed_robot_and_offset_tool_envelopes() {
    for mode in [JogMode::Joint, JogMode::Linear] {
        let mut input = request(); input.mode = mode; input.axes = [0., 1., 0., 0., 0., 0.]; input.speed_scale = 0.1;
        input.base_pose.position = [2000., 3000., 200.]; input.tool_pose.position = [0., 0., 300.]; input.tool_radius_mm = 4.;
        let tcp = world_tcp(&input, &input.start_joints);
        input.zones = vec![CollisionZone { id: "tool-tip".into(), enabled: true, center_mm: tcp.position, size_mm: [1.; 3] }];
        assert_eq!(failure_code(&input), "collision");
        input.zones[0].enabled = false; success(&input);
        input.zones[0].enabled = true; input.zones[0].center_mm = [0., 0., 50.]; success(&input);
        input.zones[0].center_mm = [2000., 3000., 250.]; input.zones[0].size_mm = [100.; 3];
        assert_eq!(failure_code(&input), "collision");
    }
}
#[test]
fn malformed_or_unbounded_commands_and_cancellation_fail_closed() {
    let changes: Vec<Box<dyn Fn(&mut JogRequest)>> = vec![
        Box::new(|r| r.axes[0] = f64::NAN), Box::new(|r| r.axes[0] = 1.01),
        Box::new(|r| r.speed_scale = f64::INFINITY), Box::new(|r| r.speed_scale = -0.1), Box::new(|r| r.speed_scale = 1.01),
        Box::new(|r| r.step_seconds = 0.051), Box::new(|r| r.step_seconds = -0.01), Box::new(|r| r.step_seconds = f64::NAN),
        Box::new(|r| r.start_joints[3] = f64::NAN), Box::new(|r| r.tool_radius_mm = -1.),
        Box::new(|r| { r.mode = JogMode::Linear; r.axes[3] = 0.5; }),
        Box::new(|r| r.base_pose.orientation = [0.; 4]),
    ];
    for change in changes { let mut input = request(); change(&mut input); let _ = failure_code(&input); }
    assert!(matches!(jog_step(&request(), &AtomicBool::new(true)), JogResult::Failure(error) if error.code == "cancelled"));
}

#[test]
fn thin_tool_obstacle_between_clear_joint_endpoints_is_detected() {
    let mut input = request();
    input.tool_pose.position = [0., 0., 2000.];
    let end = success(&input).joints;
    let middle = std::array::from_fn(|axis| (input.start_joints[axis] + end[axis]) / 2.);
    input.zones = vec![CollisionZone {
        id: "between-joint-endpoints".into(), enabled: true,
        center_mm: world_tcp(&input, &middle).position, size_mm: [0.001; 3],
    }];
    // Nonzero microscopic motion still enters the same envelope validator;
    // zero input is intentionally a no-op and is not evidence of clear geometry.
    let mut endpoint = input.clone(); endpoint.axes[0] = 1e-6;
    success(&endpoint);
    endpoint.start_joints = end; success(&endpoint);
    assert_eq!(failure_code(&input), "collision");
}
#[test]
fn linear_at_mechanical_zero_requires_axis_mode_and_cancels_during_work() {
    let mut input = request(); input.mode = JogMode::Linear; input.start_joints = [0.; 6];
    assert!(matches!(jog_step(&input, &AtomicBool::new(false)), JogResult::Failure(error) if error.code == "singular_path" && error.message.contains("轴模式")));
    input.start_joints = READY; input.axes = [0., 1., 0., 0., 0., 0.]; input.speed_scale = 0.1;
    let mut checks = 0;
    let result = jog_step_cancellable(&input, || { checks += 1; checks > 8 });
    assert!(checks > 8);
    assert!(matches!(result, JogResult::Failure(error) if error.code == "cancelled"));
}
