use emulation_core::configuration::{get_configuration, joint_quadrant};
use emulation_core::kinematics::forward_kinematics;
use emulation_core::model::{get_robot_model, robot_models};

fn close(actual: &[f64], expected: &[f64], tolerance: f64) {
    assert_eq!(actual.len(), expected.len());
    for (a, b) in actual.iter().zip(expected) {
        assert!((a - b).abs() < tolerance, "{a} != {b} within {tolerance}");
    }
}

#[test]
fn official_mechanical_zero_and_positive_axes() {
    // Independent ABB 3HAC064993-001 Rev V dimensions, not generated through IK.
    for (id, elbow, wrist, flange) in [
        (
            "irb1100-4-0.58",
            [0., 0., 607.],
            [300., 0., 617.],
            [364., 0., 617.],
        ),
        (
            "irb1100-4-0.475",
            [0., 0., 552.],
            [250., 0., 562.],
            [314., 0., 562.],
        ),
    ] {
        let model = get_robot_model(id).unwrap();
        assert_eq!(model.home_joints, [0.; 6]);
        let fk = forward_kinematics(&model, &model.home_joints).unwrap();
        close(
            &[fk.frames[0][3], fk.frames[0][7], fk.frames[0][11]],
            &[0.; 3],
            1e-8,
        );
        close(&fk.shoulder, &[0., 0., 327.], 1e-8);
        close(&fk.elbow, &elbow, 1e-8);
        close(&fk.wrist_center, &wrist, 1e-8);
        close(&fk.pose.position, &flange, 1e-8);
        let positive_axes = [
            [0., 0., 1.],
            [0., 1., 0.],
            [0., 1., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
            [1., 0., 0.],
        ];
        for (a, b) in fk.joint_axes.iter().zip(positive_axes) {
            close(a, &b, 1e-8);
        }
        let f = fk.frames[6];
        close(&[f[0], f[4], f[8]], &[0., 0., -1.], 1e-8);
        close(&[f[1], f[5], f[9]], &[0., 1., 0.], 1e-8);
        close(&[f[2], f[6], f[10]], &[1., 0., 0.], 1e-8);
        close(
            &fk.pose.orientation,
            &[
                std::f64::consts::FRAC_1_SQRT_2,
                0.,
                std::f64::consts::FRAC_1_SQRT_2,
                0.,
            ],
            1e-8,
        );
        assert!(get_configuration(&model, &model.home_joints, &fk).singular);
    }
}

#[test]
fn official_working_range_drawing_positions() {
    // ABB Rev V pp.60–61: rounded angles and coordinates need 1 mm tolerance.
    // The short-arm pos2 table says 53.8; the same-page drawing and geometry say 153.8.
    // pos0 is the flange despite the table heading and is checked separately above.
    let long = [
        [0., -88., 0., 907.2],
        [12.5, 55., 184.6, 327.],
        [90., -88., 580., 327.],
        [113., -88., 534., 100.3],
        [-28.3, -205., -304., 327.],
        [-115., 55., -112.4, 473.5],
        [-90., -88., -580., 327.],
        [-115., -88., -525.8, 81.8],
        [113., -205., 237.3, 517.1],
    ];
    let short = [
        [0., -87.7, 0., 802.],
        [9.7, 55., 153.8, 327.],
        [90., -87.7, 475., 327.],
        [113., -87.7, 437.4, 141.3],
        [-26.4, -205., -248.2, 327.],
        [-115., 55., -87.6, 453.4],
        [-90., -87.7, -475., 327.],
        [-115., -87.7, -430.7, 126.2],
        [113., -205., 188.4, 488.6],
    ];
    for (model, points) in robot_models().iter().zip([long, short]) {
        for [q2, q3, x, z] in points {
            let fk = forward_kinematics(model, &[0., q2, q3, 0., 0., 0.]).unwrap();
            close(&fk.wrist_center, &[x, 0., z], 1.);
        }
    }
}

#[test]
fn default_limits_speeds_and_quadrants_keep_complete_turns() {
    for model in robot_models() {
        assert_eq!(
            model.limits.map(|l| (l.min_deg, l.max_deg)),
            [
                (-230., 230.),
                (-115., 113.),
                (-205., 55.),
                (-230., 230.),
                (-125., 120.),
                (-400., 400.)
            ]
        );
        assert_eq!(
            model.limits[1].max_speed_deg,
            if model.reach_mm == 580. { 360. } else { 380. }
        );
        assert!(
            !get_configuration(
                &model,
                &model.ready_joints,
                &forward_kinematics(&model, &model.ready_joints).unwrap()
            )
            .singular
        );
    }
    for (angle, quadrant) in [
        (-400., -5),
        (-360., -4),
        (-180., -2),
        (-90.001, -2),
        (-90., -1),
        (-0.000001, -1),
        (0., 0),
        (89.999999, 0),
        (90., 1),
        (180., 2),
        (400., 4),
    ] {
        assert_eq!(joint_quadrant(angle), Some(quadrant));
    }
    assert_eq!(joint_quadrant(-1e-12), Some(0));
    assert_eq!(joint_quadrant(90. - 1e-12), Some(1));
    assert_eq!(joint_quadrant(90. - 1e-7), Some(0));
    assert_eq!(joint_quadrant(f64::INFINITY), None);
    assert_eq!(get_robot_model("unknown"), None);
}

#[test]
fn configuration_matches_all_eight_official_geometric_branches() {
    let model = get_robot_model("irb1100-4-0.58").unwrap();
    let geometries = [
        ([0., 0., 607.], [300., 0., 617.]),
        ([280., 0., 327.], [290., 0., 627.]),
        ([-280., 0., 327.], [-290., 0., 627.]),
        ([0., 0., 607.], [-300., 0., 617.]),
    ];
    for cfx in 0..8 {
        let mut fk = forward_kinematics(&model, &model.home_joints).unwrap();
        (fk.elbow, fk.wrist_center) = geometries[cfx / 2];
        let q = [
            0.,
            0.,
            0.,
            -100.,
            if cfx % 2 == 0 { 30. } else { -30. },
            380.,
        ];
        assert_eq!(
            get_configuration(&model, &q, &fk).configuration,
            Some([0, -2, 4, cfx as i32])
        );
    }
    let q = [90., 0., 0., 0., 30., 0.];
    let fk = forward_kinematics(&model, &q).unwrap();
    assert_eq!(
        get_configuration(&model, &q, &fk).configuration,
        Some([1, 0, 0, 0])
    );
}

#[test]
fn configuration_rejects_shoulder_elbow_and_wrist_singularities() {
    let model = get_robot_model("irb1100-4-0.58").unwrap();
    let q = [0., 0., 0., 0., 30., 0.];
    let mut fk = forward_kinematics(&model, &q).unwrap();
    fk.wrist_center = [0.01, 0., 617.];
    assert!(get_configuration(&model, &q, &fk).singular);
    fk.elbow = [280., 0., 327.];
    fk.wrist_center = [580., 0., 327.01];
    assert!(get_configuration(&model, &q, &fk).singular);
    let q = [0., 20., -30., 0., 0.05, 0.];
    let result = get_configuration(&model, &q, &forward_kinematics(&model, &q).unwrap());
    assert!(result.singular);
    assert!(result.configuration.is_none());
}
