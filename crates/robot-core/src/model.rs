//! ABB nominal geometry: 3HAC064993-001 Rev V, pp.30–31, 62, 66.
//! The standard-DH table is derived from geometry, not individual calibration.
use crate::types::{DhRow, Geometry, JointLimit, RobotModel};

fn model(id: &str, upper: f64, forearm: f64, reach: f64, shoulder_speed: f64) -> RobotModel {
    let row = |a_mm, d_mm, alpha_deg, theta_offset_deg| DhRow {
        a_mm,
        d_mm,
        alpha_deg,
        theta_offset_deg,
        theta_sign: 1.,
    };
    let limit = |min_deg, max_deg, max_speed_deg| JointLimit {
        min_deg,
        max_deg,
        max_speed_deg,
    };
    RobotModel {
        id: id.into(),
        name: format!(
            "ABB IRB 1100-4/{}",
            if reach == 580. { "0.58" } else { "0.475" }
        ),
        reach_mm: reach,
        payload_kg: 4.,
        geometry: Geometry {
            shoulder_height_mm: 327.,
            upper_arm_mm: upper,
            elbow_offset_mm: 10.,
            forearm_mm: forearm,
            flange_mm: 64.,
        },
        dh: [
            row(0., 327., -90., 0.),
            row(upper, 0., 0., -90.),
            row(10., 0., -90., 0.),
            row(0., forearm, 90., 0.),
            row(0., 0., -90., 0.),
            row(0., 64., 0., 180.),
        ],
        limits: [
            limit(-230., 230., 460.),
            limit(-115., 113., shoulder_speed),
            limit(-205., 55., 280.),
            limit(-230., 230., 560.),
            limit(-125., 120., 420.),
            limit(-400., 400., 750.),
        ],
        home_joints: [0.; 6],
        // Application example, never described as the manufacturer's zero.
        ready_joints: [0., 20., -30., 0., 40., 0.],
    }
}

pub fn get_robot_model(id: &str) -> Option<RobotModel> {
    match id {
        "irb1100-4-0.58" => Some(model(id, 280., 300., 580., 360.)),
        "irb1100-4-0.475" => Some(model(id, 225., 250., 475., 380.)),
        _ => None,
    }
}

pub fn robot_models() -> [RobotModel; 2] {
    [
        model("irb1100-4-0.58", 280., 300., 580., 360.),
        model("irb1100-4-0.475", 225., 250., 475., 380.),
    ]
}
