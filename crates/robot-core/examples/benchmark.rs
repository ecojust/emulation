//! Kernel-only batch timing. JSON and process startup are outside timed regions.
use emulation_core::{
    kinematics::{forward_kinematics, inverse_kinematics},
    model::get_robot_model,
    planner::plan_move_l,
    types::*,
};
use serde::{Deserialize, Serialize};
use std::{hint::black_box, time::Instant};

#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
enum Task {
    Fk {
        #[serde(rename = "modelId")]
        model_id: String,
        joints: Vec<JointAngles>,
    },
    Ik {
        #[serde(rename = "modelId")]
        model_id: String,
        poses: Vec<Pose>,
        seeds: Vec<JointAngles>,
    },
    Plan {
        requests: Vec<PlanRequest>,
    },
}
#[derive(Deserialize)]
struct Case {
    name: String,
    iterations: usize,
    task: Task,
}
#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Input {
    samples: usize,
    warmup_batches: usize,
    cases: Vec<Case>,
}
#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct Measurement {
    name: String,
    iterations: usize,
    batch_mean_ns: Vec<f64>,
    checksum: f64,
}

enum Prepared {
    Fk(RobotModel, Vec<JointAngles>),
    Ik(RobotModel, Vec<Pose>, Vec<JointAngles>),
    Plan(Vec<PlanRequest>),
}
impl Prepared {
    fn run(&self, iteration: usize) -> f64 {
        match self {
            Self::Fk(model, joints) => {
                let result = black_box(
                    forward_kinematics(
                        black_box(model),
                        black_box(&joints[iteration % joints.len()]),
                    )
                    .unwrap(),
                );
                black_box(result.pose.position[0] + result.pose.position[2])
            }
            Self::Ik(model, poses, seeds) => {
                let index = iteration % poses.len();
                let solutions = black_box(inverse_kinematics(
                    black_box(model),
                    black_box(&poses[index]),
                    black_box(&seeds[index]),
                ));
                black_box(
                    solutions.len() as f64
                        + solutions.first().map_or(0.0, |s| s.joints[0] / 1000.0),
                )
            }
            Self::Plan(requests) => {
                let result = black_box(plan_move_l(black_box(
                    &requests[iteration % requests.len()],
                )));
                black_box(match result {
                    PlanResult::Success(result) => {
                        result.length_mm
                            + result.orientation_travel_deg
                            + result.selected.joints[0] / 1000.0
                    }
                    PlanResult::Failure(_) => -1.0,
                })
            }
        }
    }
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .ok_or("Usage: benchmark fixtures.json")?;
    let input: Input = serde_json::from_slice(&std::fs::read(path)?)?;
    if !(5..=101).contains(&input.samples) || !(1..=20).contains(&input.warmup_batches) {
        return Err("Invalid benchmark sample counts".into());
    }
    let mut measurements = Vec::new();
    for case in input.cases {
        if case.iterations == 0 || case.iterations > 100_000 {
            return Err("Invalid benchmark iteration count".into());
        }
        let task = match case.task {
            Task::Fk { model_id, joints } => {
                if joints.is_empty() {
                    return Err("Empty FK batch".into());
                }
                Prepared::Fk(get_robot_model(&model_id).ok_or("Unknown model")?, joints)
            }
            Task::Ik {
                model_id,
                poses,
                seeds,
            } => {
                if poses.is_empty() || poses.len() != seeds.len() {
                    return Err("Invalid IK batch".into());
                }
                Prepared::Ik(
                    get_robot_model(&model_id).ok_or("Unknown model")?,
                    poses,
                    seeds,
                )
            }
            Task::Plan { requests } => {
                if requests.is_empty() {
                    return Err("Empty planning batch".into());
                }
                Prepared::Plan(requests)
            }
        };
        for _ in 0..input.warmup_batches {
            for iteration in 0..case.iterations {
                black_box(task.run(iteration));
            }
        }
        let mut batch_mean_ns = Vec::with_capacity(input.samples);
        let mut checksum = 0.0;
        for _ in 0..input.samples {
            let started = Instant::now();
            for iteration in 0..case.iterations {
                checksum += task.run(iteration);
            }
            batch_mean_ns.push(started.elapsed().as_nanos() as f64 / case.iterations as f64);
        }
        measurements.push(Measurement {
            name: case.name,
            iterations: case.iterations,
            batch_mean_ns,
            checksum: black_box(checksum),
        });
    }
    println!("{}", serde_json::to_string(&measurements)?);
    Ok(())
}
