//! JSON-lines diagnostic executable. This is a test/benchmark bridge, not a desktop sidecar.
use emulation_core::{
    collision::{check_collisions, CollisionRequest},
    endpoint_inspection::{inspect_endpoint, EndpointInspectionRequest},
    generic_endpoint_inspection::{inspect_generic_endpoint, GenericEndpointRequest},
    generic_kinematics::{forward_generic_kinematics, GenericChain},
    generic_planner::{plan_generic_move_l, GenericPlanRequest},
    generic_pose_planner::{plan_generic_pose_move_l, GenericPosePlanRequest},
    grinding::{plan_grinding, GrindingRequest},
    kinematics, model, planner,
    surface::{plan_surface, SurfaceRequest},
    types::{JointAngles, PlanRequest, Pose},
};
use serde::Deserialize;
use serde_json::{json, Value};
use std::io::{self, BufRead, Write};
use std::sync::atomic::AtomicBool;

#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
enum Command {
    Fk {
        #[serde(rename = "modelId")]
        model_id: String,
        joints: JointAngles,
    },
    Ik {
        #[serde(rename = "modelId")]
        model_id: String,
        pose: Pose,
        seed: JointAngles,
    },
    Plan {
        request: PlanRequest,
    },
    Collision {
        request: CollisionRequest,
    },
    Inspect {
        request: EndpointInspectionRequest,
    },
    GenericFk {
        chain: GenericChain,
        joints: JointAngles,
    },
    Grinding {
        request: GrindingRequest,
    },
    Surface {
        request: SurfaceRequest,
    },
    GenericInspect {
        request: GenericEndpointRequest,
    },
    GenericPosePlan {
        request: GenericPosePlanRequest,
    },
    GenericPlan {
        request: GenericPlanRequest,
    },
}

fn execute(command: Command) -> Result<Value, String> {
    match command {
        Command::Fk { model_id, joints } => {
            let robot = model::get_robot_model(&model_id).ok_or("Unsupported robot model")?;
            serde_json::to_value(kinematics::forward_kinematics(&robot, &joints)?)
                .map_err(|e| e.to_string())
        }
        Command::Ik {
            model_id,
            pose,
            seed,
        } => {
            let robot = model::get_robot_model(&model_id).ok_or("Unsupported robot model")?;
            serde_json::to_value(kinematics::inverse_kinematics(&robot, &pose, &seed))
                .map_err(|e| e.to_string())
        }
        Command::Plan { request } => {
            serde_json::to_value(planner::plan_move_l(&request)).map_err(|e| e.to_string())
        }
        Command::Collision { request } => {
            serde_json::to_value(check_collisions(&request, &AtomicBool::new(false))?)
                .map_err(|e| e.to_string())
        }
        Command::Inspect { request } => {
            serde_json::to_value(inspect_endpoint(&request)?).map_err(|e| e.to_string())
        }
        Command::GenericFk { chain, joints } => {
            serde_json::to_value(forward_generic_kinematics(&chain, &joints)?)
                .map_err(|e| e.to_string())
        }
        Command::Surface { request } => {
            serde_json::to_value(plan_surface(&request, &AtomicBool::new(false)))
                .map_err(|e| e.to_string())
        }
        Command::Grinding { request } => {
            serde_json::to_value(plan_grinding(&request, &AtomicBool::new(false)))
                .map_err(|e| e.to_string())
        }
        Command::GenericInspect { request } => {
            serde_json::to_value(inspect_generic_endpoint(&request)?).map_err(|e| e.to_string())
        }
        Command::GenericPosePlan { request } => {
            serde_json::to_value(plan_generic_pose_move_l(&request, &AtomicBool::new(false)))
                .map_err(|e| e.to_string())
        }
        Command::GenericPlan { request } => {
            serde_json::to_value(plan_generic_move_l(&request, &AtomicBool::new(false)))
                .map_err(|e| e.to_string())
        }
    }
}

fn main() -> io::Result<()> {
    let stdin = io::stdin();
    let mut stdout = io::BufWriter::new(io::stdout().lock());
    for line in stdin.lock().lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let answer = if line.len() > 1_048_576 {
            json!({ "error": "Command exceeds 1 MiB diagnostic limit" })
        } else {
            match serde_json::from_str::<Command>(&line)
                .map_err(|e| e.to_string())
                .and_then(execute)
            {
                Ok(value) => json!({ "result": value }),
                Err(error) => json!({ "error": error }),
            }
        };
        serde_json::to_writer(&mut stdout, &answer)?;
        writeln!(stdout)?;
        stdout.flush()?;
    }
    Ok(())
}
