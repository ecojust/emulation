//! One CPU worker with one pending motion/export, endpoint IK, and mesh inspection.
//! Background endpoint and collision inspection never cancel a solve or export.
//! Foreground work may interrupt inspection, including a cold mesh-index build.
//! Cancellation never creates an unbounded spawn_blocking queue.
use emulation_core::{
    collision::{check_collisions, CollisionRequest, CollisionResult},
    endpoint_inspection::{inspect_endpoint_with_cancel, EndpointInspectionRequest},
    generic_planner::{plan_generic_move_l, GenericPlanRequest, GenericPlanResult},
    grinding::{plan_grinding, GrindingRequest, GrindingResult},
    planner::plan_move_l_with_cancel,
    program::{export_program_with_cancel, ProgramExportRequest, ProgramFile},
    types::{IkSolution, PlanRequest, PlanResult},
};
use std::{
    collections::VecDeque,
    panic::{catch_unwind, AssertUnwindSafe},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Condvar, Mutex,
    },
};
use tokio::sync::oneshot;

pub enum Work {
    Plan(PlanRequest),
    GenericPlan(GenericPlanRequest),
    Grinding(GrindingRequest),
    Export(ProgramExportRequest),
    Collision(CollisionRequest),
    InspectEndpoint(EndpointInspectionRequest),
}
pub enum Reply {
    Plan(PlanResult),
    GenericPlan(GenericPlanResult),
    Grinding(GrindingResult),
    Export(ProgramFile),
    Collision(Vec<CollisionResult>),
    InspectEndpoint(Vec<IkSolution>),
}
type Response = Result<Reply, String>;
struct Job {
    id: String,
    work: Work,
    cancelled: Arc<AtomicBool>,
    reply: oneshot::Sender<Response>,
}
#[derive(Default)]
struct Queue {
    pending: Option<Job>,
    background: Option<Job>,
    endpoint: Option<Job>,
    active: Option<(String, Arc<AtomicBool>, bool)>,
    cancelled_ids: VecDeque<String>,
    stopped: bool,
}
#[derive(Default)]
struct Shared {
    queue: Mutex<Queue>,
    changed: Condvar,
}
pub struct Engine {
    shared: Arc<Shared>,
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}
impl Engine {
    pub fn new() -> Self {
        let shared = Arc::new(Shared::default());
        let worker = shared.clone();
        std::thread::Builder::new()
            .name("robot-motion".into())
            .spawn(move || loop {
                let job = {
                    let mut queue = worker.queue.lock().unwrap_or_else(|e| e.into_inner());
                    while queue.pending.is_none()
                        && queue.endpoint.is_none()
                        && queue.background.is_none()
                        && !queue.stopped
                    {
                        queue = worker
                            .changed
                            .wait(queue)
                            .unwrap_or_else(|e| e.into_inner());
                    }
                    if queue.stopped {
                        break;
                    }
                    let job = queue
                        .pending
                        .take()
                        .or_else(|| queue.endpoint.take())
                        .or_else(|| queue.background.take())
                        .expect("pending job checked");
                    queue.active = Some((
                        job.id.clone(),
                        job.cancelled.clone(),
                        matches!(&job.work, Work::Collision(_) | Work::InspectEndpoint(_)),
                    ));
                    job
                };
                let result = catch_unwind(AssertUnwindSafe(|| match job.work {
                    Work::Plan(request) => Ok(Reply::Plan(plan_move_l_with_cancel(
                        &request,
                        &job.cancelled,
                    ))),
                    Work::GenericPlan(request) => Ok(Reply::GenericPlan(plan_generic_move_l(
                        &request,
                        &job.cancelled,
                    ))),
                    Work::Grinding(request) => {
                        Ok(Reply::Grinding(plan_grinding(&request, &job.cancelled)))
                    }
                    Work::Export(request) => export_program_with_cancel(&request, &job.cancelled)
                        .map(Reply::Export)
                        .map_err(|e| e.to_string()),
                    Work::Collision(request) => {
                        check_collisions(&request, &job.cancelled).map(Reply::Collision)
                    }
                    Work::InspectEndpoint(request) => {
                        inspect_endpoint_with_cancel(&request, &job.cancelled)
                            .map(Reply::InspectEndpoint)
                    }
                }))
                .unwrap_or_else(|_| Err("原生运动计算异常，请调整输入后重试".into()));
                let result = if job.cancelled.load(Ordering::Acquire) {
                    Err("计算已取消".into())
                } else {
                    result
                };
                let _ = job.reply.send(result);
                let mut queue = worker.queue.lock().unwrap_or_else(|e| e.into_inner());
                if queue
                    .active
                    .as_ref()
                    .is_some_and(|(id, _, _)| id == &job.id)
                {
                    queue.active = None;
                }
            })
            .expect("cannot create native motion worker");
        Self { shared }
    }

    pub fn submit(&self, id: String, work: Work) -> Result<oneshot::Receiver<Response>, String> {
        if id.is_empty()
            || id.len() > 100
            || !id.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-')
        {
            return Err("无效的计算请求标识".into());
        }
        let (reply, receiver) = oneshot::channel();
        let mut queue = self
            .shared
            .queue
            .lock()
            .map_err(|_| "原生运动引擎状态异常")?;
        if queue.stopped || queue.cancelled_ids.contains(&id) {
            return Err("计算已取消".into());
        }
        let is_background = matches!(&work, Work::Collision(_) | Work::InspectEndpoint(_));
        if !is_background {
            if let Some((_, cancelled, _)) = &queue.active {
                cancelled.store(true, Ordering::Release);
            }
        }
        let slot = match &work {
            Work::InspectEndpoint(_) => &mut queue.endpoint,
            Work::Collision(_) => &mut queue.background,
            _ => &mut queue.pending,
        };
        if let Some(previous) = slot.take() {
            let _ = previous.reply.send(Err("计算已取消".into()));
        }
        *slot = Some(Job {
            id,
            work,
            cancelled: Arc::new(AtomicBool::new(false)),
            reply,
        });
        self.shared.changed.notify_one();
        Ok(receiver)
    }

    pub fn cancel(&self, id: &str) {
        if let Ok(mut queue) = self.shared.queue.lock() {
            if let Some((active_id, cancelled, _)) = &queue.active {
                if active_id == id {
                    cancelled.store(true, Ordering::Release);
                }
            }
            if queue.pending.as_ref().is_some_and(|job| job.id == id) {
                if let Some(job) = queue.pending.take() {
                    let _ = job.reply.send(Err("计算已取消".into()));
                }
            }
            if queue.background.as_ref().is_some_and(|job| job.id == id) {
                if let Some(job) = queue.background.take() {
                    let _ = job.reply.send(Err("计算已取消".into()));
                }
            }
            if queue.endpoint.as_ref().is_some_and(|job| job.id == id) {
                if let Some(job) = queue.endpoint.take() {
                    let _ = job.reply.send(Err("计算已取消".into()));
                }
            }
            // Includes cancellation arriving just before its async submit command.
            if !queue.cancelled_ids.iter().any(|known| known == id) {
                queue.cancelled_ids.push_back(id.to_owned());
                if queue.cancelled_ids.len() > 64 {
                    queue.cancelled_ids.pop_front();
                }
            }
        }
    }

    pub fn is_cancelled(&self, id: &str) -> bool {
        self.shared
            .queue
            .lock()
            .map(|queue| queue.stopped || queue.cancelled_ids.iter().any(|known| known == id))
            .unwrap_or(true)
    }
}

impl Drop for Engine {
    fn drop(&mut self) {
        if let Ok(mut queue) = self.shared.queue.lock() {
            queue.stopped = true;
            if let Some((_, cancelled, _)) = &queue.active {
                cancelled.store(true, Ordering::Release);
            }
            if let Some(job) = queue.pending.take() {
                let _ = job.reply.send(Err("应用正在关闭".into()));
            }
            if let Some(job) = queue.background.take() {
                let _ = job.reply.send(Err("应用正在关闭".into()));
            }
            if let Some(job) = queue.endpoint.take() {
                let _ = job.reply.send(Err("应用正在关闭".into()));
            }
            self.shared.changed.notify_one();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use emulation_core::{
        kinematics::forward_kinematics, model::get_robot_model, program::ControllerTarget,
        types::Target,
    };

    fn request() -> PlanRequest {
        let model = get_robot_model("irb1100-4-0.58").unwrap();
        let pose = forward_kinematics(&model, &model.ready_joints)
            .unwrap()
            .pose;
        PlanRequest {
            model_id: model.id,
            start_joints: model.ready_joints,
            target: Target {
                position: [pose.position[0], 30.0, pose.position[2]],
                orientation: Some(pose.orientation),
            },
            speed_mm_per_second: 100.0,
        }
    }

    fn generic_request() -> GenericPlanRequest {
        let chains: Vec<emulation_core::generic_kinematics::GenericChain> = serde_json::from_str(
            include_str!("../../tests/fixtures/catalog/generic-chains.json"),
        )
        .unwrap();
        GenericPlanRequest {
            chain: chains[0].clone(),
            start_joints: [20., 15., -25., 25., 35., 10.],
            target_joints: [21., 14.5, -24.25, 25.5, 34.25, 11.],
            speed_mm_per_second: 100.,
        }
    }

    fn grinding_request() -> GrindingRequest {
        use emulation_core::{grinding::GrindingRobot, types::Pose};
        let motion = request();
        let pose = forward_kinematics(
            &get_robot_model(&motion.model_id).unwrap(),
            &motion.start_joints,
        )
        .unwrap()
        .pose;
        let identity = Pose {
            position: [0.; 3],
            orientation: [1., 0., 0., 0.],
        };
        GrindingRequest {
            robot: GrindingRobot::Abb {
                model_id: motion.model_id,
            },
            base_pose: identity,
            tool_pose: identity,
            tool_radius_mm: 1.,
            start_joints: motion.start_joints,
            targets: vec![
                Pose {
                    position: [pose.position[0], 4., pose.position[2]],
                    ..pose
                },
                Pose {
                    position: [pose.position[0], 8., pose.position[2]],
                    ..pose
                },
            ],
            orientation: Some(pose.orientation),
            zones: vec![],
            clearance_mm: 1.,
            speed_mm_per_second: 100.,
            allow_detours: false,
        }
    }

    #[test]
    fn grinding_worker_preserves_result_type_and_recovers_after_invalid_input() {
        let engine = Engine::new();
        let mut invalid = grinding_request();
        invalid.clearance_mm = -1.;
        let result = engine
            .submit("invalid-grinding".into(), Work::Grinding(invalid))
            .unwrap()
            .blocking_recv()
            .unwrap()
            .unwrap();
        assert!(
            matches!(result,Reply::Grinding(GrindingResult::Failure(error)) if error.code=="invalid_input")
        );
        let request = grinding_request();
        let result = engine
            .submit("valid-grinding".into(), Work::Grinding(request.clone()))
            .unwrap()
            .blocking_recv()
            .unwrap()
            .unwrap();
        let Reply::Grinding(GrindingResult::Success(plan)) = result else {
            panic!("expected verified native TCP path");
        };
        assert_eq!(plan.path[0].joints, request.start_joints);
        assert_eq!(plan.waypoints.len(), 2);
        assert!((plan.waypoints[1].pose.position[1] - 8.).abs() < 0.01);
        assert!(plan.checked_intervals > 0);
        assert_eq!(plan.waypoints[0].kind, "transfer");
        assert_eq!(plan.waypoints[1].kind, "process");
    }

    #[test]
    fn grinding_is_bounded_foreground_work_with_precancel_and_latest_request_wins() {
        let active = Arc::new(AtomicBool::new(false));
        let engine = Engine {
            shared: Arc::new(Shared {
                queue: Mutex::new(Queue {
                    active: Some(("inspection".into(), active.clone(), true)),
                    ..Queue::default()
                }),
                changed: Condvar::new(),
            }),
        };
        engine.cancel("precancel-grinding");
        assert!(engine
            .submit(
                "precancel-grinding".into(),
                Work::Grinding(grinding_request())
            )
            .is_err());
        let first = engine
            .submit("grinding-first".into(), Work::Grinding(grinding_request()))
            .unwrap();
        assert!(active.load(Ordering::Acquire));
        let latest = engine
            .submit("grinding-latest".into(), Work::Grinding(grinding_request()))
            .unwrap();
        assert!(first.blocking_recv().unwrap().is_err());
        engine.cancel("grinding-first");
        assert_eq!(
            engine
                .shared
                .queue
                .lock()
                .unwrap()
                .pending
                .as_ref()
                .unwrap()
                .id,
            "grinding-latest"
        );
        engine.cancel("grinding-latest");
        assert!(latest.blocking_recv().unwrap().is_err());
        assert!(engine.shared.queue.lock().unwrap().pending.is_none());
    }

    #[test]
    fn background_endpoint_does_not_cancel_active_grinding_but_new_motion_does() {
        let cancelled = Arc::new(AtomicBool::new(false));
        let engine = Engine {
            shared: Arc::new(Shared {
                queue: Mutex::new(Queue {
                    active: Some(("grinding-active".into(), cancelled.clone(), false)),
                    ..Queue::default()
                }),
                changed: Condvar::new(),
            }),
        };
        let _background = engine
            .submit(
                "background-endpoint".into(),
                Work::InspectEndpoint(endpoint_request()),
            )
            .unwrap();
        assert!(!cancelled.load(Ordering::Acquire));
        let _new = engine
            .submit("new-motion".into(), Work::Plan(request()))
            .unwrap();
        assert!(cancelled.load(Ordering::Acquire));
    }

    #[test]
    fn generic_foreground_work_uses_the_native_urdf_planner() {
        let engine = Engine::new();
        let request = generic_request();
        let response = engine
            .submit("generic-plan".into(), Work::GenericPlan(request.clone()))
            .unwrap()
            .blocking_recv()
            .unwrap()
            .unwrap();
        let Reply::GenericPlan(GenericPlanResult::Success(plan)) = response else {
            panic!("expected a native URDF Cartesian path");
        };
        assert!(plan.path.len() > 2);
        assert_eq!(plan.path[0].joints, request.start_joints);
        assert_eq!(plan.path.last().unwrap().joints, request.target_joints);
    }

    #[test]
    fn generic_work_has_foreground_priority_and_independent_cancellation() {
        let cancelled = Arc::new(AtomicBool::new(false));
        let engine = Engine {
            shared: Arc::new(Shared {
                queue: Mutex::new(Queue {
                    active: Some(("background-endpoint".into(), cancelled.clone(), true)),
                    ..Queue::default()
                }),
                changed: Condvar::new(),
            }),
        };
        let receiver = engine
            .submit(
                "generic-foreground".into(),
                Work::GenericPlan(generic_request()),
            )
            .unwrap();
        assert!(cancelled.load(Ordering::Acquire));
        {
            let queue = engine.shared.queue.lock().unwrap();
            assert_eq!(queue.pending.as_ref().unwrap().id, "generic-foreground");
            assert!(queue.endpoint.is_none());
        }
        engine.cancel("generic-foreground");
        assert!(receiver.blocking_recv().unwrap().is_err());
        assert!(engine.shared.queue.lock().unwrap().pending.is_none());
    }

    #[test]
    fn native_worker_returns_a_real_planned_path() {
        let engine = Engine::new();
        let response = engine
            .submit("plan-1".into(), Work::Plan(request()))
            .unwrap()
            .blocking_recv()
            .unwrap()
            .unwrap();
        let Reply::Plan(PlanResult::Success(plan)) = response else {
            panic!("expected a verified native path")
        };
        assert!(plan.path.len() >= 7);
        assert!((plan.selected.pose.position[1] - 30.0).abs() < 1e-6);
    }

    #[test]
    fn cancellation_before_async_submission_prevents_late_work() {
        let engine = Engine::new();
        engine.cancel("cancelled-before-submit");
        assert!(engine
            .submit("cancelled-before-submit".into(), Work::Plan(request()))
            .is_err());
        assert!(engine.is_cancelled("cancelled-before-submit"));
    }

    #[test]
    fn cancel_for_an_old_request_does_not_cancel_new_work() {
        let engine = Engine::new();
        engine.cancel("old-request");
        let response = engine
            .submit("new-request".into(), Work::Plan(request()))
            .unwrap()
            .blocking_recv()
            .unwrap()
            .unwrap();
        assert!(matches!(response, Reply::Plan(PlanResult::Success(_))));
        assert!(!engine.is_cancelled("new-request"));
    }

    #[test]
    fn worker_exports_a_revalidated_module_and_recovers_from_an_unsupported_controller() {
        let engine = Engine::new();
        let mut export = ProgramExportRequest {
            program_name: "WorkerTest".into(),
            controller: ControllerTarget::FanucLs,
            motion: request(),
        };
        assert!(engine
            .submit("unsupported".into(), Work::Export(export.clone()))
            .unwrap()
            .blocking_recv()
            .unwrap()
            .is_err());
        export.controller = ControllerTarget::AbbRapid;
        let response = engine
            .submit("export".into(), Work::Export(export))
            .unwrap()
            .blocking_recv()
            .unwrap()
            .unwrap();
        let Reply::Export(file) = response else {
            panic!("expected a generated module")
        };
        assert_eq!(file.file_name, "WorkerTest.mod");
        assert!(file.content.contains("CJointT"));
        assert!(file.content.contains("ENDMODULE"));
    }

    #[test]
    fn background_inspection_does_not_cancel_foreground_export() {
        use emulation_core::collision::{CollisionCandidate, CollisionRequest, CollisionZone};
        let engine = Engine::new();
        let motion = request();
        let export = engine
            .submit(
                "foreground-export".into(),
                Work::Export(ProgramExportRequest {
                    program_name: "Foreground".into(),
                    controller: ControllerTarget::AbbRapid,
                    motion: motion.clone(),
                }),
            )
            .unwrap();
        let inspection = engine
            .submit(
                "background-inspection".into(),
                Work::Collision(CollisionRequest {
                    base_pose: None,
                    model_id: motion.model_id,
                    zones: vec![CollisionZone {
                        id: "all-links".into(),
                        enabled: true,
                        center_mm: [0., 0., 400.],
                        size_mm: [4000., 4000., 4000.],
                    }],
                    solutions: vec![CollisionCandidate {
                        id: "endpoint".into(),
                        joints: motion.start_joints,
                    }],
                }),
            )
            .unwrap();
        assert!(matches!(
            export.blocking_recv().unwrap().unwrap(),
            Reply::Export(_)
        ));
        let Reply::Collision(results) = inspection.blocking_recv().unwrap().unwrap() else {
            panic!("expected native mesh inspection");
        };
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].colliding_links, vec![0, 1, 2, 3, 4, 5, 6]);
        assert!(results[0].uncertain_links.is_empty());
    }

    #[test]
    fn foreground_work_interrupts_active_inspection() {
        // Inspect the scheduler policy without racing a CPU-dependent mesh build.
        let cancelled = Arc::new(AtomicBool::new(false));
        let engine = Engine {
            shared: Arc::new(Shared {
                queue: Mutex::new(Queue {
                    active: Some(("inspection".into(), cancelled.clone(), true)),
                    ..Queue::default()
                }),
                changed: Condvar::new(),
            }),
        };
        let _receiver = engine
            .submit("interactive-motion".into(), Work::Plan(request()))
            .unwrap();
        assert!(cancelled.load(Ordering::Acquire));
        assert_eq!(
            engine
                .shared
                .queue
                .lock()
                .unwrap()
                .pending
                .as_ref()
                .unwrap()
                .id,
            "interactive-motion"
        );
    }

    #[test]
    fn inspection_cancellation_before_submit_is_preserved() {
        use emulation_core::collision::{CollisionRequest, CollisionZone};
        let engine = Engine::new();
        engine.cancel("obsolete-inspection");
        assert!(engine
            .submit(
                "obsolete-inspection".into(),
                Work::Collision(CollisionRequest {
                    base_pose: None,
                    model_id: "irb1100-4-0.58".into(),
                    zones: vec![CollisionZone {
                        id: "disabled".into(),
                        enabled: false,
                        center_mm: [0.; 3],
                        size_mm: [100.; 3]
                    }],
                    solutions: vec![],
                })
            )
            .is_err());
    }

    fn endpoint_request() -> EndpointInspectionRequest {
        let model = get_robot_model("irb1100-4-0.58").unwrap();
        EndpointInspectionRequest {
            model_id: model.id.clone(),
            pose: forward_kinematics(&model, &model.ready_joints)
                .unwrap()
                .pose,
            seed_joints: model.ready_joints,
        }
    }

    #[test]
    fn native_endpoint_work_returns_alternatives_without_a_motion_plan() {
        let engine = Engine::new();
        let response = engine
            .submit(
                "endpoint-ik".into(),
                Work::InspectEndpoint(endpoint_request()),
            )
            .unwrap()
            .blocking_recv()
            .unwrap()
            .unwrap();
        let Reply::InspectEndpoint(solutions) = response else {
            panic!("expected endpoint IK alternatives");
        };
        assert!(solutions.len() > 8);
        assert!(solutions
            .iter()
            .all(|solution| solution.position_error_mm <= 0.01));
        engine.cancel("obsolete-endpoint");
        assert!(engine
            .submit(
                "obsolete-endpoint".into(),
                Work::InspectEndpoint(endpoint_request())
            )
            .is_err());
    }

    #[test]
    fn endpoint_and_collision_have_independent_bounded_slots_and_do_not_cancel_foreground() {
        let foreground_cancelled = Arc::new(AtomicBool::new(false));
        let engine = Engine {
            shared: Arc::new(Shared {
                queue: Mutex::new(Queue {
                    active: Some((
                        "foreground-export".into(),
                        foreground_cancelled.clone(),
                        false,
                    )),
                    ..Queue::default()
                }),
                changed: Condvar::new(),
            }),
        };
        let old = engine
            .submit(
                "old-endpoint".into(),
                Work::InspectEndpoint(endpoint_request()),
            )
            .unwrap();
        let _collision = engine
            .submit(
                "mesh-inspection".into(),
                Work::Collision(CollisionRequest {
                    base_pose: None,
                    model_id: "irb1100-4-0.58".into(),
                    zones: vec![],
                    solutions: vec![],
                }),
            )
            .unwrap();
        let latest = engine
            .submit(
                "latest-endpoint".into(),
                Work::InspectEndpoint(endpoint_request()),
            )
            .unwrap();
        assert!(old.blocking_recv().unwrap().is_err());
        assert!(!foreground_cancelled.load(Ordering::Acquire));
        {
            let queue = engine.shared.queue.lock().unwrap();
            assert!(queue.pending.is_none());
            assert_eq!(queue.endpoint.as_ref().unwrap().id, "latest-endpoint");
            assert_eq!(queue.background.as_ref().unwrap().id, "mesh-inspection");
        }
        engine.cancel("latest-endpoint");
        assert!(latest.blocking_recv().unwrap().is_err());
        let queue = engine.shared.queue.lock().unwrap();
        assert!(queue.endpoint.is_none());
        assert!(queue.background.is_some());
        assert!(!foreground_cancelled.load(Ordering::Acquire));
    }
}
