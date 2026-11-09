//! One CPU worker with one pending motion/export and one pending inspection.
//! Background collision inspection must never cancel a user's solve or export.
//! Cancellation never creates an unbounded spawn_blocking queue.
use emulation_core::{
    collision::{check_collisions, CollisionRequest, CollisionResult},
    planner::plan_move_l_with_cancel,
    program::{export_program_with_cancel, ProgramExportRequest, ProgramFile},
    types::{PlanRequest, PlanResult},
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
    Export(ProgramExportRequest),
    Collision(CollisionRequest),
}
pub enum Reply {
    Plan(PlanResult),
    Export(ProgramFile),
    Collision(Vec<CollisionResult>),
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
                    while queue.pending.is_none() && queue.background.is_none() && !queue.stopped {
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
                        .or_else(|| queue.background.take())
                        .expect("pending job checked");
                    queue.active = Some((
                        job.id.clone(),
                        job.cancelled.clone(),
                        matches!(&job.work, Work::Collision(_)),
                    ));
                    job
                };
                let result = catch_unwind(AssertUnwindSafe(|| match job.work {
                    Work::Plan(request) => Ok(Reply::Plan(plan_move_l_with_cancel(
                        &request,
                        &job.cancelled,
                    ))),
                    Work::Export(request) => export_program_with_cancel(&request, &job.cancelled)
                        .map(Reply::Export)
                        .map_err(|e| e.to_string()),
                    Work::Collision(request) => {
                        check_collisions(&request, &job.cancelled).map(Reply::Collision)
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
        let is_background = matches!(&work, Work::Collision(_));
        if !is_background {
            if let Some((_, cancelled, active_background)) = &queue.active {
                if !*active_background {
                    cancelled.store(true, Ordering::Release);
                }
            }
        }
        let slot = if is_background {
            &mut queue.background
        } else {
            &mut queue.pending
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
            panic!("expected native bounding-box inspection");
        };
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].colliding_links, vec![0, 1, 2, 3, 4, 5, 6]);
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
}
