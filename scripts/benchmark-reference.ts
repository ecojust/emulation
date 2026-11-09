/** TypeScript reference kernel measured with the exact same serialized fixtures as Rust. */
import { readFileSync } from 'node:fs';
import { performance } from 'node:perf_hooks';
import { forwardKinematics, inverseKinematics } from '../src/robot/kinematics';
import { ROBOT_MODELS, getRobotModel } from '../src/robot/model';
import { planMoveL } from '../src/robot/planner';
import type { JointAngles, PlanRequest, Pose, RobotId } from '../src/robot/types';

type Task = { op: 'fk'; modelId: RobotId; joints: JointAngles[] } | { op: 'ik'; modelId: RobotId; poses: Pose[]; seeds: JointAngles[] } | { op: 'plan'; requests: PlanRequest[] };
interface Case { name: string; iterations: number; task: Task }
export interface Input { samples: number; warmupBatches: number; cases: Case[] }
export function fixtures(): Input {
  const cases: Case[] = [];
  for (const model of Object.values(ROBOT_MODELS)) {
    const seeds: JointAngles[] = [model.readyJoints, [20, 20, -40, 25, 40, 370], [-70, -10, -85, 35, -30, 90], [90, 40, -100, -45, 60, -150]];
    const poses = seeds.map(seed => forwardKinematics(model, seed).pose);
    const start = poses[0];
    const request: PlanRequest = { modelId: model.id, startJoints: model.readyJoints, target: { position: [start.position[0] - 15, 30, start.position[2] - 10], orientation: start.orientation }, speedMmPerSecond: 100 };
    const rotated: JointAngles = [...model.readyJoints]; rotated[5] += 60;
    cases.push(
      { name: `${model.id}/fk`, iterations: 2000, task: { op: 'fk', modelId: model.id, joints: seeds } },
      { name: `${model.id}/ik`, iterations: 100, task: { op: 'ik', modelId: model.id, poses, seeds } },
      { name: `${model.id}/movel-35mm`, iterations: 20, task: { op: 'plan', requests: [request] } },
      { name: `${model.id}/rotation-60deg`, iterations: 10, task: { op: 'plan', requests: [{ ...request, target: forwardKinematics(model, rotated).pose }] } },
      { name: `${model.id}/auto-unreachable`, iterations: 20, task: { op: 'plan', requests: [{ ...request, target: { position: [2000, 0, 500], orientation: null } }] } },
    );
    if (model.id === 'irb1100-4-0.58') cases.push({ name: `${model.id}/auto-direction-search`, iterations: 5, task: { op: 'plan', requests: [{ ...request, target: { position: [200, 0, 100], orientation: null } }] } });
  }
  return { samples: 31, warmupBatches: 5, cases };
}
function prepare(task: Task): (iteration: number) => number {
  if (task.op === 'fk') {
    const model = getRobotModel(task.modelId);
    return iteration => { const result = forwardKinematics(model, task.joints[iteration % task.joints.length]); return result.pose.position[0] + result.pose.position[2]; };
  }
  if (task.op === 'ik') {
    const model = getRobotModel(task.modelId);
    return iteration => { const index = iteration % task.poses.length; const solutions = inverseKinematics(model, task.poses[index], task.seeds[index]); return solutions.length + (solutions[0]?.joints[0] ?? 0) / 1000; };
  }
  return iteration => { const result = planMoveL(task.requests[iteration % task.requests.length]); return result.status === 'success' ? result.lengthMm + result.orientationTravelDeg + result.selected.joints[0] / 1000 : -1; };
}
function measure(input: Input) {
  return input.cases.map(item => {
    const operation = prepare(item.task);
    let warmupChecksum = 0;
    for (let batch = 0; batch < input.warmupBatches; batch++) for (let i = 0; i < item.iterations; i++) warmupChecksum += operation(i);
    const batchMeanNs: number[] = [];
    let checksum = 0;
    for (let sample = 0; sample < input.samples; sample++) {
      const started = performance.now();
      for (let i = 0; i < item.iterations; i++) checksum += operation(i);
      batchMeanNs.push((performance.now() - started) * 1e6 / item.iterations);
    }
    if (!Number.isFinite(warmupChecksum) || !Number.isFinite(checksum)) throw new Error('Benchmark produced a non-finite checksum');
    return { name: item.name, iterations: item.iterations, batchMeanNs, checksum };
  });
}
if (process.argv[2] === '--fixtures') process.stdout.write(JSON.stringify(fixtures()));
else if (process.argv[2]) process.stdout.write(JSON.stringify(measure(JSON.parse(readFileSync(process.argv[2], 'utf8')))));
