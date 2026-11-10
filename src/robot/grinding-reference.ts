import { getRobotModel } from './model';
import { forwardKinematics, inverseKinematics, jointsWithinLimits, matrixToQuaternion, multiplyMatrices, normalizeQuaternion, quaternionDistanceDeg, quaternionToMatrix, slerpQuaternion } from './kinematics';
import { getConfiguration } from './configuration';
import { forwardGenericUnchecked, GenericMotionCancelled, genericJointsWithinLimits, solveGenericTaskUnchecked } from './generic-kinematics';
import { intersectsDangerZone, linkBoundingBoxes, type LinkBoundingBox } from './collision-geometry';
import { GRINDING_LIMITS as LIMIT, validateGrindingRequest } from './grinding-validation';
import type { GrindingFailure, GrindingRequest, GrindingResult, GrindingSuccess, GrindingWaypoint } from './grinding-types';
import type { JointAngles, Mat4, MotionSample, Pose, Quaternion, RobConf, Vec3 } from './types';

const IDENTITY: Mat4 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const norm = (value: readonly number[]) => Math.hypot(...value);
class PlanningFailure extends Error {
  constructor(readonly code: string, message: string, public failedTarget?: number) { super(message); }
}
const fail = (code: string, message: string): never => { throw new PlanningFailure(code, message); };
function poseMatrix(pose: Pose): Mat4 {
  const m = quaternionToMatrix(pose.orientation);
  for (let i = 0; i < 3; i++) m[i * 4 + 3] = pose.position[i];
  return m;
}
const matrixPose = (m: Mat4): Pose => ({ position: [m[3], m[7], m[11]], orientation: matrixToQuaternion(m) });
function inverse(m: Mat4): Mat4 {
  const out = [...IDENTITY];
  for (let row = 0; row < 3; row++) {
    for (let col = 0; col < 3; col++) out[row * 4 + col] = m[col * 4 + row];
    out[row * 4 + 3] = -[0, 1, 2].reduce((sum, k) => sum + out[row * 4 + k] * m[k * 4 + 3], 0);
  }
  return out;
}
function rotation(axis: Vec3, degrees: number): Mat4 {
  const [x, y, z] = axis, s = Math.sin(degrees * Math.PI / 180), c = Math.cos(degrees * Math.PI / 180), d = 1 - c;
  return [c+x*x*d,x*y*d-z*s,x*z*d+y*s,0,y*x*d+z*s,c+y*y*d,y*z*d-x*s,0,z*x*d-y*s,z*y*d+x*s,c+z*z*d,0,0,0,0,1];
}
function transformedBox(m: Mat4, center: Vec3, half: Vec3, index = 0): LinkBoundingBox {
  return { index, centerMm: [0, 1, 2].map(row => m[row * 4 + 3] + center.reduce((sum, p, k) => sum + m[row * 4 + k] * p, 0)) as Vec3,
    axes: [[m[0], m[4], m[8]], [m[1], m[5], m[9]], [m[2], m[6], m[10]]], halfSizeMm: half };
}
interface Leg { samples: MotionSample[]; duration: number; length: number; cost: number }
const sameJoints = (a: JointAngles, b: JointAngles) => a.every((v,i) => Math.abs(v-b[i]) <= 0.001);
const compareJoints = (a: JointAngles, b: JointAngles) => a.map((v,i) => v-b[i]).find(v => v !== 0) ?? 0;
const jointCost = (a: JointAngles,b: JointAngles) => a.reduce((sum,v,i) => sum + Math.abs(v-b[i]) * (i >= 3 ? 1.5 : 1),0);
class Context {
  base: Mat4; baseInverse: Mat4; tool: Mat4; toolInverse: Mat4;
  radiusBound: number; speedLimits: number[];
  ikCalls = 0; checkedIntervals = 0;
  constructor(readonly request: GrindingRequest, readonly cancel: () => boolean) {
    this.base = poseMatrix(request.basePose); this.baseInverse = inverse(this.base);
    this.tool = poseMatrix(request.toolPose); this.toolInverse = inverse(this.tool);
    let reach = 0;
    if (request.robot.kind === 'abb') {
      const model = getRobotModel(request.robot.modelId);
      if (!jointsWithinLimits(model, request.startJoints)) fail('joint_limit', '起点关节超出限位。');
      const boxes = linkBoundingBoxes(model.id, [0, 0, 0, 0, 0, 0]);
      reach = 2 * model.dh.reduce((sum, row) => sum + Math.hypot(row.aMm, row.dMm), 0)
        + Math.max(...boxes.map(box => norm(box.centerMm) + norm(box.halfSizeMm)));
      this.speedLimits = model.limits.map(limit => limit.maxSpeedDeg);
    } else {
      const { chain, bodies } = request.robot;
      if (!genericJointsWithinLimits(chain, request.startJoints)) fail('joint_limit', '起点关节超出限位。');
      reach = chain.steps.reduce((sum, step) => sum + Math.hypot(step.origin[3], step.origin[7], step.origin[11]), 0)
        + Math.max(...bodies.map(body => norm(body.centerMm) + norm(body.halfSizeMm)));
      this.speedLimits = chain.limits.map(limit => limit.maxSpeedDeg);
    }
    this.radiusBound = reach + norm(request.toolPose.position) + Math.sqrt(3) * Math.max(request.toolRadiusMm, 1e-6);
  }
  check() { if (this.cancel()) fail('cancelled', '路径规划已取消，未返回部分结果。'); }
  flange(q: JointAngles): Mat4 {
    this.check();
    return this.request.robot.kind === 'abb' ? forwardKinematics(getRobotModel(this.request.robot.modelId), q).frames[6]
      : forwardGenericUnchecked(this.request.robot.chain, q, this.cancel).transform;
  }
  tcp(q: JointAngles): Pose { return matrixPose(multiplyMatrices(multiplyMatrices(this.base, this.flange(q)), this.tool)); }
  config(q: JointAngles): RobConf | null {
    if (this.request.robot.kind !== 'abb') return null;
    const model = getRobotModel(this.request.robot.modelId), result = getConfiguration(model, q, forwardKinematics(model, q));
    if (result.singular) fail('singular_path', '路径进入奇异区域，未生成连续路径。');
    return result.configuration;
  }
  ikCandidates(pose: Pose, seed: JointAngles, explore: boolean): JointAngles[] {
    this.check();
    const baseTcp = matrixPose(multiplyMatrices(this.baseInverse,poseMatrix(pose)));
    const target = matrixPose(multiplyMatrices(poseMatrix(baseTcp),this.toolInverse));
    let candidates: JointAngles[] = [];
    const budget = () => { this.check(); if (++this.ikCalls > LIMIT.ikCalls) fail('resource_limit','路径搜索达到逆解预算，未返回部分路径。'); };
    if (this.request.robot.kind === 'abb') {
      budget(); const config = this.config(seed); if (!config) fail('singular_path','起点没有有效机器人配置。');
      // Only cfX denotes a physical branch; quadrant counters may change smoothly.
      candidates = inverseKinematics(getRobotModel(this.request.robot.modelId),target,seed).filter(solution => solution.configuration[3] === config![3]).map(solution => solution.joints);
    } else {
      const { chain } = this.request.robot, seeds = [seed];
      if (explore) for (let axis = 0; axis < 6; axis++) for (const offset of [-10,10]) { const next = [...seed] as JointAngles; next[axis] = Math.max(chain.limits[axis].minDeg,Math.min(chain.limits[axis].maxDeg,next[axis]+offset)); seeds.push(next); }
      for (const trial of seeds) { budget(); const q = solveGenericTaskUnchecked(chain,baseTcp,trial,this.tool,false,this.cancel); if (q && !candidates.some(old => sameJoints(old,q))) candidates.push(q); }
    }
    const hadCandidates = candidates.length > 0;
    candidates = candidates.filter(q => q.every((v,i) => Math.abs(v-seed[i]) <= LIMIT.jointStepDeg + 1e-8));
    if (hadCandidates && !candidates.length) fail('configuration_change','相邻采样关节变化超过15°，需要细化路径。');
    candidates.sort((a,b) => jointCost(seed,a)-jointCost(seed,b) || compareJoints(a,b));
    if (!candidates.length) fail('unreachable','未找到保持目标姿态且连续的逆解，不能跳换关节分支。');
    return candidates;
  }
  boxes(q: JointAngles): LinkBoundingBox[] {
    this.check();
    let boxes: LinkBoundingBox[];
    if (this.request.robot.kind === 'abb') {
      boxes = linkBoundingBoxes(this.request.robot.modelId, q).map(box => {
        const a = box.axes, p = box.centerMm;
        const m = [a[0][0],a[1][0],a[2][0],p[0],a[0][1],a[1][1],a[2][1],p[1],a[0][2],a[1][2],a[2][2],p[2],0,0,0,1];
        return transformedBox(multiplyMatrices(this.base, m), [0, 0, 0], box.halfSizeMm, box.index);
      });
    } else {
      const { chain, bodies } = this.request.robot, frames: Mat4[] = [[...IDENTITY]];
      let m = [...IDENTITY];
      for (const step of chain.steps) {
        this.check(); m = multiplyMatrices(m, step.origin);
        if (step.jointIndex !== null) m = multiplyMatrices(m, rotation(step.axis, q[step.jointIndex]));
        frames.push(m);
      }
      boxes = bodies.map((body, i) => transformedBox(multiplyMatrices(this.base, frames[body.stepIndex]), body.centerMm, body.halfSizeMm, i));
    }
    const offset = this.request.toolPose.position, radius = Math.max(this.request.toolRadiusMm, 1e-6);
    boxes.push(transformedBox(multiplyMatrices(this.base, this.flange(q)), offset.map(v => v / 2) as Vec3, offset.map(v => Math.abs(v) / 2 + radius) as Vec3));
    return boxes;
  }
  clear(boxes: LinkBoundingBox[], padding: number): boolean {
    return boxes.every(box => !this.request.zones.some(zone => zone.enabled && intersectsDangerZone({ ...box, halfSizeMm: box.halfSizeMm.map(value => value + padding) as Vec3 }, zone)));
  }
  certify(a: JointAngles, b: JointAngles, depth = 0, expected: [Quaternion, Quaternion] | null = null) {
    this.check();
    if (++this.checkedIntervals > LIMIT.intervalChecks) fail('resource_limit', '连续包络检查达到预算，不能确认路径安全。');
    const mid = a.map((value, i) => (value + b[i]) / 2) as JointAngles;
    const boxes = this.boxes(mid), sweep = this.radiusBound * a.reduce((sum, value, i) => sum + Math.abs(value - b[i]) * Math.PI / 180, 0) / 2;
    const expectedMid = expected ? slerpQuaternion(expected[0],expected[1],0.5) : null;
    if (this.request.robot.kind === 'abb' && this.config(mid)?.[3] !== this.config(a)?.[3]) fail('configuration_change','关节插值跨越机器人分支，未生成路径。');
    let orientationClear = true;
    if (expected) {
      const error = quaternionDistanceDeg(this.tcp(mid).orientation,expectedMid!);
      if (error > 0.1 + 1e-8) fail('orientation_tolerance','关节插值偏离目标姿态插值超过0.1°，需要细化路径。');
      orientationClear = error + a.reduce((sum,value,i) => sum+Math.abs(value-b[i]),0)/2 + quaternionDistanceDeg(expected[0],expected[1])/2 <= 0.1;
    }
    if (orientationClear && this.clear(boxes, this.request.clearanceMm + sweep + 1e-7)) return;
    if (!this.clear(boxes, this.request.clearanceMm)) fail('collision', '机器人或工具的保守包络与禁入区相交。');
    if (depth >= LIMIT.certDepth) fail('uncertain', '连续运动包络或姿态插值无法在检查精度内完成验证，未生成路径。');
    this.certify(a,mid,depth+1,expected ? [expected[0],expectedMid!] : null); this.certify(mid,b,depth+1,expected ? [expectedMid!,expected[1]] : null);
  }
  legs(start: JointAngles,target: Pose,explore: boolean): Leg[] {
    const origin = this.tcp(start), length = norm(target.position.map((v,i) => v-origin.position[i]));
    const angle = quaternionDistanceDeg(origin.orientation,target.orientation);
    const initial = Math.max(1,Math.ceil(length/LIMIT.translationStepMm),Math.ceil(angle/LIMIT.orientationStepDeg));
    let lastError = new PlanningFailure('unreachable','未找到连续运动。');
    for (let refinement = 0; refinement <= LIMIT.legRefinements; refinement++) {
      const count = initial * 2 ** refinement; if (count >= LIMIT.samples) fail('sample_limit','单段路径超过采样上限。');
      try { return this.legsAtResolution(start,origin,target,length,angle,count,explore); }
      catch (error) { if (error instanceof PlanningFailure && ['orientation_tolerance','configuration_change'].includes(error.code)) lastError = error; else throw error; }
    }
    throw lastError;
  }
  legsAtResolution(start: JointAngles,origin: Pose,target: Pose,length: number,angle: number,count: number,explore: boolean): Leg[] {
    let beam: Leg[] = [{ samples: [],duration: Math.max(length/this.request.speedMmPerSecond,angle/500,0.001),length,cost:length*0.01 }];
    for (let index = 1; index <= count; index++) {
      this.check(); const t = index/count;
      const wanted: Pose = { position: origin.position.map((v,i) => v+t*(target.position[i]-v)) as Vec3,orientation:slerpQuaternion(origin.orientation,target.orientation,t) };
      const previousOrientation = slerpQuaternion(origin.orientation,target.orientation,(index-1)/count);
      const next: Leg[] = []; let error = new PlanningFailure('unreachable','未找到连续运动。');
      for (const state of beam) {
        const previous = state.samples.at(-1)?.joints ?? start; let candidates: JointAngles[];
        try { candidates = this.ikCandidates(wanted,previous,explore); } catch (e) { if (retryable(e)) { error=e; continue; } throw e; }
        let accepted=0;
        for (const q of candidates) {
          try { this.certify(previous,q,0,[previousOrientation,wanted.orientation]); } catch (e) { if (retryable(e)) { error=e; continue; } throw e; }
          const actual = this.tcp(q);
          if (norm(actual.position.map((v,i)=>v-wanted.position[i])) > 0.002 || quaternionDistanceDeg(actual.orientation,wanted.orientation) > 0.002) { error=new PlanningFailure('unreachable','实际TCP未达到输入位姿精度，未生成路径。'); continue; }
          let duration=state.duration; for (let j=0;j<6;j++) duration=Math.max(duration,Math.abs(q[j]-previous[j])/this.speedLimits[j]*count);
          if (!Number.isFinite(duration)) fail('invalid_input','关节速度产生非有限运动时间，未生成路径。');
          next.push({ samples:[...state.samples,{t,pose:actual,joints:q}],duration,length,cost:state.cost+jointCost(previous,q) });
          if (++accepted===LIMIT.beamWidth) break;
        }
      }
      next.sort((a,b)=>a.cost-b.cost||compareJoints(a.samples.at(-1)!.joints,b.samples.at(-1)!.joints));
      const distinct: Leg[]=[]; for (const state of next) if (!distinct.some(old=>sameJoints(old.samples.at(-1)!.joints,state.samples.at(-1)!.joints))) { distinct.push(state); if (distinct.length===LIMIT.beamWidth) break; }
      if (!distinct.length) throw error; beam=distinct;
    }
    return beam;
  }
}
const retryable = (error: unknown): error is PlanningFailure => error instanceof PlanningFailure && ['collision', 'uncertain', 'unreachable', 'configuration_change', 'singular_path', 'orientation_tolerance','sample_limit'].includes(error.code);
interface SearchState { current:JointAngles; path:MotionSample[]; waypoints:GrindingWaypoint[]; duration:number; length:number; cost:number; usedDetour:boolean }
function prune(states:SearchState[]):SearchState[] {
  states.sort((a,b)=>a.cost-b.cost||compareJoints(a.current,b.current)); const kept:SearchState[]=[];
  for (const state of states) if (!kept.some(other=>sameJoints(other.current,state.current))) { kept.push(state); if (kept.length===LIMIT.beamWidth) break; }
  return kept;
}
function routes(request:GrindingRequest,start:Pose,target:Pose,explore:boolean):Pose[][] {
  const result:Pose[][]=[[target]]; if (!explore||request.allowDetours===false||!request.zones.some(z=>z.enabled)) return result;
  for (const extra of [25,75]) for (let axis=0;axis<3;axis++) for (const sign of [-1,1]) {
    const edges=request.zones.filter(z=>z.enabled).map(z=>z.centerMm[axis]+sign*z.sizeMm[axis]/2);
    const edge=sign<0?Math.min(...edges):Math.max(...edges),plane=edge+sign*(request.toolRadiusMm+request.clearanceMm+extra);
    const p1:Pose={position:[...start.position],orientation:slerpQuaternion(start.orientation,target.orientation,1/3)},p2:Pose={position:[...target.position],orientation:slerpQuaternion(start.orientation,target.orientation,2/3)};
    p1.position[axis]=plane;p2.position[axis]=plane;
    if ([p1,p2].every(p=>p.position.every(x=>Number.isFinite(x)&&Math.abs(x)<=1e6))) result.push([p1,p2,target]);
  }
  return result;
}
function search(context:Context,initial:SearchState,explore:boolean):SearchState {
  let beam=[initial];
  for (const [targetIndex,input] of context.request.targets.entries()) {
    const target:Pose={position:[...input.position],orientation:normalizeQuaternion(input.orientation)};
    let next:SearchState[]=[],failure=new PlanningFailure('unreachable','有界搜索未找到保留全部目标位姿的连续路径。');
    for (const state of beam) {
      const start=context.tcp(state.current);
      for (const route of routes(context.request,start,target,explore)) {
        let origin=start,estimate=state.path.length;
        for(const pose of route){estimate+=Math.max(1,Math.ceil(norm(pose.position.map((v,i)=>v-origin.position[i]))/LIMIT.translationStepMm),Math.ceil(quaternionDistanceDeg(origin.orientation,pose.orientation)/LIMIT.orientationStepDeg));origin=pose;}
        if(estimate>LIMIT.samples){failure=new PlanningFailure('sample_limit','整条路径超过采样上限，未返回部分结果。');continue;}
        const detour=route.length>1;let partial=[state];
        for (const [routeIndex,pose] of route.entries()) {
          const children:SearchState[]=[];
          for (const prefix of partial) {
            let legs:Leg[];try { legs=context.legs(prefix.current,pose,explore); } catch(error) { if(retryable(error)){failure=error;continue;}throw error; }
            for(const leg of legs){
              if(prefix.path.length+leg.samples.length>LIMIT.samples){failure=new PlanningFailure('sample_limit','整条路径超过采样上限，未返回部分结果。');continue;}
              const endpoint=leg.samples.at(-1)!,duration=prefix.duration+leg.duration,length=prefix.length+leg.length,cost=prefix.cost+leg.cost;
              if(![duration,length,cost].every(Number.isFinite))fail('resource_limit','累计路径长度或时间超过数值范围。');
              children.push({current:endpoint.joints,path:[...prefix.path,...leg.samples.map(sample=>({...sample,t:prefix.duration+sample.t*leg.duration}))],
                waypoints:[...prefix.waypoints,{pose,joints:endpoint.joints,configuration:context.config(endpoint.joints),targetIndex:routeIndex===route.length-1?targetIndex:null,kind:!detour&&targetIndex>0?'process':'transfer'}],duration,length,cost,usedDetour:prefix.usedDetour||detour});
            }
          }
          partial=prune(children);if(!partial.length)break;
        }
        next=prune([...next,...partial]);
        if(targetIndex+1===context.request.targets.length&&next.length)return next[0];
      }
    }
    if(!next.length){failure.failedTarget=targetIndex;throw failure;}beam=prune(next);
  }
  if(!beam.length)fail('unreachable','未找到完整路径。');return beam[0];
}
/** Bounded candidate/route beam; every accepted edge retains swept-volume and orientation certification. */
export function planGrindingReference(input:GrindingRequest,cancel=()=>false):GrindingResult {
  try {
    if(cancel())fail('cancelled','路径规划已取消，未返回部分结果。');
    let request:GrindingRequest;try{request=validateGrindingRequest(input);}catch(error){return{status:'failure',code:'invalid_input',message:(error as Error).message};}
    const context=new Context(request,cancel),orientation=request.orientation?normalizeQuaternion(request.orientation):null;
    context.config(request.startJoints);context.certify(request.startJoints,request.startJoints);
    const initial:SearchState={current:request.startJoints,path:[{t:0,pose:context.tcp(request.startJoints),joints:request.startJoints}],waypoints:[],duration:0,length:0,cost:0,usedDetour:false};
    let found:SearchState;try{found=search(context,initial,false);}catch(error){if(!retryable(error))throw error;found=search(context,initial,true);}
    context.check();let previousTime=-1;
    const path=found.path.map(sample=>({...sample,t:sample.t/found.duration}));
    for(const sample of path){if(!Number.isFinite(sample.t)||sample.t<0||sample.t>1||sample.t<=previousTime)fail('resource_limit','路径采样时间超出数值精度，未返回部分路径。');previousTime=sample.t;}
    const warnings=['输入目标的位置和姿态保持不变；使用有界候选搜索选择连续关节路径，不保证全局最优或穷举全部解。','已验证声明的机器人／工具保守包络与禁入区在关节插值全程分离；姿态跟随目标间的四元数插值；未包含自碰撞、实际工件接触、控制器或动力学验收。'];
    if(found.usedDetour)warnings.push('绕行点为转移运动，须在加工工艺中关闭打磨接触；不得将绕行当作表面加工轨迹。');
    return{status:'success',path,waypoints:found.waypoints,durationSeconds:found.duration,lengthMm:found.length,orientation,checkedIntervals:context.checkedIntervals,warnings};
  }catch(error){
    if(error instanceof GenericMotionCancelled)return{status:'failure',code:'cancelled',message:'路径规划已取消，未返回部分结果。'};
    if(error instanceof PlanningFailure){const result:GrindingFailure={status:'failure',code:error.code==='sample_limit'?'resource_limit':error.code,message:error.message};if(error.failedTarget!==undefined)result.failedTarget=error.failedTarget;return result;}throw error;
  }
}
