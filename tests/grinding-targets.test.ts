import { execFileSync } from 'node:child_process';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import chainsJson from './fixtures/catalog/generic-chains.json';
import redundantFixture from './fixtures/grinding/redundant-lookahead.json';
import { forwardGenericKinematics, inverseGenericKinematics } from '../src/robot/generic-kinematics';
import { planGrindingReference } from '../src/robot/grinding-reference';
import { validateGrindingRequest, validateGrindingResult } from '../src/robot/grinding-validation';
import { composePose, identityPose } from '../src/workspace/frame-graph';
import { getRobotModel } from '../src/robot/model';
import { forwardKinematics, quaternionDistanceDeg, slerpQuaternion } from '../src/robot/kinematics';
import { getConfiguration } from '../src/robot/configuration';
import type { GrindingRequest, GrindingResult, GrindingSuccess } from '../src/robot/grinding-types';
import type { GenericChain } from '../src/robot/generic-motion-types';
import type { JointAngles, Pose, RobotId, Vec3 } from '../src/robot/types';
const nativePath=process.env.EMULATION_NATIVE_CORE||resolve(process.env.CARGO_TARGET_DIR||'target','debug','core-cli');
function native(request:GrindingRequest):GrindingResult {
  const response=JSON.parse(execFileSync(nativePath,{input:JSON.stringify({op:'grinding',request})+'\n',encoding:'utf8',timeout:60_000,maxBuffer:8*1024*1024}));
  expect(response.error).toBeUndefined();return response.result;
}
function checked(request:GrindingRequest,result=planGrindingReference(request)):GrindingSuccess {
  expect(result.status,JSON.stringify(result)).toBe('success');if(result.status!=='success')throw new Error(result.message);
  expect(validateGrindingResult(result,request)).toBe(result);
  const targets=result.waypoints.filter(point=>point.targetIndex!==null);
  expect(targets).toHaveLength(request.targets.length);
  targets.forEach((point,i)=>{expect(point.targetIndex).toBe(i);expect(point.pose.position).toEqual(request.targets[i].position);expect(Math.min(...[1,-1].map(sign=>Math.hypot(...point.pose.orientation.map((v,a)=>v-sign*request.targets[i].orientation[a]))))).toBeLessThan(1e-12);});
  return result;
}
function actual(request:GrindingRequest,joints:JointAngles):Pose {
  const flange=request.robot.kind==='abb'?forwardKinematics(getRobotModel(request.robot.modelId),joints).pose:forwardGenericKinematics(request.robot.chain,joints).pose;
  return composePose(composePose(request.basePose,flange),request.toolPose);
}
function request(modelId:RobotId='irb1100-4-0.58'):GrindingRequest {
  const model=getRobotModel(modelId);
  const value:GrindingRequest={robot:{kind:'abb',modelId:model.id},basePose:{position:[450,-230,120],orientation:[Math.cos(.3),0,0,Math.sin(.3)]},toolPose:{position:[30,15,70],orientation:[Math.cos(.15),Math.sin(.15),0,0]},toolRadiusMm:2,startJoints:[...model.readyJoints],targets:[],clearanceMm:1,speedMmPerSecond:100,zones:[],orientation:null,allowDetours:false};
  value.targets=[[1,20,-30,0,40,2],[2,20,-30,0,40,4]].map(q=>actual(value,q as JointAngles));return value;
}
function verifyInterpolation(request:GrindingRequest,result:GrindingSuccess) {
  let startIndex=0;
  for(const point of result.waypoints){
    const endIndex=result.path.findIndex((s,i)=>i>startIndex&&s.joints.every((v,a)=>Math.abs(v-point.joints[a])<1e-6));expect(endIndex).toBeGreaterThan(startIndex);
    const start=result.path[startIndex],end=result.path[endIndex];
    for(let i=startIndex+1;i<=endIndex;i++)for(const fraction of [.25,.5,.75]){
      const a=result.path[i-1],b=result.path[i],q=a.joints.map((v,j)=>v+(b.joints[j]-v)*fraction) as JointAngles;
      const local=(a.t+(b.t-a.t)*fraction-start.t)/(end.t-start.t);
      expect(quaternionDistanceDeg(actual(request,q).orientation,slerpQuaternion(start.pose.orientation,point.pose.orientation,local))).toBeLessThanOrEqual(.1+1e-8);
    }startIndex=endIndex;
  }
}
describe.each(['irb1100-4-0.475','irb1100-4-0.58'] as const)('%s target-preserving grinding',modelId=>{
  it('preserves distinct target poses under base/tool transforms in native and reference trajectories',()=>{
    const input=request(modelId),before=structuredClone(input),reference=checked(input),result=checked(input,native(input));expect(input).toEqual(before);
    expect(quaternionDistanceDeg(result.waypoints[0].pose.orientation,result.waypoints[1].pose.orientation)).toBeGreaterThan(1);
    expect(result.path).toHaveLength(reference.path.length);verifyInterpolation(input,result);verifyInterpolation(input,reference);
  });
  it('allows smooth cf1/cf4/cf6 quadrant crossings while retaining cfX',()=>{
    const input=request(modelId);input.basePose=identityPose();input.toolPose=identityPose();input.startJoints=[89.6,20,-30,89.6,40,89.6];
    input.targets=[[90.1,20,-30,90.1,40,90.1],[90.4,20,-30,90.4,40,90.4]].map(q=>actual(input,q as JointAngles));
    const result=checked(input),initial=getConfiguration(getRobotModel(modelId),input.startJoints,forwardKinematics(getRobotModel(modelId),input.startJoints)).configuration!;
    expect(initial.slice(0,3)).toEqual([0,0,0]);expect(result.waypoints.at(-1)!.configuration!.slice(0,3)).toEqual([1,1,1]);expect(result.waypoints.every(p=>p.configuration![3]===initial[3])).toBe(true);checked(input,native(input));
  });
});
it('uses each generic model and preserves its changing target directions',()=>{
  for(const raw of chainsJson){const chain=structuredClone(raw) as GenericChain,input=request();input.startJoints=[20,15,-25,25,35,10];
    input.robot={kind:'generic',chain,bodies:[0,...chain.steps.flatMap((step,index)=>step.jointIndex===null?[]:[index+1])].map(stepIndex=>({stepIndex,centerMm:[0,0,0] as Vec3,halfSizeMm:[1,1,1] as Vec3}))};
    input.targets=[1,2].map(factor=>actual(input,input.startJoints.map((v,i)=>v+factor*[.3,-.2,.3,.2,-.2,.5][i]) as JointAngles));
    const result=checked(input);checked(input,native(input));expect(result.waypoints.every(p=>p.configuration===null)).toBe(true);verifyInterpolation(input,result);
  }
});
it('rejects incompatible legacy orientation overrides and wrist branch flips without a partial path',()=>{
  const input=request();input.orientation=input.targets[0].orientation;expect(()=>validateGrindingRequest(input)).toThrow('不会覆盖');
  for(const value of [planGrindingReference(input),native(input)]){expect(value).toMatchObject({status:'failure',code:'invalid_input'});expect(value).not.toHaveProperty('path');}
  input.orientation=null;input.basePose=identityPose();input.toolPose=identityPose();input.startJoints=[0,20,-30,0,-.2,0];input.targets=[actual(input,[0,20,-30,0,.2,0])];
  for(const value of [planGrindingReference(input),native(input)]){expect(value.status).toBe('failure');expect(value).not.toHaveProperty('path');}
});
it('recovers a redundant-chain greedy dead end with bounded continuous candidates',()=>{
  // Synthetic planar six-axis chain, not a manufacturer model. The original
  // single seeded solution reached target0 but could not continue to target1.
  const input=structuredClone(redundantFixture) as unknown as GrindingRequest;
  if(input.robot.kind!=='generic')throw new Error('fixture');const chain=input.robot.chain;
  let greedy=input.startJoints,failed=false;
  for(const target of input.targets){const origin=forwardGenericKinematics(chain,greedy).pose,count=Math.max(1,Math.ceil(Math.hypot(...target.position.map((v,i)=>v-origin.position[i]))/2),Math.ceil(quaternionDistanceDeg(origin.orientation,target.orientation)));
    for(let i=1;i<=count;i++){const t=i/count,pose={position:origin.position.map((v,a)=>v+(target.position[a]-v)*t) as Vec3,orientation:slerpQuaternion(origin.orientation,target.orientation,t)},q=inverseGenericKinematics(chain,pose,greedy);if(!q){failed=true;break;}greedy=q;}if(failed)break;
  }expect(failed).toBe(true);
  const result=checked(input),nativeResult=checked(input,native(input));expect(result.checkedIntervals).toBeLessThanOrEqual(100000);expect(nativeResult.checkedIntervals).toBeLessThanOrEqual(100000);expect(result.waypoints).toHaveLength(2);
},60_000);
it('rejects altered target poses, invalid branches, excessive speeds and partial failure payloads',()=>{
  const input=request(),result=checked(input);
  for(const mutate of [(r:GrindingSuccess)=>{r.waypoints[0].pose.position[0]+=.0001;},(r:GrindingSuccess)=>{r.waypoints[1].pose.orientation=[...r.waypoints[0].pose.orientation];},(r:GrindingSuccess)=>{r.waypoints.reverse();},(r:GrindingSuccess)=>{r.durationSeconds=1e-8;}]){const bad=structuredClone(result);mutate(bad);expect(()=>validateGrindingResult(bad,input)).toThrow();}
  expect(()=>validateGrindingResult({status:'failure',code:'cancelled',message:'cancelled',path:result.path},input)).toThrow();
  let polls=0;const cancelled=planGrindingReference(structuredClone(redundantFixture) as unknown as GrindingRequest,()=>++polls>1500);expect(cancelled).toMatchObject({status:'failure',code:'cancelled'});expect(cancelled).not.toHaveProperty('waypoints');
});
