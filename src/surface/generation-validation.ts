import { quaternionDistanceDeg, quaternionToMatrix } from '../robot/kinematics';
import type { Quaternion, Vec3 } from '../robot/types';
import type { SurfaceAnchor, SurfaceMesh } from './types';
import { SURFACE_LIMITS } from './types';
import type { SurfaceGeneratedPoint, SurfaceRequest, SurfaceResult, SurfaceSuccess } from './generation-types';

const record = (value: unknown): value is Record<string, any> => Boolean(value) && typeof value === 'object' && !Array.isArray(value);
function requireValue(condition: unknown, message: string): asserts condition { if (!condition) throw new Error('表面路径数据无效：' + message); }
const finiteTuple = (value: unknown, count: number): value is number[] => Array.isArray(value) && value.length === count && value.every(item => typeof item === 'number' && Number.isFinite(item));
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const dot = (a: Vec3, b: Vec3) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross = (a: Vec3,b: Vec3):Vec3 => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const vector = (values: number[], offset: number): Vec3 => [values[offset], values[offset+1], values[offset+2]];
function unit(a: Vec3): Vec3 { const length=Math.hypot(...a); requireValue(length>1e-12,'零法向量'); return a.map(value=>value/length) as Vec3; }
function validateAnchor(value: unknown, triangles: number): asserts value is SurfaceAnchor {
  requireValue(record(value) && Number.isInteger(value.triangleIndex) && value.triangleIndex>=0 && value.triangleIndex<triangles,'三角面引用');
  requireValue(finiteTuple(value.barycentric,3) && value.barycentric.every(n=>n>=-1e-8 && n<=1+1e-8) && Math.abs(value.barycentric.reduce((a,b)=>a+b,0)-1)<=1e-6,'三角面重心坐标');
}
/** Independently reconstruct the actual mesh location and normal of an anchor. */
export function surfaceAnchorValue(mesh: SurfaceMesh, anchor: SurfaceAnchor, normalSign: 1|-1=1): {position:Vec3; normal:Vec3} {
  validateAnchor(anchor,mesh.positions.length/9);
  const position:Vec3=[0,0,0], normal:Vec3=[0,0,0], offset=anchor.triangleIndex*9;
  for(let vertex=0;vertex<3;vertex++)for(let axis=0;axis<3;axis++) {
    position[axis]+=anchor.barycentric[vertex]*mesh.positions[offset+vertex*3+axis];
    normal[axis]+=anchor.barycentric[vertex]*mesh.normals[offset+vertex*3+axis]*normalSign;
  }
  return {position,normal:unit(normal)};
}
/** Reconstruct the regional chart independently of the solver's BVH and row tracing. */
function regionalChart(request: SurfaceRequest) {
  const positions=request.boundary.map(anchor=>surfaceAnchorValue(request.mesh,anchor).position),corners=request.controlIndices.map(index=>positions[index]);
  const origin=corners[0],x=unit(sub(corners[1],origin)),z=unit(cross(x,sub(corners[3],origin))),y=cross(z,x);
  const project=(p:Vec3):[number,number]=>{const d=sub(p,origin);return[dot(d,x),dot(d,y)];};
  const boundary=positions.map(project),controls=request.controlIndices.map(index=>boundary[index]);
  const edges=request.controlIndices.map((start,k)=>{
    const points:[number,number][]=[];for(let index=start;;index=(index+1)%boundary.length){points.push(boundary[index]);if(index===request.controlIndices[(k+1)%4])break;}
    if(k>=2)points.reverse();const knots=[0];for(let i=1;i<points.length;i++)knots.push(knots[i-1]+Math.hypot(points[i][0]-points[i-1][0],points[i][1]-points[i-1][1]));
    const length=knots[knots.length-1];requireValue(length>1e-8,'区域控制边界长度');for(let i=0;i<knots.length;i++)knots[i]/=length;
    return(t:number)=>{const found=knots.findIndex(v=>v>=t),index=Math.max(1,Math.min(found<0?knots.length:found,knots.length-1)),fraction=Math.max(0,Math.min(1,(t-knots[index-1])/(knots[index]-knots[index-1])));return points[index-1].map((v,axis)=>v+(points[index][axis]-v)*fraction);};
  });
  const at=(u:number,v:number)=>{const b=edges[0](u),r=edges[1](v),t=edges[2](u),l=edges[3](v);return[0,1].map(i=>(1-v)*b[i]+v*t[i]+(1-u)*l[i]+u*r[i]-((1-u)*(1-v)*controls[0][i]+u*(1-v)*controls[1][i]+u*v*controls[2][i]+(1-u)*v*controls[3][i]));};
  const inside=(p:number[])=>{let yes=false;for(let i=0;i<boundary.length;i++){const a=boundary[i],b=boundary[(i+1)%boundary.length],dx=b[0]-a[0],dy=b[1]-a[1],length=dx*dx+dy*dy,t=length?Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length)):0;if(Math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)<1e-6)return true;if((a[1]>p[1])!==(b[1]>p[1])&&p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0])yes=!yes;}return yes;};
  const opposite=corners[1].map((v,i)=>v+corners[3][i]-origin[i]) as Vec3;
  let planarRectangle=Math.abs(dot(x,unit(sub(corners[3],origin))))<=1e-8&&Math.hypot(...sub(opposite,corners[2]))<=1e-6;
  // Exact pitch evidence is available only for a coplanar mesh and straight
  // rectangular boundaries, not merely because two endpoints share a face.
  if(planarRectangle)for(let offset=0;offset<request.mesh.positions.length;offset+=3)if(Math.abs(dot(sub(vector(request.mesh.positions,offset),origin),z))>1e-6){planarRectangle=false;break;}
  if(planarRectangle)for(let edge=0;edge<4;edge++) {
    const start=corners[edge],end=corners[(edge+1)%4],length=Math.hypot(...sub(end,start)),direction=unit(sub(end,start));let previousDistance=-1e-8;
    for(let index=request.controlIndices[edge];;index=(index+1)%positions.length){
      const offset=sub(positions[index],start),distance=dot(offset,direction);
      if(Math.hypot(...cross(offset,direction))>1e-6||distance<previousDistance-1e-8||distance>length+1e-8){planarRectangle=false;break;}
      previousDistance=distance;if(index===request.controlIndices[(edge+1)%4])break;
    }
  }
  return{project,at,inside,planarRectangle};
}
/** Copy a bounded snapshot before asynchronous native or browser computation. */
export function validateSurfaceRequest(raw: unknown): SurfaceRequest {
  requireValue(record(raw) && record(raw.mesh),'请求');
  const {positions,normals}=raw.mesh;
  requireValue(Array.isArray(positions) && positions.length>=9 && positions.length%9===0 && positions.length<=SURFACE_LIMITS.triangles*9,'模型三角面数量');
  requireValue(Array.isArray(normals) && normals.length===positions.length,'模型法向量数量');
  requireValue(positions.every(n=>typeof n==='number' && Number.isFinite(n) && Math.abs(n)<=SURFACE_LIMITS.coordinateMm),'模型坐标');
  requireValue(normals.every(n=>typeof n==='number' && Number.isFinite(n)),'模型法向量');
  for(let offset=0;offset<positions.length;offset+=9) {
    const a=vector(positions,offset),b=vector(positions,offset+3),c=vector(positions,offset+6),face=cross(sub(b,a),sub(c,a));
    requireValue(Math.hypot(...face)>SURFACE_LIMITS.minimumDoubleAreaMm2,'退化三角面');
    for(let vertex=0;vertex<3;vertex++) { const n=vector(normals,offset+vertex*3); requireValue(Math.abs(Math.hypot(...n)-1)<=1e-4 && dot(n,face)>0,'法向量须为与模型绕序一致的单位向量'); }
  }
  requireValue(Array.isArray(raw.boundary) && raw.boundary.length>=4 && raw.boundary.length<=256,'区域边界需要4至256个点');
  raw.boundary.forEach((anchor:unknown)=>validateAnchor(anchor,positions.length/9));
  requireValue(finiteTuple(raw.controlIndices,4) && new Set(raw.controlIndices).size===4 && raw.controlIndices.every(index=>Number.isInteger(index)&&index>=0&&index<raw.boundary.length),'区域需要4个不同控制点');
  const order=raw.controlIndices.map(index=>(index-raw.controlIndices[0]+raw.boundary.length)%raw.boundary.length);
  requireValue(order[0]===0 && order.slice(1).every((index,i)=>index>order[i]),'控制点必须按边界顺序排列');
  requireValue(record(raw.sampling) && ['spacing','divisions'].includes(raw.sampling.mode),'UV采样模式');
  for(const value of [raw.sampling.u,raw.sampling.v])requireValue(Number.isFinite(value)&&value>0&&value<=1e6&&(raw.sampling.mode!=='divisions'||Number.isInteger(value)&&value<=255),'UV数值');
  requireValue(raw.normalSign===1||raw.normalSign===-1,'法向方向');
  requireValue(Number.isFinite(raw.maxNormalStepDeg)&&raw.maxNormalStepDeg>0&&raw.maxNormalStepDeg<=45,'最大姿态变化角');
  requireValue(Number.isInteger(raw.maxPoints)&&raw.maxPoints>=4&&raw.maxPoints<=256,'路径点数量上限');
  return {mesh:{positions:[...positions],normals:[...normals]},boundary:raw.boundary.map((anchor:SurfaceAnchor)=>({triangleIndex:anchor.triangleIndex,barycentric:[...anchor.barycentric]})),controlIndices:[...raw.controlIndices] as [number,number,number,number],sampling:{mode:raw.sampling.mode,u:raw.sampling.u,v:raw.sampling.v},normalSign:raw.normalSign,maxNormalStepDeg:raw.maxNormalStepDeg,maxPoints:raw.maxPoints};
}
/** Native and worker results are untrusted until mesh provenance and frames agree. */
export function validateSurfaceResult(raw: unknown, request: SurfaceRequest): SurfaceResult {
  requireValue(record(raw),'返回结果');
  if(raw.status==='failure') {
    requireValue(typeof raw.code==='string'&&raw.code.length>0&&raw.code.length<=100&&typeof raw.message==='string'&&raw.message.length>0&&raw.message.length<=2000,'失败原因');
    requireValue(raw.points===undefined,'失败结果不能包含部分路径');
    return {status:'failure',code:raw.code,message:raw.message};
  }
  requireValue(raw.status==='success' && Array.isArray(raw.points) && raw.points.length>=4&&raw.points.length<=request.maxPoints,'返回点数量');
  requireValue(Array.isArray(raw.rowOffsets)&&raw.rowOffsets.length>=2&&raw.rowOffsets.length<=raw.points.length&&raw.rowOffsets[0]===0&&raw.rowOffsets.every((offset:number,i:number)=>Number.isInteger(offset)&&offset>=0&&offset<raw.points.length&&(i===0||offset>raw.rowOffsets[i-1])),'UV行引用');
  requireValue(Array.isArray(raw.warnings)&&raw.warnings.length<=32&&raw.warnings.every((text:unknown)=>typeof text==='string'&&text.length<=2000),'结果说明');
  for(const metric of [raw.maxSpacingErrorMm,raw.maxRegularSpacingErrorMm,raw.maxOrientationStepDeg])requireValue(Number.isFinite(metric)&&metric>=0,'间距或姿态误差');
  const points:SurfaceGeneratedPoint[]=[],chart=regionalChart(request); let measuredStep=0;
  for(let index=0;index<raw.points.length;index++) {
    const point=raw.points[index];requireValue(record(point)&&record(point.pose)&&finiteTuple(point.pose.position,3)&&finiteTuple(point.pose.orientation,4),'返回位姿');
    requireValue(Math.abs(Math.hypot(...point.pose.orientation)-1)<=1e-6&&finiteTuple(point.normal,3)&&Math.abs(Math.hypot(...point.normal)-1)<=1e-6,'返回方向');
    validateAnchor(point.anchor,request.mesh.positions.length/9);
    const expected=surfaceAnchorValue(request.mesh,point.anchor,request.normalSign);
    requireValue(Math.hypot(...sub(point.pose.position as Vec3,expected.position))<=1e-5,'生成点未落在声明的模型表面');
    requireValue(Math.hypot(...sub(point.normal as Vec3,expected.normal))<=1e-5,'生成法向与模型表面不符');
    const matrix=quaternionToMatrix(point.pose.orientation as Quaternion),z:Vec3=[matrix[2],matrix[6],matrix[10]];
    requireValue(Math.hypot(...sub(z,expected.normal))<=1e-5,'工具Z轴没有跟随表面法向');
    requireValue(finiteTuple(point.uv,2)&&point.uv.every(n=>n>=-1e-8&&n<=1+1e-8),'UV坐标');
    const projected=chart.project(point.pose.position as Vec3),expectedUv=chart.at(point.uv[0],point.uv[1]);
    requireValue(chart.inside(projected)&&Math.hypot(projected[0]-expectedUv[0],projected[1]-expectedUv[1])<=1e-5,'生成点与点选区域或UV参数不符');
    const nextRow=raw.rowOffsets.findIndex((offset:number)=>offset>index),row=nextRow<0?raw.rowOffsets.length-1:nextRow-1;
    requireValue(point.row===row,'路径行顺序');
    if(index>0) {
      const previous=points[index-1],orientation=point.pose.orientation as Quaternion;
      requireValue(previous.pose.orientation.reduce((sum,value,i)=>sum+value*orientation[i],0)>=-1e-8,'姿态四元数符号跳变');
      const step=quaternionDistanceDeg(previous.pose.orientation,orientation);
      measuredStep=Math.max(measuredStep,step);
      requireValue(step<=request.maxNormalStepDeg+1e-5,'路径姿态发生突变');
      if(previous.row===row)requireValue((row%2===0?1:-1)*(point.uv[0]-previous.uv[0])>0&&Math.abs(point.uv[1]-previous.uv[1])<1e-6,'UV行内顺序');
      else requireValue(point.uv[1]>previous.uv[1],'UV行方向');
    }
    points.push({pose:{position:[...point.pose.position] as Vec3,orientation:[...point.pose.orientation] as Quaternion},normal:[...point.normal] as Vec3,anchor:{triangleIndex:point.anchor.triangleIndex,barycentric:[...point.anchor.barycentric]},uv:[...point.uv] as [number,number],row});
  }
  for(let row=0;row<raw.rowOffsets.length;row++) {
    const start=raw.rowOffsets[row],end=raw.rowOffsets[row+1]??points.length;
    requireValue(end-start>=2,'每条UV行至少需要两个端点');
    requireValue(Math.abs(points[start].uv[0]-(row%2?1:0))<=1e-6&&Math.abs(points[end-1].uv[0]-(row%2?0:1))<=1e-6,'UV行未覆盖区域两侧');
  }
  requireValue(Math.abs(points[0].uv[1])<=1e-6&&Math.abs(points[points.length-1].uv[1]-1)<=1e-6,'UV路径未覆盖区域首尾');
  if(request.sampling.mode==='spacing') {
    let exactMax=0,exactRegular=0;
    for(let row=0;row<raw.rowOffsets.length;row++) {
      const start=raw.rowOffsets[row],end=raw.rowOffsets[row+1]??points.length;
      for(let index=start+1;index<end;index++) {
        // Distances are generated from U=0 before odd rows are reversed. Their
        // shorter terminal interval is therefore at the beginning of odd rows.
        const chord=Math.hypot(...sub(points[index].pose.position,points[index-1].pose.position)),regular=row%2===1?index>start+1:index+1<end;
        // Every U interval has arc length <= the requested pitch. A large V or
        // terminal-error report cannot excuse a missing interior U sample.
        requireValue(chord<=request.sampling.u+1e-5,'U间距小于实际弦长下界');
        if(chart.planarRectangle){const error=Math.abs(chord-request.sampling.u);exactMax=Math.max(exactMax,error);if(regular)exactRegular=Math.max(exactRegular,error);}
      }
      if(row===0)continue;
      const previousStart=raw.rowOffsets[row-1],regular=row+1<raw.rowOffsets.length,reported=regular?raw.maxRegularSpacingErrorMm:raw.maxSpacingErrorMm;
      for(let index=start;index<end;index++)for(let before=previousStart;before<start;before++)if(points[index].uv[0]===points[before].uv[0]) {
        const chord=Math.hypot(...sub(points[index].pose.position,points[before].pose.position));requireValue(chord<=request.sampling.v+reported+1e-5,'V间距误差报告小于实际弦长下界');
      }
      if(chart.planarRectangle){const current=chart.project(points[start].pose.position),previous=chart.project(points[previousStart].pose.position),distance=current[1]-previous[1],error=Math.abs(distance-request.sampling.v);requireValue(distance<=request.sampling.v+1e-5,'平面矩形区域缺少中间V采样行');exactMax=Math.max(exactMax,error);if(regular)exactRegular=Math.max(exactRegular,error);}
    }
    if(chart.planarRectangle)requireValue(Math.abs(raw.maxSpacingErrorMm-exactMax)<=1e-5&&Math.abs(raw.maxRegularSpacingErrorMm-exactRegular)<=1e-5,'平面矩形区域的间距误差报告不符');
  }
  requireValue(raw.maxRegularSpacingErrorMm<=raw.maxSpacingErrorMm+1e-6,'间距误差报告');
  requireValue(Math.abs(measuredStep-raw.maxOrientationStepDeg)<=1e-4&&measuredStep<=request.maxNormalStepDeg+1e-5,'姿态连续性报告');
  return {status:'success',points,rowOffsets:[...raw.rowOffsets],maxSpacingErrorMm:raw.maxSpacingErrorMm,maxRegularSpacingErrorMm:raw.maxRegularSpacingErrorMm,maxOrientationStepDeg:raw.maxOrientationStepDeg,warnings:[...raw.warnings]} as SurfaceSuccess;
}
