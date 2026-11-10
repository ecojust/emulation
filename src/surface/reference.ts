/** Browser reference of robot-core/surface.rs. Desktop generation always uses Rust. */
import { matrixToQuaternion, quaternionDistanceDeg } from '../robot/kinematics';
import type { Quaternion, Vec3 } from '../robot/types';
import type { SurfaceAnchor, SurfaceMesh } from './types';
import type { SurfaceGeneratedPoint, SurfaceRequest, SurfaceResult } from './generation-types';

type V2=[number,number];type Bounds=[number,number,number,number];
const EPS=1e-8,MAX_WORK=8_000_000;
class SurfaceError extends Error { constructor(readonly code:string,message:string){super(message);} }
const fail=(code:string,message:string):never=>{throw new SurfaceError(code,message);};
const plus=(a:Vec3,b:Vec3):Vec3=>a.map((n,i)=>n+b[i]) as Vec3;
const sub=(a:Vec3,b:Vec3):Vec3=>a.map((n,i)=>n-b[i]) as Vec3;
const scale=(a:Vec3,s:number):Vec3=>a.map(n=>n*s) as Vec3;
const mix=(a:Vec3,b:Vec3,t:number):Vec3=>a.map((n,i)=>n+(b[i]-n)*t) as Vec3;
const mix2=(a:V2,b:V2,t:number):V2=>a.map((n,i)=>n+(b[i]-n)*t) as V2;
const cross2=(a:V2,b:V2)=>a[0]*b[1]-a[1]*b[0];
const sub2=(a:V2,b:V2):V2=>[a[0]-b[0],a[1]-b[1]];
const distance2=(a:V2,b:V2)=>Math.hypot(a[0]-b[0],a[1]-b[1]);
const dot=(a:Vec3,b:Vec3)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a:Vec3,b:Vec3):Vec3=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=(a:Vec3)=>Math.hypot(Math.hypot(a[0],a[1]),a[2]);
const unit=(a:Vec3):Vec3|null=>{const n=norm(a);return Number.isFinite(n)&&n>=1e-10?scale(a,1/n):null;};
const angle=(a:Vec3,b:Vec3)=>Math.acos(Math.max(-1,Math.min(1,dot(a,b))))*180/Math.PI;
const rawVertex=(values:number[],triangle:number,vertex:number):Vec3=>{const i=triangle*9+vertex*3;return[values[i],values[i+1],values[i+2]];};
const clamp=(n:number,a:number,b:number)=>Math.max(a,Math.min(b,n));
const uniqueSorted=(values:number[],epsilon=1e-12)=>{values.sort((a,b)=>a-b);const result:number[]=[];for(const value of values)if(!result.length||Math.abs(value-result[result.length-1])>=epsilon)result.push(value);return result;};
interface Sample {anchor:SurfaceAnchor;position:Vec3;normal:Vec3}
function anchorSample(mesh:SurfaceMesh,anchor:SurfaceAnchor,sign:number):Sample {
  if(anchor.triangleIndex>=mesh.positions.length/9||anchor.barycentric.some(v=>!Number.isFinite(v)||v< -1e-7||v>1+1e-7)||Math.abs(anchor.barycentric.reduce((a,b)=>a+b,0)-1)>1e-6)fail('invalid_input','表面点的三角形或重心坐标无效。');
  let position:Vec3=[0,0,0],normal:Vec3=[0,0,0];
  for(let vertex=0;vertex<3;vertex++){position=plus(position,scale(rawVertex(mesh.positions,anchor.triangleIndex,vertex),anchor.barycentric[vertex]));normal=plus(normal,scale(rawVertex(mesh.normals,anchor.triangleIndex,vertex),anchor.barycentric[vertex]));}
  const n=unit(normal);if(!n)fail('invalid_input','表面法向量无效。');return{anchor,position,normal:scale(n!,sign)};
}
class Budget {work=0;constructor(readonly cancel:()=>boolean){}check(amount:number){this.work+=amount;if(this.cancel())fail('cancelled','表面路径生成已取消。');if(this.work>MAX_WORK)fail('resource_limit','表面区域过于复杂，请减少模型三角形或缩小点选区域。');}}
class Triangle {
  constructor(readonly original:number,readonly p:[V2,V2,V2],readonly bounds:Bounds,readonly denominator:number){}
  barycentric(p:V2):Vec3 {const b=cross2(sub2(p,this.p[0]),sub2(this.p[2],this.p[0]))/this.denominator,c=cross2(sub2(this.p[1],this.p[0]),sub2(p,this.p[0]))/this.denominator;return[1-b-c,b,c];}
  clip(a:V2,b:V2):[number,number]|null {const ba=this.barycentric(a),bb=this.barycentric(b);let lo=0,hi=1;for(let i=0;i<3;i++){const d=bb[i]-ba[i];if(Math.abs(d)<1e-14){if(ba[i]< -1e-9)return null;}else{const t=-ba[i]/d;if(d>0)lo=Math.max(lo,t);else hi=Math.min(hi,t);}}return hi>=lo-1e-10&&hi>=0&&lo<=1?[Math.max(lo,0),Math.min(hi,1)]:null;}
}
const overlaps=(a:Bounds,b:Bounds)=>a[0]<=b[2]+EPS&&a[2]+EPS>=b[0]&&a[1]<=b[3]+EPS&&a[3]+EPS>=b[1];
class Node {
  constructor(readonly bounds:Bounds,readonly children:[Node,Node]|null,readonly triangles:number[]){}
  static build(indices:number[],triangles:Triangle[],budget:Budget):Node {budget.check(indices.length);const bounds:Bounds=[Infinity,Infinity,-Infinity,-Infinity];for(const i of indices){const b=triangles[i].bounds;bounds[0]=Math.min(bounds[0],b[0]);bounds[1]=Math.min(bounds[1],b[1]);bounds[2]=Math.max(bounds[2],b[2]);bounds[3]=Math.max(bounds[3],b[3]);}if(indices.length<=12)return new Node(bounds,null,indices);const axis=bounds[3]-bounds[1]>bounds[2]-bounds[0]?1:0;indices.sort((a,b)=>(triangles[a].bounds[axis]+triangles[a].bounds[axis+2])-(triangles[b].bounds[axis]+triangles[b].bounds[axis+2])||a-b);const middle=Math.floor(indices.length/2);return new Node(bounds,[Node.build(indices.slice(0,middle),triangles,budget),Node.build(indices.slice(middle),triangles,budget)],[]);}
  candidates(bounds:Bounds,result:number[],budget:Budget):void {budget.check(1);if(!overlaps(this.bounds,bounds))return;if(this.children){this.children[0].candidates(bounds,result,budget);this.children[1].candidates(bounds,result,budget);}else result.push(...this.triangles);}
}
class Polyline {
  readonly knots:number[];
  constructor(readonly points:V2[]){const knots=[0];for(let i=1;i<points.length;i++)knots.push(knots[knots.length-1]+distance2(points[i-1],points[i]));const total=knots[knots.length-1];if(total<=EPS)fail('invalid_region','控制点之间的边界长度为零。');this.knots=knots.map(n=>n/total);}
  at(t:number):V2 {const first=this.knots.findIndex(v=>v>=t),i=clamp(first<0?this.knots.length:first,1,this.knots.length-1),d=this.knots[i]-this.knots[i-1];return mix2(this.points[i-1],this.points[i],clamp((t-this.knots[i-1])/d,0,1));}
}
class Chart {
  constructor(readonly boundary:V2[],readonly edges:[Polyline,Polyline,Polyline,Polyline],readonly corners:[V2,V2,V2,V2],readonly origin:Vec3,readonly x:Vec3,readonly y:Vec3,readonly z:Vec3){}
  project(p:Vec3):V2 {const d=sub(p,this.origin);return[dot(d,this.x),dot(d,this.y)];}
  at(u:number,v:number):V2 {const b=this.edges[0].at(u),r=this.edges[1].at(v),t=this.edges[2].at(u),l=this.edges[3].at(v),c=this.corners;return[0,1].map(i=>(1-v)*b[i]+v*t[i]+(1-u)*l[i]+u*r[i]-((1-u)*(1-v)*c[0][i]+u*(1-v)*c[1][i]+u*v*c[2][i]+(1-u)*v*c[3][i])) as V2;}
  knots(horizontal:boolean):number[]{const[a,b]=horizontal?[0,2]:[1,3];return uniqueSorted([...this.edges[a].knots,...this.edges[b].knots]);}
}
function pointSegmentDistance(p:V2,a:V2,b:V2):number {const ab=sub2(b,a),ap=sub2(p,a),d=ab[0]*ab[0]+ab[1]*ab[1],t=d>0?(ap[0]*ab[0]+ap[1]*ab[1])/d:0;return distance2(p,mix2(a,b,clamp(t,0,1)));}
function inside(p:V2,polygon:V2[],strict:boolean):boolean {let yes=false;for(let i=0;i<polygon.length;i++){const a=polygon[i],b=polygon[(i+1)%polygon.length];if(pointSegmentDistance(p,a,b)<1e-6)return!strict;if((a[1]>p[1])!==(b[1]>p[1])&&p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0])yes=!yes;}return yes;}
function intersectionParameter(a:V2,b:V2,c:V2,d:V2):number|null {const ab=sub2(b,a),cd=sub2(d,c),det=cross2(ab,cd);if(Math.abs(det)<1e-12)return null;const t=cross2(sub2(c,a),cd)/det,u=cross2(sub2(c,a),ab)/det;return t>=-1e-10&&t<=1+1e-10&&u>=-1e-10&&u<=1+1e-10?clamp(t,0,1):null;}
function edgeInside(a:V2,b:V2,polygon:V2[]):boolean {const cuts=[0,1];for(let i=0;i<polygon.length;i++){const t=intersectionParameter(a,b,polygon[i],polygon[(i+1)%polygon.length]);if(t!==null)cuts.push(t);}cuts.sort((a,b)=>a-b);return cuts.some((t,i)=>i>0&&t-cuts[i-1]>1e-10&&inside(mix2(a,b,(t+cuts[i-1])/2),polygon,true));}
function buildChart(request:SurfaceRequest,budget:Budget):Chart {
  const n=request.boundary.length,controls=request.controlIndices;if(n<4||n>256)fail('invalid_input','点选边界需包含4至256个点。');if(controls.some(i=>i>=n))fail('invalid_region','控制点索引无效。');const offsets=controls.map(i=>(i+n-controls[0])%n);if(!(offsets[0]===0&&offsets[1]>0&&offsets[2]>offsets[1]&&offsets[3]>offsets[2]))fail('invalid_region','4个控制点必须按边界顺序排列且互不重复。');
  const samples=request.boundary.map(a=>anchorSample(request.mesh,a,request.normalSign)),origin=samples[controls[0]].position,x=unit(sub(samples[controls[1]].position,origin));if(!x)fail('invalid_region','U方向控制点重合。');const z=unit(cross(x!,sub(samples[controls[3]].position,origin)));if(!z)fail('invalid_region','控制点共线，无法定义UV区域。');const y=cross(z!,x!),project=(p:Vec3):V2=>{const d=sub(p,origin);return[dot(d,x!),dot(d,y)];},boundary=samples.map(s=>project(s.position));
  for(let i=0;i<n;i++){if(distance2(boundary[i],boundary[(i+1)%n])<1e-6)fail('invalid_region','边界在UV投影中重合或折叠。');for(let j=i+1;j<n;j++){budget.check(1);if(j===i+1||(i===0&&j===n-1))continue;const[a,b,c,d]=[boundary[i],boundary[(i+1)%n],boundary[j],boundary[(j+1)%n]];if(intersectionParameter(a,b,c,d)!==null||pointSegmentDistance(c,a,b)<1e-6||pointSegmentDistance(d,a,b)<1e-6)fail('invalid_region','点选边界自交或重叠，请按边缘顺序重新点选。');}}
  const edges:Polyline[]=[];for(let k=0;k<4;k++){const points:V2[]=[];let i=controls[k];for(;;){points.push(boundary[i]);if(i===controls[(k+1)%4])break;i=(i+1)%n;}if(k>=2)points.reverse();edges.push(new Polyline(points));}
  const chart=new Chart(boundary,edges as [Polyline,Polyline,Polyline,Polyline],controls.map(i=>boundary[i]) as [V2,V2,V2,V2],origin,x!,y,z!),us=chart.knots(true),vs=chart.knots(false);
  for(let ui=1;ui<us.length;ui++)for(let vi=1;vi<vs.length;vi++)for(const a of [0,.5,1])for(const b of [0,.5,1]){budget.check(1);const u=[us[ui-1],us[ui]],v=[vs[vi-1],vs[vi]],uu=u[0]+(u[1]-u[0])*a,vv=v[0]+(v[1]-v[0])*b,du=sub2(chart.at(u[1],vv),chart.at(u[0],vv)),dv=sub2(chart.at(uu,v[1]),chart.at(uu,v[0]));if(cross2(du,dv)<=1e-10||!inside(chart.at(uu,vv),chart.boundary,false))fail('folded_region','控制点形成折叠UV区域，请选择较小且单值的表面区域。');}
  return chart;
}
function validateRequest(request:SurfaceRequest,budget:Budget):void {
  const m=request.mesh;if(!m||!Array.isArray(m.positions)||!m.positions.length||m.positions.length%9||m.positions.length/9>100_000||!Array.isArray(m.normals)||m.normals.length!==m.positions.length)fail('invalid_input','模型需包含1至100000个有效三角形以及对应顶点法向。');
  if(!Number.isInteger(request.maxPoints)||request.maxPoints<4||request.maxPoints>256||![1,-1].includes(request.normalSign)||!Number.isFinite(request.maxNormalStepDeg)||request.maxNormalStepDeg<=0||request.maxNormalStepDeg>45)fail('invalid_input','表面路径的数量或法向连续性设置无效。');
  const s=request.sampling,valid=s.mode==='spacing'?[s.u,s.v].every(v=>Number.isFinite(v)&&v>0&&v<=1e6):s.mode==='divisions'&&[s.u,s.v].every(v=>Number.isFinite(v)&&v>=1&&v<=255&&Number.isInteger(v));if(!valid)fail('invalid_input','UV间距必须是正的毫米数值，或有效的整数细分数量。');
  for(let i=0;i<m.positions.length/9;i++){budget.check(1);const p=[rawVertex(m.positions,i,0),rawVertex(m.positions,i,1),rawVertex(m.positions,i,2)];if(p.flat().some(v=>!Number.isFinite(v)||Math.abs(v)>1e6))fail('invalid_input','模型坐标超出有效毫米范围。');const face=unit(cross(sub(p[1],p[0]),sub(p[2],p[0])));if(!face)fail('invalid_input','模型含退化三角形，请先修复网格。');for(let j=0;j<3;j++){const n=rawVertex(m.normals,i,j);if(n.some(v=>!Number.isFinite(v))||Math.abs(norm(n)-1)>1e-4||dot(n,face!)<=1e-6)fail('invalid_input','模型法向必须归一化且与三角形绕序同向。');}}
}
function meshTriangles(request:SurfaceRequest,chart:Chart,budget:Budget):Triangle[] {
  const first=request.boundary[request.controlIndices[0]].triangleIndex,p=[rawVertex(request.mesh.positions,first,0),rawVertex(request.mesh.positions,first,1),rawVertex(request.mesh.positions,first,2)],orientation=Math.sign(dot(cross(sub(p[1],p[0]),sub(p[2],p[0])),chart.z));if(orientation===0)fail('folded_region','所选表面与UV投影方向平行。');
  const triangles:Triangle[]=[];for(let i=0;i<request.mesh.positions.length/9;i++){budget.check(1);const p=[chart.project(rawVertex(request.mesh.positions,i,0)),chart.project(rawVertex(request.mesh.positions,i,1)),chart.project(rawVertex(request.mesh.positions,i,2))] as [V2,V2,V2],denominator=cross2(sub2(p[1],p[0]),sub2(p[2],p[0]));if(denominator*orientation<=1e-10)continue;const bounds:Bounds=[Math.min(...p.map(n=>n[0])),Math.min(...p.map(n=>n[1])),Math.max(...p.map(n=>n[0])),Math.max(...p.map(n=>n[1]))];triangles.push(new Triangle(i,p,bounds,denominator));}
  const originalIndices=new Set(triangles.map(t=>t.original));if(!triangles.length||request.boundary.some(a=>!originalIndices.has(a.triangleIndex)))fail('folded_region','边界跨越了表面背面或投影折叠，请缩小区域。');return triangles;
}
interface Edge {a:Vec3;b:Vec3;na:Vec3;nb:Vec3;face:Vec3;forward:boolean}
const vertexKey=(p:Vec3)=>p.map(v=>v<0?-Math.round(-v*1e6):Math.round(v*1e6));
function compareKey(a:number[],b:number[]):number {for(let i=0;i<3;i++){const c=a[i]-b[i];if(c)return c;}return 0;}
function checkTopology(request:SurfaceRequest,chart:Chart,triangles:Triangle[],budget:Budget):void {
  const edges=new Map<string,Edge[]>();for(const triangle of triangles){budget.check(1);const i=triangle.original,p=[rawVertex(request.mesh.positions,i,0),rawVertex(request.mesh.positions,i,1),rawVertex(request.mesh.positions,i,2)],face=unit(cross(sub(p[1],p[0]),sub(p[2],p[0])))!;for(const pair of [[0,1],[1,2],[2,0]]){let[a,b]=pair;const ka=vertexKey(p[a]),kb=vertexKey(p[b]),forward=compareKey(ka,kb)<0;if(!forward)[a,b]=[b,a];const key=(forward?[ka,kb]:[kb,ka]).map(k=>k.join(',')).join('|'),adjacent=edges.get(key)??[];adjacent.push({a:p[a],b:p[b],na:rawVertex(request.mesh.normals,i,a),nb:rawVertex(request.mesh.normals,i,b),face,forward});edges.set(key,adjacent);}}
  for(const adjacent of edges.values()){budget.check(1);const edge=adjacent[0],manifold=adjacent.length===2&&adjacent[0].forward!==adjacent[1].forward,smooth=manifold&&angle(adjacent[0].face,adjacent[1].face)<=45+1e-7&&angle(adjacent[0].na,adjacent[1].na)<=request.maxNormalStepDeg+1e-6&&angle(adjacent[0].nb,adjacent[1].nb)<=request.maxNormalStepDeg+1e-6;if(smooth)continue;budget.check(chart.boundary.length);if(!edgeInside(chart.project(edge.a),chart.project(edge.b),chart.boundary))continue;if(!manifold)fail('surface_hole','点选区域跨越网格孔洞、非流形边或投影边界，请修复网格或缩小区域。');fail('normal_discontinuity','点选区域跨越锐边或不连续法向，无法生成连续加工姿态。');}
}
interface CurveSection {t0:number;t1:number;start:Sample;end:Sample;length:number;offset:number}
class SurfaceCurve {
  constructor(readonly horizontal:boolean,readonly fixed:number,readonly sections:CurveSection[],readonly length:number){}
  sample(distance:number,mesh:SurfaceMesh,sign:number):[Sample,V2] {distance=clamp(distance,0,this.length);const first=this.sections.findIndex(s=>s.offset+s.length>=distance),index=Math.min(first<0?this.sections.length:first,this.sections.length-1),s=this.sections[index],t=clamp((distance-s.offset)/s.length,0,1),anchor={triangleIndex:s.start.anchor.triangleIndex,barycentric:mix(s.start.anchor.barycentric,s.end.anchor.barycentric,t)},sample=anchorSample(mesh,anchor,sign),parameter=s.t0+(s.t1-s.t0)*t;return[sample,this.horizontal?[parameter,this.fixed]:[this.fixed,parameter]];}
}
class Context {
  readonly byOriginal:Map<number,Triangle>;
  constructor(readonly request:SurfaceRequest,readonly chart:Chart,readonly triangles:Triangle[],readonly tree:Node,readonly budget:Budget){this.byOriginal=new Map(triangles.map(t=>[t.original,t]));}
  hit(p:V2):Sample {const candidates:number[]=[];this.tree.candidates([p[0],p[1],p[0],p[1]],candidates,this.budget);let result:Sample|null=null;for(const i of candidates){this.budget.check(1);const t=this.triangles[i],bary=t.barycentric(p);if(bary.some(v=>v< -1e-8||v>1+1e-8))continue;const clamped=bary.map(v=>clamp(v,0,1)),sum=clamped.reduce((a,b)=>a+b,0),anchor={triangleIndex:t.original,barycentric:clamped.map(v=>v/sum) as Vec3},sample=anchorSample(this.request.mesh,anchor,this.request.normalSign);if(result){if(norm(sub(sample.position,result.position))>1e-5)fail('ambiguous_surface','UV投影落在多层表面上，请点选不重叠的单值表面区域。');if(angle(sample.normal,result.normal)>this.request.maxNormalStepDeg+1e-6)fail('normal_discontinuity','相邻表面的法向不连续。');if(sample.anchor.triangleIndex<result.anchor.triangleIndex)result=sample;}else result=sample;}if(!result)fail('surface_hole','UV路径遇到模型孔洞或超出表面，未生成悬空路径点。');return result!;}
  curve(horizontal:boolean,fixed:number):SurfaceCurve {const knots=this.chart.knots(horizontal),sections:CurveSection[]=[];let length=0;
    for(let part=1;part<knots.length;part++){const pair=[knots[part-1],knots[part]],p=horizontal?this.chart.at(pair[0],fixed):this.chart.at(fixed,pair[0]),q=horizontal?this.chart.at(pair[1],fixed):this.chart.at(fixed,pair[1]);if(distance2(p,q)<1e-9)fail('folded_region','UV曲线发生折叠或长度为零。');const candidates:number[]=[];this.tree.candidates([Math.min(p[0],q[0]),Math.min(p[1],q[1]),Math.max(p[0],q[0]),Math.max(p[1],q[1])],candidates,this.budget);const rawCuts=[0,1];for(const i of candidates){this.budget.check(1);const clipped=this.triangles[i].clip(p,q);if(clipped&&clipped[1]-clipped[0]>1e-11)rawCuts.push(...clipped);}const cuts=uniqueSorted(rawCuts,1e-10);
      for(let segment=1;segment<cuts.length;segment++){const cut=[cuts[segment-1],cuts[segment]];if(cut[1]-cut[0]<=1e-10)continue;this.budget.check(1);const midpoint=mix2(p,q,(cut[0]+cut[1])/2);if(!inside(midpoint,this.chart.boundary,false))fail('folded_region','UV曲线越过点选边界。');const sample=this.hit(midpoint),triangle=this.byOriginal.get(sample.anchor.triangleIndex)!;
        const anchor=(t:number):SurfaceAnchor=>{const bary=triangle.barycentric(mix2(p,q,t)).map(n=>clamp(n,0,1)),sum=bary.reduce((a,b)=>a+b,0);return{triangleIndex:triangle.original,barycentric:bary.map(n=>n/sum) as Vec3};};
        const start=anchorSample(this.request.mesh,anchor(cut[0]),this.request.normalSign),end=anchorSample(this.request.mesh,anchor(cut[1]),this.request.normalSign),d=norm(sub(end.position,start.position));if(d<1e-10)continue;if(sections.length){const previous=sections[sections.length-1];if(norm(sub(previous.end.position,start.position))>1e-5)fail('surface_hole','表面曲线在相邻三角形之间断开。');if(angle(previous.end.normal,start.normal)>this.request.maxNormalStepDeg+1e-6)fail('normal_discontinuity','表面边界上的法向发生跳变。');}
        sections.push({t0:pair[0]+(pair[1]-pair[0])*cut[0],t1:pair[0]+(pair[1]-pair[0])*cut[1],start,end,length:d,offset:length});length+=d;
      }
    }
    if(!sections.length||!Number.isFinite(length)||length<=1e-8)fail('invalid_region','所选UV曲线没有有效表面长度。');return new SurfaceCurve(horizontal,fixed,sections,length);
  }
}
function distances(length:number,setting:number,mode:string,limit:number):number[]{const divisions=mode==='spacing'?Math.max(Math.ceil(length/setting),1):setting;if(!Number.isFinite(divisions)||divisions+1>limit)fail('resource_limit','UV细分点数过多，请缩小区域或增大间距。');const result:number[]=[];for(let i=0;i<=divisions;i++)result.push(mode==='spacing'?Math.min(i*setting,length):length*i/divisions);if(result.length>1&&length-result[result.length-2]<1e-8){result.pop();result[result.length-1]=length;}return result;}
type Previous=[Vec3,Vec3,Quaternion];
function transportOrientation(normal:Vec3,chartX:Vec3,previous:Previous|null):[Vec3,Quaternion]{let x:Vec3;if(previous){const[previousNormal,previousX]=previous,v=cross(previousNormal,normal),d=dot(previousNormal,normal);if(d<=-.999999)fail('normal_discontinuity','相邻法向发生翻转。');const vx=cross(v,previousX);x=plus(plus(previousX,vx),scale(cross(v,vx),1/(1+d)));}else x=chartX;const tangent=unit(sub(x,scale(normal,dot(x,normal))));if(!tangent)fail('normal_discontinuity','无法建立连续的表面切向坐标系。');const y=cross(normal,tangent!),matrix=[tangent![0],y[0],normal[0],0,tangent![1],y[1],normal[1],0,tangent![2],y[2],normal[2],0,0,0,0,1];let q=matrixToQuaternion(matrix);if(q.some(v=>!Number.isFinite(v)))fail('normal_discontinuity','表面姿态生成失败。');if(previous&&q.reduce((sum,n,i)=>sum+n*previous[2][i],0)<0)q=q.map(n=>-n) as Quaternion;return[tangent!,q];}

/** Exact triangle-edge curve tracing and arc-length sampling, matching the native algorithm. */
export function planSurfaceReference(request:SurfaceRequest,cancel=()=>false):SurfaceResult {
  try {
    const budget=new Budget(cancel);budget.check(0);validateRequest(request,budget);const chart=buildChart(request,budget),triangles=meshTriangles(request,chart,budget);checkTopology(request,chart,triangles,budget);const tree=Node.build(triangles.map((_,i)=>i),triangles,budget),context=new Context(request,chart,triangles,tree,budget);
    for(const anchor of request.boundary){const original=anchorSample(request.mesh,anchor,request.normalSign),hit=context.hit(chart.project(original.position));if(norm(sub(original.position,hit.position))>1e-5)fail('ambiguous_surface','点选边界不属于同一个可投影表面。');}
    const vcurve=context.curve(false,.5),vdistances=distances(vcurve.length,request.sampling.v,request.sampling.mode,Math.floor(request.maxPoints/2)),points:SurfaceGeneratedPoint[]=[],rowOffsets:number[]=[];let previous:Previous|null=null,maxOrientationStepDeg=0,maxSpacingErrorMm=0,maxRegularSpacingErrorMm=0;const rowParameters:number[]=[];
    for(let row=0;row<vdistances.length;row++){budget.check(1);const[,uv]=vcurve.sample(vdistances[row],request.mesh,request.normalSign),v=uv[1],curve=context.curve(true,v),uds=distances(curve.length,request.sampling.u,request.sampling.mode,request.maxPoints);if(points.length+uds.length>request.maxPoints)fail('resource_limit','UV路径超过允许点数，请缩小区域或增大UV间距。');
      if(request.sampling.mode==='spacing'){
        for(let i=1;i<uds.length;i++){const error=Math.abs(uds[i]-uds[i-1]-request.sampling.u);maxSpacingErrorMm=Math.max(maxSpacingErrorMm,error);if(i+1<uds.length)maxRegularSpacingErrorMm=Math.max(maxRegularSpacingErrorMm,error);}
      }
      rowParameters.push(v);if(row%2===1)uds.reverse();rowOffsets.push(points.length);
      for(const distance of uds){const[sample,uv]=curve.sample(distance,request.mesh,request.normalSign),[x,orientation]=transportOrientation(sample.normal,chart.x,previous);if(previous){const change=quaternionDistanceDeg(previous[2],orientation);maxOrientationStepDeg=Math.max(maxOrientationStepDeg,change);if(angle(previous[0],sample.normal)>request.maxNormalStepDeg+1e-6||change>request.maxNormalStepDeg+1e-6)fail('normal_discontinuity','相邻加工姿态变化超过限制，请减小UV间距或缩小曲率较大的区域。');}previous=[sample.normal,x,orientation];points.push({pose:{position:sample.position,orientation},normal:sample.normal,anchor:sample.anchor,uv,row});}
    }
    if(request.sampling.mode==='spacing')for(const u of uniqueSorted(points.map(point=>point.uv[0]))){
      const transverse=context.curve(false,u),at=(v:number)=>{let distance=0;for(const section of transverse.sections){if(v>=section.t1)distance=section.offset+section.length;else if(v>section.t0){distance=section.offset+section.length*(v-section.t0)/(section.t1-section.t0);break;}else break;}return distance;};
      for(let row=1;row<rowParameters.length;row++){budget.check(1);const error=Math.abs(at(rowParameters[row])-at(rowParameters[row-1])-request.sampling.v);maxSpacingErrorMm=Math.max(maxSpacingErrorMm,error);if(row+1<rowParameters.length)maxRegularSpacingErrorMm=Math.max(maxRegularSpacingErrorMm,error);}
    }
    budget.check(0);return{status:'success',points,rowOffsets,maxSpacingErrorMm,maxRegularSpacingErrorMm,maxOrientationStepDeg,warnings:['位置来自原模型三角形；姿态的工具+Z轴沿表面法向，切向通过平行传输保持连续，蛇形换行不翻转工具。','U沿各行曲面折线弧长等距，V沿区域中心曲面弧长等距；末段可能短于设定值，曲面度量变化会使其他位置的行距不同，已检查每个生成点U坐标处的全部相邻行间距并报告最大误差。','这是单值投影区域的几何采样；直线指令连接曲面点存在弦差，机器人逆解、运动连续性与碰撞需另行验证。']};
  }catch(error){if(error instanceof SurfaceError)return{status:'failure',code:error.code,message:error.message};throw error;}
}
