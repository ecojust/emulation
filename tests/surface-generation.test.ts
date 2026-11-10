import { execFileSync } from 'node:child_process';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import { planSurfaceReference } from '../src/surface/reference';
import { validateSurfaceResult } from '../src/surface/generation-validation';
import { quaternionDistanceDeg, quaternionToMatrix } from '../src/robot/kinematics';
import type { SurfaceAnchor, SurfaceMesh } from '../src/surface/types';
import type { SurfaceRequest, SurfaceResult, SurfaceSuccess } from '../src/surface/generation-types';
import type { Vec3 } from '../src/robot/types';

function grid(nx=2,ny=2,height=(x:number)=>0,gradient=(x:number)=>0,omit=(x:number,y:number)=>false):SurfaceMesh {
  const positions:number[]=[],normals:number[]=[];
  for(let y=0;y<ny;y++)for(let x=0;x<nx;x++){
    if(omit(x,y))continue;const vertices=[[x*10,y*10],[(x+1)*10,y*10],[(x+1)*10,(y+1)*10],[x*10,(y+1)*10]];
    for(const index of [0,1,2,0,2,3]){const[a,b]=vertices[index];positions.push(a,b,height(a));const n=[-gradient(a),0,1],length=Math.hypot(...n);normals.push(...n.map(v=>v/length));}
  }
  return {positions,normals};
}
function anchor(mesh:SurfaceMesh,x:number,y:number):SurfaceAnchor {
  for(let i=0;i<mesh.positions.length;i+=3)if(Math.abs(mesh.positions[i]-x)<1e-8&&Math.abs(mesh.positions[i+1]-y)<1e-8){const vertex=i/3%3;return{triangleIndex:Math.floor(i/9),barycentric:[0,1,2].map(v=>v===vertex?1:0) as Vec3};}
  throw new Error('fixture anchor missing');
}
function input(mesh=grid(),width=20,height=20):SurfaceRequest {
  return{mesh,boundary:[[0,0],[width,0],[width,height],[0,height]].map(([x,y])=>anchor(mesh,x,y)),controlIndices:[0,1,2,3],sampling:{mode:'spacing',u:10,v:10},normalSign:1,maxNormalStepDeg:15,maxPoints:256};
}
function narrowRidge():SurfaceRequest {
  const mesh:SurfaceMesh={positions:[],normals:[]};
  const point=(x:number,y:number)=>[x*2,y*10,x===3?.25*Math.min(y,4-y):0];
  for(let y=0;y<4;y++)for(let x=0;x<20;x++)for(const [a,b] of [[x,y],[x+1,y],[x+1,y+1],[x,y],[x+1,y+1],[x,y+1]]){mesh.positions.push(...point(a,b));mesh.normals.push(0,0,1);}
  const request=input(mesh,40,40);request.sampling={mode:'spacing',u:2,v:10};return request;
}
const nativePath=process.env.EMULATION_NATIVE_CORE||resolve(process.env.CARGO_TARGET_DIR||'target','debug',process.platform==='win32'?'core-cli.exe':'core-cli');
function native(request:SurfaceRequest):SurfaceResult {const response=JSON.parse(execFileSync(nativePath,{input:JSON.stringify({op:'surface',request})+'\n',encoding:'utf8',timeout:30000,maxBuffer:8*1024*1024}));expect(response.error).toBeUndefined();return response.result;}
function checked(request:SurfaceRequest,compute:(request:SurfaceRequest)=>SurfaceResult):SurfaceSuccess {const result=compute(request);expect(result.status,JSON.stringify(result)).toBe('success');if(result.status!=='success')throw new Error(result.message);expect(validateSurfaceResult(result,request)).toEqual(result);return result;}
const matrix=quaternionToMatrix([Math.cos(.2),0,Math.sin(.2),0]);
function transformed(request:SurfaceRequest):SurfaceRequest {
  const output=structuredClone(request);for(const property of ['positions','normals'] as const)for(let i=0;i<output.mesh[property].length;i+=3){const vector=output.mesh[property].slice(i,i+3),value=[0,1,2].map(row=>[0,1,2].reduce((sum,column)=>sum+matrix[row*4+column]*vector[column],0)+(property==='positions'?[200,-70,80][row]:0));output.mesh[property].splice(i,3,...value);}return output;
}

describe.each([['browser',planSurfaceReference],['native',native]] as const)('%s surface sampling',(name,compute)=>{
  it('creates exact millimetre serpentine spacing, surface anchors and stable tool orientation',()=>{
    const request=input(),before=structuredClone(request),result=checked(request,compute);expect(request).toEqual(before);expect(result.rowOffsets).toEqual([0,3,6]);expect(result.points.map(p=>p.pose.position)).toEqual([[0,0,0],[10,0,0],[20,0,0],[20,10,0],[10,10,0],[0,10,0],[0,20,0],[10,20,0],[20,20,0]]);
    expect(result.maxSpacingErrorMm).toBeLessThan(1e-8);for(let i=1;i<result.points.length;i++)expect(quaternionDistanceDeg(result.points[i-1].pose.orientation,result.points[i].pose.orientation)).toBeLessThan(1e-6);
  });
  it('retains short terminal intervals after serpentine reversal without inflating regular spacing errors',()=>{
    for(const pitch of [3,1.5,5]){
      const request=input();request.mesh.positions=request.mesh.positions.map(value=>value/5);request.sampling={mode:'spacing',u:pitch,v:pitch};
      const result=checked(request,compute);expect(result.maxRegularSpacingErrorMm).toBeLessThan(1e-8);
      const start=result.rowOffsets[1],end=result.rowOffsets[2]??result.points.length;
      expect(result.points[start].uv[0]).toBe(1);expect(result.points[end-1].uv[0]).toBe(0);
      const first=Math.hypot(...result.points[start+1].pose.position.map((value,i)=>value-result.points[start].pose.position[i]));
      expect(first).toBeLessThanOrEqual(pitch+1e-8);
    }
  });
  it('keeps transformed surface positions and reversed tool normals coherent',()=>{
    const request=transformed(input());request.normalSign=-1;const result=checked(request,compute);expect(result.points[0].pose.position).toEqual([200,-70,80]);expect(result.points.every(p=>p.normal[2]<0)).toBe(true);
  });
  it('uses surface arc lengths on a curved mesh while preserving continuous normals at row reversals',()=>{
    const request=input(grid(4,2,x=>.002*x*x,x=>.004*x),40,20),result=checked(request,compute);
    expect(result.points.some(p=>p.pose.position[2]>.1)).toBe(true);expect(result.maxOrientationStepDeg).toBeLessThan(15);expect(result.maxRegularSpacingErrorMm).toBeLessThan(.01);
    for(let row=0;row<result.rowOffsets.length;row++){const start=result.rowOffsets[row],end=result.rowOffsets[row+1]??result.points.length;expect(result.points[end-1].uv[0]).toBeCloseTo(row%2?0:1,8);}
  });
  it('uses additional boundary picks and four marked corners without modifying the source mesh',()=>{
    const request=input(),before=structuredClone(request.mesh);request.boundary=[[0,0],[10,0],[20,0],[20,20],[10,20],[0,20]].map(([x,y])=>anchor(request.mesh,x,y));request.controlIndices=[0,2,3,5];
    const result=checked(request,compute);expect(result.points).toHaveLength(9);expect(request.mesh).toEqual(before);
  });
  it('rejects interior holes even when coarse raster samples could miss them',()=>{
    const request=input(grid(4,4,()=>0,()=>0,(x,y)=>x===1&&y===1),40,40);request.sampling={mode:'spacing',u:40,v:40};const result=compute(request);expect(result).toMatchObject({status:'failure',code:'surface_hole'});expect(result).not.toHaveProperty('points');
  });
  it('rejects ambiguous stacked surfaces, self-crossed boundaries and excessive point counts',()=>{
    const overlap=input(),second=structuredClone(overlap.mesh);for(let i=2;i<second.positions.length;i+=3)second.positions[i]+=10;overlap.mesh.positions.push(...second.positions);overlap.mesh.normals.push(...second.normals);expect(compute(overlap)).toMatchObject({status:'failure',code:'ambiguous_surface'});
    const crossed=input();[crossed.boundary[1],crossed.boundary[2]]=[crossed.boundary[2],crossed.boundary[1]];expect(compute(crossed).status).toBe('failure');
    const dense=input();dense.sampling.u=.01;expect(compute(dense)).toMatchObject({status:'failure',code:'resource_limit'});
  });
  it('reports narrow transverse spacing distortion at generated U positions between fixed probe lines',()=>{
    const result=checked(narrowRidge(),compute);expect(result.maxRegularSpacingErrorMm).toBeGreaterThan(.002);
  });
  it('rejects abrupt normal changes instead of smoothing the tool away from the declared surface normal',()=>{
    const request=input(grid(4,2,x=>.002*x*x,x=>.004*x),40,20);request.maxNormalStepDeg=.1;expect(compute(request)).toMatchObject({status:'failure',code:'normal_discontinuity'});
  });
});

it('native and browser paths agree on positions, normals, UV order, spacing and continuous tool orientation',()=>{
  for(const request of [input(),transformed(input()),input(grid(4,2,x=>.002*x*x,x=>.004*x),40,20),narrowRidge()]){
    const reference=checked(request,planSurfaceReference),actual=checked(request,native);expect(actual.rowOffsets).toEqual(reference.rowOffsets);expect(actual.points).toHaveLength(reference.points.length);
    actual.points.forEach((point,index)=>{const expected=reference.points[index];expect(Math.hypot(...point.pose.position.map((v,i)=>v-expected.pose.position[i]))).toBeLessThan(1e-7);expect(quaternionDistanceDeg(point.pose.orientation,expected.pose.orientation)).toBeLessThan(1e-4);expect(point.uv[0]).toBeCloseTo(expected.uv[0],7);expect(point.uv[1]).toBeCloseTo(expected.uv[1],7);});
    expect(actual.maxSpacingErrorMm).toBeCloseTo(reference.maxSpacingErrorMm,7);expect(actual.maxRegularSpacingErrorMm).toBeCloseTo(reference.maxRegularSpacingErrorMm,7);
  }
});
it('reference observes cancellation during mesh work and returns no partial points',()=>{let checks=0;const result=planSurfaceReference(input(grid(20,20),200,200),()=>++checks>12);expect(result).toMatchObject({status:'failure',code:'cancelled'});expect(result).not.toHaveProperty('points');});
