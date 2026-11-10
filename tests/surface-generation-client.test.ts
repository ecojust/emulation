import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import type { SurfaceRequest, SurfaceSuccess } from '../src/surface/generation-types';
import { validateSurfaceRequest, validateSurfaceResult } from '../src/surface/generation-validation';
import { planSurfaceReference } from '../src/surface/reference';
const transport=vi.hoisted(()=>({invoke:vi.fn(),isTauri:vi.fn(()=>true)}));
vi.mock('@tauri-apps/api/core',()=>transport);
import { generateSurfacePath } from '../src/surface/generate';

function request():SurfaceRequest {
  return {mesh:{positions:[0,0,0,20,0,0,20,20,0,0,0,0,20,20,0,0,20,0],normals:Array.from({length:6},()=>[0,0,1]).flat()},boundary:[{triangleIndex:0,barycentric:[1,0,0]},{triangleIndex:0,barycentric:[0,1,0]},{triangleIndex:0,barycentric:[0,0,1]},{triangleIndex:1,barycentric:[0,0,1]}],controlIndices:[0,1,2,3],sampling:{mode:'spacing',u:10,v:10},normalSign:1,maxNormalStepDeg:5,maxPoints:64};
}
function success():SurfaceSuccess {
  return {status:'success',points:[0,10,20].flatMap((y,row)=>(row%2?[20,10,0]:[0,10,20]).map(x=>({pose:{position:[x,y,0] as [number,number,number],orientation:[1,0,0,0] as [number,number,number,number]},normal:[0,0,1] as [number,number,number],anchor:y<=x?{triangleIndex:0,barycentric:[1-x/20,(x-y)/20,y/20] as [number,number,number]}:{triangleIndex:1,barycentric:[1-y/20,x/20,(y-x)/20] as [number,number,number]},uv:[x/20,y/20] as [number,number],row}))),rowOffsets:[0,3,6],maxSpacingErrorMm:0,maxRegularSpacingErrorMm:0,maxOrientationStepDeg:0,warnings:[]};
}

describe('independent surface result validation',()=>{
  it('accepts a surface-attached serpentine path and copies immutable request/result values',()=>{
    const input=request(),saved=validateSurfaceRequest(input),output=success(),validated=validateSurfaceResult(output,saved);
    expect(validated).toEqual(output);input.mesh.positions[0]=77;input.boundary[0].barycentric[0]=0;
    expect(saved.mesh.positions[0]).toBe(0);expect(saved.boundary[0].barycentric[0]).toBe(1);
    output.points[0].pose.position[0]=77;expect(validated.status==='success'&&validated.points[0].pose.position[0]).toBe(0);
  });
  it('rejects invalid mesh winding, degenerate faces, malformed anchors and crossed control order before dispatch',()=>{
    for(const mutate of [(r:SurfaceRequest)=>{r.mesh.normals[2]=-1;},(r:SurfaceRequest)=>{r.mesh.positions.fill(0);},(r:SurfaceRequest)=>{r.boundary[0].triangleIndex=8;},(r:SurfaceRequest)=>{r.boundary[0].barycentric=[1,1,0];},(r:SurfaceRequest)=>{r.controlIndices=[0,2,1,3];},(r:SurfaceRequest)=>{r.sampling.u=0;},(r:SurfaceRequest)=>{r.maxPoints=257;}]){
      const input=request();mutate(input);expect(()=>validateSurfaceRequest(input)).toThrow();
    }
  });
  it('rejects forged geometry, normal direction, UV order, quaternion sign flips and false orientation reports',()=>{
    for(const mutate of [(r:SurfaceSuccess)=>{r.points[1].pose.position[2]=.1;},(r:SurfaceSuccess)=>{r.points[1].anchor.triangleIndex=9;},(r:SurfaceSuccess)=>{r.points[1].normal=[0,0,-1];},(r:SurfaceSuccess)=>{r.points[1].pose.orientation=[0,1,0,0];},(r:SurfaceSuccess)=>{r.points[1].pose.orientation=[-1,0,0,0];},(r:SurfaceSuccess)=>{r.points[1].uv[0]=.01;r.points[2].uv[0]=0;},(r:SurfaceSuccess)=>{r.points[1].row=1;},(r:SurfaceSuccess)=>{r.maxOrientationStepDeg=1;},(r:SurfaceSuccess)=>{r.maxSpacingErrorMm=NaN;}]){
      const output=success();mutate(output);expect(()=>validateSurfaceResult(output,request())).toThrow();
    }
    expect(()=>validateSurfaceResult({status:'failure',code:'hole',message:'hole',points:[]},request())).toThrow();
  });
  it('rejects a valid mesh anchor falsely labelled with another UV position',()=>{
    const output=success();output.points[1].anchor={triangleIndex:0,barycentric:[0,1,0]};output.points[1].pose.position=[20,0,0];
    expect(()=>validateSurfaceResult(output,request())).toThrow('区域或UV');
  });
  it('independently detects a false zero-spacing report for an oversampled planar rectangle',()=>{
    const output=success();output.points=[0,10,20].flatMap((y,row)=>(row%2?[20,15,10,5,0]:[0,5,10,15,20]).map(x=>({pose:{position:[x,y,0] as [number,number,number],orientation:[1,0,0,0] as [number,number,number,number]},normal:[0,0,1] as [number,number,number],anchor:y<=x?{triangleIndex:0,barycentric:[1-x/20,(x-y)/20,y/20] as [number,number,number]}:{triangleIndex:1,barycentric:[1-y/20,x/20,(y-x)/20] as [number,number,number]},uv:[x/20,y/20] as [number,number],row})));output.rowOffsets=[0,5,10];
    expect(()=>validateSurfaceResult(output,request())).toThrow('平面矩形');
    output.maxSpacingErrorMm=5;output.maxRegularSpacingErrorMm=5;expect(validateSurfaceResult(output,request()).status).toBe('success');
  });
  it('rejects a missing grid sample whose claimed pitch is shorter than the actual chord',()=>{
    const output=success();output.points.splice(1,1);output.rowOffsets=[0,2,5];
    expect(()=>validateSurfaceResult(output,request())).toThrow('弦长下界');
  });
  it('accepts real odd-row boundary remainders without classifying them as regular-pitch errors',()=>{
    const input=request();input.mesh.positions=input.mesh.positions.map(value=>value/5);input.sampling={mode:'spacing',u:3,v:3};
    const output=planSurfaceReference(input);expect(output.status).toBe('success');if(output.status!=='success')throw new Error(output.message);
    expect(output.points.slice(3,6).map(point=>point.pose.position[0])).toEqual([4,3,0]);
    expect(output.maxSpacingErrorMm).toBeCloseTo(2,8);expect(output.maxRegularSpacingErrorMm).toBeLessThan(1e-8);
    expect(validateSurfaceResult(output,input)).toEqual(output);
    const missing=structuredClone(output);missing.points.splice(4,1);missing.rowOffsets=[0,3,5];
    expect(()=>validateSurfaceResult(missing,input)).toThrow('U间距小于实际弦长下界');
    const missingRow=structuredClone(output);missingRow.points.splice(3,3);missingRow.rowOffsets=[0,3];
    for(const point of missingRow.points.slice(3))point.row=1;
    // Restore serpentine order on the remaining row so only the missing V row
    // causes rejection, not its traversal direction.
    missingRow.points.splice(3,3,...missingRow.points.slice(3).reverse());
    expect(()=>validateSurfaceResult(missingRow,input)).toThrow(/V间距|中间V采样行/);
  });
  it('accepts deliberate inward normal orientation only when both frame and actual normal agree',()=>{
    const input=request();input.normalSign=-1;const output=success();for(const point of output.points){point.normal=[0,0,-1];point.pose.orientation=[0,1,0,0];}
    expect(validateSurfaceResult(output,input)).toEqual(output);
  });
});

describe('surface native isolation and cancellable worker transport',()=>{
  let worker:ReturnType<typeof vi.fn>;
  beforeEach(()=>{transport.invoke.mockReset();transport.isTauri.mockReturnValue(true);worker=vi.fn();vi.stubGlobal('Worker',worker);});
  afterEach(()=>{vi.unstubAllGlobals();vi.useRealTimers();});
  it('dispatches the bounded native surface command and never silently falls back on failure',async()=>{
    transport.invoke.mockResolvedValue(success());expect((await generateSurfacePath(request())).status).toBe('success');
    expect(transport.invoke).toHaveBeenCalledWith('generate_surface_path',expect.objectContaining({requestId:expect.any(String),request:request()}));
    transport.invoke.mockRejectedValue('native surface offline');await expect(generateSurfacePath(request())).rejects.toThrow('native surface offline');expect(worker).not.toHaveBeenCalled();
  });
  it('submits no invalid or pre-cancelled work',async()=>{
    const input=request();input.mesh.normals=[];await expect(generateSurfacePath(input)).rejects.toThrow();
    const controller=new AbortController();controller.abort();await expect(generateSurfacePath(request(),controller.signal)).rejects.toMatchObject({name:'AbortError'});
    expect(transport.invoke).not.toHaveBeenCalled();expect(worker).not.toHaveBeenCalled();
  });
  it('cancels only its request ID and ignores late native geometry',async()=>{
    let finish:(value:unknown)=>void=()=>{};transport.invoke.mockImplementation(command=>command==='generate_surface_path'?new Promise(resolve=>{finish=resolve;}):Promise.resolve());
    const controller=new AbortController(),pending=generateSurfacePath(request(),controller.signal),rejected=expect(pending).rejects.toMatchObject({name:'AbortError'});
    const {requestId}=transport.invoke.mock.calls[0][1];controller.abort();await rejected;
    expect(transport.invoke).toHaveBeenCalledWith('cancel_motion',{requestId});finish(success());expect(worker).not.toHaveBeenCalled();
  });
  it('rejects forged native results through the independent mesh validator',async()=>{
    const wrong=success();wrong.points[0].pose.position[2]=1;transport.invoke.mockResolvedValue(wrong);
    await expect(generateSurfacePath(request())).rejects.toThrow('模型表面');expect(worker).not.toHaveBeenCalled();
    expect(transport.invoke).toHaveBeenCalledWith('cancel_motion',expect.objectContaining({requestId:expect.any(String)}));
  });
  it('times out and cancels a native task',async()=>{
    vi.useFakeTimers();transport.invoke.mockImplementation(()=>new Promise(()=>{}));const pending=generateSurfacePath(request()),rejected=expect(pending).rejects.toThrow('超时');
    const {requestId}=transport.invoke.mock.calls[0][1];await vi.advanceTimersByTimeAsync(60001);await rejected;expect(transport.invoke).toHaveBeenCalledWith('cancel_motion',{requestId});
  });
  it('filters stale browser responses and terminates the worker after success',async()=>{
    transport.isTauri.mockReturnValue(false);const instance={onmessage:null as null|((event:MessageEvent)=>void),onerror:null,onmessageerror:null,postMessage:vi.fn(),terminate:vi.fn()};worker.mockImplementation(function(){return instance;});
    const pending=generateSurfacePath(request()),sent=instance.postMessage.mock.calls[0][0];
    instance.onmessage!({data:{requestId:'stale',result:success()}} as MessageEvent);expect(instance.terminate).not.toHaveBeenCalled();
    instance.onmessage!({data:{requestId:sent.requestId,result:success()}} as MessageEvent);expect((await pending).status).toBe('success');expect(instance.terminate).toHaveBeenCalledTimes(1);expect(transport.invoke).not.toHaveBeenCalled();
  });
  it('terminates a cancelled browser worker and ignores its late response',async()=>{
    transport.isTauri.mockReturnValue(false);const instance={onmessage:null as null|((event:MessageEvent)=>void),onerror:null,onmessageerror:null,postMessage:vi.fn(),terminate:vi.fn()};worker.mockImplementation(function(){return instance;});
    const controller=new AbortController(),pending=generateSurfacePath(request(),controller.signal),rejected=expect(pending).rejects.toMatchObject({name:'AbortError'}),sent=instance.postMessage.mock.calls[0][0];
    controller.abort();await rejected;expect(instance.terminate).toHaveBeenCalledTimes(1);instance.onmessage!({data:{requestId:sent.requestId,result:success()}} as MessageEvent);expect(transport.invoke).not.toHaveBeenCalled();
  });
});
