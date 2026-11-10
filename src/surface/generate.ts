import { invoke, isTauri } from '@tauri-apps/api/core';
import { validateSurfaceRequest, validateSurfaceResult } from './generation-validation';
import type { SurfaceRequest, SurfaceResult } from './generation-types';
export type { SurfaceRequest, SurfaceResult, SurfaceSuccess, SurfaceFailure, SurfaceGeneratedPoint } from './generation-types';
export { validateSurfaceRequest, validateSurfaceResult } from './generation-validation';
const TIMEOUT_MS=60_000;

/** Desktop always uses the bounded Rust worker; an IPC error never falls back to JS. */
export function generateSurfacePath(request: SurfaceRequest, signal?: AbortSignal): Promise<SurfaceResult> {
  if(signal?.aborted)return Promise.reject(new DOMException('表面路径生成已取消','AbortError'));
  let validated:SurfaceRequest;
  try { validated=validateSurfaceRequest(request); } catch(error) { return Promise.reject(error); }
  const native=isTauri();
  return new Promise((resolve,reject)=>{
    const requestId=crypto.randomUUID();let settled=false,worker:Worker|null=null;
    const cleanup=()=>{clearTimeout(timer);signal?.removeEventListener('abort',abort);worker?.terminate();};
    const stop=(error:Error)=>{if(settled)return;settled=true;cleanup();if(native)void invoke('cancel_motion',{requestId}).catch(()=>{});reject(error);};
    const abort=()=>stop(new DOMException('表面路径生成已取消','AbortError'));
    const timer=setTimeout(()=>stop(new Error('表面路径生成超时，请减少区域或增大U/V间距。')),TIMEOUT_MS);
    signal?.addEventListener('abort',abort,{once:true});
    const accept=(value:unknown)=>{if(settled)return;try{const result=validateSurfaceResult(value,validated);settled=true;cleanup();resolve(result);}catch(error){stop(error instanceof Error?error:new Error('表面路径返回结果无效。'));}};
    if(signal?.aborted){abort();return;}
    if(native){void invoke('generate_surface_path',{requestId,request:validated}).then(accept,error=>stop(new Error(typeof error==='string'?error:'原生表面路径生成失败。')));return;}
    try {
      worker=new Worker(new URL('./generate.worker.ts',import.meta.url),{type:'module'});
      worker.onmessage=(event:MessageEvent<{requestId:string;result?:unknown;error?:string}>)=>{if(settled||event.data?.requestId!==requestId)return;if(event.data.error)stop(new Error(event.data.error));else accept(event.data.result);};
      worker.onerror=()=>stop(new Error('表面路径计算未完成，请重试。'));
      worker.onmessageerror=()=>stop(new Error('表面路径结果无法读取。'));
      worker.postMessage({requestId,request:validated});
    }catch(error){stop(error instanceof Error?error:new Error('无法创建表面路径任务。'));}
  });
}
