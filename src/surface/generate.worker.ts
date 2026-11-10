/// <reference lib="webworker" />
import { planSurfaceReference } from './reference';
import type { SurfaceRequest } from './generation-types';
const scope=self as unknown as DedicatedWorkerGlobalScope;
scope.onmessage=(event:MessageEvent<{requestId:string;request:SurfaceRequest}>)=>{
  const {requestId,request}=event.data;
  try{scope.postMessage({requestId,result:planSurfaceReference(request)});}
  catch(error){scope.postMessage({requestId,error:error instanceof Error?error.message:'表面路径生成失败。'});}
};
