import { checkGenericCollisionReference, type GenericCollisionRequest } from './generic-collision-geometry';
self.onmessage = async (event: MessageEvent<{requestId: string; request: GenericCollisionRequest}>) => {
 const {requestId,request}=event.data;
 try {self.postMessage({requestId,results:await checkGenericCollisionReference(request)});}
 catch(error){self.postMessage({requestId,error:error instanceof Error?error.message:'干涉检测无法完成'});}
};
