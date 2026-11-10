import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { DOMParser } from '@xmldom/xmldom';
import { BoxGeometry, BufferGeometry, Float32BufferAttribute } from 'three';
import { parseUrdf } from '../src/catalog/urdf';
import { urdfLinkMatrices, urdfOriginMatrix } from '../src/catalog/kinematics';
import { buildGenericCollisionDescriptor } from '../src/catalog/collision-descriptors';
import catalog from '../src/catalog/catalog.json';
import { buildCollisionMesh } from '../src/robot/collision-mesh';
import { checkGenericCollisionReference, genericCollisionDescriptor, genericCollisionTransforms, validateGenericCollisionRequest, type GenericCollisionRequest } from '../src/robot/generic-collision-geometry';
import { verifiedGenericCollisionResults } from '../src/robot/generic-collision';
import type { JointAngles, Pose } from '../src/robot/types';
const ids = ['fanuc-crx10ia-l', 'fanuc-r2000ib-165f', 'yaskawa-gp7', 'yaskawa-gp110'];
const q: JointAngles = [10, 15, -20, 25, 30, -35];
const base: Pose = { position: [250,-170,80], orientation: [Math.SQRT1_2,0,0,Math.SQRT1_2] };
const request = (modelId: string): GenericCollisionRequest => ({modelId,basePose:base,solutions:[{id:'first',joints:q}],zones:[]});
describe('generic exact endpoint geometry', () => {
  for (const id of ids) it(`${id} includes every URDF visual and places each fixed branch independently`, () => {
    globalThis.DOMParser = DOMParser as unknown as typeof globalThis.DOMParser;
    const robot = parseUrdf(readFileSync(`tests/fixtures/catalog/${id}.urdf`,'utf8'));
    const descriptor = buildGenericCollisionDescriptor({id,name:id,description:robot,geometries:new Map()},catalog.models.find(m=>m.id===id)!.files);
    expect(JSON.parse(JSON.stringify(descriptor))).toEqual(genericCollisionDescriptor(id));
    const frames=genericCollisionTransforms(id,q), independent=urdfLinkMatrices(robot,q);
    let index=0;
    for(const link of robot.links) for(const visual of link.visuals) {
      const expected=independent.get(link.name)!.clone().multiply(urdfOriginMatrix(visual.origin)).transpose().toArray();
      for(const axis of [3,7,11]) expected[axis]*=1000;
      expect(Math.max(...frames[index].map((v,i)=>Math.abs(v-expected[i])))).toBeLessThan(1e-8);
      expect(descriptor.parts[index++].scale).toEqual(visual.scale.map(v=>v*1000));
    }
    expect(index).toBe(descriptor.parts.length);
    if(id==='fanuc-r2000ib-165f') {expect(index).toBe(9);expect(descriptor.parts.some(p=>p.name==='balancer')).toBe(true);}
  });
  for(const id of ids) it(`${id} checks all transformed parts, closed containment and disabled zones`, async()=>{
    const descriptor=genericCollisionDescriptor(id), geometries=descriptor.parts.map(()=>new BoxGeometry(10,10,10));
    const model={descriptor,meshes:geometries.map(g=>buildCollisionMesh(g))};
    try {
      const r=request(id), frames=genericCollisionTransforms(id,q,base), last=frames.at(-1)!;
      r.zones=[{id:'part',enabled:true,centerMm:[last[3],last[7],last[11]],sizeMm:[1,1,1]}, {id:'far',enabled:true,centerMm:[50000,50000,50000],sizeMm:[1,1,1]}, {id:'disabled',enabled:false,centerMm:[last[3],last[7],last[11]],sizeMm:[1,1,1]}];
      const before=JSON.stringify(r), result=await checkGenericCollisionReference(r,model);
      expect(result[0].collidingLinks).toContain(descriptor.parts.length-1);
      expect(result[0].uncertainLinks).toEqual([]);
      expect(result[0].zones.map(z=>z.zoneId)).toEqual(['part']);
      expect(verifiedGenericCollisionResults(result,r)).toEqual(result);
      expect(JSON.stringify(r)).toBe(before);
    } finally {geometries.forEach(g=>g.dispose());model.meshes.forEach(m=>m.geometry.dispose());}
  });
  it('retains open-mesh uncertainty instead of treating an interior as clear',async()=>{
    const descriptor=genericCollisionDescriptor('yaskawa-gp7'), box=new BoxGeometry(10,10,10).toNonIndexed();
    const positions=Array.from(box.getAttribute('position').array).slice(0,-9);
    const open=new BufferGeometry().setAttribute('position',new Float32BufferAttribute(positions,3));
    const model={descriptor,meshes:descriptor.parts.map((_,i)=>buildCollisionMesh(i===0?open:box))};
    try {
      const r=request('yaskawa-gp7');delete r.basePose;
      r.zones=[{id:'inside',enabled:true,centerMm:[0,0,0],sizeMm:[1,1,1]}];
      const result=await checkGenericCollisionReference(r,model);
      expect(result[0].uncertainLinks).toContain(0);expect(result[0].collidingLinks).not.toContain(0);
      expect(result[0].zones[0].uncertainLinks).toContain(0);
      expect(verifiedGenericCollisionResults(result,r)).toEqual(result);
    } finally {box.dispose();open.dispose();model.meshes.forEach(mesh=>mesh.geometry.dispose());}
  });
  it('rejects invalid input and forged generic part IDs without a false clear result',()=>{
    const r=request('yaskawa-gp7');
    expect(()=>validateGenericCollisionRequest({...r,modelId:'unknown'})).toThrow();
    expect(()=>validateGenericCollisionRequest({...r,solutions:[{id:'bad',joints:[9999,0,0,0,0,0]}]})).toThrow();
    expect(()=>validateGenericCollisionRequest({...r,zones:[{id:'bad',enabled:true,centerMm:[0,0,0],sizeMm:[0,1,1]}]})).toThrow();
    expect(()=>verifiedGenericCollisionResults([{id:'first',collidingLinks:[7],uncertainLinks:[],zones:[]}],r)).toThrow();
    expect(()=>verifiedGenericCollisionResults([{id:'wrong',collidingLinks:[],uncertainLinks:[],zones:[]}],r)).toThrow();
    expect(()=>verifiedGenericCollisionResults([],r)).toThrow();
  });
});
