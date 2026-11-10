import { readFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { cpus } from 'node:os';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { createHash } from 'node:crypto';
import { execFileSync, spawnSync } from 'node:child_process';
import { performance } from 'node:perf_hooks';

// Optional CSG dependencies stay outside the application; pass their scratch node_modules directory.
// This is the exploratory one-ray signed-containment BVH prototype measured during
// algorithm selection, not the final product's three-ray uncertain-interior kernel.
const script = fileURLToPath(import.meta.url);
const root = resolve(dirname(script), '..');
const childMode = process.argv[2] === '--child';
const args = process.argv.slice(childMode ? 3 : 2);
if (args.length < 3) throw new Error('Usage: node scripts/benchmark-collision.mjs <scratch-node_modules> <bvh|bsp|bvh-csg> <case> [irb1100-4-0.475|irb1100-4-0.58]');
const [packages, backend, caseName, modelId = 'irb1100-4-0.475'] = args;
if (!['bvh', 'bsp', 'bvh-csg'].includes(backend) || !['irb1100-4-0.475', 'irb1100-4-0.58'].includes(modelId)) throw new Error('Unsupported benchmark backend or model');
if (!childMode) {
  const child = spawnSync(process.execPath, ['--max-old-space-size=1536', script, '--child', ...args], { encoding: 'utf8', timeout: 60_000, maxBuffer: 4 * 1024 * 1024 });
  process.stdout.write(child.stdout || '');
  process.stderr.write(child.stderr || '');
  if (child.error) process.stderr.write(JSON.stringify({ phase: 'terminated', error: child.error.message, signal: child.signal }) + '\n');
  process.exit(child.status ?? 1);
}
const packageDirectory = resolve(packages);
const requireScratch = createRequire(join(packageDirectory, '..', 'package.json'));
const importScratch = (name, entry) => import(pathToFileURL(join(packageDirectory, name, entry)).href);
const THREE = await importScratch('three', 'build/three.module.js');
const { STLLoader } = await importScratch('three', 'examples/jsm/loaders/STLLoader.js');
const { mergeGeometries } = await importScratch('three', 'examples/jsm/utils/BufferGeometryUtils.js');
const { CSG } = requireScratch('three-csg-ts');
const { MeshBVH, SAH } = await importScratch('three-mesh-bvh', 'src/index.js');
const { Brush, Evaluator, INTERSECTION } = await importScratch('three-bvh-csg', 'src/index.js');
const versions = Object.fromEntries(['three', 'three-csg-ts', 'three-mesh-bvh', 'three-bvh-csg'].map(name => [name, JSON.parse(readFileSync(join(packageDirectory, name, 'package.json'), 'utf8')).version]));
console.log(JSON.stringify({ phase: 'environment', node: process.version, platform: process.platform, arch: process.arch, cpu: cpus()[0]?.model, availableCpus: cpus().length, versions, sourceManifestSha256: createHash('sha256').update(readFileSync(join(root, 'vendor/work-r-labs/SOURCE.json'))).digest('hex'), queryDefinition: '7 parsed meshes and cached structures; timings exclude file parsing/structure construction; BSP/CSG produce intersection polygons, BVH asks boolean surface plus one-ray containment', timeoutMs: 60000, heapLimitMiB: 1536 }));
const started = performance.now();
const synthetic = caseName.startsWith('cube-') || caseName.startsWith('hollow-');
const zero = [0,0,0,0,0,0];
const joints = caseName === 'ready-multiple' ? [0,20,-30,0,40,0] : zero;
const definitions = {
  'empty-corner': {center:[75,75,80],size:[4,4,4]},
  surface: {center:[0,70,100],size:[4,4,4]},
  contained: {center:[0,0,50],size:[2,2,2]},
  'ready-multiple': {center:[120,0,430],size:[180,180,260]},
  'link5-cavity': {center:[modelId.endsWith('0.58')?351.75:301.75,-16.708984375,modelId.endsWith('0.58')?592.6882934570312:537.6882934570312],size:[.1,.1,.1]},
  'cube-contact': {center:[2,0,0],size:[2,2,2]},
  'cube-gap': {center:[2.001,0,0],size:[2,2,2]},
  'cube-contained': {center:[0,0,0],size:[.2,.2,.2]},
  'hollow-cavity': {center:[0,0,0],size:[.2,.2,.2]},
  'hollow-wall': {center:[.7,0,0],size:[.2,.2,.2]},
};
const fixture = definitions[caseName];
if(!fixture) throw Error('unknown case');
const center = new THREE.Vector3(...fixture.center);
const size = new THREE.Vector3(...fixture.size);
const box = new THREE.Box3().setFromCenterAndSize(center,size);
const material = new THREE.MeshBasicMaterial();
const zoneMesh = new THREE.Mesh(new THREE.BoxGeometry(...fixture.size),material);
zoneMesh.position.copy(center); zoneMesh.updateMatrixWorld();
let geometries, deltas, closed;
if(synthetic){
  let geometry = new THREE.BoxGeometry(2,2,2).toNonIndexed();
  if(caseName.startsWith('hollow-')){
    const inner = new THREE.BoxGeometry(1,1,1).toNonIndexed();
    const array=inner.getAttribute('position').array;
    for(let i=0;i<array.length;i+=9) for(let j=0;j<3;j++) [array[i+3+j],array[i+6+j]]=[array[i+6+j],array[i+3+j]];
    inner.computeVertexNormals();
    geometry=mergeGeometries([geometry,inner],false);
  }
  geometries=[geometry]; deltas=[new THREE.Matrix4()]; closed=[true];
} else {
  const commands=[zero,joints].map(joints=>JSON.stringify({op:'fk',modelId,joints})).join('\n')+'\n';
  const fks=execFileSync(process.env.EMULATION_NATIVE_CORE || resolve(process.env.CARGO_TARGET_DIR || join(root, 'target'), 'debug', process.platform === 'win32' ? 'core-cli.exe' : 'core-cli'),{input:commands,encoding:'utf8'}).trim().split('\n').map(line=>JSON.parse(line).result);
  deltas=fks[1].frames.map((frame,index)=>new THREE.Matrix4().set(...frame).multiply(new THREE.Matrix4().set(...fks[0].frames[index]).invert()));
  const loader=new STLLoader();
  geometries=Array.from({length:7},(_,i)=>{const bytes=readFileSync(`${root}/public/models/${modelId}/${i===0?'base_link':`link_${i}`}.stl`); return loader.parse(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));});
  closed=Array.from({length:7},(_,i)=>i===0||i===5||i===6||(i===2&&modelId.endsWith('0.58')));
}
const triangleCounts=geometries.map(g=>g.getAttribute('position').count/3);
geometries.forEach(g=>g.computeBoundingBox());
// Baseline uses cached source BVHs and moves query/triangles, never rebuilds BVHs on a new pose.
let structures;
const prepareStart=performance.now();
if(backend==='bvh') structures=geometries.map(g=>new MeshBVH(g,{strategy:SAH,targetLeafSize:10}));
else if(backend==='bsp') structures=geometries.map((g,i)=>{const mesh=new THREE.Mesh(g,material);mesh.matrix.copy(deltas[i]);return CSG.fromMesh(mesh);});
else if(backend==='bvh-csg') structures=geometries.map((g,i)=>{const brush=new Brush(g.clone(),material); brush.matrixAutoUpdate=false;brush.matrix.copy(deltas[i]);brush.matrixWorld.copy(deltas[i]);brush.prepareGeometry();return brush;});
else throw Error('unknown backend');
const preparationMs=performance.now()-prepareStart;
console.log(JSON.stringify({phase:'prepared',backend,caseName,modelId,triangleCounts,preparationMs,parseAndSetupMs:prepareStart-started}));

const broadIndices=[];
for(let i=0;i<geometries.length;i++) if(geometries[i].boundingBox.clone().applyMatrix4(deltas[i]).intersectsBox(box)) broadIndices.push(i);
function bvhQuery(index){
 const bvh=structures[index],delta=deltas[index], scratchBox=new THREE.Box3(), triangle=new THREE.Triangle();
 let testedTriangles=0;
 const surface=bvh.shapecast({
  intersectsBounds:bounds=>scratchBox.copy(bounds).applyMatrix4(delta).intersectsBox(box),
  intersectsTriangle:local=>{testedTriangles++;triangle.a.copy(local.a).applyMatrix4(delta);triangle.b.copy(local.b).applyMatrix4(delta);triangle.c.copy(local.c).applyMatrix4(delta);return box.intersectsTriangle(triangle);}
 });
 if(surface) return {index,hit:true,kind:'surface',testedTriangles};
 if(!closed[index]) return {index,hit:null,kind:'uncertain-open-mesh',testedTriangles};
 const localCenter=center.clone().applyMatrix4(delta.clone().invert());
 const ray=new THREE.Ray(localCenter,new THREE.Vector3(1,.371,.213).normalize());
 const hits=bvh.raycast(ray,THREE.DoubleSide).sort((a,b)=>a.distance-b.distance);
 let winding=0;
 for(let i=0;i<hits.length;){let j=i,sum=0;for(;j<hits.length&&Math.abs(hits[j].distance-hits[i].distance)<1e-7;j++)sum+=Math.sign(hits[j].face.normal.dot(ray.direction));winding+=Math.sign(sum);i=j;}
 return {index,hit:winding!==0,kind:winding?'contained':'clear',testedTriangles,rayCrossings:hits.length,signedCrossings:winding};
}
function polygonVolume(polygons){let volume=0;for(const p of polygons){const a=p.vertices[0].pos;for(let i=1;i<p.vertices.length-1;i++){const b=p.vertices[i].pos,c=p.vertices[i+1].pos;volume+=a.x*(b.y*c.z-b.z*c.y)+a.y*(b.z*c.x-b.x*c.z)+a.z*(b.x*c.y-b.y*c.x);}}return Math.abs(volume/6);}
function geometryVolume(g){const p=g.getAttribute('position'),idx=g.index;let volume=0;const a=new THREE.Vector3(),b=new THREE.Vector3(),c=new THREE.Vector3();const count=idx?idx.count:p.count;for(let i=0;i<count;i+=3){a.fromBufferAttribute(p,idx?idx.getX(i):i);b.fromBufferAttribute(p,idx?idx.getX(i+1):i+1);c.fromBufferAttribute(p,idx?idx.getX(i+2):i+2);volume+=a.dot(b.cross(c));}return Math.abs(volume/6);}
const evaluator=new Evaluator();evaluator.attributes=['position','normal'];evaluator.useGroups=false;
const zoneBrush=new Brush(zoneMesh.geometry,material);zoneBrush.position.copy(center);zoneBrush.updateMatrixWorld();
const zoneCsg=CSG.fromMesh(zoneMesh);
function query(){return broadIndices.map(index=>{
 if(backend==='bvh')return bvhQuery(index);
 if(backend==='bsp'){const intersection=structures[index].intersect(zoneCsg),polygons=intersection.toPolygons();return {index,polygonCount:polygons.length,volume:polygonVolume(polygons),closedInput:closed[index]};}
 const intersection=evaluator.evaluate(structures[index],zoneBrush,INTERSECTION);const geometry=intersection.geometry;const result={index,triangleCount:(geometry.index?geometry.index.count:geometry.attributes.position.count)/3,volume:geometryVolume(geometry),closedInput:closed[index]};geometry.dispose();return result;
});}
const timings=[],rounds=backend==='bvh'?30:3;let results;
for(let i=0;i<rounds;i++){const start=performance.now();try{results=query();}catch(error){console.log(JSON.stringify({phase:'failed',backend,caseName,message:String(error),elapsedMs:performance.now()-start,broadIndices,heapMiB:process.memoryUsage().heapUsed/1048576}));process.exitCode=2;break;}timings.push(performance.now()-start);}
if(results){const sorted=[...timings].sort((a,b)=>a-b);console.log(JSON.stringify({phase:'complete',backend,caseName,modelId,broadIndices,results,rounds:timings.length,firstMs:timings[0],medianMs:sorted[Math.floor(sorted.length/2)],maxMs:Math.max(...timings),totalMs:performance.now()-started,heapMiB:process.memoryUsage().heapUsed/1048576,maxRssMiB:process.resourceUsage().maxRSS/1024}));}
