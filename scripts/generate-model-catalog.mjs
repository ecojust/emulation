import { createHash } from 'node:crypto';
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

// The catalog is reviewed source data, not an online search index. Adding another
// model requires updating this list and reviewing its geometry and source notices.
const revision = 'ab6aa489855256de07ba46c7b45887fdbc42b607';
const repository = 'https://github.com/work-r-labs/robots';
const rawRoot = `https://raw.githubusercontent.com/work-r-labs/robots/${revision}/`;
const spdxRevision = '31846481de7ea17d92f2f46fad76abe58b2e9bb0';
const spdxUrl = `https://raw.githubusercontent.com/spdx/license-list-data/${spdxRevision}/text/BSD-3-Clause.txt`;
const root = fileURLToPath(new URL('../', import.meta.url));
let cacheDirectory = join(tmpdir(), 'emulation-model-catalog', revision);
let check = false;
let offline = false;
const args = process.argv.slice(2);
for (let i = 0; i < args.length; i++) {
  if (args[i] === '--check') check = true;
  else if (args[i] === '--offline') offline = true;
  else if (args[i] === '--cache-dir' && args[i + 1]) cacheDirectory = resolve(args[++i]);
  else throw new Error('Usage: node scripts/generate-model-catalog.mjs [--check] [--offline] [--cache-dir DIRECTORY]');
}

const definitions = [
  {
    id: 'irb1100-4-0.475', brand: 'ABB', name: 'ABB IRB 1100-4/0.475',
    directory: 'ABB/IRB1100_4kg_475_STD_v1', urdfPath: 'IRB1100_4kg_475_STD.urdf',
    bundledMotionModelId: 'irb1100-4-0.475',
    originalNotice: 'sources/ABB/IRB1100_4kg-475-STD/IRB1100_4kg-475-STD_description/LICENSE',
  },
  {
    id: 'irb1100-4-0.58', brand: 'ABB', name: 'ABB IRB 1100-4/0.58',
    directory: 'ABB/IRB1100_4kg_580_STD_v1', urdfPath: 'IRB1100_4kg_580_STD.urdf',
    bundledMotionModelId: 'irb1100-4-0.58',
    originalNotice: 'sources/ABB/IRB1100_4kg-580-STD/IRB1100_4kg-580-STD_description/LICENSE',
  },
  {
    id: 'fanuc-crx10ia-l', brand: 'FANUC', name: 'FANUC CRX-10iA/L',
    directory: 'Fanuc/CRX_10iA_L_v1', urdfPath: 'CRX_10iA_L.urdf',
    originalNotice: 'sources/Fanuc/CRX-Series/CRX-10iA_L_v01_description/LICENSE',
  },
  {
    id: 'fanuc-r2000ib-165f', brand: 'FANUC', name: 'FANUC R-2000iB/165F',
    directory: 'Fanuc/R_2000iB_165F_v1', urdfPath: 'R_2000iB_165F.urdf',
    originalNotice: 'sources/Fanuc/R2000-Series/R-2000iB165F_description/LICENSE',
    additionalMeshes: ['balancer.stl', 'balancer_shaft.stl'],
  },
  {
    id: 'yaskawa-gp7', brand: 'Yaskawa', name: 'Yaskawa MOTOMAN GP7',
    directory: 'Yaskawa/GP7_v1', urdfPath: 'GP7.urdf',
    originalNotice: 'sources/Yaskawa/GP Series/motoman_gp7_support/package.xml',
    originalMeshDirectory: 'sources/Yaskawa/GP Series/motoman_gp7_support/meshes/visual/',
  },
  {
    id: 'yaskawa-gp110', brand: 'Yaskawa', name: 'Yaskawa MOTOMAN GP110',
    directory: 'Yaskawa/GP110_v1', urdfPath: 'GP110.urdf',
    originalNotice: 'sources/Yaskawa/GP Series/motoman_gp110_support/package.xml',
    originalMeshDirectory: 'sources/Yaskawa/GP Series/motoman_gp110_support/meshes/gp110/visual/',
  },
];

const sha256 = bytes => createHash('sha256').update(bytes).digest('hex');
const gitBlobSha1 = bytes => createHash('sha1').update(`blob ${bytes.length}\0`).update(bytes).digest('hex');
const rawUrl = path => rawRoot + path.split('/').map(encodeURIComponent).join('/');
const bytesCache = new Map();
const vendorFiles = new Map();
const provenance = [];

async function readOptional(path) {
  try { return await readFile(path); }
  catch (error) { if (error.code === 'ENOENT') return undefined; throw error; }
}

async function fetchBytes(url, maximumBytes) {
  if (offline) throw new Error(`Offline cache is missing ${url}`);
  const response = await fetch(url, { redirect: 'error', signal: AbortSignal.timeout(45_000), headers: { 'User-Agent': 'emulation-model-catalog-generator' } });
  if (!response.ok) throw new Error(`Download failed (${response.status}): ${url}`);
  const declaredLength = Number(response.headers.get('content-length'));
  if (declaredLength && declaredLength > maximumBytes) throw new Error(`Oversized response: ${url}`);
  const chunks = [];
  let length = 0;
  for await (const chunk of response.body) {
    length += chunk.length;
    if (length > maximumBytes) throw new Error(`Oversized response: ${url}`);
    chunks.push(chunk);
  }
  return Buffer.concat(chunks, length);
}

const treeCachePath = join(cacheDirectory, 'pinned-tree.json');
let treeBytes = await readOptional(treeCachePath);
if (!treeBytes) {
  treeBytes = await fetchBytes(`https://api.github.com/repos/work-r-labs/robots/git/trees/${revision}?recursive=1`, 4_000_000);
  await mkdir(dirname(treeCachePath), { recursive: true });
  await writeFile(treeCachePath, treeBytes);
}
const tree = JSON.parse(treeBytes.toString('utf8'));
if (tree.sha !== revision || tree.truncated || !Array.isArray(tree.tree)) throw new Error('Pinned source tree is missing or truncated.');
const treeFiles = new Map(tree.tree.filter(entry => entry.type === 'blob').map(entry => [entry.path, entry]));

async function sourceBytes(sourcePath, bundledPath) {
  if (bytesCache.has(sourcePath)) return bytesCache.get(sourcePath);
  const metadata = treeFiles.get(sourcePath);
  if (!metadata || metadata.size > 10_000_000) throw new Error(`Unreviewed or oversized source file: ${sourcePath}`);
  const destination = join(cacheDirectory, sourcePath);
  const bytes = (bundledPath && await readOptional(join(root, bundledPath))) || await readOptional(destination) || await fetchBytes(rawUrl(sourcePath), metadata.size);
  if (bytes.length !== metadata.size || gitBlobSha1(bytes) !== metadata.sha) throw new Error(`Pinned source verification failed: ${sourcePath}`);
  if (sourcePath.endsWith('.stl')) {
    const triangles = bytes.length >= 84 ? bytes.readUInt32LE(80) : 0;
    if (!triangles || bytes.length !== 84 + triangles * 50) throw new Error(`Invalid binary STL: ${sourcePath}`);
    for (let triangle = 0; triangle < triangles; triangle++) {
      for (let coordinate = 0; coordinate < 9; coordinate++) {
        if (!Number.isFinite(bytes.readFloatLE(84 + triangle * 50 + 12 + coordinate * 4))) throw new Error(`Non-finite STL vertex: ${sourcePath}`);
      }
    }
  }
  await mkdir(dirname(destination), { recursive: true });
  await writeFile(destination, bytes);
  bytesCache.set(sourcePath, bytes);
  return bytes;
}

function assetRecord(path, bytes, url) {
  return { path, bytes: bytes.length, sha256: sha256(bytes), gitBlobSha1: gitBlobSha1(bytes), url };
}

async function sourceAsset(sourcePath, path, bundledPath) {
  return assetRecord(path, await sourceBytes(sourcePath, bundledPath), rawUrl(sourcePath));
}

async function standardBsdAsset() {
  const path = join(cacheDirectory, 'BSD-3-Clause.txt');
  const bytes = await readOptional(path) || await fetchBytes(spdxUrl, 1460);
  if (bytes.length !== 1460 || sha256(bytes) !== '5a93d5831e1297ab10fe643e1a631e83be392896da14ee2951285a79012df69d') throw new Error('Pinned SPDX BSD-3-Clause template verification failed.');
  await mkdir(dirname(path), { recursive: true });
  await writeFile(path, bytes);
  vendorFiles.set('vendor/work-r-labs/model-notices/BSD-3-Clause-template.txt', bytes);
  return assetRecord('notices/BSD-3-Clause-template.txt', bytes, spdxUrl);
}

const models = [];
const fixtureRows = [];
for (const definition of definitions) {
  const prefix = `library/${definition.directory}/`;
  const meshNames = ['base_link.stl', ...Array.from({ length: 6 }, (_, index) => `link_${index + 1}.stl`), ...(definition.additionalMeshes || [])];
  const sourcePaths = [definition.urdfPath, ...meshNames.map(name => `meshes/${name}`)];
  const files = [];
  for (const path of sourcePaths) {
    const bundledPath = definition.bundledMotionModelId && path.startsWith('meshes/') ? `public/models/${definition.id}/${path.slice('meshes/'.length)}` : undefined;
    files.push(await sourceAsset(prefix + path, path, bundledPath));
  }
  const urdfBytes = await sourceBytes(prefix + definition.urdfPath);
  const meshReferences = [...urdfBytes.toString('utf8').matchAll(/<mesh\s+filename="([^"]+)"/g)].map(match => match[1]);
  if (!meshReferences.length || meshReferences.some(path => !sourcePaths.includes(path)) || meshNames.some(name => !meshReferences.includes(`meshes/${name}`))) {
    throw new Error(`URDF mesh references differ from the reviewed asset list: ${definition.id}`);
  }
  if (!definition.bundledMotionModelId) {
    vendorFiles.set(`tests/fixtures/catalog/${definition.id}.urdf`, urdfBytes);
    fixtureRows.push(`| ${definition.id}.urdf | [Pinned source](${rawUrl(prefix + definition.urdfPath)}) | \`${sha256(urdfBytes)}\` |`);
  }
  const workrNotice = await sourceAsset('LICENSE', 'notices/work-r-labs-MIT.txt');
  files.push(workrNotice);
  const licenses = [{ name: 'Workr Robot Asset Library — MIT (2025 work_r)', spdx: 'MIT', path: workrNotice.path, url: workrNotice.url }];
  if (definition.originalMeshDirectory) {
    const notice = await sourceAsset(definition.originalNotice, 'notices/motoman-package.xml');
    files.push(notice);
    vendorFiles.set(`vendor/work-r-labs/model-notices/${definition.id}-package.xml`, await sourceBytes(definition.originalNotice));
    licenses.push({ name: 'ROS-Industrial Motoman — declared BSD-3-Clause (original package.xml)', spdx: 'BSD-3-Clause', path: notice.path, url: notice.url });
    const terms = await standardBsdAsset();
    files.push(terms);
    licenses.push({ name: 'BSD-3-Clause standard terms template — copyright placeholders are not model attribution', spdx: 'BSD-3-Clause', path: terms.path, url: terms.url });
    const originalFiles = tree.tree.filter(entry => entry.type === 'blob' && entry.path.startsWith(definition.originalMeshDirectory));
    for (const name of meshNames) {
      const libraryPath = `${prefix}meshes/${name}`;
      const metadata = treeFiles.get(libraryPath);
      const matches = originalFiles.filter(entry => entry.sha === metadata.sha && entry.size === metadata.size);
      if (matches.length !== 1) throw new Error(`Original Motoman mesh provenance is ambiguous: ${libraryPath}`);
      provenance.push({ modelId: definition.id, libraryPath, originalPath: matches[0].path, bytes: metadata.size, gitBlobSha1: metadata.sha, sha256: files.find(file => file.path === `meshes/${name}`).sha256 });
    }
  } else {
    const notice = await sourceAsset(definition.originalNotice, 'notices/source-model-MIT.txt');
    files.push(notice);
    const noticeBytes = await sourceBytes(definition.originalNotice);
    const commonNotice = vendorFiles.get('vendor/work-r-labs/model-notices/source-model-MIT.txt');
    if (commonNotice && !commonNotice.equals(noticeBytes)) throw new Error('Source model MIT notices differ; preserve them separately.');
    vendorFiles.set('vendor/work-r-labs/model-notices/source-model-MIT.txt', noticeBytes);
    licenses.push({ name: 'Original model package — MIT (2018 Toshinori Kitamura)', spdx: 'MIT', path: notice.path, url: notice.url });
  }
  models.push({
    id: definition.id, brand: definition.brand, name: definition.name,
    ...(definition.bundledMotionModelId ? { bundledMotionModelId: definition.bundledMotionModelId } : {}),
    urdfPath: definition.urdfPath, files, licenses,
    downloadBytes: files.reduce((sum, file) => sum + file.bytes, 0),
  });
}

const output = { schemaVersion: 1, source: { repository, revision }, models };
vendorFiles.set('src/catalog/catalog.json', Buffer.from(JSON.stringify(output, null, 2) + '\n'));
vendorFiles.set('vendor/work-r-labs/model-notices/motoman-mesh-provenance.json', Buffer.from(JSON.stringify({ repository, revision, meshes: provenance }, null, 2) + '\n'));
vendorFiles.set('tests/fixtures/catalog/README.md', Buffer.from(`# Catalog URDF fixtures\n\nThese four small URDF files are unchanged bytes from [Workr Robot Asset Library](${repository}/tree/${revision}), revision \`${revision}\`. No STL meshes are included in the fixtures. They exercise the actual URDF structure, visual offsets, metre/millimetre scales, and fixed branches in offline parser/FK tests; they do not establish controller or motion-planner compatibility.\n\n| Fixture | Source | SHA-256 |\n| --- | --- | --- |\n${fixtureRows.join('\n')}\n\nSource notices and the distinction between Workr MIT and original Motoman BSD-3-Clause declarations are preserved in [model-notices](../../../vendor/work-r-labs/model-notices/README.md). FANUC source packages retain the original Toshinori Kitamura MIT notice. The original Motoman package declarations and standard BSD-3-Clause terms template are retained without inventing copyright holders.\n\nRegenerate these fixtures, notices and the catalog together with \`node scripts/generate-model-catalog.mjs\`; verify without modifying them with \`--check\`.\n`));
for (const [path, bytes] of vendorFiles) {
  const destination = join(root, path);
  if (check) {
    const existing = await readOptional(destination);
    if (!existing?.equals(bytes)) throw new Error(`Generated file is stale: ${path}`);
  } else {
    await mkdir(dirname(destination), { recursive: true });
    await writeFile(destination, bytes);
  }
}
console.log(`${check ? 'Verified' : 'Generated'} ${models.length} pinned models, ${models.reduce((sum, model) => sum + model.files.length, 0)} asset entries, ${models.reduce((sum, model) => sum + model.downloadBytes, 0)} total bytes. Mesh downloads remain outside the repository: ${cacheDirectory}`);
