import { build } from 'esbuild';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../', import.meta.url));
const check = process.argv.includes('--check');
if (process.argv.slice(2).some(argument => argument !== '--check')) throw new Error('Usage: node scripts/generate-generic-chains.mjs [--check]');
const ids = ['fanuc-crx10ia-l', 'fanuc-r2000ib-165f', 'yaskawa-gp7', 'yaskawa-gp110'];
const manifest = JSON.parse(await readFile(join(root, 'src/catalog/catalog.json'), 'utf8'));
const sources = [];
for (const id of ids) {
  const model = manifest.models.find(model => model.id === id);
  const source = model?.files.find(file => file.path === model.urdfPath);
  const file = `tests/fixtures/catalog/${id}.urdf`;
  const bytes = await readFile(join(root, file));
  const sha256 = createHash('sha256').update(bytes).digest('hex');
  if (!source || bytes.length !== source.bytes || sha256 !== source.sha256) throw new Error(`Pinned URDF fixture changed: ${id}`);
  sources.push({ id, file, bytes: bytes.length, sha256, url: source.url });
}

const temporary = await mkdtemp(join(tmpdir(), 'emulation-generic-chains-'));
let chains, collision;
try {
  const entry = `
    import { readFileSync } from 'node:fs';
    import { DOMParser } from '@xmldom/xmldom';
    import { parseUrdf } from './src/catalog/urdf';
    import { buildGenericChain } from './src/robot/generic-motion-chain';
    import { buildGenericCollisionDescriptor } from './src/catalog/collision-descriptors';
    globalThis.DOMParser = DOMParser;
    const sources = ${JSON.stringify(sources)};
    const models = sources.map(source => ({
      id: source.id, name: source.id,
      description: parseUrdf(readFileSync(source.file, 'utf8')), geometries: new Map(),
    }));
    const manifest = ${JSON.stringify(manifest)};
    process.stdout.write(JSON.stringify({chains: models.map(buildGenericChain), collision: models.map(model => buildGenericCollisionDescriptor(model, manifest.models.find(item => item.id === model.id).files))}));
  `;
  const bundle = join(temporary, 'generate.cjs');
  await build({ stdin: { contents: entry, resolveDir: root, loader: 'ts' }, bundle: true, platform: 'node', format: 'cjs', outfile: bundle, logLevel: 'silent' });
  ({chains, collision} = JSON.parse(execFileSync(process.execPath, [bundle], { cwd: root, encoding: 'utf8', timeout: 30_000, maxBuffer: 1_000_000 })));
} finally {
  await rm(temporary, { recursive: true, force: true });
}

const generated = {
  'src/catalog/generic-collision.json': collision,
  'tests/fixtures/catalog/generic-chains.json': chains,
  'tests/fixtures/catalog/generic-chains.source.json': {
    schemaVersion: 1,
    source: manifest.source,
    generator: 'scripts/generate-generic-chains.mjs',
    coordinates: 'Original URDF root-to-tool ancestry; row-major millimetres; no geometry or controller corrections',
    sources,
  },
};
for (const [file, value] of Object.entries(generated)) {
  const output = JSON.stringify(value, null, 2) + '\n';
  if (check) {
    if (await readFile(join(root, file), 'utf8') !== output) throw new Error(`Generated generic chain is stale: ${file}`);
  } else await writeFile(join(root, file), output);
}
console.log(`${check ? 'Verified' : 'Generated'} ${chains.length} exact URDF tool chains from pinned source fixtures.`);
