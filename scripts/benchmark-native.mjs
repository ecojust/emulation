import { execFileSync } from 'node:child_process';
import { mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir, cpus, platform, arch, release } from 'node:os';
import { resolve, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { build } from 'esbuild';
import { createHash } from 'node:crypto';

const root = fileURLToPath(new URL('../', import.meta.url));
const cargo = process.env.CARGO || 'cargo';
const temporary = mkdtempSync(join(tmpdir(), 'emulation-benchmark-'));
try {
  execFileSync(cargo, ['build', '--release', '--locked', '-p', 'emulation-core', '--example', 'benchmark'], { cwd: root, stdio: ['ignore', 'inherit', 'inherit'] });
  const binary = resolve(root, process.env.CARGO_TARGET_DIR || 'target', 'release', 'examples', platform() === 'win32' ? 'benchmark.exe' : 'benchmark');
  const reference = join(temporary, 'reference.mjs');
  await build({ entryPoints: [resolve(root, 'scripts/benchmark-reference.ts')], bundle: true, platform: 'node', format: 'esm', outfile: reference });
  const fixtureText = execFileSync(process.execPath, [reference, '--fixtures'], { encoding: 'utf8' });
  const fixtureFile = join(temporary, 'fixtures.json');
  writeFileSync(fixtureFile, fixtureText);
  const run = (executable, args) => JSON.parse(execFileSync(executable, args, { encoding: 'utf8', maxBuffer: 10 * 1024 * 1024 }));
  // Separate processes, no concurrent measurement. Reverse order on the second round to expose order bias.
  const rounds = [
    { rust: run(binary, [fixtureFile]), javascript: run(process.execPath, [reference, fixtureFile]) },
    (() => { const javascript = run(process.execPath, [reference, fixtureFile]); return { javascript, rust: run(binary, [fixtureFile]) }; })(),
  ];
  const quantile = (values, fraction) => { const sorted = [...values].sort((a, b) => a - b); return sorted[Math.ceil(sorted.length * fraction) - 1]; };
  const cases = rounds[0].rust.map((measurement, index) => {
    const rustNs = rounds.flatMap(round => round.rust[index].batchMeanNs);
    const jsNs = rounds.flatMap(round => round.javascript[index].batchMeanNs);
    for (const round of rounds) {
      if (round.rust[index].name !== measurement.name || round.javascript[index].name !== measurement.name) throw new Error('Mismatched benchmark cases');
      if (!Number.isFinite(round.rust[index].checksum) || !Number.isFinite(round.javascript[index].checksum)) throw new Error('Invalid checksum');
      const checksumDifference = Math.abs(round.rust[index].checksum - round.javascript[index].checksum);
      if (checksumDifference > 1e-8 * Math.max(1, Math.abs(round.javascript[index].checksum))) throw new Error(`Different computation results for ${measurement.name}: ${checksumDifference}`);
    }
    const rustMedianNs = quantile(rustNs, .5), jsMedianNs = quantile(jsNs, .5);
    return { name: measurement.name, iterationsPerBatch: measurement.iterations, rustMedianUs: rustMedianNs / 1000, rustP95Us: quantile(rustNs, .95) / 1000, jsMedianUs: jsMedianNs / 1000, jsP95Us: quantile(jsNs, .95) / 1000, rustMedianOpsPerSecond: 1e9 / rustMedianNs, jsMedianOpsPerSecond: 1e9 / jsMedianNs, medianSpeedup: jsMedianNs / rustMedianNs, rustBatchMeanNs: rustNs, jsBatchMeanNs: jsNs, checksums: rounds.map(round => ({ rust: round.rust[index].checksum, javascript: round.javascript[index].checksum })) };
  });
  const fingerprints = ['Cargo.toml', 'Cargo.lock', 'crates/robot-core/src/math.rs', 'crates/robot-core/src/model.rs', 'crates/robot-core/src/configuration.rs', 'crates/robot-core/src/kinematics.rs', 'crates/robot-core/src/planner.rs', 'crates/robot-core/examples/benchmark.rs', 'src/robot/kinematics.ts', 'src/robot/planner.ts', 'scripts/benchmark-reference.ts', 'scripts/benchmark-native.mjs'];
  const sourceSha256 = Object.fromEntries(fingerprints.map(path => [path, createHash('sha256').update(readFileSync(resolve(root, path))).digest('hex')]));
  const report = { sourceSha256, measuredAt: new Date().toISOString(), commit: execFileSync('git', ['rev-parse', 'HEAD'], { cwd: root, encoding: 'utf8' }).trim(), worktreeDirty: Boolean(execFileSync('git', ['status', '--porcelain'], { cwd: root, encoding: 'utf8' }).trim()), host: { platform: platform(), arch: arch(), kernel: release(), cpu: cpus()[0]?.model, logicalCpus: cpus().length, node: process.version, rustc: execFileSync('rustc', ['--version'], { encoding: 'utf8' }).trim() }, build: { profile: 'release', optLevel: 3, lto: 'thin', codegenUnits: 1, nativeCpuTuning: false }, method: { samplesPerRound: JSON.parse(fixtureText).samples, warmupBatches: JSON.parse(fixtureText).warmupBatches, rounds: ['Rust then Node', 'Node then Rust'], metric: 'Per-operation batch mean; median and p95 over warmed batches', excludes: ['process startup', 'JSON serialization', 'Tauri IPC', 'WebView rendering'] }, cases };
  const output = process.argv[2];
  if (output) writeFileSync(resolve(output), JSON.stringify(report, null, 2) + '\n');
  process.stdout.write(JSON.stringify(report, null, 2) + '\n');
} finally { rmSync(temporary, { recursive: true, force: true }); }
