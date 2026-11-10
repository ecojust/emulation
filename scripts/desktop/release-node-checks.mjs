import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { mkdtempSync, mkdirSync, readFileSync, rmSync, symlinkSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { test } from 'node:test';
import { assertRun, platforms, selectPlatforms, validateArtifact } from './release.mjs';

const sha = 'a'.repeat(40), repository = 'ecojust/emulation';
const run = { path: '.github/workflows/check.yml', head_sha: sha, head_branch: 'master', head_repository: { full_name: repository }, event: 'push', status: 'completed', conclusion: 'success' };
test('release inputs require the exact successful master workflow, not a PR merge or another source', () => {
  assert.doesNotThrow(() => assertRun(run, 'check.yml', sha, repository, true));
  for (const patch of [
    { head_sha: 'b'.repeat(40) }, { head_branch: 'feature' }, { event: 'pull_request' },
    { path: '.github/workflows/other.yml' }, { status: 'in_progress' }, { conclusion: 'failure' },
    { head_repository: { full_name: 'another/repository' } },
  ]) assert.throws(() => assertRun({ ...run, ...patch }, 'check.yml', sha, repository, true));
  assert.doesNotThrow(() => assertRun({ ...run, path: '.github/workflows/desktop.yml', conclusion: 'failure' }, 'desktop.yml', sha, repository, false));
  assert.doesNotThrow(() => assertRun({ ...run, path: '.github/workflows/desktop.yml', status: 'in_progress', conclusion: null }, 'desktop.yml', sha, repository, false));
  assert.throws(() => assertRun({ ...run, path: '.github/workflows/desktop.yml', status: 'queued', conclusion: null }, 'desktop.yml', sha, repository, false));
  assert.throws(() => assertRun({ ...run, path: '.github/workflows/desktop.yml', conclusion: 'cancelled' }, 'desktop.yml', sha, repository, false));
});

const jobs = platforms.map(platform => ({ name: platform.job, status: 'completed', conclusion: platform.required ? 'success' : 'failure' }));
test('both required platforms must pass; Intel is included only after its entire job passes', () => {
  assert.deepEqual(selectPlatforms(jobs).map(platform => platform.artifact), ['macOS-arm64', 'Windows-x64']);
  assert.equal(selectPlatforms(jobs.map(job => job.name === 'macOS Intel' ? { ...job, status: 'in_progress', conclusion: null } : job)).length, 2);
  assert.throws(() => selectPlatforms(jobs.map(job => job.name === 'Windows x64' ? { ...job, status: 'in_progress', conclusion: null } : job)));
  assert.equal(selectPlatforms(jobs.map(job => ({ ...job, conclusion: 'success' }))).length, 3);
  assert.throws(() => selectPlatforms(jobs.filter(job => job.name !== 'Windows x64')));
  assert.throws(() => selectPlatforms([...jobs, jobs[0]]));
  for (const conclusion of ['failure', 'cancelled', 'skipped']) assert.throws(() => selectPlatforms(jobs.map(job => job.name === 'Windows x64' ? { ...job, conclusion } : job)));
});

function fixture(t, platform = platforms[0]) {
  const directory = mkdtempSync(join(tmpdir(), 'emulation-release-'));
  t.after(() => rmSync(directory, { recursive: true, force: true }));
  const bytes = Buffer.from('verified installer fixture'), name = `Emulation_0.2.1_${platform.artifact}${platform.suffix}`;
  mkdirSync(join(directory, 'bundle'));
  const installer = join(directory, 'bundle', name), manifestPath = join(directory, 'artifact-manifest.json');
  writeFileSync(installer, bytes);
  const manifest = { application: 'Emulation', version: '0.2.1', target: platform.target, sourceCommit: sha,
    signing: platform.suffix === '.dmg' ? 'ad-hoc integrity signature; not Apple notarized' : 'unsigned',
    validation: 'Installed or DMG-copied native executable passed --smoke-test and real WebView --ui-smoke-test on the target architecture.',
    files: [{ name, bytes: bytes.length, sha256: createHash('sha256').update(bytes).digest('hex') }] };
  const save = () => writeFileSync(manifestPath, JSON.stringify(manifest)); save();
  return { directory, installer, manifestPath, manifest, save, platform };
}

test('checks actual installer bytes against version, source, target and installed validation', t => {
  for (const platform of platforms) {
    const f = fixture(t, platform);
    assert.equal(validateArtifact(f.directory, platform, '0.2.1', sha).installer, f.installer);
    assert.throws(() => validateArtifact(f.directory, platform, '0.2.0', sha));
    assert.throws(() => validateArtifact(f.directory, platform, '0.2.1', 'b'.repeat(40)));
    f.manifest.validation = 'Build only'; f.save();
    assert.throws(() => validateArtifact(f.directory, platform, '0.2.1', sha));
  }
});

test('rejects changed bytes even when length and filenames still agree', t => {
  const f = fixture(t), bytes = readFileSync(f.installer); bytes[0] ^= 1; writeFileSync(f.installer, bytes);
  assert.throws(() => validateArtifact(f.directory, f.platform, '0.2.1', sha), /SHA-256/);
});

test('rejects length, architecture and metadata mismatches', t => {
  for (const change of [
    m => { m.files[0].bytes++; }, m => { m.target = platforms[1].target; },
    m => { m.files[0].sha256 = 'bad'; }, m => { m.signing = 'notarized'; },
    m => { m.files[0].name = '../outside.dmg'; }, m => { m.files[0].name = 'installer.dmg#label'; },
  ]) {
    const f = fixture(t); change(f.manifest); f.save();
    assert.throws(() => validateArtifact(f.directory, f.platform, '0.2.1', sha));
  }
});

test('rejects extra artifacts and symlinks instead of uploading unverified files', t => {
  const extra = fixture(t); writeFileSync(join(extra.directory, 'extra.dmg'), 'unchecked');
  assert.throws(() => validateArtifact(extra.directory, extra.platform, '0.2.1', sha));
  const link = fixture(t); rmSync(link.installer); symlinkSync(link.manifestPath, link.installer);
  assert.throws(() => validateArtifact(link.directory, link.platform, '0.2.1', sha), /symbolic/);
});
