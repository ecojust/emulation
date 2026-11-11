import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { appendFileSync, copyFileSync, lstatSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { basename, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const installedValidation = 'Installed or DMG-copied native executable passed --smoke-test and real WebView --ui-smoke-test on the target architecture.';
export const platforms = [
  { artifact: 'macOS-arm64', job: 'macOS Apple Silicon', target: 'aarch64-apple-darwin', suffix: '.dmg', required: true },
  { artifact: 'Windows-x64', job: 'Windows x64', target: 'x86_64-pc-windows-msvc', suffix: '-setup.exe', required: true },
  { artifact: 'macOS-x64', job: 'macOS Intel', target: 'x86_64-apple-darwin', suffix: '.dmg', required: false },
];
const sha256 = bytes => createHash('sha256').update(bytes).digest('hex');
function requireCondition(condition, message) { if (!condition) throw new Error(message); }

export function assertRun(run, workflow, sha, repository, successful) {
  requireCondition(run.path === `.github/workflows/${workflow}`, `Expected ${workflow} workflow.`);
  requireCondition(run.head_sha === sha && run.head_branch === 'master', 'Run must verify the exact requested master commit. PR merge builds are not release inputs.');
  requireCondition(run.head_repository?.full_name === repository, 'Run must originate in this repository.');
  requireCondition(['push', 'workflow_dispatch'].includes(run.event), 'Unsupported run event.');
  if (successful) requireCondition(run.status === 'completed' && run.conclusion === 'success', 'Required checks have not completed successfully.');
  else requireCondition(run.status === 'in_progress' && !run.conclusion || run.status === 'completed' && ['success', 'failure'].includes(run.conclusion), 'Desktop workflow was queued, cancelled, or otherwise unavailable.');
}

export function selectPlatforms(jobs) {
  return platforms.filter(platform => {
    const matches = jobs.filter(job => job.name === platform.job);
    requireCondition(matches.length === 1, `Expected one current ${platform.job} job.`);
    const passed = matches[0].status === 'completed' && matches[0].conclusion === 'success';
    requireCondition(passed || !platform.required, `${platform.job} must pass its complete installed-client checks.`);
    return passed;
  });
}

function filesBelow(directory) {
  const result = [];
  for (const name of readdirSync(directory)) {
    const path = join(directory, name), stat = lstatSync(path);
    requireCondition(!stat.isSymbolicLink(), 'Artifact must not contain symbolic links.');
    if (stat.isDirectory()) result.push(...filesBelow(path));
    else { requireCondition(stat.isFile(), 'Artifact contains a non-regular file.'); result.push(path); }
  }
  return result;
}

export function validateArtifact(directory, platform, version, sha) {
  const files = filesBelow(directory);
  const manifests = files.filter(path => basename(path) === 'artifact-manifest.json');
  requireCondition(files.length === 2 && manifests.length === 1, `Expected exactly one installer and manifest for ${platform.artifact}.`);
  requireCondition(lstatSync(manifests[0]).size < 128 * 1024, 'Manifest is too large.');
  const manifest = JSON.parse(readFileSync(manifests[0], 'utf8'));
  requireCondition(manifest.application === 'Emulation' && manifest.version === version && manifest.target === platform.target && manifest.sourceCommit === sha, 'Manifest source/version/platform does not match the release.');
  requireCondition(manifest.validation === installedValidation, 'Manifest does not attest the complete installed-client check.');
  const signing = platform.suffix === '.dmg' ? 'ad-hoc integrity signature; not Apple notarized' : 'unsigned';
  requireCondition(manifest.signing === signing, 'Unexpected signing description.');
  requireCondition(Array.isArray(manifest.files) && manifest.files.length === 1, 'Manifest must list exactly one installer.');
  const entry = manifest.files[0];
  requireCondition(typeof entry.name === 'string' && /^[A-Za-z0-9][A-Za-z0-9_. -]*$/.test(entry.name) && entry.name.endsWith(platform.suffix), 'Invalid installer filename.');
  requireCondition(Number.isSafeInteger(entry.bytes) && entry.bytes > 0 && entry.bytes <= 300 * 1024 * 1024 && /^[a-f0-9]{64}$/.test(entry.sha256), 'Invalid installer checksum metadata.');
  const installer = files.find(path => path !== manifests[0]);
  requireCondition(basename(installer) === entry.name && lstatSync(installer).size === entry.bytes, 'Installer filename or length differs from its manifest.');
  requireCondition(sha256(readFileSync(installer)) === entry.sha256, 'Installer SHA-256 differs from its manifest.');
  return { installer, manifestPath: manifests[0], manifest };
}

function gh(args, input) {
  try { return execFileSync('gh', args, { input, encoding: 'utf8', maxBuffer: 8 * 1024 * 1024, stdio: ['pipe', 'pipe', 'pipe'] }); }
  catch (error) {
    // Do not print redirect URLs, credentials, or arbitrary CLI response bodies.
    const missing = String(error.stderr).includes('(HTTP 404)');
    throw Object.assign(new Error(`GitHub command failed (${missing ? 'not found' : `exit ${error.status ?? 'unknown'}`}).`), { missing });
  }
}
function api(path, payload) { return JSON.parse(gh(['api', path, ...(payload ? ['--method', 'POST', '--input', '-'] : [])], payload ? JSON.stringify(payload) : undefined)); }
function optionalApi(path) { try { return api(path); } catch (error) { if (error.missing) return null; throw error; } }
function pages(path, key) {
  // One encoded JSON object per line works with both older and current gh CLI.
  const output = gh(['api', path, '--paginate', '--jq', `${key ? `.${key}` : '.'}[] | tojson`]).trim();
  return output ? output.split('\n').map(line => JSON.parse(line)) : [];
}
function releaseForTag(repository, tag) {
  // The published-release-by-tag endpoint does not reliably expose drafts.
  const matches = pages(`repos/${repository}/releases?per_page=100`).filter(item => item.tag_name === tag);
  requireCondition(matches.length <= 1, 'Multiple releases use the requested tag.');
  return matches[0] ?? null;
}
function tagCommit(repository, tag) {
  const ref = optionalApi(`repos/${repository}/git/ref/tags/${tag}`);
  if (!ref) return null;
  let object = ref.object;
  for (let depth = 0; depth < 8 && object.type === 'tag'; depth++) object = api(`repos/${repository}/git/tags/${object.sha}`).object;
  requireCondition(object.type === 'commit', 'Release tag must resolve to a commit.');
  return object.sha;
}

function validateRelease() {
  const repository = process.env.GH_REPO, sha = process.env.RELEASE_SOURCE_SHA;
  const desktopId = process.env.RELEASE_DESKTOP_RUN_ID, checkId = process.env.RELEASE_CHECK_RUN_ID;
  requireCondition(/^[A-Za-z0-9_.-]+\/[A-Za-z0-9_.-]+$/.test(repository ?? ''), 'Invalid repository.');
  requireCondition(/^[a-f0-9]{40}$/.test(sha ?? ''), 'Expected a full source commit SHA.');
  requireCondition(/^\d+$/.test(desktopId ?? '') && /^\d+$/.test(checkId ?? ''), 'Invalid workflow run IDs.');
  const checkout = execFileSync('git', ['rev-parse', 'HEAD'], { encoding: 'utf8' }).trim();
  requireCondition(checkout === sha, 'Checked-out source differs from the requested commit.');
  const version = JSON.parse(readFileSync('package.json', 'utf8')).version;
  requireCondition(/^\d+\.\d+\.\d+$/.test(version), 'Only stable three-part application versions can be released here.');
  requireCondition(JSON.parse(readFileSync('src-tauri/tauri.conf.json', 'utf8')).version === version, 'Desktop and package versions differ.');
  const ancestry = api(`repos/${repository}/compare/${sha}...master`);
  requireCondition(['identical', 'ahead'].includes(ancestry.status) && ancestry.merge_base_commit?.sha === sha, 'Release source must be on master history.');
  const desktop = api(`repos/${repository}/actions/runs/${desktopId}`), check = api(`repos/${repository}/actions/runs/${checkId}`);
  assertRun(desktop, 'desktop.yml', sha, repository, false);
  assertRun(check, 'check.yml', sha, repository, true);
  const selected = selectPlatforms(pages(`repos/${repository}/actions/runs/${desktopId}/jobs?filter=latest&per_page=100`, 'jobs'));
  const artifacts = pages(`repos/${repository}/actions/runs/${desktopId}/artifacts?per_page=100`, 'artifacts');
  for (const platform of selected) {
    const matches = artifacts.filter(item => item.name === `emulation-${version}-${platform.artifact}`);
    requireCondition(matches.length === 1 && !matches[0].expired, `Missing current verified artifact for ${platform.artifact}.`);
  }
  return { repository, sha, desktopId, checkId, version, tag: `v${version}`, selected };
}

function publishRelease(context) {
  const { repository, sha, desktopId, checkId, version, tag, selected } = context;
  // Verify every archive before creating or modifying any release or tag.
  const verified = selected.map(platform => ({ platform, ...validateArtifact(resolve('release-input', platform.artifact), platform, version, sha) }));
  const directory = resolve('release-assets');
  mkdirSync(directory, { recursive: true });
  requireCondition(readdirSync(directory).length === 0, 'Release staging directory must be empty.');
  const staged = [];
  for (const item of verified) {
    for (const [source, name] of [[item.installer, item.manifest.files[0].name], [item.manifestPath, `Emulation_${version}_${item.platform.artifact}.manifest.json`]]) {
      requireCondition(!staged.some(path => basename(path) === name), 'Duplicate release asset name.');
      const destination = join(directory, name); copyFileSync(source, destination); staged.push(destination);
    }
  }
  const sums = join(directory, 'SHA256SUMS.txt');
  writeFileSync(sums, staged.map(path => `${sha256(readFileSync(path))}  ${basename(path)}`).sort().join('\n') + '\n'); staged.push(sums);
  const intelVerified = selected.some(platform => platform.artifact === 'macOS-x64');
  const notes = [
    "新增手机示教器：在项目顶部点击“示教器”，手机通过同一局域网扫码打开，无需安装 App。虚拟摇杆支持 TCP 线模式、J1–J6 轴模式、世界／工具坐标和速度调节，电脑同步显示仿真机器人运动。",
    "",
    "松手、切到后台或断连后停止当前操作，恢复连接后需重新按住摇杆；返回主页或主动断开会结束会话。运动计算由 Rust 执行，按当前机器人型号及基座／工具配置计算，示教器保持独立模块，便于后续扩展。",
    "",
    "新增高对比度明／暗主题，主页、工作台、代码编辑器和三维场景同步适配并保存选择；切换主题保持机器人姿态和相机。手机页面跟随手机系统主题。",
    "",
    "兼容与边界：",
    "- 手机与电脑需要处于可互通的局域网；首次使用时允许应用的局域网访问，并检查系统防火墙连接许可。",
    "- 同一会话只连接一个手机；手机连接期间，电脑端场景对象和属性编辑暂时锁定，断开后可继续编辑。",
    "- 仅控制本机仿真，不连接真实机器人。运动检查覆盖现有机器人／工具包络与禁入区，不认证导入工件碰撞、自碰撞或实际控制器执行。",
    "- 项目文档保持格式 2，兼容读取旧格式；v0.4.1 及更早客户端不能打开新版保存的项目。",
    '',
    '安装包及校验：',
    '- macOS Apple Silicon 与 Windows x64 均已完成实际安装、原生引擎和 WebView／3D 检查。',
    intelVerified ? '- macOS Intel 也已完成相同的完整安装检查，本次附带 x64 DMG。' : '- macOS Intel 在本次发布准备阶段尚无完整验收通过的产物，因此未附带该平台安装包；后续状态见原生构建记录，仍需完成可用 WebGL2 环境中的图形检查。',
    '- macOS 包采用 ad-hoc 完整性签名，未做 Apple Developer ID 公证；Windows 安装包未进行商业代码签名。',
    '- 每个平台附带源提交和文件摘要 manifest；SHA256SUMS.txt 可校验安装包。',
    '',
    `Source commit: \`${sha}\``,
    `[完整测试](https://github.com/${repository}/actions/runs/${checkId}) · [原生构建及安装验证](https://github.com/${repository}/actions/runs/${desktopId})`,
  ].join('\n');
  const notesPath = resolve('release-notes.md'); writeFileSync(notesPath, notes + '\n');
  const prior = releaseForTag(repository, tag);
  requireCondition(!prior || prior.draft && !prior.prerelease && prior.target_commitish === sha, 'Refusing to replace a published release, prerelease draft, or draft for another source.');
  const existingTag = tagCommit(repository, tag);
  requireCondition(!existingTag || existingTag === sha, 'Release tag already identifies another commit.');
  if (!existingTag) api(`repos/${repository}/git/refs`, { ref: `refs/tags/${tag}`, sha });
  if (!prior) gh(['release', 'create', tag, '--repo', repository, '--verify-tag', '--draft', '--target', sha, '--title', `Emulation ${tag}`, '--notes-file', notesPath]);
  else gh(['release', 'edit', tag, '--repo', repository, '--draft', '--notes-file', notesPath]);
  gh(['release', 'upload', tag, ...staged, '--repo', repository, '--clobber']);
  const draft = releaseForTag(repository, tag);
  requireCondition(draft.draft === true, 'Release must remain a draft until asset verification completes.');
  requireCondition(draft.assets.length === staged.length, 'Draft contains unexpected assets; leaving it unpublished.');
  for (const path of staged) {
    const bytes = readFileSync(path), entry = draft.assets.find(item => item.name === basename(path));
    requireCondition(entry?.state === 'uploaded' && entry.size === bytes.length, 'Uploaded release asset is incomplete.');
    if (entry.digest) requireCondition(entry.digest === `sha256:${sha256(bytes)}`, 'Uploaded release asset digest differs.');
    else {
      let remote;
      try { remote = execFileSync('gh', ['api', `repos/${repository}/releases/assets/${entry.id}`, '-H', 'Accept: application/octet-stream'], { maxBuffer: 320 * 1024 * 1024, stdio: ['ignore', 'pipe', 'pipe'] }); }
      catch { throw new Error('Could not verify uploaded release bytes; leaving the draft unpublished.'); }
      requireCondition(sha256(remote) === sha256(bytes), 'Uploaded release asset bytes differ.');
    }
  }
  // A rerun/cancellation during download or upload must not be reported as verified.
  const finalState = validateRelease();
  requireCondition(selected.every(platform => finalState.selected.some(item => item.target === platform.target)), 'Platform validation changed during publication; leaving the draft unpublished.');
  requireCondition(tagCommit(repository, tag) === sha, 'Release tag changed during publication; leaving the draft unpublished.');
  gh(['release', 'edit', tag, '--repo', repository, '--draft=false', '--latest', '--notes-file', notesPath]);
  const published = api(`repos/${repository}/releases/tags/${tag}`);
  requireCondition(!published.draft && !published.prerelease && published.tag_name === tag, 'Release publication was not confirmed.');
  console.log(`Published ${published.html_url} from ${sha}.`);
  if (process.env.GITHUB_STEP_SUMMARY) appendFileSync(process.env.GITHUB_STEP_SUMMARY, `Published [${tag}](${published.html_url}) from \`${sha}\`.\n\nVerified platforms: ${selected.map(platform => platform.artifact).join(', ')}.\n`);
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    requireCondition(['validate', 'publish'].includes(process.argv[2]), 'Use validate or publish.');
    const context = validateRelease();
    if (process.argv[2] === 'validate') {
      if (process.env.GITHUB_OUTPUT) appendFileSync(process.env.GITHUB_OUTPUT, `version=${context.version}\nintel_verified=${context.selected.some(platform => platform.artifact === 'macOS-x64')}\n`);
      console.log(`Verified ${context.sha}, ${context.tag}, and ${context.selected.map(platform => platform.artifact).join(', ')}.`);
    } else publishRelease(context);
  } catch (error) { console.error(error.message); process.exitCode = 1; }
}
