import { expect, test, type Page } from '@playwright/test';
import { prepareCatalogFixtures, readCatalog } from './catalog-fixtures';
import { generateSource, openCode } from './programming-helpers';
import type { Pose } from '../../src/robot/types';
import { bodyCanvas, bodyJoints, bodyPose, captureCurrent, createPath, expectBodyJoints, expectNoPointMotionActions, runPathFromMenu, selectPathForPoints, observeBrowser, openScene, openWorkspace, pathObjects, pathPoints, selectPath, selectRobot, selectRobotObject, setJoint, type CapturedPoint } from './workspace-helpers';

test.setTimeout(120_000);
const ready = [0, 20, -30, 0, 40, 0];
const runState = (page: Page) => page.locator('[data-run-phase]');
const stopButton = (page: Page) => page.getByRole('button', { name: '停止运行', exact: true });
type Recorded = CapturedPoint & { pose: Pose; joints: number[] };
type Sample = { phase: string; pointId: string; line: number; cycle: number; joints: number[]; pose: Pose };

async function recordJoint1(page: Page, degrees: number, speed = 10): Promise<Recorded> {
  await setJoint(page, 1, degrees);
  await page.getByRole('combobox', { name: '记录速度', exact: true }).selectOption(String(speed));
  await expectBodyJoints(page, [degrees, ...ready.slice(1)]);
  const point = await captureCurrent(page);
  return { ...point, pose: await bodyPose(page), joints: await bodyJoints(page) };
}
async function startSampling(page: Page) {
  await page.evaluate(() => {
    const scope = window as unknown as { emulationRunSamples: unknown[]; emulationRunSampling: boolean };
    scope.emulationRunSamples = []; scope.emulationRunSampling = true;
    const frame = () => {
      const state = document.querySelector<HTMLElement>('[data-run-phase]');
      const canvas = document.querySelector<HTMLElement>('canvas[data-robot-joints]');
      if (state && canvas) scope.emulationRunSamples.push({ phase: state.dataset.runPhase, pointId: state.dataset.runPointId,
        line: Number(state.dataset.runLine), cycle: Number(state.dataset.runCycle),
        joints: JSON.parse(canvas.dataset.robotJoints!), pose: JSON.parse(canvas.dataset.robotPose!) });
      if (scope.emulationRunSampling) requestAnimationFrame(frame);
    };
    requestAnimationFrame(frame);
  });
}
async function stopSampling(page: Page): Promise<Sample[]> {
  return page.evaluate(() => {
    const scope = window as unknown as { emulationRunSamples: Sample[]; emulationRunSampling: boolean };
    scope.emulationRunSampling = false;
    return scope.emulationRunSamples;
  });
}
async function run(page: Page, loop = false, pathId?: string) {
  await openScene(page);
  await selectPath(page, pathId ?? (await pathObjects(page).first().getAttribute('data-object-id'))!);
  await page.getByRole('checkbox', { name: '循环运行', exact: true }).setChecked(loop);
  await startSampling(page);
  await runPathFromMenu(page, pathId ?? (await pathObjects(page).first().getAttribute('data-object-id'))!, { keyboard: loop });
  await expect(page.locator('[data-active-view]')).toHaveAttribute('data-active-view', 'scene');
}
async function assertStopped(page: Page, from: number[], to: number[], keyboard = false, controlledClock = false) {
  if (keyboard) await stopButton(page).press('Enter');
  else await stopButton(page).click();
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'idle');
  // Read actual rendered joints on 15 scene frames. The observation never
  // changes App state; only the two short Stop tests control animation time.
  await bodyCanvas(page).evaluate(element => {
    const scope = window as unknown as { emulationStoppedFrames: number[][] };
    scope.emulationStoppedFrames = [];
    const sample = () => {
      scope.emulationStoppedFrames.push(JSON.parse(element.getAttribute('data-robot-joints')!));
      if (scope.emulationStoppedFrames.length < 15) requestAnimationFrame(sample);
    };
    requestAnimationFrame(sample);
  });
  if (controlledClock) await page.clock.runFor(320);
  await expect.poll(() => page.evaluate(() => (window as unknown as { emulationStoppedFrames: number[][] }).emulationStoppedFrames.length)).toBe(15);
  const values = await page.evaluate(() => (window as unknown as { emulationStoppedFrames: number[][] }).emulationStoppedFrames);
  const final = values.at(-1)!;
  expect(Math.hypot(...final.map((value, i) => value - from[i]))).toBeGreaterThan(.1);
  expect(Math.hypot(...final.map((value, i) => value - to[i]))).toBeGreaterThan(.1);
  for (const sample of values.slice(3)) expect(Math.max(...sample.map((value, i) => Math.abs(value - final[i])))).toBeLessThan(1e-7);
  return final;
}
function deviation(position: number[], from: number[], to: number[]) {
  const delta = to.map((value, i) => value - from[i]);
  const lengthSquared = delta.reduce((sum, value) => sum + value * value, 0);
  const t = position.reduce((sum, value, i) => sum + (value - from[i]) * delta[i], 0) / lengthSquared;
  return { t, error: Math.hypot(...position.map((value, i) => value - from[i] - t * delta[i])) };
}

test('the context menu runs its own Path despite another selection and plans every ABB line before switching to ordered Cartesian playback and locks edits without engine badges', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const first = await recordJoint1(page, 6), second = await recordJoint1(page, 12);
  // A different Path contains an incompatible wrist branch. Running the first
  // Path must neither plan nor play this unrelated point.
  const otherPath = await createPath(page);
  await setJoint(page, 5, -40);
  await captureCurrent(page, otherPath);
  await setJoint(page, 5, 40);
  await selectPathForPoints(page, [first, second]);
  await setJoint(page, 1, 0);
  await expectBodyJoints(page, ready);
  const start = await bodyPose(page);
  await expect(page.getByText('浏览器参考引擎', { exact: true })).toHaveCount(0);
  await expect(page.getByText('Rust 原生引擎', { exact: true })).toHaveCount(0);
  await selectPath(page, otherPath);
  await expect(page.locator(`[data-object-kind="path"][data-object-id="${otherPath}"]`)).toHaveClass(/selected/);
  await startSampling(page);
  await runPathFromMenu(page, first.pathId);
  await expect(runState(page)).toHaveAttribute('data-run-path-id', first.pathId);
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'running', { timeout: 20_000 });
  await expect(page.getByRole('button', { name: '选择机器人型号', exact: true })).toHaveCount(0);
  await expect(page.getByLabel('项目机器人型号', { exact: true })).toHaveAttribute('data-model-id', 'irb1100-4-0.58');
  await expect(page.getByRole('button', { name: '程序', exact: true })).toBeDisabled();
  await selectRobotObject(page); // Reading another inspector must not cancel playback.
  for (let joint = 1; joint <= 6; joint++) await expect(page.getByRole('spinbutton', { name: `机器人关节 J${joint}`, exact: true })).toBeDisabled();
  await expect(page.getByRole('button', { name: '拖动法兰', exact: true })).toBeDisabled();
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'complete', { timeout: 30_000 });
  await expectBodyJoints(page, second.joints, .001);
  const samples = (await stopSampling(page)).filter(sample => sample.phase === 'running');
  const order = samples.map(sample => sample.pointId).filter((id, i, ids) => i === 0 || id !== ids[i - 1]);
  expect(order).toEqual([first.id, second.id]);
  for (const [index, point] of [first, second].entries()) {
    const from = index === 0 ? start.position : first.pose.position;
    const interior = samples.filter(sample => sample.pointId === point.id).map(sample => deviation(sample.pose.position, from, point.pose.position)).filter(sample => sample.t > .15 && sample.t < .85);
    expect(interior.length).toBeGreaterThan(0);
    // Pure J1 interpolation would trace an arc (>0.5 mm at these angles).
    // Rendered FK must follow the actual checked Cartesian segment instead.
    expect(Math.max(...interior.map(sample => sample.error))).toBeLessThan(.2);
  }
  await selectPath(page, first.pathId);
  await expect(pathPoints(page, first.pathId)).toHaveCount(2);
  await expect(page.getByRole('button', { name: '添加当前点到 Path', exact: true })).toBeEnabled();
  await page.screenshot({ path: '/tmp/emulation-program-run-complete.png' });
  expect(errors).toEqual([]);
});

test('Loop closes from the last point to the first without replaying approach and Stop holds an intermediate pose', async ({ page }) => {
  await page.clock.install();
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const first = await recordJoint1(page, 2), second = await recordJoint1(page, 4);
  await setJoint(page, 1, 0);
  await expectBodyJoints(page, ready);
  await run(page, true);
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'running', { timeout: 20_000 });
  // Keep a real UI selection while the clock drives continuous scene frames.
  // Pointer actionability can stall waiting for stable animation frames here.
  const activePath = page.locator(`[data-object-kind="path"][data-object-id="${first.pathId}"]`);
  await expect(activePath).toBeVisible();
  await expect(activePath).toBeEnabled();
  await activePath.press('Enter');
  await expect(activePath).toHaveClass(/selected/);
  await expect(page.getByRole('button', { name: '添加当前点到 Path', exact: true })).toBeDisabled();
  await expect(page.getByRole('button', { name: '转换为程序', exact: true })).toBeDisabled();
  await expect(page.getByRole('button', { name: '删除 Path', exact: true })).toBeDisabled();
  await expect(page.getByRole('checkbox', { name: '循环运行', exact: true })).toBeDisabled();
  await activePath.press('Shift+F10');
  const menu = page.getByRole('menu', { name: 'Path 右键菜单', exact: true });
  await expect(menu).toBeVisible();
  for (const name of ['运行', '添加当前点到 Path', '转换为程序', '删除 Path']) await expect(menu.getByRole('menuitem', { name, exact: true })).toBeDisabled();
  await menu.press('Escape');
  await expect(menu).toHaveCount(0);
  await expect(activePath).toBeFocused();
  await pathPoints(page, first.pathId).first().focus();
  await page.keyboard.press('Delete');
  await expect(pathPoints(page, first.pathId)).toHaveCount(2);
  await expect.poll(async () => Number(await runState(page).getAttribute('data-run-cycle')), { timeout: 25_000 }).toBeGreaterThanOrEqual(2);
  // The first two cycles above use wall time. Freeze only the Stop observation
  // window: a real mouse click's actionability wait can consume this short leg.
  await stopButton(page).focus();
  await page.clock.pauseAt(new Date(await page.evaluate(() => Date.now()) + 10_000));
  await page.clock.runFor(32); // Let React and the scene both observe the clock transition.
  for (let index = 0; index < 40; index++) {
    const q = (await bodyJoints(page))[0];
    if (q > 2.5 && q < 3.5) break;
    await page.clock.runFor(100);
  }
  expect((await bodyJoints(page))[0]).toBeGreaterThan(2.5);
  expect((await bodyJoints(page))[0]).toBeLessThan(3.5);
  await assertStopped(page, first.joints, second.joints, true, true);
  await page.clock.resume();
  const repeated = (await stopSampling(page)).filter(sample => sample.phase === 'running' && sample.cycle >= 1);
  expect(repeated.length).toBeGreaterThan(0);
  expect(new Set(repeated.map(sample => sample.pointId))).toEqual(new Set([first.id, second.id]));
  for (const sample of repeated) {
    expect(sample.joints[0]).toBeGreaterThanOrEqual(2 - .001);
    expect(sample.joints[0]).toBeLessThanOrEqual(4 + .001);
  }
  await page.locator(`[data-object-kind="path"][data-object-id="${first.pathId}"]`).press('Enter');
  await expect(page.getByRole('checkbox', { name: '循环运行', exact: true })).toBeChecked();
  await expect(page.getByRole('button', { name: '运行 Path', exact: true })).toBeEnabled();
  expect(errors).toEqual([]);
});

test('a later singular MoveL rejects the whole run before any successful earlier leg can move the body', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  await recordJoint1(page, 2, 50);
  await setJoint(page, 5, -40);
  await captureCurrent(page);
  await selectRobotObject(page);
  await page.getByRole('button', { name: '示例起始姿态', exact: true }).click();
  await expectBodyJoints(page, ready);
  const start = await bodyPose(page);
  await run(page);
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'error', { timeout: 20_000 });
  await expect(page.getByRole('status', { name: '程序运行状态', exact: true })).toContainText(/第 2 条|第2条/);
  await expectBodyJoints(page, ready);
  expect(await bodyPose(page)).toEqual(start);
  const samples = await stopSampling(page);
  expect(samples.some(sample => sample.phase === 'running')).toBe(false);
  for (const sample of samples) expect(Math.max(...sample.joints.map((value, i) => Math.abs(value - ready[i])))).toBeLessThan(1e-7);
  await expectNoPointMotionActions(page);
  await page.screenshot({ path: '/tmp/emulation-program-rejected.png' });
  expect(errors).toEqual([]);
});

test('a Path point context menu positions the recorded joints with stoppable view-only motion', async ({ page }) => {
  await page.clock.install();
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const point = await recordJoint1(page, 160, 50);
  const from = [-160, ...ready.slice(1)];
  await selectPathForPoints(page, [point]);
  await setJoint(page, 1, -160);
  await openScene(page);
  await expectBodyJoints(page, from);
  const target = page.locator(`[data-object-kind="target"][data-object-id="${point.id}"]`);
  await target.click();
  await expect(bodyCanvas(page)).toHaveAttribute('data-selected-point-id', point.id);
  await bodyCanvas(page).screenshot(); // Finish drawing the selected marker before the short animation.
  await target.click({ button: 'right' });
  // Keep the real menu and keyboard events, but control this ~1 second view-only
  // animation's time so CDP latency cannot remove Stop before the input arrives.
  await page.clock.pauseAt(new Date(await page.evaluate(() => Date.now()) + 10_000));
  await page.getByRole('menuitem', { name: '移动至此', exact: true }).press('Enter');
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'positioning');
  await page.clock.runFor(300);
  expect((await bodyJoints(page))[0]).toBeGreaterThan(-159.5);
  expect((await bodyJoints(page))[0]).toBeLessThan(159.5);
  await assertStopped(page, from, point.joints, true, true);
  await target.click({ button: 'right' });
  await page.getByRole('menuitem', { name: '移动至此', exact: true }).click();
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'positioning');
  // Keep the same controlled clock through completion: resumed virtual RAF
  // delivery can lag a wall-clock assertion on software-rendered CI workers.
  await page.clock.runFor(1500);
  await expectBodyJoints(page, point.joints, .001);
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'complete');
  await expectNoPointMotionActions(page);
  await page.screenshot({ path: '/tmp/emulation-point-positioning.png' });
  expect(errors).toEqual([]);
});

test('a real GP7 records INFORM Cartesian targets and runs its own URDF chain without ABB configuration or controller export', async ({ page }) => {
  const errors = observeBrowser(page);
  const catalog = await readCatalog();
  const model = catalog.models.find(model => model.id === 'yaskawa-gp7')!;
  const fixtures = await prepareCatalogFixtures({ ...catalog, models: [model] });
  await page.route(url => fixtures.has(url.href), async route => {
    await route.fulfill({ status: 200, contentType: 'application/octet-stream', headers: { 'access-control-allow-origin': '*' }, body: fixtures.get(route.request().url())! });
  });
  await openWorkspace(page);
  await selectRobot(page, model.id);
  await expect(bodyCanvas(page)).toHaveAttribute('data-urdf-model', model.id);
  const seed = [20, 15, -25, 25, 35, 10];
  const delta = [1, -.5, .75, .5, -.75, 1];
  async function setPose(joints: number[]) {
    await selectRobotObject(page);
    for (let i = 0; i < 6; i++) await page.getByRole('spinbutton', { name: `机器人关节 J${i + 1}`, exact: true }).fill(String(joints[i]));
    await page.getByRole('combobox', { name: '记录速度', exact: true }).selectOption('10');
    await expectBodyJoints(page, joints);
  }
  await setPose(seed.map((value, i) => value + delta[i]));
  const first = await captureCurrent(page), firstPose = await bodyPose(page);
  const lastJoints = seed.map((value, i) => value + 2 * delta[i]);
  await setPose(lastJoints);
  const second = await captureCurrent(page), lastPose = await bodyPose(page);
  const expected = [547.3587776286031, 242.02369993286896, 516.5348915917687];
  for (let i = 0; i < 3; i++) expect(lastPose.position[i]).toBeCloseTo(expected[i], 5);
  await expect(pathPoints(page, first.pathId)).toHaveCount(2);
  expect(first.name).toBe('P001');
  expect(second.name).toBe('P002');
  await expect(page.getByLabel('目标 robconf', { exact: true })).toHaveCount(0);
  await expect(page.getByRole('textbox', { name: '目标 Yaskawa 位置数据', exact: true })).toHaveValue(/C00001=/);
  const code = await generateSource(page);
  expect(code).toContain('MOVL C00000 V=10');
  expect(code).toContain('MOVL C00001 V=10');
  expect(code).toContain('///RCONF {{RCONF_P001}}');
  expect(code).toContain('///RCONF {{RCONF_P002}}');
  expect(code).toContain('///TOOL {{TOOL}}');
  expect(code).not.toMatch(/robconf|robtarget|MoveL/);
  await selectPathForPoints(page, [first, second]);
  await setPose(seed);
  const start = await bodyPose(page);
  await run(page);
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'complete', { timeout: 30_000 });
  await expectBodyJoints(page, lastJoints, .001);
  await expect.poll(async () => Math.hypot(...(await bodyPose(page)).position.map((value, i) => value - lastPose.position[i]))).toBeLessThan(.01);
  const samples = (await stopSampling(page)).filter(sample => sample.phase === 'running');
  expect(samples.some(sample => Math.hypot(...sample.pose.position.map((value, i) => value - start.position[i])) > .5)).toBe(true);
  for (const [id, from, to] of [[first.id, start.position, firstPose.position], [second.id, firstPose.position, lastPose.position]] as [string, number[], number[]][]) {
    const interior = samples.filter(sample => sample.pointId === id).map(sample => deviation(sample.pose.position, from, to)).filter(sample => sample.t > .1 && sample.t < .9);
    expect(interior.length).toBeGreaterThan(0);
    expect(Math.max(...interior.map(sample => sample.error))).toBeLessThan(.2);
  }
  await expect(page.getByRole('button', { name: '导出此 MoveL', exact: true })).toHaveCount(0);
  await page.screenshot({ path: '/tmp/emulation-gp7-program-complete.png' });
  await openScene(page);
  await expect(page.locator('[data-object-kind="target"][data-executing="true"]')).toHaveCount(0);
  await openCode(page);
  await expect(page.getByRole('button', { name: '运行源码', exact: true })).toHaveCount(0);
  await page.screenshot({ path: '/tmp/emulation-gp7-inform-template.png' });
  // Finished and explicitly stopped runs both clear the live Path-point marker.
  await selectPath(page, first.pathId);
  await page.getByRole('checkbox', { name: '循环运行', exact: true }).check();
  await page.getByRole('button', { name: '运行 Path', exact: true }).click();
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'running', { timeout: 20_000 });
  await stopButton(page).press('Enter');
  await expect(runState(page)).toHaveAttribute('data-run-phase', 'idle');
  await openScene(page);
  await expect(page.locator('[data-object-kind="target"][data-executing="true"]')).toHaveCount(0);
  expect(errors).toEqual([]);
});
