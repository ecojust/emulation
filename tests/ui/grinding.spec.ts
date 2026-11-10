import { openDefaultProject, waitForProjectScene } from './project-helpers';
import { expect, test, type Page } from '@playwright/test';
import type { JointAngles, Pose } from '../../src/robot/types';
import { forwardKinematics } from '../../src/robot/kinematics';
import { getRobotModel } from '../../src/robot/model';
import { prepareCatalogFixtures, readCatalog } from './catalog-fixtures';
import { bodyCanvas, bodyJoints, bodyPose, captureCurrent, createPath, expectBodyJoints, observeBrowser, openPathMenu, openWorkspace, pathPoints, runPathFromMenu, selectRobot, selectRobotObject } from './workspace-helpers';

test.setTimeout(120_000);
const planner = (page: Page) => page.getByRole('dialog', { name: '打磨路径规划', exact: true });
const result = (page: Page) => planner(page).getByRole('status', { name: '路径规划结果', exact: true });
async function disableZones(page: Page) {
  for (const zone of await page.locator('[data-object-kind="zone"]').all()) {
    await zone.click();
    await page.getByRole('checkbox', { name: '显示干涉区域', exact: true }).uncheck();
  }
}
async function openPlanner(page: Page, pathId: string, initial: Pose, next: Pose = { position: [initial.position[0], initial.position[1] + 20, initial.position[2]], orientation: initial.orientation.map(value => -value) as Pose['orientation'] }) {
  const menu = await openPathMenu(page, pathId);
  await menu.getByRole('menuitem', { name: '路径规划', exact: true }).click();
  await expect(planner(page)).toBeVisible();
  const input = [initial, next].map(pose => [...pose.position, ...pose.orientation].join(' ')).join('\n');
  await planner(page).getByRole('textbox', { name: '目标点列表', exact: true }).fill(input);
  await expect(planner(page).getByRole('button', { name: '使用首点方向', exact: true })).toHaveCount(0);
  await expect(planner(page).getByRole('spinbutton', { name: /^统一工具方向 / })).toHaveCount(0);
  await planner(page).getByRole('spinbutton', { name: '最小安全间距', exact: true }).fill('2');
  return { input, next };
}
async function renderedPoints(page: Page, pathId: string): Promise<Pose[]> {
  await expect.poll(async () => {
    const paths = JSON.parse((await bodyCanvas(page).getAttribute('data-scene-paths')) || '[]') as { id: string; points: Pose[] }[];
    return paths.some(path => path.id === pathId && path.points.length > 0);
  }).toBe(true);
  const paths = JSON.parse((await bodyCanvas(page).getAttribute('data-scene-paths'))!) as { id: string; points: Pose[] }[];
  return paths.find(path => path.id === pathId)!.points;
}
function sameDirection(actual: number[], expected: number[]) {
  return Math.abs(1 - Math.abs(actual.reduce((sum, value, i) => sum + value * expected[i], 0)));
}

test('grinding preserves distinct target orientations through preview, apply and execution without changing the recorded Path before apply', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  await disableZones(page);
  const point = await captureCurrent(page), before = await bodyPose(page);
  const originalJoints = await bodyJoints(page);
  const nextJoints = [...originalJoints] as JointAngles;
  nextJoints[0] += 1; nextJoints[5] += 3;
  const nextTarget = forwardKinematics(getRobotModel('irb1100-4-0.58'), nextJoints).pose;
  expect(sameDirection(nextTarget.orientation, before.orientation)).toBeGreaterThan(1e-6);
  const original = await renderedPoints(page, point.pathId);
  const { input, next } = await openPlanner(page, point.pathId, before, nextTarget);
  await planner(page).getByRole('button', { name: '生成路径', exact: true }).click();
  await expect(result(page)).toHaveAttribute('data-grinding-status', 'success', { timeout: 40_000 });
  await expect(planner(page).getByRole('button', { name: '应用到路径', exact: true })).toBeEnabled();
  expect(await renderedPoints(page, point.pathId)).toEqual(original);
  await expect(pathPoints(page, point.pathId)).toHaveCount(1);
  expect(await bodyPose(page)).toEqual(before);
  const cameraBefore = JSON.parse((await bodyCanvas(page).getAttribute('data-camera-position'))!) as number[];
  await planner(page).getByRole('button', { name: '预览路径', exact: true }).click();
  await expect(planner(page)).toHaveAttribute('data-planning-mode', 'preview');
  await expect(planner(page)).toHaveAttribute('aria-modal', 'false');
  // A real drag through the exposed canvas must reach OrbitControls.
  const viewport = (await bodyCanvas(page).boundingBox())!;
  const from = { x: viewport.x + viewport.width * .4, y: viewport.y + viewport.height * .35 };
  await page.mouse.move(from.x, from.y);
  await page.mouse.down();
  await page.mouse.move(from.x + 70, from.y + 25, { steps: 6 });
  await page.mouse.up();
  await expect.poll(async () => {
    const camera = JSON.parse((await bodyCanvas(page).getAttribute('data-camera-position'))!) as number[];
    return Math.hypot(...camera.map((value, i) => value - cameraBefore[i]));
  }).toBeGreaterThan(.01);
  expect(await bodyJoints(page)).toEqual(originalJoints);
  expect(await bodyPose(page)).toEqual(before);
  expect(await renderedPoints(page, point.pathId)).toEqual(original);
  await planner(page).getByRole('button', { name: '返回编辑', exact: true }).click();
  await expect(planner(page)).toHaveAttribute('data-planning-mode', 'edit');
  await expect(planner(page)).toHaveAttribute('aria-modal', 'true');
  await expect(result(page)).toHaveAttribute('data-grinding-status', 'success');
  await expect(planner(page).getByRole('textbox', { name: '目标点列表', exact: true })).toHaveValue(input);
  await expect(planner(page).getByRole('button', { name: '应用到路径', exact: true })).toBeEnabled();
  // Editing an input must invalidate a completed result, even before another job starts.
  await planner(page).getByRole('spinbutton', { name: '最小安全间距', exact: true }).fill('3');
  await expect(planner(page).getByRole('button', { name: '应用到路径', exact: true })).toBeDisabled();
  await expect(planner(page).getByRole('button', { name: '预览路径', exact: true })).toBeDisabled();
  expect(await renderedPoints(page, point.pathId)).toEqual(original);
  await planner(page).getByRole('button', { name: '生成路径', exact: true }).click();
  await expect(result(page)).toHaveAttribute('data-grinding-status', 'success', { timeout: 40_000 });
  await planner(page).getByRole('button', { name: '应用到路径', exact: true }).click();
  await expect(planner(page)).toHaveCount(0);
  await expect.poll(async () => (await renderedPoints(page, point.pathId)).length).toBeGreaterThanOrEqual(2);
  const points = await renderedPoints(page, point.pathId);
  let previousIndex = -1;
  for (const target of [before, next]) {
    const index = points.findIndex((pose, i) => i > previousIndex && Math.hypot(...pose.position.map((value, axis) => value - target.position[axis])) < .01 && sameDirection(pose.orientation, target.orientation) < 1e-7);
    expect(index).toBeGreaterThan(previousIndex); previousIndex = index;
  }
  expect(await bodyPose(page)).toEqual(before);
  await runPathFromMenu(page, point.pathId);
  await expect(page.locator('[data-run-phase]')).toHaveAttribute('data-run-phase', 'complete', { timeout: 30_000 });
  await expect.poll(async () => Math.hypot(...(await bodyPose(page)).position.map((value, i) => value - next.position[i]))).toBeLessThan(.01);
  expect(sameDirection((await bodyPose(page)).orientation, next.orientation)).toBeLessThan(1e-7);
  await page.screenshot({ path: '/tmp/emulation-grinding-path.png' });
  expect(input.split('\n')).toHaveLength(2);
  expect(errors).toEqual([]);
});

test('a forbidden volume covering the robot rejects grinding and closing the planner preserves all original Path points', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const point = await captureCurrent(page), before = await bodyPose(page);
  const original = await renderedPoints(page, point.pathId);
  await page.locator('[data-object-kind="zone"]').first().click();
  for (const axis of ['X', 'Y', 'Z']) {
    await page.getByRole('spinbutton', { name: `危险区中心 ${axis}`, exact: true }).fill('0');
    await page.getByRole('spinbutton', { name: `危险区尺寸 ${axis}`, exact: true }).fill('5000');
  }
  await page.getByRole('checkbox', { name: '显示干涉区域', exact: true }).check();
  await openPlanner(page, point.pathId, before);
  await planner(page).getByRole('button', { name: '生成路径', exact: true }).click();
  await expect(result(page)).toHaveAttribute('data-grinding-status', 'failure', { timeout: 40_000 });
  await expect(planner(page).getByRole('button', { name: '应用到路径', exact: true })).toBeDisabled();
  await planner(page).getByRole('button', { name: '关闭路径规划', exact: true }).click();
  await expect(planner(page)).toHaveCount(0);
  await expect(pathPoints(page, point.pathId)).toHaveCount(1);
  expect(await renderedPoints(page, point.pathId)).toEqual(original);
  expect(await bodyPose(page)).toEqual(before);
  expect(errors).toEqual([]);
});


test('real GP7 mesh bounds and its own URDF chain generate and run a uniform-direction grinding Path', async ({ page }) => {
  const errors = observeBrowser(page);
  const catalog = await readCatalog(), model = catalog.models.find(entry => entry.id === 'yaskawa-gp7')!;
  const fixtures = await prepareCatalogFixtures({ ...catalog, models: [model] });
  await page.route(url => fixtures.has(url.href), async route => {
    await route.fulfill({ status: 200, contentType: 'application/octet-stream', headers: { 'access-control-allow-origin': '*' }, body: fixtures.get(route.request().url())! });
  });
  await openWorkspace(page);
  await selectRobot(page, model.id);
  await expect(bodyCanvas(page)).toHaveAttribute('data-urdf-model', model.id);
  await disableZones(page);
  await selectRobotObject(page);
  const seed = [20, 15, -25, 25, 35, 10];
  for (let i = 0; i < 6; i++) await page.getByRole('spinbutton', { name: `机器人关节 J${i + 1}`, exact: true }).fill(String(seed[i]));
  await expectBodyJoints(page, seed);
  const before = await bodyPose(page), point = await captureCurrent(page);
  const original = await renderedPoints(page, point.pathId);
  await openPlanner(page, point.pathId, before);
  const targets: Pose[] = [5, 15].map(delta => ({ position: [before.position[0], before.position[1] + delta, before.position[2]], orientation: before.orientation }));
  await planner(page).getByRole('textbox', { name: '目标点列表', exact: true }).fill(targets.map(pose => [...pose.position, ...pose.orientation].join(' ')).join('\n'));
  await planner(page).getByRole('button', { name: '生成路径', exact: true }).click();
  await expect(result(page)).toHaveAttribute('data-grinding-status', 'success', { timeout: 40_000 });
  // Read the actual Three.js line, not the input or the planner's response.
  await expect.poll(async () => {
    const preview = JSON.parse((await bodyCanvas(page).getAttribute('data-preview-path')) || 'null') as { visible: boolean; count: number; startMm: number[]; endMm: number[] } | null;
    if (!preview?.visible || preview.count < 2 || preview.startMm?.length !== 3 || preview.endMm?.length !== 3) return Infinity;
    return Math.max(Math.hypot(...preview.startMm.map((value, i) => value - before.position[i])),
      Math.hypot(...preview.endMm.map((value, i) => value - targets[1].position[i])));
  }).toBeLessThan(.001);
  expect(await renderedPoints(page, point.pathId)).toEqual(original);
  await expect(pathPoints(page, point.pathId)).toHaveCount(1);
  await expectBodyJoints(page, seed);
  await planner(page).getByRole('button', { name: '应用到路径', exact: true }).click();
  await expect(planner(page)).toHaveCount(0);
  await expect.poll(async () => (await renderedPoints(page, point.pathId)).length).toBeGreaterThanOrEqual(2);
  const generated = await renderedPoints(page, point.pathId);
  for (const target of targets) expect(generated.some(pose => Math.hypot(...pose.position.map((value, i) => value - target.position[i])) < .01 && sameDirection(pose.orientation, target.orientation) < 1e-7)).toBe(true);
  await runPathFromMenu(page, point.pathId);
  await expect(page.locator('[data-run-phase]')).toHaveAttribute('data-run-phase', 'complete', { timeout: 30_000 });
  await expect.poll(async () => Math.hypot(...(await bodyPose(page)).position.map((value, i) => value - targets[1].position[i]))).toBeLessThan(.01);
  expect(sameDirection((await bodyPose(page)).orientation, targets[1].orientation)).toBeLessThan(1e-7);
  expect(Math.hypot(...(await bodyJoints(page)).map((value, i) => value - seed[i]))).toBeGreaterThan(.1);
  expect(errors).toEqual([]);
});


test('an empty Path retains every pasted pose and ABB configuration, and failed replanning preserves the saved targets', async ({ page }) => {
  const errors = observeBrowser(page);
  // This case also runs against the production bundle, where DEV canvas
  // diagnostics are absent. Verify the default body through visible controls.
  await openDefaultProject(page);
  await waitForProjectScene(page, 'irb1100-4-0.58');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  const model = getRobotModel('irb1100-4-0.58');
  const before = forwardKinematics(model, model.readyJoints).pose;
  async function expectUnchangedBody() {
    await selectRobotObject(page);
    for (const [i, joint] of model.readyJoints.entries()) expect(Number(await page.getByRole('spinbutton', { name: `机器人关节 J${i + 1}`, exact: true }).inputValue())).toBe(joint);
    const displayed = await page.getByLabel('当前 TCP', { exact: true }).locator('[data-axis] strong').allTextContents();
    expect(displayed).toHaveLength(3);
    for (const [i, value] of displayed.entries()) expect(Math.abs(Number(value) - before.position[i])).toBeLessThan(.006);
    const orientation = (await page.getByLabel('当前四元数', { exact: true }).textContent())!.split('·').map(Number);
    expect(orientation).toHaveLength(4);
    for (const [i, value] of orientation.entries()) expect(Math.abs(value - before.orientation[i])).toBeLessThan(1e-6);
  }
  await expectUnchangedBody();
  await disableZones(page);
  const pathId = await createPath(page);
  async function displayedTargets() {
    const ids = await pathPoints(page, pathId).evaluateAll(nodes => nodes.map(node => node.getAttribute('data-object-id')!));
    const saved: { id: string; pose: Pose; configuration: number[] }[] = [];
    for (const id of ids) {
      await page.locator(`[data-object-kind="target"][data-object-id="${id}"]`).click();
      const inspector = page.locator(`[data-target-point-id="${id}"]`);
      await expect(inspector).toBeVisible();
      saved.push({ id, pose: JSON.parse((await inspector.getAttribute('data-world-pose'))!) as Pose,
        configuration: JSON.parse((await inspector.getByLabel('目标 robconf', { exact: true }).textContent())!) as number[] });
    }
    return saved;
  }
  expect(Math.hypot(...before.orientation.map((value, index) => value - (index === 0 ? 1 : 0)))).toBeGreaterThan(.1);
  const targets = [1, 2].map(delta => {
    const joints = [...model.readyJoints] as JointAngles;
    joints[0] += delta; joints[5] += delta * 2;
    return forwardKinematics(model, joints).pose;
  });
  expect(sameDirection(targets[0].orientation, targets[1].orientation)).toBeGreaterThan(1e-6);
  const text = (poses: Pose[]) => poses.map(pose => [...pose.position, ...pose.orientation].join(' ')).join('\n');
  const menu = await openPathMenu(page, pathId);
  await menu.getByRole('menuitem', { name: '路径规划', exact: true }).click();
  await expect(planner(page)).toBeVisible();
  await planner(page).getByRole('textbox', { name: '目标点列表', exact: true }).fill(text(targets));
  await expect(planner(page).getByRole('spinbutton', { name: /^统一工具方向 / })).toHaveCount(0);
  await planner(page).getByRole('button', { name: '生成路径', exact: true }).click();
  await expect(result(page)).toHaveAttribute('data-grinding-status', 'success', { timeout: 40_000 });
  await expect(pathPoints(page, pathId)).toHaveCount(0);
  await planner(page).getByRole('button', { name: '应用到路径', exact: true }).click();
  await expect(planner(page)).toHaveCount(0);
  await expect.poll(() => pathPoints(page, pathId).count()).toBeGreaterThanOrEqual(2);
  const generated = await displayedTargets();
  let previousIndex = -1;
  for (const target of targets) {
    const index = generated.findIndex(({ pose }, i) => i > previousIndex && Math.hypot(...pose.position.map((value, axis) => value - target.position[axis])) < .01 && sameDirection(pose.orientation, target.orientation) < 1e-7);
    expect(index).toBeGreaterThan(previousIndex); previousIndex = index;
  }
  for (const { configuration } of generated) {
    expect(configuration).toHaveLength(4);
    expect(configuration.every(Number.isInteger)).toBe(true);
    expect(configuration[3]).toBe(generated[0].configuration[3]);
  }
  await expectUnchangedBody();

  const nextMenu = await openPathMenu(page, pathId);
  await nextMenu.getByRole('menuitem', { name: '路径规划', exact: true }).click();
  await expect(planner(page)).toBeVisible();
  const unreachable: Pose = { position: [100000, 100000, 100000], orientation: targets[1].orientation };
  const rejectedInput = text([targets[0], unreachable]);
  await planner(page).getByRole('textbox', { name: '目标点列表', exact: true }).fill(rejectedInput);
  await planner(page).getByRole('button', { name: '生成路径', exact: true }).click();
  await expect(result(page)).toHaveAttribute('data-grinding-status', 'failure', { timeout: 40_000 });
  await expect(planner(page).getByRole('textbox', { name: '目标点列表', exact: true })).toHaveValue(rejectedInput);
  await expect(planner(page).getByRole('button', { name: '应用到路径', exact: true })).toBeDisabled();
  await expect(planner(page).getByRole('button', { name: '预览路径', exact: true })).toBeDisabled();
  await planner(page).getByRole('button', { name: '关闭路径规划', exact: true }).click();
  await expect(planner(page)).toHaveCount(0);
  expect(await displayedTargets()).toEqual(generated);
  await expectUnchangedBody();
  expect(errors).toEqual([]);
});
