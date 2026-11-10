import { expect, test } from '@playwright/test';
import { forwardKinematics } from '../../src/robot/kinematics';
import { getRobotModel } from '../../src/robot/model';
import { bodyJoints, bodyPose, captureCurrent, checkedSegment, expectBodyJoints, expectCopyState, makePath, observeBrowser, openProgram, openScene, openWorkspace, programRows, readRapid, scene, selectPoint, selectRobot, selectRobotObject, setJoint } from './workspace-helpers';

test.setTimeout(90_000);
const ready = [0, 20, -30, 0, 40, 0];

test('robot selection exposes actual state and DH while mechanical zero cannot be recorded as a valid MoveL', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const objects = page.getByRole('complementary', { name: '场景对象', exact: true });
  await expect(objects.getByLabel('当前 TCP', { exact: true })).toHaveCount(0);
  await expect(page.getByRole('spinbutton', { name: /^起始关节|^目标 [XYZ]/ })).toHaveCount(0);
  await expectBodyJoints(page, ready);
  const dh = page.locator('details').filter({ hasText: 'DH 参数与关节范围' });
  await dh.locator('summary').click();
  await expect(dh.getByRole('row')).toHaveCount(7);
  await page.getByRole('button', { name: '机械零位', exact: true }).click();
  await expectBodyJoints(page, [0, 0, 0, 0, 0, 0]);
  for (const [axis, coordinate] of [['X', '364.00'], ['Y', '0.00'], ['Z', '617.00']]) await expect(page.getByLabel('当前 TCP', { exact: true }).locator(`[data-axis="${axis}"] strong`)).toHaveText(coordinate);
  await expect(page.getByRole('status', { name: '记录姿态提示', exact: true })).toContainText(/奇异|有效.*配置/);
  await openProgram(page);
  await page.getByRole('listbox', { name: 'MoveL 程序指令', exact: true }).click({ button: 'right' });
  await expect(page.getByRole('menuitem', { name: '插入当前姿态（MoveL）', exact: true })).toBeDisabled();
  await page.keyboard.press('Escape');
  await expect(programRows(page)).toHaveCount(0);
  await expectCopyState(page, false);
  await page.screenshot({ path: '/tmp/emulation-object-zero.png' });
  expect(errors).toEqual([]);
});

test('real context capture and reversed multiselection create ordered Path objects without moving the selected point into the robot', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const first = await captureCurrent(page);
  const firstPose = await bodyPose(page);
  await setJoint(page, 1, 3);
  const second = await captureCurrent(page);
  const secondPose = await bodyPose(page);
  const pathId = await makePath(page, [first, second]);
  const pathTree = page.locator(`details.object-path`).filter({ has: page.locator(`[data-object-id="${pathId}"]`) });
  await expect(pathTree.locator('[data-object-kind="target"]')).toHaveText([first.name, second.name]);
  for (const point of [first, second]) await expect(page.getByRole('button', { name: `选择目标点 ${point.name}`, exact: true })).toBeInViewport();
  const sectionLabel = page.locator('.object-section-label').filter({ hasText: '危险区域' }).locator('span');
  expect((await sectionLabel.boundingBox())!.height).toBeLessThan(25);
  await expect(page.getByRole('heading', { name: 'Path 1', exact: true })).toBeVisible();
  await selectPoint(page, first);
  await expectBodyJoints(page, [3, ...ready.slice(1)]);
  await expect(page.getByRole('textbox', { name: '目标 robtarget', exact: true })).toHaveAttribute('readonly', '');
  await expect(scene(page)).toHaveAttribute('data-selected-point-id', first.id);
  await expect.poll(async () => {
    const paths = JSON.parse((await scene(page).getAttribute('data-scene-paths'))!);
    return paths.map((path: { id: string; points: { id: string }[] }) => ({ id: path.id, ids: path.points.map(point => point.id) }));
  }).toEqual([{ id: pathId, ids: [first.id, second.id] }]);
  const rendered = JSON.parse((await scene(page).getAttribute('data-scene-paths'))!)[0];
  expect(rendered.visible).toBe(true);
  for (let i = 0; i < 2; i++) {
    for (let axis = 0; axis < 3; axis++) expect(rendered.points[i].position[axis]).toBeCloseTo([firstPose, secondPose][i].position[axis], 7);
  }
  expect(Math.hypot(...secondPose.position.map((value, i) => value - firstPose.position[i]))).toBeGreaterThan(20);
  await expectCopyState(page, false);
  await expect(page.getByRole('button', { name: '播放路径', exact: true })).toBeDisabled();
  await expect.poll(async () => page.locator('[data-endpoint-joints]').evaluateAll(rows => rows.length > 0 && rows.every(row => ['hit', 'clear', 'uncertain'].includes(row.getAttribute('data-collision-state')!)))).toBe(true);
  await page.screenshot({ path: '/tmp/emulation-object-path.png' });
  await page.screenshot({ path: 'docs/images/workspace.png' });
  await openProgram(page);
  const template = page.locator('details.program-template');
  if ((await template.getAttribute('open')) === null) await template.locator('summary').click();
  await expect(page.getByRole('textbox', { name: 'ABB RAPID 程序模板', exact: true })).toHaveValue(/CONST robtarget P001/);
  await page.screenshot({ path: 'docs/images/program-workspace.png' });
  expect(errors).toEqual([]);
});

test('a recorded point requires a real MoveL check before copy or playback and preserves its full pose', async ({ page, context }) => {
  const errors = observeBrowser(page);
  await context.grantPermissions(['clipboard-read', 'clipboard-write']);
  await openWorkspace(page);
  await checkedSegment(page);
  const rapid = await readRapid(page);
  await page.getByRole('button', { name: '复制此 MoveL', exact: true }).click();
  expect(await page.evaluate(() => navigator.clipboard.readText())).toBe(rapid);
  expect(rapid).toContain('ConfL \\On;');
  expect(rapid).toContain('tool0\\WObj:=wobj0');
  expect(rapid.match(/9E9/gi)).toHaveLength(6);
  await expectBodyJoints(page, [3, ...ready.slice(1)]);
  await page.getByRole('button', { name: '播放此 MoveL', exact: true }).click();
  await expect(page.getByRole('slider', { name: '路径进度', exact: true })).toHaveValue('100', { timeout: 15_000 });
  const target = forwardKinematics(getRobotModel('irb1100-4-0.58'), ready as [number, number, number, number, number, number]).pose;
  await expect.poll(async () => Math.hypot(...(await bodyPose(page)).position.map((value, i) => value - target.position[i]))).toBeLessThan(.01);
  await page.screenshot({ path: '/tmp/emulation-object-checked-segment.png' });
  expect(errors).toEqual([]);
});

test('joint edits keep the last valid body, invalidate checked motion, and explicitly update an existing stored point', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const point = await checkedSegment(page);
  await setJoint(page, 1, 1000);
  await expect(page.getByRole('alert')).toContainText('超出关节范围');
  await expectBodyJoints(page, [3, ...ready.slice(1)]);
  await expectCopyState(page, false);
  await openProgram(page);
  await expect(page.getByRole('button', { name: '＋ 插入当前姿态', exact: true })).toBeDisabled();
  await setJoint(page, 1, '');
  await expectBodyJoints(page, [3, ...ready.slice(1)]);
  await setJoint(page, 1, 6);
  await selectPoint(page, point);
  const original = await page.getByRole('textbox', { name: '目标 robtarget', exact: true }).inputValue();
  await page.getByRole('button', { name: '用当前姿态更新点', exact: true }).click();
  await expect(page.locator('[data-target-point-id]')).toHaveAttribute('data-target-point-id', point.id);
  await expect(page.getByRole('textbox', { name: '目标 robtarget', exact: true })).not.toHaveValue(original);
  await expectBodyJoints(page, [6, ...ready.slice(1)]);
  await expectCopyState(page, false);
  await expect(page.getByRole('button', { name: '播放路径', exact: true })).toBeDisabled();
  await openProgram(page);
  await expect(programRows(page)).toHaveCount(1);
  await expect(programRows(page).first()).toHaveAttribute('data-program-point-id', point.id);
  expect(errors).toEqual([]);
});

test('noncontinuous row deletion preserves the remaining program order and removes empty Path references', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const first = await captureCurrent(page);
  await setJoint(page, 1, 3);
  const middle = await captureCurrent(page);
  await setJoint(page, 1, 6);
  const last = await captureCurrent(page);
  const pathId = await makePath(page, [first, last]);
  await openProgram(page);
  await page.getByRole('option', { name: `MoveL ${first.name}`, exact: true }).click();
  await page.getByRole('option', { name: `MoveL ${last.name}`, exact: true }).click({ modifiers: ['ControlOrMeta'] });
  await page.keyboard.press('Delete');
  await expect(programRows(page)).toHaveCount(1);
  await expect(programRows(page).first()).toHaveAttribute('data-program-point-id', middle.id);
  await expect(programRows(page).first()).toHaveAttribute('data-program-path-id', '');
  await expect(page.locator(`[data-object-id="${pathId}"]`)).toHaveCount(0);
  await expect(page.getByRole('heading', { name: '机器人属性', exact: true })).toBeVisible();
  const fourth = await captureCurrent(page);
  expect(fourth.name).toBe('P004');
  // Keep P004 inspected while removing a different selected row. A cancellation
  // must schedule a fresh inspection even though the inspected point is unchanged.
  await page.getByRole('option', { name: `MoveL ${middle.name}`, exact: true }).click();
  const kept = page.getByRole('option', { name: `MoveL ${fourth.name}`, exact: true });
  await kept.click({ modifiers: ['ControlOrMeta'] });
  await kept.click({ modifiers: ['ControlOrMeta'] });
  await page.getByRole('button', { name: '重新检查端点解', exact: true }).click();
  await kept.focus();
  await page.keyboard.press('Delete');
  await expect(programRows(page)).toHaveCount(1);
  await expect(page.locator('[data-target-point-id]')).toHaveAttribute('data-target-point-id', fourth.id);
  await expect.poll(() => page.locator('[data-endpoint-joints]').count()).toBeGreaterThan(0);
  await expect.poll(async () => page.locator('[data-endpoint-joints]').evaluateAll(rows => rows.every(row => ['hit', 'clear', 'uncertain'].includes(row.getAttribute('data-collision-state')!)))).toBe(true);
  expect(errors).toEqual([]);
});

test('model switching isolates recorded programs and selections while returning restores the original points', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const longPoint = await captureCurrent(page);
  await makePath(page, [longPoint]);
  await openProgram(page);
  await programRows(page).first().click({ button: 'right' });
  await expect(page.getByRole('menu', { name: '程序右键菜单', exact: true })).toBeVisible();
  await selectRobot(page, 'irb1100-4-0.475');
  await expect(page.getByRole('menu', { name: '程序右键菜单', exact: true })).toHaveCount(0);
  await openProgram(page);
  await expect(programRows(page)).toHaveCount(0);
  await expect(page.locator('[data-object-kind="path"]')).toHaveCount(0);
  const shortPoint = await captureCurrent(page);
  expect(shortPoint.id).not.toBe(longPoint.id);
  await selectRobot(page, 'irb1100-4-0.58');
  await openProgram(page);
  await expect(programRows(page)).toHaveCount(1);
  await expect(programRows(page).first()).toHaveAttribute('data-program-point-id', longPoint.id);
  await expect(programRows(page).first()).toHaveAttribute('aria-selected', 'false');
  await expect(page.getByRole('heading', { name: '机器人属性', exact: true })).toBeVisible();
  await expectCopyState(page, false);
  await selectRobot(page, 'irb1100-4-0.475');
  await selectRobotObject(page);
  await page.getByRole('button', { name: '机械零位', exact: true }).click();
  await expect.poll(async () => (await bodyPose(page)).position.map(value => Math.round(value))).toEqual([314, 0, 562]);
  await openProgram(page);
  await expect(programRows(page).first()).toHaveAttribute('data-program-point-id', shortPoint.id);
  await openScene(page);
  await page.screenshot({ path: '/tmp/emulation-object-model-scope.png' });
  expect(errors).toEqual([]);
});
