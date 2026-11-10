import { expect, test, type Locator, type Page } from '@playwright/test';
import { Euler, Quaternion, Vector3 } from 'three';
import type { Pose } from '../../src/robot/types';
import { generateSource, readSource } from './programming-helpers';
import {
  bodyCanvas, bodyJoints, captureCurrent, observeBrowser, openPathMenu, openProgram,
  openScene, openWorkspace, setJoint,
} from './workspace-helpers';

test.setTimeout(180_000);
const objects = (page: Page, kind: string) => page.locator(`[data-object-kind="${kind}"]`);
const object = (page: Page, kind: string, id: string) => page.locator(`[data-object-kind="${kind}"][data-object-id="${id}"]`);
async function rendered<T>(page: Page, name: string): Promise<T> {
  return JSON.parse((await bodyCanvas(page).getAttribute(name))!);
}
function poseError(actual: Pose, expected: Pose) {
  const dot = Math.abs(actual.orientation.reduce((sum, value, i) => sum + value * expected.orientation[i], 0));
  return Math.max(Math.hypot(...actual.position.map((value, i) => value - expected.position[i])), Math.abs(1 - dot));
}
function composePose(parent: Pose, local: Pose): Pose {
  const [w, x, y, z] = parent.orientation, [lw, lx, ly, lz] = local.orientation;
  const rotation = new Quaternion(x, y, z, w);
  const position = new Vector3(...local.position).applyQuaternion(rotation).add(new Vector3(...parent.position));
  rotation.multiply(new Quaternion(lx, ly, lz, lw));
  return { position: [position.x, position.y, position.z], orientation: [rotation.w, rotation.x, rotation.y, rotation.z] };
}
async function expectPose(page: Page, attribute: string, expected: Pose) {
  await expect.poll(async () => poseError(await rendered<Pose>(page, attribute), expected)).toBeLessThan(1e-5);
}
async function toolPose(page: Page, id: string) {
  const tools = await rendered<{ id: string; pose: Pose; tcpPose: Pose }[]>(page, 'data-scene-tools');
  return tools.find(tool => tool.id === id)!;
}
async function pathPose(page: Page, id: string) {
  const paths = await rendered<{ id: string; points: Pose[] }[]>(page, 'data-scene-paths');
  return paths.find(path => path.id === id)!.points[0];
}
async function applyPose(page: Page, label: string, position: number[], rotation?: number[]) {
  for (const [index, axis] of ['X', 'Y', 'Z'].entries()) {
    await page.getByRole('spinbutton', { name: `${label}位置 ${axis}`, exact: true }).fill(String(position[index]));
  }
  if (rotation) for (const [index, axis] of ['Rz', 'Ry', 'Rx'].entries()) {
    await page.getByRole('spinbutton', { name: `${label}姿态 ${axis}`, exact: true }).fill(String(rotation[index]));
  }
  await page.getByRole('button', { name: `应用${label}位姿`, exact: true }).click();
}
async function removalDialog(page: Page, node: Locator, action: string, title = action, keyboard = false) {
  if (keyboard) await node.press('Shift+F10');
  else await node.click({ button: 'right' });
  const menu = page.getByRole('menu');
  await expect(menu).toBeVisible();
  await menu.getByRole('menuitem', { name: action, exact: true }).click();
  const dialog = page.getByRole('dialog', { name: title, exact: true });
  await expect(dialog).toBeVisible();
  return dialog;
}
async function addMountedTool(page: Page) {
  await page.getByRole('button', { name: '添加工具', exact: true }).click();
  const id = (await objects(page, 'tool').last().getAttribute('data-object-id'))!;
  const tool = object(page, 'tool', id);
  await tool.dragTo(page.getByRole('button', { name: '选择末端法兰', exact: true }));
  await expect(tool).toHaveAttribute('data-parent-kind', 'flange');
  await tool.click();
  await page.getByRole('button', { name: '对齐法兰', exact: true }).click();
  await applyPose(page, '工具 TCP 偏置', [15, -10, 65], [20, -10, 5]);
  const rotation = new Quaternion().setFromEuler(new Euler(5 * Math.PI / 180, -10 * Math.PI / 180, 20 * Math.PI / 180, 'ZYX'));
  const requestedTcp: Pose = { position: [15, -10, 65], orientation: [rotation.w, rotation.x, rotation.y, rotation.z] };
  // Both rendered TCP values can still agree on the old 50 mm offset immediately
  // after Apply. Wait for the requested transform, then snapshot deletion inputs.
  await expect.poll(async () => {
    const snapshot = await bodyCanvas(page).evaluate((canvas, toolId) => ({
      tool: (JSON.parse(canvas.getAttribute('data-scene-tools')!) as { id: string; pose: Pose; tcpPose: Pose }[]).find(item => item.id === toolId),
      flange: JSON.parse(canvas.getAttribute('data-flange-pose')!) as Pose,
      tcp: JSON.parse(canvas.getAttribute('data-robot-pose')!) as Pose,
    }), id);
    if (!snapshot.tool) return Infinity;
    const expectedTcp = composePose(snapshot.flange, requestedTcp);
    return Math.max(poseError(snapshot.tool.pose, snapshot.flange), poseError(snapshot.tool.tcpPose, expectedTcp), poseError(snapshot.tcp, expectedTcp));
  }).toBeLessThan(1e-5);
  return { id, node: tool };
}

test('reference and active-tool context deletion preserves child world poses and recorded source', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const point = await captureCurrent(page), source = await generateSource(page, point.pathId);
  await openScene(page);
  await page.getByRole('button', { name: '添加参考坐标系', exact: true }).click();
  const frameId = (await objects(page, 'frame').last().getAttribute('data-object-id'))!;
  const frame = object(page, 'frame', frameId), path = object(page, 'path', point.pathId);
  await applyPose(page, '参考坐标系', [100, -60, 25], [35, 15, -10]);
  await objects(page, 'base').dragTo(frame);
  await path.dragTo(frame);
  await expect(path).toHaveAttribute('data-parent-id', frameId);
  await frame.click();
  await applyPose(page, '参考坐标系', [135, -60, 25], [35, 15, -10]);
  await expect.poll(async () => (await rendered<Pose>(page, 'data-robot-base-pose')).position[0]).toBeCloseTo(35, 5);
  const tool = await addMountedTool(page);
  const baseBefore = await rendered<Pose>(page, 'data-robot-base-pose');
  const pointBefore = await pathPose(page, point.pathId), toolBefore = await toolPose(page, tool.id);
  const cancelled = await removalDialog(page, frame, '删除参考坐标系');
  await cancelled.getByRole('button', { name: '取消', exact: true }).click();
  await expect(frame).toBeVisible();
  await expect(path).toHaveAttribute('data-parent-id', frameId);
  const confirmed = await removalDialog(page, frame, '删除参考坐标系', undefined, true);
  await confirmed.getByRole('button', { name: '删除', exact: true }).click();
  await expect(frame).toHaveCount(0);
  await expect(objects(page, 'robot')).toHaveAttribute('data-parent-kind', 'world');
  await expect(path).toHaveAttribute('data-parent-kind', 'world');
  await expectPose(page, 'data-robot-base-pose', baseBefore);
  await expect.poll(async () => poseError(await pathPose(page, point.pathId), pointBefore)).toBeLessThan(1e-5);
  await expect.poll(async () => poseError((await toolPose(page, tool.id)).pose, toolBefore.pose)).toBeLessThan(1e-5);
  await expect.poll(async () => poseError((await toolPose(page, tool.id)).tcpPose, toolBefore.tcpPose)).toBeLessThan(1e-5);
  await expect(tool.node).toHaveAttribute('data-parent-kind', 'flange');

  const toolDialog = await removalDialog(page, tool.node, '删除工具', undefined, true);
  await toolDialog.getByRole('button', { name: '删除', exact: true }).click();
  await expect(tool.node).toHaveCount(0);
  await expect.poll(() => rendered(page, 'data-scene-tools')).toEqual([]);
  await expectPose(page, 'data-robot-pose', await rendered<Pose>(page, 'data-flange-pose'));
  await expect.poll(async () => poseError(await pathPose(page, point.pathId), pointBefore)).toBeLessThan(1e-5);
  await expect(object(page, 'target', point.id)).toBeVisible();
  await openProgram(page);
  expect(await readSource(page)).toBe(source);
  expect(errors).toEqual([]);
});

test('zone, target and Path context deletion requires confirmation and preserves generated modules', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const point = await captureCurrent(page), source = await generateSource(page, point.pathId);
  await openScene(page);
  const zoneId = (await objects(page, 'zone').first().getAttribute('data-object-id'))!;
  const zone = object(page, 'zone', zoneId);
  const zoneDialog = await removalDialog(page, zone, '删除禁入区', undefined, true);
  await zoneDialog.press('Escape');
  await expect(zone).toBeVisible();
  await (await removalDialog(page, zone, '删除禁入区')).getByRole('button', { name: '删除', exact: true }).click();
  await expect(zone).toHaveCount(0);
  await expect.poll(async () => (await rendered<{ id: string }[]>(page, 'data-danger-zones')).some(item => item.id === zoneId)).toBe(false);
  const target = object(page, 'target', point.id), path = object(page, 'path', point.pathId);
  await (await removalDialog(page, target, '删除点', '删除目标点')).getByRole('button', { name: '取消', exact: true }).click();
  await expect(target).toBeVisible();
  await (await removalDialog(page, target, '删除点', '删除目标点', true)).getByRole('button', { name: '删除', exact: true }).click();
  await expect(target).toHaveCount(0);
  await expect(path).toBeVisible();
  await expect(path.locator('small')).toHaveText('0');
  const second = await captureCurrent(page, point.pathId);
  await (await removalDialog(page, path, '删除 Path')).getByRole('button', { name: '取消', exact: true }).click();
  await expect(object(page, 'target', second.id)).toBeVisible();
  await (await removalDialog(page, path, '删除 Path', undefined, true)).getByRole('button', { name: '删除', exact: true }).click();
  await expect(path).toHaveCount(0);
  await expect(object(page, 'target', second.id)).toHaveCount(0);
  await expect.poll(async () => (await rendered<{ id: string }[]>(page, 'data-scene-paths')).some(item => item.id === point.pathId)).toBe(false);
  await expect(objects(page, 'robot')).toBeVisible();
  await openProgram(page);
  expect(await readSource(page)).toBe(source);
  expect(errors).toEqual([]);
});

test('only the whole robot can be removed, mounted tools retain their actual world poses and the same model can be restored', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const point = await captureCurrent(page), source = await generateSource(page, point.pathId);
  await openScene(page);
  const zoneIds = await objects(page, 'zone').evaluateAll(nodes => nodes.map(node => node.getAttribute('data-object-id')));
  await page.getByRole('button', { name: '添加参考坐标系', exact: true }).click();
  const frameId = (await objects(page, 'frame').last().getAttribute('data-object-id'))!;
  const frame = object(page, 'frame', frameId), path = object(page, 'path', point.pathId);
  await objects(page, 'robot').dragTo(frame);
  await path.dragTo(frame);
  await frame.click();
  await applyPose(page, '参考坐标系', [70, -40, 30], [50, 10, -15]);
  await expect.poll(async () => (await rendered<Pose>(page, 'data-robot-base-pose')).position[0]).toBeCloseTo(70, 5);
  const tool = await addMountedTool(page), attached = await toolPose(page, tool.id);
  await setJoint(page, 1, 12);
  await expect.poll(async () => (await bodyJoints(page))[0]).toBeCloseTo(12, 5);
  await expect.poll(async () => poseError((await toolPose(page, tool.id)).pose, attached.pose)).toBeGreaterThan(1);
  const mounted = await toolPose(page, tool.id), pointBefore = await pathPose(page, point.pathId);
  const baseBefore = await rendered<Pose>(page, 'data-robot-base-pose');
  const jointsBefore = await bodyJoints(page);

  // A base, individual joint or flange is part of the body, not an independently removable object.
  for (const node of [objects(page, 'base'), objects(page, 'joint').first(), objects(page, 'flange')]) {
    await node.click({ button: 'right' });
    await expect(page.getByRole('menu')).toHaveCount(0);
    await page.keyboard.press('Escape');
    await node.press('Shift+F10');
    await expect(page.getByRole('menu')).toHaveCount(0);
    await page.keyboard.press('Escape');
  }
  await (await removalDialog(page, objects(page, 'robot'), '删除机器人')).getByRole('button', { name: '取消', exact: true }).click();
  await expect(objects(page, 'joint')).toHaveCount(6);
  await expect(bodyCanvas(page)).toHaveAttribute('data-robot-visible', 'true');
  await expect(tool.node).toHaveAttribute('data-parent-kind', 'flange');
  await (await removalDialog(page, objects(page, 'robot'), '删除机器人', undefined, true)).getByRole('button', { name: '删除', exact: true }).click();
  for (const kind of ['robot', 'base', 'joint', 'flange']) await expect(objects(page, kind)).toHaveCount(0);
  await expect(bodyCanvas(page)).toHaveAttribute('data-robot-visible', 'false');
  await expect(page.getByRole('heading', { name: '机器人本体已删除', exact: true })).toBeVisible();
  const restore = page.getByRole('button', { name: '恢复机器人', exact: true });
  await expect(restore).toBeVisible(); await expect(restore).toBeEnabled();
  await expect(tool.node).toHaveAttribute('data-parent-kind', 'world');
  await expect.poll(async () => poseError((await toolPose(page, tool.id)).pose, mounted.pose)).toBeLessThan(1e-5);
  await expect.poll(async () => poseError((await toolPose(page, tool.id)).tcpPose, mounted.tcpPose)).toBeLessThan(1e-5);
  await expect(frame).toBeVisible();
  await expect(path).toHaveAttribute('data-parent-id', frameId);
  await expect.poll(async () => poseError(await pathPose(page, point.pathId), pointBefore)).toBeLessThan(1e-5);
  expect(await objects(page, 'zone').evaluateAll(nodes => nodes.map(node => node.getAttribute('data-object-id')))).toEqual(zoneIds);

  const pathMenu = await openPathMenu(page, point.pathId);
  for (const name of ['添加当前点到 Path', '运行', '路径规划']) await expect(pathMenu.getByRole('menuitem', { name, exact: true })).toBeDisabled();
  await page.keyboard.press('Escape');
  await object(page, 'target', point.id).click({ button: 'right' });
  await expect(page.getByRole('menuitem', { name: '移动至此', exact: true })).toBeDisabled();
  await page.keyboard.press('Escape');
  await expect(object(page, 'target', point.id)).toHaveClass(/selected/);
  await expect(restore).toBeVisible(); await expect(restore).toBeEnabled();
  await openProgram(page);
  expect(await readSource(page)).toBe(source);

  await openScene(page);
  await restore.click();
  await expect(restore).toHaveCount(0);
  await expect(objects(page, 'robot')).toHaveClass(/selected/);
  await expect(page.getByLabel('项目机器人型号', { exact: true })).toHaveAttribute('data-model-id', 'irb1100-4-0.58');
  await expect(page.getByRole('dialog', { name: '机器人模型目录', exact: true })).toHaveCount(0);
  await expect(objects(page, 'joint')).toHaveCount(6);
  await expect(bodyCanvas(page)).toHaveAttribute('data-robot-visible', 'true');
  await expect(objects(page, 'robot')).toHaveAttribute('data-parent-id', frameId);
  await expectPose(page, 'data-robot-base-pose', baseBefore);
  expect(await bodyJoints(page)).toEqual(jointsBefore);
  await expect(tool.node).toHaveAttribute('data-parent-kind', 'world');
  await expect.poll(async () => poseError((await toolPose(page, tool.id)).tcpPose, mounted.tcpPose)).toBeLessThan(1e-5);
  await expectPose(page, 'data-robot-pose', await rendered<Pose>(page, 'data-flange-pose'));
  const restoredMenu = await openPathMenu(page, point.pathId);
  await expect(restoredMenu.getByRole('menuitem', { name: '添加当前点到 Path', exact: true })).toBeEnabled();
  await expect(restoredMenu.getByRole('menuitem', { name: '运行', exact: true })).toBeEnabled();
  await page.keyboard.press('Escape');
  await openProgram(page);
  expect(await readSource(page)).toBe(source);
  expect(errors).toEqual([]);
});
