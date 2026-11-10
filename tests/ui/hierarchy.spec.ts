import { expect, test, type Page } from '@playwright/test';
import { Quaternion, Vector3 } from 'three';
import type { Pose } from '../../src/robot/types';
import { generateSource, readSource } from './programming-helpers';
import { bodyCanvas, bodyJoints, captureCurrent, observeBrowser, openProgram, openScene, openWorkspace, selectPoint, selectRobot, setJoint } from './workspace-helpers';

test.setTimeout(150_000);
const objects = (page: Page, kind: string) => page.locator(`[data-object-kind="${kind}"]`);
async function rendered<T>(page: Page, name: string): Promise<T> {
  return JSON.parse((await bodyCanvas(page).getAttribute(name))!);
}
function poseError(actual: Pose, expected: Pose) {
  const dot = Math.abs(actual.orientation.reduce((sum, value, i) => sum + value * expected.orientation[i], 0));
  return Math.max(Math.hypot(...actual.position.map((value, i) => value - expected.position[i])), Math.abs(1 - dot));
}
async function expectPose(page: Page, name: string, expected: Pose) {
  await expect.poll(async () => poseError(await rendered<Pose>(page, name), expected)).toBeLessThan(1e-5);
}
async function applyReference(page: Page, position: number[], rotation: number[]) {
  for (const [i, axis] of ['X', 'Y', 'Z'].entries()) await page.getByRole('spinbutton', { name: `参考坐标系位置 ${axis}`, exact: true }).fill(String(position[i]));
  for (const [i, axis] of ['Rz', 'Ry', 'Rx'].entries()) await page.getByRole('spinbutton', { name: `参考坐标系姿态 ${axis}`, exact: true }).fill(String(rotation[i]));
  await page.getByRole('button', { name: '应用参考坐标系位姿', exact: true }).click();
}
async function pathPose(page: Page, pathId: string): Promise<Pose> {
  await expect.poll(async () => (await rendered<{ id: string; points: Pose[] }[]>(page, 'data-scene-paths')).some(path => path.id === pathId && path.points.length > 0)).toBe(true);
  const paths = await rendered<{ id: string; points: Pose[] }[]>(page, 'data-scene-paths');
  return paths.find(path => path.id === pathId)!.points[0];
}

test('dragging base and a complete Path under a rotated reference frame preserves world poses, then parent edits move both consistently', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  await expect(objects(page, 'joint')).toHaveCount(6);
  for (let index = 1; index <= 6; index++) await expect(page.getByRole('button', { name: `选择关节轴 ${index}`, exact: true })).toBeVisible();
  await expect(page.getByRole('button', { name: '选择末端法兰', exact: true })).toBeVisible();
  const point = await captureCurrent(page);
  const originalBody = await rendered<Pose>(page, 'data-robot-pose');
  const originalBase = await rendered<Pose>(page, 'data-robot-base-pose');
  const originalPoint = await pathPose(page, point.pathId);
  const originalJoints = await bodyJoints(page);
  await page.getByRole('button', { name: '添加参考坐标系', exact: true }).click();
  const frame = objects(page, 'frame').last(), id = (await frame.getAttribute('data-object-id'))!;
  await applyReference(page, [120, -80, 30], [90, 0, 0]);
  const base = page.getByRole('button', { name: '选择机器人基座', exact: true });
  await base.dragTo(frame);
  await expect(base).toHaveAttribute('data-parent-kind', 'frame');
  await expect(base).toHaveAttribute('data-parent-id', id);
  await expectPose(page, 'data-robot-base-pose', originalBase);
  await expectPose(page, 'data-robot-pose', originalBody);
  expect(await bodyJoints(page)).toEqual(originalJoints);
  await base.click();
  const local = JSON.parse((await page.locator('[data-local-pose][data-world-pose]').getAttribute('data-local-pose'))!) as Pose;
  // Independent analytic inverse of Rz(90°) plus the nonzero parent translation.
  expect(Math.hypot(local.position[0] - 80, local.position[1] - 120, local.position[2] + 30)).toBeLessThan(1e-6);
  expect(Math.abs(local.orientation[0] * Math.SQRT1_2 - local.orientation[3] * Math.SQRT1_2)).toBeCloseTo(1, 8);
  const path = page.locator(`[data-object-kind="path"][data-object-id="${point.pathId}"]`);
  await path.dragTo(frame);
  await expect(path).toHaveAttribute('data-parent-kind', 'frame');
  await expect(path).toHaveAttribute('data-parent-id', id);
  await expect.poll(async () => poseError(await pathPose(page, point.pathId), originalPoint)).toBeLessThan(1e-5);
  await frame.click();
  await applyReference(page, [150, -80, 30], [90, 0, 0]);
  const translate = (pose: Pose): Pose => ({ position: [pose.position[0] + 30, pose.position[1], pose.position[2]], orientation: pose.orientation });
  await expectPose(page, 'data-robot-base-pose', translate(originalBase));
  await expectPose(page, 'data-robot-pose', translate(originalBody));
  await expect.poll(async () => poseError(await pathPose(page, point.pathId), translate(originalPoint))).toBeLessThan(1e-5);
  expect(await bodyJoints(page)).toEqual(originalJoints);

  await selectRobot(page, 'irb1100-4-0.475');
  await expect(objects(page, 'frame')).toHaveCount(0);
  await expect(objects(page, 'path')).toHaveCount(0);
  await expect(objects(page, 'joint')).toHaveCount(6);
  await expectPose(page, 'data-robot-base-pose', { position: [0, 0, 0], orientation: [1, 0, 0, 0] });
  await selectRobot(page, 'irb1100-4-0.58');
  await expect(page.locator(`[data-object-kind="frame"][data-object-id="${id}"]`)).toBeVisible();
  await expectPose(page, 'data-robot-base-pose', translate(originalBase));
  await expect.poll(async () => poseError(await pathPose(page, point.pathId), translate(originalPoint))).toBeLessThan(1e-5);
  expect(await page.evaluate(() => Math.max(document.documentElement.scrollHeight - innerHeight, document.documentElement.scrollWidth - innerWidth))).toBeLessThanOrEqual(1);
  await page.screenshot({ path: '/tmp/emulation-reference-hierarchy.png' });
  expect(errors).toEqual([]);
});

async function toolPose(page: Page, id: string): Promise<{ pose: Pose; tcpPose: Pose }> {
  return (await rendered<{ id: string; pose: Pose; tcpPose: Pose }[]>(page, 'data-scene-tools')).find(tool => tool.id === id)!;
}
function offsetPose(flange: Pose, offset: number[]): Pose {
  const [w, x, y, z] = flange.orientation;
  const transformed = new Vector3().fromArray(offset).applyQuaternion(new Quaternion(x, y, z, w));
  return { position: [flange.position[0] + transformed.x, flange.position[1] + transformed.y, flange.position[2] + transformed.z], orientation: flange.orientation };
}

test('a tool dragged onto the flange keeps its world pose, TCP offset is used for teaching, and detaching preserves its independent pose', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  await page.getByRole('button', { name: '添加工具', exact: true }).click();
  const tool = objects(page, 'tool').last(), id = (await tool.getAttribute('data-object-id'))!;
  await expect.poll(async () => Boolean(await toolPose(page, id))).toBe(true);
  const before = await toolPose(page, id);
  const originalBody = await bodyJoints(page);
  await tool.dragTo(page.getByRole('button', { name: '选择末端法兰', exact: true }));
  await expect(tool).toHaveAttribute('data-parent-kind', 'flange');
  await expect.poll(async () => poseError((await toolPose(page, id)).pose, before.pose)).toBeLessThan(1e-5);
  expect(await bodyJoints(page)).toEqual(originalBody);
  await tool.click();
  await page.getByRole('button', { name: '对齐法兰', exact: true }).click();
  for (const axis of ['X', 'Y']) await page.getByRole('spinbutton', { name: `工具 TCP 偏置位置 ${axis}`, exact: true }).fill('0');
  await page.getByRole('spinbutton', { name: '工具 TCP 偏置位置 Z', exact: true }).fill('50');
  await page.getByRole('button', { name: '应用工具 TCP 偏置位姿', exact: true }).click();
  const flange = await rendered<Pose>(page, 'data-flange-pose'), tcp = offsetPose(flange, [0, 0, 50]);
  await expectPose(page, 'data-robot-pose', tcp);
  await expect.poll(async () => poseError((await toolPose(page, id)).tcpPose, tcp)).toBeLessThan(1e-5);
  const point = await captureCurrent(page);
  const recorded = await pathPose(page, point.pathId);
  expect(poseError(recorded, tcp)).toBeLessThan(1e-5);
  await setJoint(page, 1, 3);
  await expect.poll(async () => (await bodyJoints(page))[0]).toBeCloseTo(3, 5);
  const nextFlange = await rendered<Pose>(page, 'data-flange-pose');
  await expectPose(page, 'data-robot-pose', offsetPose(nextFlange, [0, 0, 50]));
  expect(poseError(await pathPose(page, point.pathId), recorded)).toBeLessThan(1e-5);
  await selectPoint(page, point);
  const pointNode = page.locator(`[data-object-kind="target"][data-object-id="${point.id}"]`);
  await pointNode.click({ button: 'right' });
  await page.getByRole('menuitem', { name: '移动至此', exact: true }).click();
  await expect(page.locator('[data-run-phase]')).toHaveAttribute('data-run-phase', 'complete', { timeout: 20_000 });
  await expectPose(page, 'data-robot-pose', recorded);
  const moduleSource = await generateSource(page, point.pathId);
  const toolLiteral = /PERS\s+tooldata\s+SimTool\s*:=\s*(.+);/.exec(moduleSource);
  expect(toolLiteral).not.toBeNull();
  const toolData = JSON.parse(toolLiteral![1].replace(/\bTRUE\b/g, 'true')) as [boolean, [number[], number[]], unknown];
  expect(toolData[0]).toBe(true);
  expect(toolData[1][0]).toEqual([0, 0, 50]);
  expect(toolData[1][1]).toEqual([1, 0, 0, 0]);
  expect(moduleSource).toContain('fine, SimTool');
  await openScene(page);
  await expectPose(page, 'data-robot-pose', recorded);
  await tool.click();
  const attached = await toolPose(page, id);
  await page.getByRole('combobox', { name: '挂接到', exact: true }).selectOption('world');
  await expect(tool).toHaveAttribute('data-parent-kind', 'world');
  await expect.poll(async () => poseError((await toolPose(page, id)).pose, attached.pose)).toBeLessThan(1e-5);
  await setJoint(page, 1, 6);
  await expect.poll(async () => (await bodyJoints(page))[0]).toBeCloseTo(6, 5);
  await expect.poll(async () => poseError((await toolPose(page, id)).tcpPose, attached.tcpPose)).toBeLessThan(1e-5);
  await expectPose(page, 'data-robot-pose', await rendered<Pose>(page, 'data-flange-pose'));
  // All active transforms are identity again, but this point was captured with
  // a 50 mm tool. Its saved joints cannot be reused as a valid flange target.
  await selectPoint(page, point);
  await pointNode.click({ button: 'right' });
  await page.getByRole('menuitem', { name: '移动至此', exact: true }).click();
  await expect(page.locator('[data-run-phase]')).toHaveAttribute('data-run-phase', 'complete', { timeout: 20_000 });
  await expectPose(page, 'data-robot-pose', recorded);
  await expectPose(page, 'data-flange-pose', recorded);
  expect(Math.hypot(...(await bodyJoints(page)).map((value, i) => value - originalBody[i]))).toBeGreaterThan(.1);
  await expect.poll(async () => poseError((await toolPose(page, id)).pose, attached.pose)).toBeLessThan(1e-5);
  await expect.poll(async () => poseError((await toolPose(page, id)).tcpPose, attached.tcpPose)).toBeLessThan(1e-5);
  expect(poseError(await pathPose(page, point.pathId), recorded)).toBeLessThan(1e-5);
  await openProgram(page);
  expect(await readSource(page)).toBe(moduleSource);
  await openScene(page);
  expect(errors).toEqual([]);
});
