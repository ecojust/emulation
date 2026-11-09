import { expect, type Page } from '@playwright/test';
import type { Pose } from '../../src/robot/types';

export const scene = (page: Page) => page.getByLabel('ABB IRB 1100 三维仿真场景');
export const programRows = (page: Page) => page.getByRole('listbox', { name: /程序指令$/ }).getByRole('option');
export const bodyCanvas = (page: Page) => page.locator('canvas[data-robot-joints]');
export type CapturedPoint = { id: string; name: string };
export function observeBrowser(page: Page) {
  const errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  return errors;
}
export async function openWorkspace(page: Page) {
  await page.goto('/');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await expect(scene(page)).toHaveAttribute('data-camera-distance', /\d/);
}
export async function openProgram(page: Page) {
  await page.getByRole('button', { name: '程序', exact: true }).click();
  await expect(page.getByRole('region', { name: /程序编辑区$/ })).toBeVisible();
}
export async function openScene(page: Page) {
  await page.getByRole('button', { name: '场景', exact: true }).click();
  await expect(page.getByRole('region', { name: '机器人视口', exact: true })).toBeVisible();
}
export async function selectRobotObject(page: Page) {
  await page.getByRole('button', { name: '选择机器人', exact: true }).click();
  await expect(page.getByRole('heading', { name: '机器人属性', exact: true })).toBeVisible();
}
export async function setJoint(page: Page, joint: number, value: number | string) {
  await selectRobotObject(page);
  await page.getByRole('spinbutton', { name: `机器人关节 J${joint}`, exact: true }).fill(String(value));
}
export async function bodyJoints(page: Page): Promise<number[]> {
  return JSON.parse((await bodyCanvas(page).getAttribute('data-robot-joints'))!);
}
export async function bodyPose(page: Page): Promise<Pose> {
  return JSON.parse((await bodyCanvas(page).getAttribute('data-robot-pose'))!);
}
export async function expectBodyJoints(page: Page, joints: readonly number[], epsilon = 1e-5) {
  await expect.poll(async () => {
    const actual = await bodyJoints(page);
    return actual?.length === joints.length ? Math.max(...actual.map((value, i) => Math.abs(value - joints[i]))) : Infinity;
  }).toBeLessThan(epsilon);
}
export async function captureCurrent(page: Page): Promise<CapturedPoint> {
  await openProgram(page);
  const rows = programRows(page), before = await rows.count();
  if (before) await rows.last().click({ button: 'right' });
  else await page.getByRole('listbox', { name: /程序指令$/ }).click({ button: 'right' });
  await page.getByRole('menuitem', { name: /^插入当前姿态（/ }).click();
  await expect(rows).toHaveCount(before + 1);
  const row = rows.last();
  const point = { id: (await row.getAttribute('data-program-point-id'))!, name: (await row.getAttribute('data-program-point-name')) ?? (await row.getAttribute('aria-label'))!.replace('MoveL ', '') };
  // Capturing and selecting happen in one React event; the new object must not
  // be discarded by selection validation against the previous points list.
  await expect(page.locator('[data-target-point-id]')).toHaveAttribute('data-target-point-id', point.id);
  await expect(row).toHaveAttribute('aria-selected', 'true');
  return point;
}
export async function selectPoint(page: Page, point: CapturedPoint) {
  const tree = page.locator(`[data-object-kind="target"][data-object-id="${point.id}"]`);
  if (await tree.count()) await tree.click();
  else {
    await openProgram(page);
    await page.locator(`[data-program-point-id="${point.id}"]`).click();
  }
  await expect(page.locator('[data-target-point-id]')).toHaveAttribute('data-target-point-id', point.id);
  await openScene(page);
}
export async function makePath(page: Page, points: CapturedPoint[]) {
  await openProgram(page);
  // Deliberately reverse click order: execution order must remain program order.
  for (let i = points.length - 1; i >= 0; i--) {
    await page.locator(`[data-program-point-id="${points[i].id}"]`).click(i === points.length - 1 ? {} : { modifiers: ['ControlOrMeta'] });
  }
  await page.locator(`[data-program-point-id="${points[0].id}"]`).click({ button: 'right' });
  await page.getByRole('menuitem', { name: `转为 Path（${points.length} 条）`, exact: true }).click();
  const path = page.locator('[data-object-kind="path"]').last();
  await expect(path).toHaveClass(/selected/);
  return (await path.getAttribute('data-object-id'))!;
}
export async function expectCopyState(page: Page, enabled: boolean) {
  const button = page.getByRole('button', { name: '复制此 MoveL', exact: true });
  if (enabled) await expect(button).toBeEnabled();
  else await expect(page.getByRole('textbox', { name: 'RAPID MoveL 指令', exact: true })).toHaveCount(0);
}
export async function readRapid(page: Page) {
  return page.getByRole('textbox', { name: 'RAPID MoveL 指令', exact: true }).inputValue();
}
export async function checkedSegment(page: Page) {
  const point = await captureCurrent(page);
  await setJoint(page, 1, 3);
  await selectPoint(page, point);
  await page.getByRole('button', { name: '检查到此点的 MoveL', exact: true }).click();
  await expect(page.getByRole('heading', { name: '此 MoveL 已通过运动学检查', exact: true })).toBeVisible();
  await expectCopyState(page, true);
  return point;
}
export async function selectRobot(page: Page, id: string) {
  await page.getByRole('button', { name: '选择机器人型号', exact: true }).click();
  const dialog = page.getByRole('dialog', { name: '机器人模型目录', exact: true });
  await expect(dialog).toBeVisible();
  await dialog.locator(`[data-catalog-model-id="${id}"]`).getByRole('button', { name: /^加载 / }).click();
  await expect(dialog).toBeHidden();
}
