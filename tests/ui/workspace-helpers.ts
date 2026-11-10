import { enterProject, openDefaultProject, waitForProjectScene } from './project-helpers';
import { expect, type Page } from '@playwright/test';
import type { Pose } from '../../src/robot/types';

export const scene = (page: Page) => page.getByLabel('ABB IRB 1100 三维仿真场景');
export const pathObjects = (page: Page) => page.locator('[data-object-kind="path"]');
export const pathPoints = (page: Page, pathId?: string) => page.locator(pathId
  ? `[data-object-kind="target"][data-path-id="${pathId}"]` : '[data-object-kind="target"]');
export const bodyCanvas = (page: Page) => page.locator('canvas[data-robot-joints]');
export type CapturedPoint = { id: string; name: string; pathId: string };
export function observeBrowser(page: Page) {
  const errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  return errors;
}
export async function openWorkspace(page: Page) {
  await openDefaultProject(page);
  await waitForProjectScene(page, 'irb1100-4-0.58');
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
  await openScene(page);
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
export async function createPath(page: Page): Promise<string> {
  await openScene(page);
  const before = await pathObjects(page).count();
  await page.getByRole('button', { name: '添加 Path', exact: true }).click();
  await expect(pathObjects(page)).toHaveCount(before + 1);
  const path = pathObjects(page).last();
  await expect(path).toHaveClass(/selected/);
  return (await path.getAttribute('data-object-id'))!;
}
export async function selectPath(page: Page, pathId: string) {
  await openScene(page);
  const path = page.locator(`[data-object-kind="path"][data-object-id="${pathId}"]`);
  await path.click();
  await expect(path).toHaveClass(/selected/);
}
export async function captureCurrent(page: Page, pathId?: string): Promise<CapturedPoint> {
  await openScene(page);
  const targetPath = pathId ?? (await pathObjects(page).count() ? (await pathObjects(page).first().getAttribute('data-object-id'))! : await createPath(page));
  const path = page.locator(`[data-object-kind="path"][data-object-id="${targetPath}"]`);
  const points = pathPoints(page, targetPath), before = await points.count();
  await path.click({ button: 'right' });
  await page.getByRole('menu', { name: 'Path 右键菜单', exact: true })
    .getByRole('menuitem', { name: '添加当前点到 Path', exact: true }).click();
  await expect(points).toHaveCount(before + 1);
  const target = points.last();
  const point = { id: (await target.getAttribute('data-object-id'))!, name: (await target.getAttribute('data-point-name'))!, pathId: targetPath };
  // Teaching preserves an active robot Jog. Inspect the new point explicitly
  // in this shared helper; direct gesture tests verify that teaching itself does not select it.
  if (!await target.evaluate(element => element.classList.contains('selected'))) await target.click();
  await expect(page.locator('[data-target-point-id]')).toHaveAttribute('data-target-point-id', point.id);
  await expect(target).toHaveClass(/selected/);
  return point;
}
export async function selectPoint(page: Page, point: CapturedPoint) {
  await openScene(page);
  const tree = page.locator(`[data-object-kind="target"][data-object-id="${point.id}"]`);
  const details = page.locator('details.object-path').filter({ has: tree });
  if (await details.count() && await details.getAttribute('open') === null) await details.locator('summary').click();
  await tree.click();
  await expect(page.locator('[data-target-point-id]')).toHaveAttribute('data-target-point-id', point.id);
}
/** Select the points' existing Path; no program-to-Path conversion exists. */
export async function selectPathForPoints(page: Page, points: CapturedPoint[]) {
  expect(points.length).toBeGreaterThan(0);
  const pathId = points[0].pathId;
  expect(points.every(point => point.pathId === pathId)).toBe(true);
  await selectPath(page, pathId);
  const ids = await pathPoints(page, pathId).evaluateAll(elements => elements.map(element => element.getAttribute('data-object-id')));
  expect(ids.filter(id => points.some(point => point.id === id))).toEqual(points.map(point => point.id));
  return pathId;
}
/** The target panel no longer exposes a separate single-leg motion workflow. */
export async function expectNoPointMotionActions(page: Page) {
  await expect(page.getByRole('button', { name: /^检查到此点的 / })).toHaveCount(0);
  for (const name of ['复制此 MoveL', '导出此 MoveL', '播放此 MoveL', '播放路径']) {
    await expect(page.getByRole('button', { name, exact: true })).toHaveCount(0);
  }
  await expect(page.getByRole('slider', { name: '路径进度', exact: true })).toHaveCount(0);
  await expect(page.getByRole('textbox', { name: 'RAPID MoveL 指令', exact: true })).toHaveCount(0);
}
/** Open the context menu without preselecting its Path through a left click. */
export async function openPathMenu(page: Page, pathId: string, keyboard = false) {
  await openScene(page);
  const path = page.locator(`[data-object-kind="path"][data-object-id="${pathId}"]`);
  if (keyboard) await path.press('Shift+F10');
  else await path.click({ button: 'right' });
  const menu = page.getByRole('menu', { name: 'Path 右键菜单', exact: true });
  await expect(menu).toBeVisible();
  return menu;
}
export async function runPathFromMenu(page: Page, pathId: string, options: { keyboard?: boolean } = {}) {
  const menu = await openPathMenu(page, pathId, options.keyboard);
  const run = menu.getByRole('menuitem', { name: '运行', exact: true });
  await expect(run).toBeEnabled();
  if (options.keyboard) await run.press('Enter');
  else await run.click();
  await expect(menu).toHaveCount(0);
  await expect(page.locator('[data-run-phase]')).toHaveAttribute('data-run-path-id', pathId);
}
const robotProjects = new WeakMap<Page, Map<string, string>>();
/** A project has one fixed robot. Change robots through the real project home,
 * reopening each model's own project so scene and source isolation is exercised. */
export async function selectRobot(page: Page, id: string) {
  let projects = robotProjects.get(page);
  if (!projects) { projects = new Map(); robotProjects.set(page, projects); }
  const current = page.getByLabel('项目机器人型号', { exact: true });
  const currentId = await current.getAttribute('data-model-id');
  const projectId = (await page.locator('[data-project-id]').getAttribute('data-project-id'))!;
  if (!currentId || !projectId) throw new Error('Workspace does not expose its fixed project/robot identity');
  projects.set(currentId, projectId);
  if (currentId !== id) {
    await page.getByRole('button', { name: '返回主页', exact: true }).click();
    const home = page.getByRole('main', { name: '项目管理', exact: true });
    await expect(home).toBeVisible();
    const savedId = projects.get(id);
    const card = savedId ? page.locator(`[data-project-card="${savedId}"] .project-card-open`) : null;
    if (card && await card.count()) await card.click();
    else projects.set(id, await enterProject(page, 'Robot ' + id, id));
  }
  await waitForProjectScene(page, id);
}
