import { expect, test, type Page } from '@playwright/test';
import { PerspectiveCamera, Vector3 } from 'three';
import type { Pose, Vec3 } from '../../src/robot/types';
import { bodyCanvas, bodyJoints, bodyPose, createPath, observeBrowser, openWorkspace, pathPoints, selectRobotObject } from './workspace-helpers';

test.setTimeout(90_000);
async function attribute<T>(page: Page, key: string): Promise<T> { return JSON.parse((await bodyCanvas(page).getAttribute(key))!); }
async function dragWorldX(page: Page) {
  const canvas = bodyCanvas(page);
  await expect.poll(async () => (await attribute<Record<string, { x: number; y: number }>>(page, 'data-jog-handles')).X).toBeTruthy();
  const handle = (await attribute<Record<string, { x: number; y: number }>>(page, 'data-jog-handles')).X;
  const box = (await canvas.boundingBox())!;
  const camera = new PerspectiveCamera(Number(await canvas.getAttribute('data-camera-fov')), Number(await canvas.getAttribute('data-camera-aspect')), .01, 100);
  camera.up.fromArray(await attribute<Vec3>(page, 'data-camera-up'));
  camera.position.fromArray(await attribute<Vec3>(page, 'data-camera-position'));
  camera.lookAt(new Vector3(...await attribute<Vec3>(page, 'data-camera-target'))); camera.updateMatrixWorld();
  const before = await bodyPose(page), projection = new Vector3(...before.position).multiplyScalar(.001).project(camera);
  const center = { x: box.x + (projection.x + 1) * box.width / 2, y: box.y + (1 - projection.y) * box.height / 2 };
  const length = Math.hypot(handle.x - center.x, handle.y - center.y);
  await page.mouse.move(handle.x, handle.y); await page.mouse.down();
  await expect(canvas).toHaveAttribute('data-jog-dragging', 'true');
  await page.mouse.move(handle.x + (handle.x - center.x) / length * 4, handle.y + (handle.y - center.y) / length * 4, { steps: 4 });
  await page.mouse.up();
  await expect(canvas).toHaveAttribute('data-jog-dragging', 'false');
  await expect(page.getByRole('status', { name: '拖动状态', exact: true })).toHaveText('拖动控制柄调整当前 TCP；无效目标不会移动机器人。');
  // React can show the completed solve before Three renders its last accepted
  // joints. Wait for that frame before freezing the exact pose for teaching.
  const acceptedJoints = await page.getByRole('spinbutton', { name: /^机器人关节 J[1-6]$/ })
    .evaluateAll(inputs => inputs.map(input => Number((input as HTMLInputElement).value)));
  expect(acceptedJoints).toHaveLength(6);
  await expect.poll(async () => (await bodyJoints(page)).map(value => Number(value.toFixed(6))))
    .toEqual(acceptedJoints);
  await expect.poll(async () => Math.abs((await bodyPose(page)).position[0] - before.position[0])).toBeGreaterThan(.5);
}
function poseError(actual: Pose, expected: Pose) {
  return Math.max(Math.hypot(...actual.position.map((n, i) => n - expected.position[i])),
    1 - Math.abs(actual.orientation.reduce((sum, n, i) => sum + n * expected.orientation[i], 0)));
}

test('right-click and keyboard Path teaching retain the dragged flange mode, selection and accepted pose', async ({ page }) => {
  const errors = observeBrowser(page); await openWorkspace(page);
  const pathId = await createPath(page), path = page.locator(`[data-object-kind="path"][data-object-id="${pathId}"]`);
  await selectRobotObject(page);
  await page.getByRole('button', { name: '正视', exact: true }).click();
  const toggle = page.getByRole('button', { name: '拖动法兰', exact: true });
  await toggle.click();
  for (const keyboard of [false, true]) {
    await dragWorldX(page);
    const pose = await bodyPose(page), joints = await bodyJoints(page), before = await pathPoints(page, pathId).count();
    if (keyboard) await path.press('Shift+F10'); else await path.click({ button: 'right' });
    const menu = page.getByRole('menu', { name: 'Path 右键菜单', exact: true });
    await expect(menu).toBeVisible();
    await expect(page.getByRole('button', { name: '选择机器人', exact: true })).toHaveClass(/selected/);
    await expect(path).not.toHaveClass(/selected/);
    await expect(toggle).toHaveAttribute('aria-pressed', 'true');
    await expect(bodyCanvas(page)).toHaveAttribute('data-gizmo-target', 'tcp');
    await expect(bodyCanvas(page)).toHaveAttribute('data-jog-mode', 'translate');
    await expect(bodyCanvas(page)).toHaveAttribute('data-jog-space', 'world');
    await menu.getByRole('menuitem', { name: '添加当前点到 Path', exact: true }).click();
    await expect(menu).toHaveCount(0);
    await expect(pathPoints(page, pathId)).toHaveCount(before + 1);
    await expect(page.getByRole('button', { name: '选择机器人', exact: true })).toHaveClass(/selected/);
    await expect(toggle).toHaveAttribute('aria-pressed', 'true');
    await expect(bodyCanvas(page)).toHaveAttribute('data-gizmo-target', 'tcp');
    await expect.poll(() => bodyJoints(page)).toEqual(joints);
    await expect.poll(async () => {
      const paths = await attribute<{ id: string; points: Pose[] }[]>(page, 'data-scene-paths');
      const captured = paths.find(item => item.id === pathId)?.points.at(-1);
      return captured ? poseError(captured, pose) : Infinity;
    }).toBeLessThan(.00001);
  }
  await page.getByRole('button', { name: '工具坐标', exact: true }).click();
  await page.getByRole('button', { name: '旋转', exact: true }).click();
  await path.click({ button: 'right' });
  await page.getByRole('menu', { name: 'Path 右键菜单', exact: true }).press('Escape');
  await expect(bodyCanvas(page)).toHaveAttribute('data-jog-mode', 'rotate');
  await expect(bodyCanvas(page)).toHaveAttribute('data-jog-space', 'local');
  await expect(toggle).toHaveAttribute('aria-pressed', 'true');
  expect(errors).toEqual([]);
});

test('project robot is read-only on the right while scene and program tabs stay centered at desktop sizes', async ({ page }) => {
  const errors = observeBrowser(page); await openWorkspace(page);
  const model = page.getByLabel('项目机器人型号', { exact: true });
  await expect(model).toHaveAttribute('data-model-id', 'irb1100-4-0.58');
  await expect(page.getByRole('button', { name: '选择机器人型号', exact: true })).toHaveCount(0);
  expect(await model.evaluate(element => element.closest('button') === null)).toBe(true);
  for (const size of [{ width: 1000, height: 720 }, { width: 1440, height: 1000 }]) {
    await page.setViewportSize(size);
    await expect.poll(async () => {
      const header = (await page.locator('.topbar').boundingBox())!, tabs = (await page.getByLabel('工作区视图', { exact: true }).boundingBox())!;
      return Math.abs(tabs.x + tabs.width / 2 - (header.x + header.width / 2));
    }).toBeLessThanOrEqual(1);
    const tabBox = (await page.getByLabel('工作区视图', { exact: true }).boundingBox())!, modelBox = (await model.boundingBox())!;
    const homeBox = (await page.getByRole('navigation', { name: '项目导航', exact: true }).getByRole('button', { name: '返回主页', exact: true }).boundingBox())!;
    expect(homeBox.x + homeBox.width).toBeLessThan(tabBox.x);
    expect(modelBox.x).toBeGreaterThan(tabBox.x + tabBox.width);
    expect(modelBox.x + modelBox.width).toBeLessThanOrEqual(size.width);
    expect(await page.evaluate(() => Math.max(document.documentElement.scrollWidth - innerWidth, document.documentElement.scrollHeight - innerHeight))).toBeLessThanOrEqual(1);
  }
  await model.click();
  await expect(page.getByRole('dialog', { name: '机器人模型目录', exact: true })).toHaveCount(0);
  await page.screenshot({ path: '/tmp/emulation-fixed-project-navigation.png' });
  expect(errors).toEqual([]);
});
