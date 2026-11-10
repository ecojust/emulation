import { expect, test, type Page } from '@playwright/test';
import { PerspectiveCamera, Vector3 } from 'three';
import { prepareCatalogFixtures, readCatalog } from './catalog-fixtures';
import { bodyCanvas, bodyJoints, bodyPose, captureCurrent, observeBrowser, openWorkspace, selectPoint, selectRobot, selectRobotObject, setJoint } from './workspace-helpers';
import type { Pose, Vec3 } from '../../src/robot/types';
import type { RobotCatalog } from '../../src/catalog/types';

let catalog: RobotCatalog, fixtures: Map<string, Buffer>;
test.beforeAll(async () => { test.setTimeout(180_000); catalog = await readCatalog(); fixtures = await prepareCatalogFixtures(catalog); });
test.setTimeout(180_000);
async function routeModels(page: Page) {
  await page.route(url => fixtures.has(url.href), route => route.fulfill({ status: 200, contentType: 'application/octet-stream', headers: { 'access-control-allow-origin': '*' }, body: fixtures.get(route.request().url())! }));
}
async function attribute<T>(page: Page, key: string): Promise<T> { return JSON.parse((await bodyCanvas(page).getAttribute(key))!); }
async function dragTcp(page: Page) {
  const canvas = bodyCanvas(page);
  await page.getByRole('button', { name: '拖动法兰', exact: true }).click();
  await expect(canvas).toHaveAttribute('data-gizmo-target', 'tcp');
  await expect.poll(async () => (await attribute<Record<string, { x: number; y: number }>>(page, 'data-jog-handles')).X).toBeTruthy();
  const box = (await canvas.boundingBox())!, handle = (await attribute<Record<string, { x: number; y: number }>>(page, 'data-jog-handles')).X;
  const camera = new PerspectiveCamera(Number(await canvas.getAttribute('data-camera-fov')), Number(await canvas.getAttribute('data-camera-aspect')), .01, 100);
  camera.up.fromArray(await attribute<Vec3>(page, 'data-camera-up')); camera.position.fromArray(await attribute<Vec3>(page, 'data-camera-position'));
  camera.lookAt(new Vector3(...await attribute<Vec3>(page, 'data-camera-target'))); camera.updateMatrixWorld();
  const pose = await attribute<Pose>(page, 'data-jog-pose'), projected = new Vector3(...pose.position).multiplyScalar(.001).project(camera);
  const center = { x: box.x + (projected.x + 1) * box.width / 2, y: box.y + (1 - projected.y) * box.height / 2 };
  const length = Math.hypot(handle.x - center.x, handle.y - center.y);
  await page.mouse.move(handle.x, handle.y); await page.mouse.down();
  await expect(canvas).toHaveAttribute('data-jog-dragging', 'true');
  await page.mouse.move(handle.x + (handle.x - center.x) / length * 3, handle.y + (handle.y - center.y) / length * 3, { steps: 3 });
  await page.mouse.up();
  await expect(canvas).toHaveAttribute('data-jog-dragging', 'false');
  await expect(page.getByRole('status', { name: '拖动状态', exact: true })).not.toContainText(/正在检查|已保持/);
  await expect.poll(async () => Math.hypot(...(await bodyPose(page)).position.map((n, i) => n - pose.position[i]))).toBeGreaterThan(.1);
}

test('every downloadable model supports actual TCP jogging, joint frames, URDF parameters, immutable IK previews, explicit point updates and endpoint mesh checks', async ({ page }) => {
  const errors = observeBrowser(page); await routeModels(page); await openWorkspace(page);
  for (const model of catalog.models.filter(item => !item.bundledMotionModelId)) {
    await selectRobot(page, model.id);
    await page.getByRole('button', { name: '示例起始姿态', exact: true }).click();
    await expect.poll(() => bodyJoints(page)).toEqual([20, 15, -25, 25, 35, 10]);
    await page.getByRole('checkbox', { name: '显示关节坐标系', exact: true }).check();
    await expect(bodyCanvas(page)).toHaveAttribute('data-joint-frames-visible', 'true');
    await page.locator('.dh-details summary').click();
    await expect(page.getByRole('table', { name: 'URDF 运动学参数', exact: true })).toBeVisible();
    await expect(page.getByRole('table', { name: 'URDF 运动学参数', exact: true })).toContainText('固定');
    await dragTcp(page);
    const point = await captureCurrent(page);
    const recorded = await page.locator('.target-inspector').getAttribute('data-world-pose');
    const recordedLocal = await page.locator('.target-inspector').getAttribute('data-local-pose');
    const targetText = await page.locator('.target-robtarget').inputValue();
    const originalTargetJoints = (await page.getByLabel('目标关节', { exact: true }).textContent())!.split(',').map(Number);
    await setJoint(page, 1, 21);
    await expect.poll(async () => (await bodyJoints(page))[0]).toBe(21);
    const committed = await bodyJoints(page);
    await selectPoint(page, point);
    const rows = page.locator('.endpoint-solution');
    await expect.poll(() => rows.count()).toBeGreaterThan(1);
    await expect(rows.first()).not.toHaveAttribute('data-collision-state', /pending|error/);
    await expect(page.getByLabel('目标 robconf', { exact: true })).toHaveCount(0);
    const preview = rows.last(), expectedJoints = JSON.parse((await preview.getAttribute('data-endpoint-joints'))!) as number[];
    expect(Math.max(...expectedJoints.map((value, index) => Math.abs(value - originalTargetJoints[index])))).toBeGreaterThan(1);
    await preview.getByRole('button', { name: /^预览姿态/ }).scrollIntoViewIfNeeded();
    const beforeScroll = await page.locator('.control-panel').evaluate(element => element.scrollTop);
    await preview.getByRole('button', { name: /^预览姿态/ }).click();
    await expect.poll(async () => Math.max(...(await bodyJoints(page)).map((n, i) => Math.abs(n - expectedJoints[i])))).toBeLessThan(.00001);
    expect(await page.locator('.control-panel').evaluate(element => element.scrollTop)).toBeGreaterThanOrEqual(beforeScroll - 2);
    await expect(page.locator('.target-inspector')).toHaveAttribute('data-world-pose', recorded!);
    await expect(page.locator('.target-robtarget')).toHaveValue(targetText);
    await expect(page.getByRole('button', { name: '用当前姿态更新点', exact: true })).toBeEnabled();
    await page.getByRole('button', { name: '退出预览', exact: true }).click();
    await expect.poll(() => bodyJoints(page)).toEqual(committed);
    // An explicit update chooses this model's candidate joints. The target's
    // coordinate data and the real robot stay intact, without ABB robconf.
    await preview.getByRole('button', { name: /^预览姿态/ }).click();
    await expect.poll(async () => Math.max(...(await bodyJoints(page)).map((n, i) => Math.abs(n - expectedJoints[i])))).toBeLessThan(.00001);
    await page.getByRole('button', { name: '用当前姿态更新点', exact: true }).click();
    await expect(page.getByRole('status', { name: '端点预览', exact: true })).toHaveCount(0);
    await expect.poll(() => bodyJoints(page)).toEqual(committed);
    await expect(page.locator('.target-inspector')).toHaveAttribute('data-target-point-id', point.id);
    await expect(page.locator('.target-inspector')).toHaveAttribute('data-world-pose', recorded!);
    await expect(page.locator('.target-inspector')).toHaveAttribute('data-local-pose', recordedLocal!);
    if (model.brand === 'FANUC') {
      // FANUC's P record contains joints, so explicit branch selection changes
      // its three-decimal declaration while the Cartesian target stays fixed.
      await expect(page.locator('.target-robtarget')).not.toHaveValue(targetText);
      const declaration = await page.locator('.target-robtarget').inputValue();
      const joints = [...declaration.matchAll(/\bJ([1-6])=(-?\d+(?:\.\d+)?) deg/g)];
      expect(joints.map(match => Number(match[1]))).toEqual([1, 2, 3, 4, 5, 6]);
      expect(Math.max(...joints.map((match, index) => Math.abs(Number(match[2]) - expectedJoints[index])))).toBeLessThan(.000501);
    } else {
      // Yaskawa's C record is Cartesian and retains the same target pose.
      await expect(page.locator('.target-robtarget')).toHaveValue(targetText);
    }
    await expect(page.getByLabel('目标 robconf', { exact: true })).toHaveCount(0);
    await expect.poll(async () => {
      const recordedJoints = (await page.getByLabel('目标关节', { exact: true }).textContent())!.split(',').map(Number);
      return Math.max(...recordedJoints.map((value, i) => Math.abs(value - expectedJoints[i])));
    }).toBeLessThan(.000001);
    await selectRobotObject(page);
  }
  expect(errors).toEqual([]);
});
