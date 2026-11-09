import { expect, test, type Locator, type Page } from '@playwright/test';
import { PerspectiveCamera, Vector3 } from 'three';
import { quaternionDistanceDeg } from '../../src/robot/kinematics';
import type { Pose, Quaternion, Vec3 } from '../../src/robot/types';
import { expectCopyState } from './workspace-helpers';

test.setTimeout(60_000);
type Point = { x: number; y: number };
const scene = (page: Page) => page.getByLabel('ABB IRB 1100 三维仿真场景');
async function jsonAttribute<T>(canvas: Locator, name: string): Promise<T> { return JSON.parse((await canvas.getAttribute(name))!); }
async function open(page: Page) {
  const errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  await page.goto('/');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await expect(scene(page)).toHaveAttribute('data-camera-distance', /\d/);
  return errors;
}
async function tcp(page: Page): Promise<Vec3> {
  const values: number[] = [];
  for (const axis of ['X', 'Y', 'Z']) values.push(Number(await page.getByLabel('当前 TCP', { exact: true }).locator(`[data-axis="${axis}"] strong`).textContent()));
  return values as Vec3;
}
function distance(a: number[], b: number[]) { return Math.hypot(...a.map((value, i) => value - b[i])); }
async function handle(page: Page, axis: 'X' | 'Y' | 'Z'): Promise<Point> {
  await expect.poll(async () => (await jsonAttribute<Record<string, Point>>(scene(page), 'data-jog-handles'))[axis]).toBeTruthy();
  return (await jsonAttribute<Record<string, Point>>(scene(page), 'data-jog-handles'))[axis];
}
async function projectedTcp(page: Page): Promise<Point> {
  const canvas = scene(page), box = (await canvas.boundingBox())!;
  const camera = new PerspectiveCamera(Number(await canvas.getAttribute('data-camera-fov')), Number(await canvas.getAttribute('data-camera-aspect')), .01, 12);
  camera.up.fromArray(await jsonAttribute<Vec3>(canvas, 'data-camera-up'));
  camera.position.fromArray(await jsonAttribute<Vec3>(canvas, 'data-camera-position'));
  camera.lookAt(new Vector3(...await jsonAttribute<Vec3>(canvas, 'data-camera-target')));
  camera.updateMatrixWorld();
  const position = (await jsonAttribute<Pose>(canvas, 'data-jog-pose')).position;
  const point = new Vector3(...position).multiplyScalar(.001).project(camera);
  return { x: box.x + (point.x + 1) * box.width / 2, y: box.y + (1 - point.y) * box.height / 2 };
}
async function settled(page: Page) {
  await expect(scene(page)).toHaveAttribute('data-jog-dragging', 'false');
  await expect(page.getByRole('status', { name: '拖动状态', exact: true })).not.toContainText('正在检查');
  await expect.poll(async () => distance((await jsonAttribute<Pose>(scene(page), 'data-jog-pose')).position, await tcp(page))).toBeLessThan(.03);
}
async function translate(page: Page, axis: 'X' | 'Y' | 'Z', pixels = 8) {
  await expect(scene(page)).toHaveAttribute('data-jog-mode', 'translate');
  const point = await handle(page, axis), center = await projectedTcp(page);
  const magnitude = Math.hypot(point.x - center.x, point.y - center.y);
  const end = { x: point.x + (point.x - center.x) / magnitude * pixels, y: point.y + (point.y - center.y) / magnitude * pixels };
  await page.mouse.move(point.x, point.y);
  await page.mouse.down();
  await expect(scene(page)).toHaveAttribute('data-jog-dragging', 'true');
  await expect(scene(page)).toHaveAttribute('data-jog-axis', axis);
  await page.mouse.move(end.x, end.y, { steps: 8 });
  await page.mouse.up();
  await settled(page);
}

test('wheel zoom interpolates, settles and respects both distance bounds', async ({ page }) => {
  const errors = await open(page), canvas = scene(page), box = (await canvas.boundingBox())!;
  await page.mouse.move(box.x + box.width * .4, box.y + box.height * .6);
  const initial = Number(await canvas.getAttribute('data-camera-distance'));
  const pageY = await page.evaluate(() => window.scrollY);
  let listenerReady!: () => void;
  const ready = new Promise<void>(resolve => { listenerReady = resolve; });
  await page.exposeFunction('zoomSamplingReady', () => listenerReady());
  const sampling = canvas.evaluate(element => new Promise<number[]>(resolve => {
    // Start on the actual event, so CDP input latency cannot consume our samples.
    element.addEventListener('wheel', async () => {
      const values = [Number(element.getAttribute('data-camera-distance'))];
      for (let i = 0; i < 45; i++) {
        await new Promise<void>(frame => requestAnimationFrame(() => frame()));
        values.push(Number(element.getAttribute('data-camera-distance')));
      }
      resolve(values);
    }, { once: true, capture: true });
    void (window as unknown as { zoomSamplingReady: () => Promise<void> }).zoomSamplingReady();
  }));
  await ready;
  await page.mouse.wheel(0, -160);
  const values = await sampling;
  const target = Number(await canvas.getAttribute('data-zoom-target'));
  expect(Math.min(...values)).toBeLessThan(initial - .05);
  // An abrupt jump has no strict intermediate sample. Counting a minimum number
  // of distinct frames would instead impose an FPS requirement on the CI GPU.
  expect(values.some(value => value > target + 1e-5 && value < initial - 1e-5)).toBe(true);
  for (const value of values) {
    expect(Number.isFinite(value)).toBe(true);
    expect(value).toBeGreaterThanOrEqual(target - 1e-8);
    expect(value).toBeLessThanOrEqual(initial + 1e-8);
  }
  for (let i = 1; i < values.length; i++) expect(values[i]).toBeLessThanOrEqual(values[i - 1] + 1e-6);
  await expect.poll(async () => Math.abs(Number(await canvas.getAttribute('data-camera-distance')) - Number(await canvas.getAttribute('data-zoom-target')))).toBeLessThan(.0001);
  for (let i = 0; i < 10; i++) await page.mouse.wheel(0, -10000);
  await expect.poll(async () => Number(await canvas.getAttribute('data-camera-distance'))).toBeLessThan(.201);
  expect(Number(await canvas.getAttribute('data-camera-distance'))).toBeGreaterThanOrEqual(.2 - 1e-8);
  for (let i = 0; i < 15; i++) await page.mouse.wheel(0, 10000);
  await expect.poll(async () => Number(await canvas.getAttribute('data-camera-distance'))).toBeGreaterThan(4.499);
  expect(Number(await canvas.getAttribute('data-camera-distance'))).toBeLessThanOrEqual(4.5 + 1e-8);
  expect(await page.evaluate(() => window.scrollY)).toBe(pageY);
  expect(errors).toEqual([]);
});

test('a real world-X flange drag moves the robot through IK while the camera remains fixed', async ({ page }) => {
  const errors = await open(page);
  await page.getByRole('button', { name: '正视', exact: true }).click();
  await page.getByRole('button', { name: '拖动法兰', exact: true }).click();
  await handle(page, 'X');
  const initial = await tcp(page), camera = await jsonAttribute<Vec3>(scene(page), 'data-camera-position');
  const before = await scene(page).screenshot();
  await translate(page, 'X');
  const after = await tcp(page);
  expect(Math.abs(after[0] - initial[0])).toBeGreaterThan(1);
  expect(Math.abs(after[1] - initial[1])).toBeLessThan(.03);
  expect(Math.abs(after[2] - initial[2])).toBeLessThan(.03);
  expect(distance(await jsonAttribute<Vec3>(scene(page), 'data-camera-position'), camera)).toBeLessThan(1e-8);
  expect((await scene(page).screenshot()).equals(before)).toBe(false);
  await expectCopyState(page, false);
  await page.screenshot({ path: '/tmp/emulation-jog-world-x.png', fullPage: true });
  expect(errors).toEqual([]);
});

test('an unreachable cursor target holds the last valid robot pose and the gizmo snaps back on release', async ({ page }) => {
  const errors = await open(page);
  await page.getByRole('button', { name: '正视', exact: true }).click();
  await page.getByRole('button', { name: '拖动法兰', exact: true }).click();
  const point = await handle(page, 'X'), box = (await scene(page).boundingBox())!;
  const initial = await tcp(page), camera = await jsonAttribute<Vec3>(scene(page), 'data-camera-position');
  await page.mouse.move(point.x, point.y);
  await page.mouse.down();
  await expect(scene(page)).toHaveAttribute('data-jog-dragging', 'true');
  await page.mouse.move(box.x + box.width - 12, point.y);
  await expect(page.getByRole('status', { name: '拖动状态', exact: true })).toContainText('已保持上一个有效姿态');
  expect(distance(await tcp(page), initial)).toBeLessThan(.03);
  await page.mouse.up();
  await settled(page);
  expect(distance(await tcp(page), initial)).toBeLessThan(.03);
  expect(distance(await jsonAttribute<Vec3>(scene(page), 'data-camera-position'), camera)).toBeLessThan(1e-8);
  await page.screenshot({ path: '/tmp/emulation-jog-unreachable.png', fullPage: true });
  expect(errors).toEqual([]);
});

test('tool-space rotation holds the TCP fixed and tool-Z translation follows the rotated flange axis', async ({ page }) => {
  const errors = await open(page);
  await page.getByRole('button', { name: '拖动法兰', exact: true }).click();
  await page.getByRole('button', { name: '工具坐标', exact: true }).click();
  await page.getByRole('button', { name: '旋转', exact: true }).click();
  await expect(page.getByRole('button', { name: '旋转', exact: true })).toHaveAttribute('aria-pressed', 'true');
  await expect(scene(page)).toHaveAttribute('data-jog-mode', 'rotate');
  await expect(scene(page)).toHaveAttribute('data-jog-space', 'local');
  const point = await handle(page, 'Z'), center = await projectedTcp(page);
  const before = await jsonAttribute<Pose>(scene(page), 'data-jog-pose');
  const camera = await jsonAttribute<Vec3>(scene(page), 'data-camera-position');
  const x = point.x - center.x, y = point.y - center.y, angle = .12;
  await page.mouse.move(point.x, point.y);
  await page.mouse.down();
  await expect(scene(page)).toHaveAttribute('data-jog-dragging', 'true');
  await expect(scene(page)).toHaveAttribute('data-jog-axis', 'Z');
  await page.mouse.move(center.x + x * Math.cos(angle) - y * Math.sin(angle), center.y + x * Math.sin(angle) + y * Math.cos(angle), { steps: 8 });
  await page.mouse.up();
  await settled(page);
  const rotated = await jsonAttribute<Pose>(scene(page), 'data-jog-pose');
  expect(quaternionDistanceDeg(before.orientation, rotated.orientation)).toBeGreaterThan(.2);
  expect(distance(before.position, await tcp(page))).toBeLessThan(.03);
  expect(distance(await jsonAttribute<Vec3>(scene(page), 'data-camera-position'), camera)).toBeLessThan(1e-8);
  await page.getByRole('button', { name: '平移', exact: true }).click();
  await translate(page, 'Z', 6);
  const after = await jsonAttribute<Pose>(scene(page), 'data-jog-pose');
  const delta = after.position.map((value, i) => value - rotated.position[i]);
  const [w, qx, qy, qz] = rotated.orientation as Quaternion;
  const toolZ = new Vector3(2 * (qx * qz + w * qy), 2 * (qy * qz - w * qx), 1 - 2 * (qx * qx + qy * qy));
  expect(Math.hypot(...delta)).toBeGreaterThan(1);
  expect(new Vector3(...delta).cross(toolZ).length()).toBeLessThan(.03);
  expect(quaternionDistanceDeg(rotated.orientation, after.orientation)).toBeLessThan(.01);
  expect(distance(await tcp(page), after.position)).toBeLessThan(.03);
  await page.screenshot({ path: '/tmp/emulation-jog-tool-space.png', fullPage: true });
  expect(errors).toEqual([]);
});
