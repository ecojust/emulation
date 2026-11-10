import { expect, test, type Page } from '@playwright/test';
import { PerspectiveCamera, Vector3 } from 'three';
import type { Vec3 } from '../../src/robot/types';
import { bodyJoints, bodyPose, observeBrowser, openScene, openWorkspace, scene } from './workspace-helpers';

test.setTimeout(90_000);
type Point = { x: number; y: number };
async function attribute<T>(page: Page, key: string): Promise<T> { return JSON.parse((await scene(page).getAttribute(key))!); }
async function zoneHandle(page: Page, id: string, axis: string) {
  await expect(scene(page)).toHaveAttribute('data-gizmo-target', 'zone');
  await expect(scene(page)).toHaveAttribute('data-zone-id', id);
  await expect.poll(async () => (await attribute<Record<string, Point>>(page, 'data-zone-handles'))[axis]).toBeTruthy();
  return (await attribute<Record<string, Point>>(page, 'data-zone-handles'))[axis];
}
async function projectedCenter(page: Page): Promise<Point> {
  const canvas = scene(page), box = (await canvas.boundingBox())!;
  const camera = new PerspectiveCamera(Number(await canvas.getAttribute('data-camera-fov')), Number(await canvas.getAttribute('data-camera-aspect')), .01, 12);
  camera.up.fromArray(await attribute<Vec3>(page, 'data-camera-up'));
  camera.position.fromArray(await attribute<Vec3>(page, 'data-camera-position'));
  camera.lookAt(new Vector3(...await attribute<Vec3>(page, 'data-camera-target')));
  camera.updateMatrixWorld();
  const position = await attribute<Vec3>(page, 'data-zone-position-mm');
  const point = new Vector3(...position).multiplyScalar(.001).project(camera);
  return { x: box.x + (point.x + 1) * box.width / 2, y: box.y + (1 - point.y) * box.height / 2 };
}
async function beginDrag(page: Page, id: string, pixels: number) {
  const point = await zoneHandle(page, id, 'X'), center = await projectedCenter(page);
  const magnitude = Math.hypot(point.x - center.x, point.y - center.y);
  await page.mouse.move(point.x, point.y);
  await page.mouse.down();
  await expect(scene(page)).toHaveAttribute('data-zone-dragging', 'true');
  await expect(scene(page)).toHaveAttribute('data-zone-axis', 'X');
  await page.mouse.move(point.x + (point.x - center.x) / magnitude * pixels, point.y + (point.y - center.y) / magnitude * pixels, { steps: 5 });
}

test('real world-X zone dragging updates box attributes without moving the robot or camera and releases on view or object changes', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const originalPose = await bodyPose(page), originalJoints = await bodyJoints(page);
  await page.getByRole('button', { name: '正视', exact: true }).click();
  const zone = page.locator('[data-object-kind="zone"]').first();
  const id = (await zone.getAttribute('data-object-id'))!;
  await zone.click();
  await zoneHandle(page, id, 'X');
  const initial = await attribute<Vec3>(page, 'data-zone-position-mm');
  const camera = await attribute<Vec3>(page, 'data-camera-position');
  await beginDrag(page, id, 12);
  await page.mouse.up();
  await expect(scene(page)).toHaveAttribute('data-zone-dragging', 'false');
  await expect(scene(page)).toHaveAttribute('data-orbit-enabled', 'true');
  const after = await attribute<Vec3>(page, 'data-zone-position-mm');
  expect(Math.abs(after[0] - initial[0])).toBeGreaterThan(5);
  expect(after[1]).toBeCloseTo(initial[1], 7);
  expect(after[2]).toBeCloseTo(initial[2], 7);
  for (const [index, axis] of ['X', 'Y', 'Z'].entries()) expect(Number(await page.getByRole('spinbutton', { name: `危险区中心 ${axis}`, exact: true }).inputValue())).toBeCloseTo(after[index], 7);
  await expect.poll(async () => {
    const actual = (await attribute<{ id: string; minMm: number[]; maxMm: number[] }[]>(page, 'data-danger-zones')).find(item => item.id === id)!;
    return Math.max(...actual.minMm.map((value, i) => Math.abs((value + actual.maxMm[i]) / 2 - after[i])));
  }).toBeLessThan(1e-6);
  expect(await bodyPose(page)).toEqual(originalPose);
  expect(await bodyJoints(page)).toEqual(originalJoints);
  expect(Math.hypot(...(await attribute<Vec3>(page, 'data-camera-position')).map((value, i) => value - camera[i]))).toBeLessThan(1e-8);
  await expect(scene(page)).toHaveAttribute('data-jog-dragging', 'false');
  await page.screenshot({ path: '/tmp/emulation-zone-drag.png' });

  // Change view with keyboard while the actual pointer is still held. This
  // exercises cancellation rather than an ordinary mouse-up ending the drag.
  await beginDrag(page, id, 4);
  await page.getByRole('button', { name: '程序', exact: true }).focus();
  await page.keyboard.press('Enter');
  await expect(scene(page)).toHaveAttribute('data-zone-dragging', 'false');
  await expect(scene(page)).toHaveAttribute('data-orbit-enabled', 'true');
  await page.mouse.up();
  await openScene(page);
  await zoneHandle(page, id, 'X');
  await beginDrag(page, id, 4);
  await page.getByRole('button', { name: '选择机器人', exact: true }).focus();
  await page.keyboard.press('Enter');
  await expect(scene(page)).toHaveAttribute('data-zone-dragging', 'false');
  await expect(scene(page)).toHaveAttribute('data-gizmo-target', 'none');
  await page.mouse.up();
  await zone.click();
  await beginDrag(page, id, 4);
  await page.getByRole('button', { name: '删除危险区', exact: true }).focus();
  await page.keyboard.press('Enter');
  await expect(scene(page)).toHaveAttribute('data-zone-dragging', 'false');
  await expect(scene(page)).toHaveAttribute('data-gizmo-target', 'none');
  await expect.poll(async () => attribute(page, 'data-danger-zones')).toEqual([]);
  await page.mouse.up();
  expect(await bodyPose(page)).toEqual(originalPose);
  expect(await bodyJoints(page)).toEqual(originalJoints);
  await expect(scene(page)).toHaveAttribute('data-orbit-enabled', 'true');
  expect(errors).toEqual([]);
});
