import { expect, test, type Page } from '@playwright/test';
import { Quaternion, Vector3 } from 'three';
import type { Pose, Vec3 } from '../../src/robot/types';
import type { SimulationProject } from '../../src/projects/schema';
import { forwardKinematics } from '../../src/robot/kinematics';
import { getRobotModel } from '../../src/robot/model';
import { bodyCanvas, bodyPose, createPath, observeBrowser, openPathMenu, openScene, openWorkspace, pathObjects, selectPath } from './workspace-helpers';
import { generateSource, newSource, readSource } from './programming-helpers';
import { waitForProjectScene } from './project-helpers';

// This end-to-end case includes import, real pointer picks, full IK, source
// generation, project cover capture/reopen and invalidation after a model edit.
test.setTimeout(180_000);
const MODEL = 'irb1100-4-0.58';
const originalSource = 'MODULE SurfaceUser\n  PROC Keep()\n  ENDPROC\nENDMODULE';
const objects = (page: Page, kind: string) => page.locator(`[data-object-kind="${kind}"]`);
const object = (page: Page, kind: string, id: string) => page.locator(`[data-object-kind="${kind}"][data-object-id="${id}"]`);
const region = (page: Page, pathId: string) => page.locator(`[data-surface-path-id="${pathId}"]`);
interface ProjectedAnchor { triangleIndex: number; barycentric: Vec3; position: Vec3; normal: Vec3; x: number; y: number }
interface SurfaceView { id: string; revision: number; triangles: number; projected: ProjectedAnchor[] }
async function surfaces(page: Page): Promise<SurfaceView[]> { return JSON.parse((await bodyCanvas(page).getAttribute('data-scene-surfaces'))!); }
async function scenePaths(page: Page): Promise<{ id: string; points: Pose[] }[]> { return JSON.parse((await bodyCanvas(page).getAttribute('data-scene-paths'))!); }
function planeObj(pose: Pose): string {
  const [w, x, y, z] = pose.orientation, rotation = new Quaternion(x, y, z, w), center = new Vector3(...pose.position);
  const vertices = [[-15, -15], [15, -15], [15, 15], [-15, 15]].map(([u, v]) => new Vector3(u, v, 0).applyQuaternion(rotation).add(center));
  return vertices.map(vertex => 'v ' + vertex.toArray().map(n => n.toFixed(9)).join(' ')).join('\n') + '\nf 1 2 3\nf 1 3 4\n';
}
async function importPlane(page: Page, pose: Pose) {
  await page.getByRole('button', { name: '导入模型', exact: true }).click();
  const dialog = page.getByRole('dialog', { name: '导入模型', exact: true });
  await dialog.getByLabel('模型文件', { exact: true }).setInputFiles({ name: 'surface-fixture.obj', mimeType: 'text/plain', buffer: Buffer.from(planeObj(pose)) });
  await dialog.getByRole('button', { name: '导入场景', exact: true }).click();
  await expect(dialog).toHaveCount(0); await expect(objects(page, 'surface')).toHaveCount(1);
  const id = (await objects(page, 'surface').getAttribute('data-object-id'))!;
  await expect.poll(async () => (await surfaces(page)).find(item => item.id === id)?.triangles).toBe(2);
  await page.getByRole('button', { name: '聚焦模型', exact: true }).click();
  await expect.poll(async () => (await surfaces(page)).find(item => item.id === id)?.projected.length ?? 0).toBeGreaterThanOrEqual(8);
  return id;
}
async function selectRegion(page: Page, surfaceId: string) {
  const pathId = await createPath(page);
  await (await openPathMenu(page, pathId)).getByRole('menuitem', { name: '点选区域', exact: true }).click();
  await expect(bodyCanvas(page)).toHaveAttribute('data-surface-picking', 'true');
  // These interior barycentric anchors form a convex four-corner patch with U
  // along the current tool X. Coordinates come from the renderer's raycaster;
  // actual pointer events still perform every user-visible pick.
  const choices: { triangle: number; weights: Vec3 }[] = [
    { triangle: 0, weights: [.7, .15, .15] }, { triangle: 0, weights: [.15, .7, .15] },
    { triangle: 1, weights: [.15, .7, .15] }, { triangle: 1, weights: [.15, .15, .7] },
  ];
  for (const [index, choice] of choices.entries()) {
    const view = (await surfaces(page)).find(item => item.id === surfaceId)!;
    const anchor = view.projected.find(item => item.triangleIndex === choice.triangle && item.barycentric.every((value, i) => Math.abs(value - choice.weights[i]) < 1e-5));
    expect(anchor).toBeTruthy();
    await page.mouse.click(anchor!.x, anchor!.y);
    await expect(region(page, pathId).getByLabel('表面点列表').locator('li')).toHaveCount(index + 1);
  }
  await region(page, pathId).getByRole('button', { name: '结束点选', exact: true }).click();
  await expect(bodyCanvas(page)).toHaveAttribute('data-surface-picking', 'false');
  for (let i = 1; i <= 4; i++) await region(page, pathId).getByRole('checkbox', { name: `S${String(i).padStart(3, '0')}设为控制点`, exact: true }).check();
  await expect(region(page, pathId)).toContainText('控制点 4 / 4');
  return pathId;
}
async function readProject(page: Page, id: string): Promise<SimulationProject> {
  return page.evaluate(projectId => new Promise<SimulationProject>((resolve, reject) => {
    const opening = indexedDB.open('emulation-simulation-projects', 2);
    opening.onerror = () => reject(opening.error);
    opening.onsuccess = () => {
      const db = opening.result, tx = db.transaction('projects', 'readonly'), request = tx.objectStore('projects').get(projectId);
      let value: SimulationProject;
      request.onsuccess = () => { value = request.result; };
      tx.oncomplete = () => { db.close(); resolve(value); };
      tx.onabort = () => { db.close(); reject(tx.error); };
    };
  }), id);
}
async function home(page: Page) {
  await page.getByRole('button', { name: '返回主页', exact: true }).click();
  await expect(page.getByRole('main', { name: '项目管理', exact: true })).toBeVisible();
}
function normalOf(pose: Pose): Vector3 { const [w, x, y, z] = pose.orientation; return new Vector3(0, 0, 1).applyQuaternion(new Quaternion(x, y, z, w)); }

test('imported surface picks generate a smooth reachable UV Path, persist with the mesh, and become non-executable after model edits without rewriting source', async ({ page }) => {
  const errors = observeBrowser(page); await openWorkspace(page);
  const projectId = (await page.locator('[data-project-id]').getAttribute('data-project-id'))!, initialPose = await bodyPose(page);
  // The test workpiece is deliberately near the existing TCP. Disable the
  // default demonstration exclusion box explicitly, as a user would.
  await objects(page, 'zone').first().click(); await page.getByRole('checkbox', { name: '显示干涉区域', exact: true }).uncheck();
  await newSource(page, 'user.mod', originalSource); await openScene(page);
  const surfaceId = await importPlane(page, initialPose), pathId = await selectRegion(page, surfaceId);
  const before = (await scenePaths(page)).find(item => item.id === pathId)!.points;
  await (await openPathMenu(page, pathId)).getByRole('menuitem', { name: '生成 UV 加工路径', exact: true }).click();
  const dialog = page.getByRole('dialog', { name: '生成 UV 加工路径', exact: true });
  await dialog.getByRole('spinbutton', { name: 'U 方向间距', exact: true }).fill('8');
  await dialog.getByRole('spinbutton', { name: 'V 方向间距', exact: true }).fill('8');
  await dialog.getByRole('button', { name: '生成路径', exact: true }).click();
  await expect(dialog.getByRole('status', { name: 'UV 路径结果', exact: true })).toHaveAttribute('data-uv-status', 'success', { timeout: 60_000 });
  const count = Number(await dialog.getByRole('status', { name: 'UV 路径结果', exact: true }).getAttribute('data-uv-points'));
  expect(count).toBeGreaterThan(4); expect(count).toBeLessThanOrEqual(256);
  await expect(pathObjects(page)).toHaveCount(1);
  expect((await scenePaths(page)).find(item => item.id === pathId)!.points).toEqual(before);
  await dialog.getByRole('button', { name: '预览路径', exact: true }).click();
  await expect(dialog).toHaveAttribute('data-surface-uv-mode', 'preview');
  await expect.poll(async () => (await scenePaths(page)).find(item => item.id === 'surface-preview')?.points.length).toBe(count);
  await page.screenshot({ path: '/tmp/emulation-surface-uv-preview.png' });
  await dialog.getByRole('button', { name: '创建新 Path', exact: true }).click();
  await expect(dialog).toHaveCount(0); await expect(pathObjects(page)).toHaveCount(2);
  const uvId = (await pathObjects(page).last().getAttribute('data-object-id'))!;
  expect((await scenePaths(page)).find(item => item.id === pathId)!.points).toEqual(before);
  const generated = await generateSource(page, uvId);
  expect((generated.match(/\bCONST robtarget\b/g) ?? []).length).toBe(count);
  expect((generated.match(/^\s*MoveL\s/gm) ?? []).length).toBe(count);
  await home(page);
  const saved = await readProject(page, projectId), program = saved.scene.models[MODEL].program;
  const surface = saved.scene.surfaces!.find(item => item.id === surfaceId)!, uv = program.paths.find(item => item.id === uvId)!, original = program.paths.find(item => item.id === pathId)!;
  expect(surface.mesh.positions).toHaveLength(18); expect(original.surface?.points).toHaveLength(4); expect(original.surface?.controlPointIds).toHaveLength(4);
  expect(uv.surface?.resolvedRevision).toBe(uv.surface?.revision); expect(uv.pointIds).toHaveLength(count);
  expect(saved.sources[MODEL].files.find(item => item.name === 'user.mod')?.text).toBe(originalSource);
  const points = uv.surface!.points, robot = getRobotModel(MODEL), expectedNormal = normalOf(initialPose);
  for (const [index, point] of points.entries()) {
    const normal = normalOf(point.pose), offset = new Vector3(...point.pose.position).sub(new Vector3(...initialPose.position));
    expect(Math.abs(offset.dot(expectedNormal))).toBeLessThan(1e-4);
    expect(normal.dot(new Vector3(...point.pick.normal))).toBeGreaterThan(1 - 1e-8);
    const { triangleIndex, barycentric } = point.pick;
    for (let axis = 0; axis < 3; axis++) {
      const coordinate = barycentric.reduce((sum, weight, corner) => sum + weight * surface.mesh.positions[triangleIndex * 9 + corner * 3 + axis], 0);
      expect(Math.abs(coordinate - point.pose.position[axis])).toBeLessThan(1e-5);
    }
    const solved = program.points.find(item => item.id === uv.pointIds[index])!, fk = forwardKinematics(robot, solved.joints).pose;
    expect(solved.pose).toEqual(point.pose); expect(solved.configuration).toHaveLength(4);
    expect(new Vector3(...fk.position).distanceTo(new Vector3(...point.pose.position))).toBeLessThan(.1);
    if (index) {
      const a = points[index - 1].pose.orientation, b = point.pose.orientation;
      expect(new Quaternion(a[1], a[2], a[3], a[0]).angleTo(new Quaternion(b[1], b[2], b[3], b[0])) * 180 / Math.PI).toBeLessThan(5.001);
    }
  }
  await page.locator(`[data-project-card="${projectId}"] .project-card-open`).click();
  // Reopening restores the saved Program view; wait for its editor before
  // checking the scene-only model status.
  expect(await readSource(page)).toBe(generated); await waitForProjectScene(page, MODEL);
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await expect(objects(page, 'surface')).toHaveCount(1); await expect(pathObjects(page)).toHaveCount(2);
  await expect.poll(async () => (await surfaces(page)).find(item => item.id === surfaceId)?.triangles).toBe(2);
  await object(page, 'surface', surfaceId).click();
  await page.getByRole('spinbutton', { name: '工件模型位置 X', exact: true }).fill('20');
  await page.getByRole('button', { name: '应用工件模型位姿', exact: true }).click();
  await expect.poll(async () => (await surfaces(page)).find(item => item.id === surfaceId)?.revision).toBe(1);
  const menu = await openPathMenu(page, uvId);
  await expect(menu.getByRole('menuitem', { name: '运行', exact: true })).toBeDisabled();
  await expect(menu.getByRole('menuitem', { name: '转换为程序', exact: true })).toBeDisabled();
  await page.keyboard.press('Escape');
  await object(page, 'surface', surfaceId).click({ button: 'right' });
  await page.getByRole('menuitem', { name: '删除工件模型', exact: true }).click();
  await page.getByRole('dialog', { name: '删除工件模型', exact: true }).getByRole('button', { name: '删除', exact: true }).click();
  await expect(objects(page, 'surface')).toHaveCount(0); await expect(pathObjects(page)).toHaveCount(2);
  await home(page);
  const deleted = await readProject(page, projectId), stale = deleted.scene.models[MODEL].program.paths.find(item => item.id === uvId)!;
  expect(deleted.scene.surfaces).toEqual([]); expect(stale.surface?.invalidated).toBe(true); expect(stale.pointIds).toEqual([]);
  expect(stale.surface?.points).toEqual(uv.surface?.points); expect(deleted.sources).toEqual(saved.sources);
  expect(errors).toEqual([]);
});

test('bad or cancelled imports and cancelling UV editing never add a partial workpiece or rewrite the picked region', async ({ page }) => {
  const errors = observeBrowser(page); await openWorkspace(page);
  const initial = await bodyPose(page);
  await page.getByRole('button', { name: '导入模型', exact: true }).click();
  const importing = page.getByRole('dialog', { name: '导入模型', exact: true });
  await importing.getByLabel('模型文件', { exact: true }).setInputFiles({ name: 'broken.obj', mimeType: 'text/plain', buffer: Buffer.from('v 0 0 0\nf 1 1 1') });
  await importing.getByRole('button', { name: '导入场景', exact: true }).click();
  await expect(importing.getByRole('alert')).toContainText('退化'); await expect(objects(page, 'surface')).toHaveCount(0);
  await importing.getByLabel('模型文件', { exact: true }).setInputFiles({ name: 'cancelled.obj', mimeType: 'text/plain', buffer: Buffer.from(planeObj(initial)) });
  await importing.getByRole('button', { name: '取消', exact: true }).click();
  await expect(importing).toHaveCount(0); await expect(objects(page, 'surface')).toHaveCount(0);
  const surfaceId = await importPlane(page, initial), pathId = await selectRegion(page, surfaceId);
  const before = (await scenePaths(page)).find(item => item.id === pathId)!.points;
  await (await openPathMenu(page, pathId)).getByRole('menuitem', { name: '生成 UV 加工路径', exact: true }).click();
  const dialog = page.getByRole('dialog', { name: '生成 UV 加工路径', exact: true });
  await dialog.getByRole('spinbutton', { name: 'U 方向间距', exact: true }).fill('0');
  await dialog.getByRole('button', { name: '生成路径', exact: true }).click();
  await expect(dialog.getByRole('status', { name: 'UV 路径结果', exact: true })).toHaveAttribute('data-uv-status', 'failure');
  await expect(dialog.getByRole('button', { name: '创建新 Path', exact: true })).toBeDisabled();
  await dialog.getByRole('button', { name: '关闭 UV 路径生成', exact: true }).click();
  await expect(dialog).toHaveCount(0); await expect(pathObjects(page)).toHaveCount(1);
  expect((await scenePaths(page)).find(item => item.id === pathId)!.points).toEqual(before);
  await selectPath(page, pathId); await expect(region(page, pathId).getByLabel('表面点列表').locator('li')).toHaveCount(4);
  await object(page, 'surface', surfaceId).click();
  await page.getByRole('spinbutton', { name: '工件模型位置 X', exact: true }).fill('20');
  await page.getByRole('button', { name: '应用工件模型位姿', exact: true }).click();
  await expect.poll(async () => (await surfaces(page)).find(item => item.id === surfaceId)?.revision).toBe(1);
  await page.getByRole('button', { name: '聚焦模型', exact: true }).click();
  await expect.poll(async () => (await surfaces(page)).find(item => item.id === surfaceId)?.projected.length).toBe(8);
  await selectPath(page, pathId);
  const inspector = region(page, pathId), reset = inspector.getByRole('group', { name: '确认重新点选区域', exact: true });
  await expect(inspector.getByRole('button', { name: '继续点选区域', exact: true })).toBeDisabled();
  await expect(inspector.locator('.surface-error')).toBeVisible();
  await inspector.getByRole('button', { name: '重新点选区域', exact: true }).click();
  await reset.getByRole('button', { name: '保留现有区域', exact: true }).click();
  await expect(reset).toHaveCount(0);
  await expect(inspector.getByLabel('表面点列表').locator('li')).toHaveCount(4);
  expect((await scenePaths(page)).find(item => item.id === pathId)!.points).toEqual(before);
  await inspector.getByRole('button', { name: '重新点选区域', exact: true }).click();
  await reset.getByRole('button', { name: '清空并重新点选', exact: true }).click();
  await expect(inspector.getByLabel('表面点列表').locator('li')).toHaveCount(0);
  await expect(bodyCanvas(page)).toHaveAttribute('data-surface-picking', 'true');
  await expect(inspector.locator('.surface-error')).toHaveCount(0);
  const anchor = (await surfaces(page)).find(item => item.id === surfaceId)!.projected[0];
  await page.mouse.click(anchor.x, anchor.y);
  await expect(inspector.getByLabel('表面点列表').locator('li')).toHaveCount(1);
  await inspector.getByRole('button', { name: '结束点选', exact: true }).click();
  await expect(bodyCanvas(page)).toHaveAttribute('data-surface-picking', 'false');
  await expect(inspector).toContainText('控制点 0 / 4');
  await expect(pathObjects(page)).toHaveCount(1); await expect(objects(page, 'surface')).toHaveCount(1);
  expect(errors).toEqual([]);
});
