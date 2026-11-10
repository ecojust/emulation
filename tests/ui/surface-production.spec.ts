import { expect, test, type Page } from '@playwright/test';
import { Quaternion, Vector3 } from 'three';
import type { SimulationProject } from '../../src/projects/schema';
import { forwardKinematics } from '../../src/robot/kinematics';
import { getRobotModel } from '../../src/robot/model';
import type { Pose } from '../../src/robot/types';
import type { SceneSurface, SurfaceAnchor } from '../../src/surface/types';
import { evaluateSurfaceAnchor, validateSceneSurface } from '../../src/surface/validation';
import { baseFlangeToWorldTcp, identityPose, pathPointToWorld, worldPointToPath } from '../../src/workspace/frame-graph';
import type { ProgramPath } from '../../src/workspace/program';
import { validateSurfacePath } from '../../src/workspace/surface-path';
import { generateSource, newSource, readSource } from './programming-helpers';
import { openDefaultProject, waitForProjectScene } from './project-helpers';
import { observeBrowser, openPathMenu, openScene, pathObjects } from './workspace-helpers';

// Real pointer picking is covered against the development build. This separate
// acceptance test starts from a persisted region so the production worker and
// source editor are exercised without relying on development-only diagnostics.
test.skip(process.env.EMULATION_UI_PREVIEW !== '1', 'Requires the bundled production preview.');
test.setTimeout(180_000);
const MODEL = 'irb1100-4-0.58', SURFACE_ID = 'production-surface', REGION_ID = 'production-region';
const ORIGINAL_SOURCE = 'MODULE ProductionUser\n  PROC Keep()\n  ENDPROC\nENDMODULE';

async function returnHome(page: Page) {
  await page.getByRole('button', { name: '返回主页', exact: true }).click();
  await expect(page.getByRole('main', { name: '项目管理', exact: true })).toBeVisible();
}
async function readProject(page: Page, id: string): Promise<SimulationProject> {
  return page.evaluate(projectId => new Promise<SimulationProject>((resolve, reject) => {
    const opening = indexedDB.open('emulation-simulation-projects', 2);
    opening.onerror = () => reject(opening.error);
    opening.onsuccess = () => {
      const db = opening.result, tx = db.transaction('projects', 'readonly');
      const request = tx.objectStore('projects').get(projectId);
      let value: SimulationProject;
      request.onsuccess = () => { value = request.result; };
      tx.oncomplete = () => { db.close(); resolve(value); };
      tx.onabort = () => { db.close(); reject(tx.error); };
    };
  }), id);
}
async function seedProject(page: Page, project: SimulationProject) {
  await page.evaluate(({ value, summary }) => new Promise<void>((resolve, reject) => {
    const opening = indexedDB.open('emulation-simulation-projects', 2);
    opening.onerror = () => reject(opening.error);
    opening.onsuccess = () => {
      const db = opening.result, tx = db.transaction(['projects', 'summaries'], 'readwrite');
      tx.objectStore('projects').put(value); tx.objectStore('summaries').put(summary);
      tx.oncomplete = () => { db.close(); resolve(); };
      tx.onabort = () => { db.close(); reject(tx.error); };
    };
  }), { value: project, summary: {
    id: project.id, name: project.name, activeModelId: project.scene.activeModelId,
    robotPresent: project.scene.models[project.scene.activeModelId].robotPresent,
    thumbnail: project.thumbnail, updatedAt: project.updatedAt, revision: project.revision,
  } });
}
function normalOf(pose: Pose) {
  const [w, x, y, z] = pose.orientation;
  return new Vector3(0, 0, 1).applyQuaternion(new Quaternion(x, y, z, w));
}
function withSurfaceRegion(project: SimulationProject): { project: SimulationProject; tcp: Pose } {
  const saved = project.scene.models[MODEL], robot = getRobotModel(MODEL);
  const tcp = baseFlangeToWorldTcp(saved.graph, forwardKinematics(robot, saved.joints).pose);
  const [w, x, y, z] = tcp.orientation, rotation = new Quaternion(x, y, z, w);
  const vertices = [[-15, -15], [15, -15], [15, 15], [-15, 15]].map(([u, v]) =>
    new Vector3(u, v, 0).applyQuaternion(rotation).add(new Vector3(...tcp.position)).toArray());
  const normal = normalOf(tcp).toArray();
  const surface: SceneSurface = validateSceneSurface({
    id: SURFACE_ID, name: 'Production plane', revision: 0, visible: true, pose: identityPose(),
    source: { name: 'production-plane.obj', format: 'obj', unit: 'mm' },
    mesh: { positions: [0, 1, 2, 0, 2, 3].flatMap(index => vertices[index]), normals: Array.from({ length: 6 }, () => normal).flat() },
  });
  const anchors: SurfaceAnchor[] = [
    { triangleIndex: 0, barycentric: [.7, .15, .15] }, { triangleIndex: 0, barycentric: [.15, .7, .15] },
    { triangleIndex: 1, barycentric: [.15, .7, .15] }, { triangleIndex: 1, barycentric: [.15, .15, .7] },
  ];
  const samples = anchors.map(anchor => {
    const picked = evaluateSurfaceAnchor(surface, anchor);
    return {
      pose: worldPointToPath(saved.graph, REGION_ID, { position: picked.position, orientation: tcp.orientation }),
      pick: { ...anchor, ...picked, surfaceId: surface.id, surfaceRevision: surface.revision },
    };
  });
  const ids = ['production-corner-1', 'production-corner-2', 'production-corner-3', 'production-corner-4'];
  // Only geometry is seeded: all executable joints and robot configurations
  // must still come from the production worker through the UI below.
  const region: ProgramPath = { id: REGION_ID, name: 'Production region', pointIds: [], surface: validateSurfacePath({
    kind: 'region', revision: 0, surfaceId: surface.id, surfaceRevision: surface.revision,
    points: samples.map((sample, index) => ({ ...sample, id: ids[index], name: `S00${index + 1}` })), controlPointIds: ids,
  }) };
  const program = { ...saved.program, revision: saved.program.revision + 1,
    nextPathNumber: saved.program.nextPathNumber + 1, paths: [...saved.program.paths, region] };
  const seeded: SimulationProject = { ...project, revision: project.revision + 1, updatedAt: new Date().toISOString(), scene: {
    ...project.scene, activeView: 'scene', surfaces: [surface],
    // The acceptance fixture explicitly disables the demonstration box, just
    // as the user can. Production planning still checks every enabled zone.
    zones: project.scene.zones.map(zone => ({ ...zone, enabled: false })),
    models: { ...project.scene.models, [MODEL]: { ...saved, program } },
  } };
  return { project: seeded, tcp };
}

test('bundled surface workers generate, preview and persist a reachable UV program from a saved region', async ({ page }) => {
  const errors = observeBrowser(page), workers: string[] = [];
  page.on('worker', worker => workers.push(worker.url()));
  const projectId = await openDefaultProject(page);
  await waitForProjectScene(page, MODEL);
  await newSource(page, 'production-user.mod', ORIGINAL_SOURCE); await openScene(page); await returnHome(page);
  const fixture = withSurfaceRegion(await readProject(page, projectId));
  await seedProject(page, fixture.project);
  await page.locator(`[data-project-card="${projectId}"] .project-card-open`).click();
  await waitForProjectScene(page, MODEL);
  await expect(page.locator('[data-object-kind="surface"]')).toHaveCount(1);
  await expect(pathObjects(page)).toHaveCount(1);
  await (await openPathMenu(page, REGION_ID)).getByRole('menuitem', { name: '生成 UV 加工路径', exact: true }).click();
  const dialog = page.getByRole('dialog', { name: '生成 UV 加工路径', exact: true });
  await dialog.getByRole('spinbutton', { name: 'U 方向间距', exact: true }).fill('8');
  await dialog.getByRole('spinbutton', { name: 'V 方向间距', exact: true }).fill('8');
  await dialog.getByRole('button', { name: '生成路径', exact: true }).click();
  const result = dialog.getByRole('status', { name: 'UV 路径结果', exact: true });
  await expect(result).toHaveAttribute('data-uv-status', 'success', { timeout: 60_000 });
  const count = Number(await result.getAttribute('data-uv-points'));
  expect(count).toBeGreaterThan(4); expect(count).toBeLessThanOrEqual(256);
  expect(workers.some(url => /\/assets\/generate\.worker-[^/]+\.js$/.test(new URL(url).pathname))).toBe(true);
  expect(workers.some(url => /\/assets\/grinding\.worker-[^/]+\.js$/.test(new URL(url).pathname))).toBe(true);
  await dialog.getByRole('button', { name: '预览路径', exact: true }).click();
  await expect(dialog).toHaveAttribute('data-surface-uv-mode', 'preview');
  await expect(pathObjects(page)).toHaveCount(1);
  await page.screenshot({ path: '/tmp/emulation-surface-production-preview.png' });
  await dialog.getByRole('button', { name: '创建新 Path', exact: true }).click();
  await expect(dialog).toHaveCount(0); await expect(pathObjects(page)).toHaveCount(2);
  const uvId = (await pathObjects(page).last().getAttribute('data-object-id'))!;
  const generated = await generateSource(page, uvId);
  expect((generated.match(/\bCONST robtarget\b/g) ?? []).length).toBe(count);
  expect((generated.match(/^\s*MoveL\s/gm) ?? []).length).toBe(count);
  await returnHome(page);
  const persisted = await readProject(page, projectId), saved = persisted.scene.models[MODEL];
  const uv = saved.program.paths.find(path => path.id === uvId)!, sourceRegion = saved.program.paths.find(path => path.id === REGION_ID)!;
  const surface = persisted.scene.surfaces!.find(item => item.id === SURFACE_ID)!;
  expect(persisted.scene.surfaces).toEqual(fixture.project.scene.surfaces);
  expect(sourceRegion).toEqual(fixture.project.scene.models[MODEL].program.paths.find(path => path.id === REGION_ID));
  expect(uv.surface?.resolvedRevision).toBe(uv.surface?.revision); expect(uv.pointIds).toHaveLength(count);
  expect(persisted.sources[MODEL].files.find(file => file.name === 'production-user.mod')?.text).toBe(ORIGINAL_SOURCE);
  expect(persisted.sources[MODEL].files.some(file => file.text === generated)).toBe(true);
  const planeNormal = normalOf(fixture.tcp), robot = getRobotModel(MODEL);
  for (const [index, point] of uv.surface!.points.entries()) {
    const target = pathPointToWorld(saved.graph, uv.id, point.pose), anchor = evaluateSurfaceAnchor(surface, point.pick);
    expect(new Vector3(...target.position).distanceTo(new Vector3(...anchor.position))).toBeLessThan(1e-5);
    expect(Math.abs(new Vector3(...target.position).sub(new Vector3(...fixture.tcp.position)).dot(planeNormal))).toBeLessThan(1e-5);
    expect(normalOf(target).dot(new Vector3(...anchor.normal))).toBeGreaterThan(1 - 1e-8);
    const solved = saved.program.points.find(item => item.id === uv.pointIds[index])!;
    expect(solved.pose).toEqual(point.pose); expect(solved.configuration).toHaveLength(4);
    const actual = baseFlangeToWorldTcp(saved.graph, forwardKinematics(robot, solved.joints).pose);
    expect(new Vector3(...actual.position).distanceTo(new Vector3(...target.position))).toBeLessThan(.1);
    if (index) {
      const a = uv.surface!.points[index - 1].pose.orientation, b = point.pose.orientation;
      expect(new Quaternion(a[1], a[2], a[3], a[0]).angleTo(new Quaternion(b[1], b[2], b[3], b[0])) * 180 / Math.PI).toBeLessThan(5.001);
    }
  }
  await page.locator(`[data-project-card="${projectId}"] .project-card-open`).click();
  expect(await readSource(page)).toBe(generated); await waitForProjectScene(page, MODEL);
  await expect(page.locator('[data-object-kind="surface"]')).toHaveCount(1); await expect(pathObjects(page)).toHaveCount(2);
  expect(errors).toEqual([]);
});
