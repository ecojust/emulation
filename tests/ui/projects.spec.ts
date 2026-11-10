import { expect, test, type Page } from '@playwright/test';
import { enterProject } from './project-helpers';
import { prepareCatalogFixtures, readCatalog } from './catalog-fixtures';
import { codeWorkspace, moduleList, newSource, openCode, readSource } from './programming-helpers';
import { bodyCanvas, bodyJoints, captureCurrent, expectBodyJoints, observeBrowser, openScene, selectPath, selectRobotObject, setJoint } from './workspace-helpers';

test.setTimeout(180_000);
const home = (page: Page) => page.getByRole('main', { name: '项目管理', exact: true });
const objects = (page: Page, kind: string) => page.locator(`[data-object-kind="${kind}"]`);
const card = (page: Page, id: string) => page.locator(`[data-project-card="${id}"]`);
async function ready(page: Page) {
  // Keep the restored view unchanged: this suite verifies final mesh loading
  // and camera restoration while the scene is hidden behind the program editor.
  await expect(page.getByLabel('项目机器人型号', { exact: true })).toBeVisible({ timeout: 30_000 });
  await expect(page.locator('.scene-model-status')).toHaveAttribute('data-loaded-parts', '7', { timeout: 30_000 });
  await expect(bodyCanvas(page)).toHaveAttribute('data-camera-distance', /\d/);
}
async function returnHome(page: Page) {
  await page.getByRole('navigation', { name: '项目导航', exact: true }).getByRole('button', { name: '返回主页', exact: true }).click();
  await expect(home(page)).toBeVisible({ timeout: 20_000 });
}
async function noDocumentScroll(page: Page) {
  expect(await page.evaluate(() => Math.max(document.documentElement.scrollWidth - innerWidth,
    document.documentElement.scrollHeight - innerHeight, Math.abs(scrollX), Math.abs(scrollY)))).toBeLessThanOrEqual(1);
}
async function imageSource(page: Page, id: string) {
  const image = card(page, id).getByRole('img');
  await expect(image).toBeVisible();
  await expect.poll(() => image.evaluate(element => (element as HTMLImageElement).naturalWidth)).toBeGreaterThan(100);
  return (await image.getAttribute('src'))!;
}
/** Observe actual drawImage/toDataURL calls; no fake pixels or renderer return values. */
async function observeRealCoverCapture(page: Page) {
  await page.addInitScript(() => {
    const originals = { drawImage: CanvasRenderingContext2D.prototype.drawImage, toDataURL: HTMLCanvasElement.prototype.toDataURL };
    const derived = new WeakMap<HTMLCanvasElement, { joints: number[]; camera: { position: number[]; target: number[] } }>();
    (window as any).__projectRealCaptures = [];
    CanvasRenderingContext2D.prototype.drawImage = function (this: CanvasRenderingContext2D, ...args: any[]) {
      const source = args[0];
      if (source instanceof HTMLCanvasElement && source.hasAttribute('data-robot-joints')) derived.set(this.canvas, {
        joints: JSON.parse(source.getAttribute('data-robot-joints')!),
        camera: { position: JSON.parse(source.getAttribute('data-camera-position')!), target: JSON.parse(source.getAttribute('data-camera-target')!) },
      });
      return Reflect.apply(originals.drawImage, this, args);
    } as typeof CanvasRenderingContext2D.prototype.drawImage;
    HTMLCanvasElement.prototype.toDataURL = function (...args: any[]) {
      const result = Reflect.apply(originals.toDataURL, this, args);
      if (derived.has(this)) (window as any).__projectRealCaptures.push({ url: result, ...derived.get(this)! });
      return result;
    };
  });
}
async function expectRealCover(page: Page, id: string, joints: number[]) {
  const url = await imageSource(page, id);
  expect(url).toMatch(/^data:image\/jpeg;base64,/);
  expect(url.length).toBeGreaterThan(1000);
  const captures = await page.evaluate(() => (window as any).__projectRealCaptures as { url: string; joints: number[] }[]);
  expect(captures.at(-1)).toMatchObject({ url, joints });
  return url;
}
async function renderedScene(page: Page) {
  return bodyCanvas(page).evaluate(canvas => Object.fromEntries([
    'data-robot-joints', 'data-robot-pose', 'data-robot-base-pose', 'data-scene-frames', 'data-scene-tools', 'data-scene-paths', 'data-danger-zones',
  ].map(key => [key, JSON.parse(canvas.getAttribute(key)!, (_name, value) => typeof value === 'number' ? Number(value.toFixed(6)) : value)])));
}

test('home requires a name and a catalog robot, and stays inside the viewport', async ({ page }) => {
  const errors = observeBrowser(page);
  await page.setViewportSize({ width: 1000, height: 720 });
  await page.goto('/');
  await expect(home(page)).toBeVisible();
  await expect(home(page).getByRole('button', { name: '新建项目', exact: true }).first()).toBeEnabled();
  await noDocumentScroll(page);
  await home(page).getByRole('button', { name: '新建项目', exact: true }).first().click();
  const dialog = page.getByRole('dialog', { name: '新建项目', exact: true });
  const name = dialog.getByRole('textbox', { name: '项目名称', exact: true });
  const create = dialog.getByRole('button', { name: '创建并打开', exact: true });
  await expect(create).toBeDisabled();
  await name.fill('   '); await expect(create).toBeDisabled();
  const catalog = await readCatalog();
  expect(await dialog.getByRole('combobox', { name: '项目机器人', exact: true }).locator('option').evaluateAll(options => options.map(option => (option as HTMLOptionElement).value))).toEqual(catalog.models.map(model => model.id));
  await name.fill('  短臂工作站  ');
  await dialog.getByRole('combobox', { name: '项目机器人', exact: true }).selectOption('irb1100-4-0.475');
  await create.click();
  await ready(page);
  await expect(page.getByLabel('项目机器人型号', { exact: true })).toContainText('IRB 1100-4/0.475');
  await expect(page.locator('.project-context')).toHaveText('短臂工作站');
  const navigation = page.getByRole('navigation', { name: '项目导航', exact: true });
  const back = navigation.getByRole('button', { name: '返回主页', exact: true });
  await expect(back).toBeVisible();
  await expect(page.getByRole('button', { name: '返回主页', exact: true })).toHaveCount(1);
  await expect(page.locator('.header-right').getByRole('button', { name: '返回主页', exact: true })).toHaveCount(0);
  const backBox = (await back.boundingBox())!, modelBox = (await page.getByLabel('项目机器人型号', { exact: true }).boundingBox())!;
  expect(backBox.x + backBox.width).toBeLessThanOrEqual(modelBox.x);
  await noDocumentScroll(page);
  await returnHome(page);
  await expect(home(page).getByRole('button', { name: '打开项目 短臂工作站', exact: true })).toBeVisible();
  await noDocumentScroll(page);
  expect(errors).toEqual([]);
});

test('projects restore complete scenes and source independently, with new real covers from program view and isolated rename/delete', async ({ page }) => {
  const errors = observeBrowser(page);
  await observeRealCoverCapture(page);
  await page.goto('/');
  const a = await enterProject(page, '工位 A'); await ready(page);
  await setJoint(page, 1, 12);
  await page.getByRole('combobox', { name: '记录速度', exact: true }).selectOption('50');
  await page.getByRole('button', { name: '添加参考坐标系', exact: true }).click();
  const frame = objects(page, 'frame').last(), frameId = (await frame.getAttribute('data-object-id'))!;
  await page.getByRole('textbox', { name: '参考坐标系名称', exact: true }).fill('夹具 A');
  await page.getByRole('spinbutton', { name: '参考坐标系位置 X', exact: true }).fill('120');
  await page.getByRole('spinbutton', { name: '参考坐标系姿态 Rz', exact: true }).fill('30');
  await page.getByRole('button', { name: '应用参考坐标系位姿', exact: true }).click();
  await selectRobotObject(page);
  await page.getByRole('combobox', { name: '挂接到', exact: true }).selectOption('frame:' + frameId);
  await expect(objects(page, 'robot')).toHaveAttribute('data-parent-id', frameId);
  await page.getByRole('button', { name: '添加工具', exact: true }).click();
  const toolId = (await objects(page, 'tool').last().getAttribute('data-object-id'))!;
  await page.getByRole('textbox', { name: '工具名称', exact: true }).fill('打磨头 A');
  await page.getByRole('combobox', { name: '挂接到', exact: true }).selectOption('flange');
  await page.getByRole('button', { name: '对齐法兰', exact: true }).click();
  await page.getByRole('spinbutton', { name: '工具 TCP 偏置位置 Z', exact: true }).fill('25');
  await page.getByRole('button', { name: '应用工具 TCP 偏置位姿', exact: true }).click();
  const point = await captureCurrent(page);
  await selectPath(page, point.pathId);
  await page.getByRole('combobox', { name: '挂接到', exact: true }).selectOption('frame:' + frameId);
  await objects(page, 'zone').first().click();
  await page.getByRole('textbox', { name: '危险区名称', exact: true }).fill('夹具禁入区 A');
  await page.getByRole('spinbutton', { name: '危险区中心 X', exact: true }).fill('700');
  await page.getByRole('spinbutton', { name: '危险区尺寸 X', exact: true }).fill('120');
  await page.getByRole('checkbox', { name: '显示关节坐标系', exact: true }).check();
  await selectRobotObject(page); // Keep transient selection aids out of the captured cover.
  await expect(bodyCanvas(page)).toHaveAttribute('data-joint-frames-visible', 'true');
  await expect.poll(async () => JSON.parse((await bodyCanvas(page).getAttribute('data-danger-zones'))!)[0].minMm[0]).toBeCloseTo(640, 5);
  const before = await renderedScene(page), jointsA = await bodyJoints(page);
  const canvasBox = (await bodyCanvas(page).boundingBox())!;
  await page.mouse.move(canvasBox.x + canvasBox.width * .65, canvasBox.y + canvasBox.height * .6);
  const previousZoomTarget = Number(await bodyCanvas(page).getAttribute('data-zoom-target'));
  await page.mouse.wheel(0, 140);
  await expect.poll(async () => Number(await bodyCanvas(page).getAttribute('data-zoom-target'))).toBeGreaterThan(previousZoomTarget + .1);
  // Repeated equal samples can be the same delayed RAF; wait for the requested target.
  await expect.poll(() => bodyCanvas(page).evaluate(canvas => Math.abs(Number(canvas.getAttribute('data-camera-distance')) - Number(canvas.getAttribute('data-zoom-target')))), { timeout: 15_000 }).toBeLessThan(.00001);
  const camera = JSON.parse((await bodyCanvas(page).getAttribute('data-camera-position'))!) as number[];
  const sourceA = 'MODULE WorkcellA\n    ! Only project A owns this module\nENDMODULE';
  await newSource(page, 'cell.mod', sourceA);
  await expect(codeWorkspace(page)).toBeVisible();
  await returnHome(page); // Capture must render the hidden scene, never the source editor.
  const firstCover = await expectRealCover(page, a, jointsA);
  const capturedCamera = await page.evaluate(() => (window as any).__projectRealCaptures.at(-1).camera as { position: number[]; target: number[] });
  expect(Math.max(...capturedCamera.position.map((value, index) => Math.abs(value - camera[index])))).toBeLessThan(.00002);

  const b = await enterProject(page, '工位 B'); await ready(page);
  for (const kind of ['frame', 'tool', 'path', 'target']) await expect(objects(page, kind)).toHaveCount(0);
  expect((await bodyJoints(page))[0]).toBe(0);
  await openCode(page); await expect(moduleList(page).getByRole('option')).toHaveCount(0);
  const sourceB = 'MODULE WorkcellB\n    ! Project B has independent content\nENDMODULE';
  await newSource(page, 'cell.mod', sourceB);
  await setJoint(page, 1, -18);
  await expect.poll(async () => (await bodyJoints(page))[0]).toBe(-18);
  const jointsB = await bodyJoints(page);
  await returnHome(page);
  await page.screenshot({ path: '/tmp/emulation-project-home.png' });

  // Exercise partial mesh completion: a saved camera must survive the final
  // mesh attachment/default-view initialization, even when the program is open.
  let releaseLastMesh!: () => void, lastMeshRequested = false;
  const lastMeshReleased = new Promise<void>(resolve => { releaseLastMesh = resolve; });
  await page.route('**/models/irb1100-4-0.58/link_6.stl', async route => {
    lastMeshRequested = true;
    await lastMeshReleased;
    await route.continue();
  });
  try {
    await page.reload();
    expect(await enterProject(page, '工位 A')).toBe(a);
    await expect.poll(() => lastMeshRequested).toBe(true);
    await expect(page.locator('.scene-model-status')).toHaveAttribute('data-loaded-parts', '6', { timeout: 30_000 });
    await expect(codeWorkspace(page)).toBeVisible();
  } finally { releaseLastMesh(); }
  await ready(page);
  await expect(codeWorkspace(page)).toBeVisible();
  expect(await readSource(page)).toBe(sourceA);
  await openScene(page);
  await expectBodyJoints(page, jointsA);
  await expect.poll(() => renderedScene(page)).toEqual(before);
  await expect(objects(page, 'robot')).toHaveAttribute('data-parent-id', frameId);
  await expect(page.locator(`[data-object-kind="path"][data-object-id="${point.pathId}"]`)).toHaveAttribute('data-parent-id', frameId);
  await expect(page.locator(`[data-object-kind="tool"][data-object-id="${toolId}"]`)).toHaveAttribute('data-parent-kind', 'flange');
  await expect(page.getByRole('checkbox', { name: '显示关节坐标系', exact: true })).toBeChecked();
  for (const component of ['position', 'target'] as const) await expect.poll(async () => {
    const actual = JSON.parse((await bodyCanvas(page).getAttribute('data-camera-' + component))!) as number[];
    return Math.max(...actual.map((value, index) => Math.abs(value - capturedCamera[component][index])));
  }).toBeLessThan(.00001);
  await selectRobotObject(page); await expect(page.getByRole('combobox', { name: '记录速度', exact: true })).toHaveValue('50');
  await setJoint(page, 1, 35);
  await expect.poll(async () => (await bodyJoints(page))[0]).toBe(35);
  const changedJoints = await bodyJoints(page);
  await openCode(page); await returnHome(page);
  const changedCover = await expectRealCover(page, a, changedJoints);
  expect(changedCover).not.toBe(firstCover);

  await card(page, a).getByRole('button', { name: '重命名项目 工位 A', exact: true }).click();
  const rename = page.getByRole('dialog', { name: '重命名项目', exact: true });
  await rename.getByRole('textbox', { name: '项目名称', exact: true }).fill('打磨工位 A');
  await rename.getByRole('button', { name: '保存名称', exact: true }).click();
  await expect(card(page, a).getByRole('button', { name: '打开项目 打磨工位 A', exact: true })).toBeVisible();
  expect(await imageSource(page, a)).toBe(changedCover);
  await card(page, a).getByRole('button', { name: '删除项目 打磨工位 A', exact: true }).click();
  await page.getByRole('dialog', { name: '删除项目', exact: true }).getByRole('button', { name: '删除项目', exact: true }).click();
  await expect(card(page, a)).toHaveCount(0); await expect(card(page, b)).toBeVisible();
  await page.reload();
  expect(await enterProject(page, '工位 B')).toBe(b); await ready(page);
  await expectBodyJoints(page, jointsB);
  await openCode(page); expect(await readSource(page)).toBe(sourceB);
  await openScene(page);
  for (const kind of ['frame', 'tool', 'path', 'target']) await expect(objects(page, kind)).toHaveCount(0);
  expect(errors).toEqual([]);
});

test('failed cover capture keeps the editor and data available until the user retries', async ({ page }) => {
  const errors = observeBrowser(page);
  await page.goto('/');
  const id = await enterProject(page, '封面重试'); await ready(page);
  await setJoint(page, 1, 22);
  await expect.poll(async () => (await bodyJoints(page))[0]).toBe(22);
  const joints = await bodyJoints(page);
  await page.evaluate(() => {
    const original = HTMLCanvasElement.prototype.toDataURL;
    HTMLCanvasElement.prototype.toDataURL = function (...args) {
      if (args[0] === 'image/jpeg') { HTMLCanvasElement.prototype.toDataURL = original; throw new Error('capture retry regression'); }
      return Reflect.apply(original, this, args);
    };
  });
  await page.getByRole('button', { name: '返回主页', exact: true }).click();
  const failure = page.getByRole('alertdialog', { name: '项目保存失败', exact: true });
  await expect(failure).toContainText('capture retry regression');
  await expect(home(page)).toHaveCount(0); await expect(page.locator('[data-project-id]')).toHaveAttribute('data-project-id', id);
  await failure.getByRole('button', { name: '继续编辑', exact: true }).click();
  await expectBodyJoints(page, joints);
  await returnHome(page); await imageSource(page, id);
  await enterProject(page, '封面重试'); await ready(page); await expectBodyJoints(page, joints);
  expect(errors).toEqual([]);
});

test('generic project downloads stay centered, cancel in flight and retry failures without mounting another robot or creating stray module drafts', async ({ page }) => {
  const errors: string[] = []; page.on('pageerror', error => errors.push(error.message));
  const catalog = await readCatalog(), model = catalog.models.find(entry => entry.id === 'yaskawa-gp7')!;
  const fixtures = await prepareCatalogFixtures({ ...catalog, models: [model] });
  const broken = model.files.find(file => file.path.endsWith('.stl'))!.url;
  let corrupt = true, hold = true, cancelled = false;
  let releaseDownload!: () => void;
  const delayedFile = new Promise<void>(resolve => { releaseDownload = resolve; });
  const aborted: string[] = [];
  page.on('requestfailed', request => { if (request.url() === broken) aborted.push(request.failure()?.errorText ?? ''); });
  await page.route(url => fixtures.has(url.href), async route => {
    const wasHeld = hold && route.request().url() === broken;
    if (wasHeld) { hold = false; await delayedFile; }
    const bytes = Buffer.from(fixtures.get(route.request().url())!);
    if (corrupt && route.request().url() === broken) bytes[bytes.length - 1] ^= 1;
    try { await route.fulfill({ status: 200, contentType: 'application/octet-stream', headers: { 'access-control-allow-origin': '*' }, body: bytes }); }
    catch (cause) { if (!wasHeld || !cancelled) throw cause; }
  });
  await page.goto('/');
  const id = await enterProject(page, 'GP7 下载重试', model.id);
  const barrier = page.getByRole('main', { name: '打开项目', exact: true });
  const loadingCard = barrier.locator('.project-model-card');
  const progress = barrier.getByRole('progressbar', { name: '模型下载进度', exact: true });
  try {
    await expect(loadingCard).toHaveAttribute('data-model-load-state', 'downloading');
    await expect(progress).toBeVisible();
    // The first actual URDF has arrived; the following mesh response is held.
    await expect.poll(async () => Number(await progress.getAttribute('value'))).toBeGreaterThan(0);
    expect(Number(await progress.getAttribute('max'))).toBe(model.downloadBytes);
    await expect(loadingCard).toContainText(model.name);
    await expect(loadingCard).toContainText('GP7 下载重试');
    await expect(loadingCard.getByRole('status')).toContainText('正在下载机器人模型');
    await expect(page.locator('canvas')).toHaveCount(0); await expect(codeWorkspace(page)).toHaveCount(0);
    for (const size of [{ width: 1000, height: 720 }, { width: 1440, height: 1000 }]) {
      await page.setViewportSize(size);
      await expect.poll(async () => {
        const content = (await barrier.locator('.project-model-content').boundingBox())!, panel = (await loadingCard.boundingBox())!;
        return Math.max(Math.abs(panel.x + panel.width / 2 - (content.x + content.width / 2)), Math.abs(panel.y + panel.height / 2 - (content.y + content.height / 2)));
      }).toBeLessThanOrEqual(1);
      const box = (await loadingCard.boundingBox())!;
      expect(box.width).toBeLessThanOrEqual(520); expect(box.x).toBeGreaterThan(0); expect(box.y).toBeGreaterThan(0);
      expect(box.x + box.width).toBeLessThan(size.width); expect(box.y + box.height).toBeLessThan(size.height);
      await noDocumentScroll(page);
    }
    await page.screenshot({ path: '/tmp/emulation-project-model-download.png' });
    cancelled = true;
    await barrier.getByRole('button', { name: '返回项目主页', exact: true }).click();
    await expect(home(page)).toBeVisible();
  } finally { releaseDownload(); }
  await expect.poll(() => aborted.length).toBeGreaterThan(0);
  await expect(card(page, id)).toBeVisible(); await expect(card(page, id).getByRole('img')).toHaveCount(0);
  await expect(page.locator('canvas')).toHaveCount(0); await expect(codeWorkspace(page)).toHaveCount(0);
  await card(page, id).getByRole('button', { name: '打开项目 GP7 下载重试', exact: true }).click();
  await expect(barrier.getByRole('alert')).toContainText(/校验|SHA|完整/);
  await expect(loadingCard).toHaveAttribute('data-model-load-state', 'error');
  await expect(barrier.getByRole('button', { name: '重新加载模型', exact: true })).toBeEnabled();
  await expect(page.locator('canvas')).toHaveCount(0); await expect(codeWorkspace(page)).toHaveCount(0);
  await noDocumentScroll(page);
  corrupt = false;
  await barrier.getByRole('button', { name: '重新加载模型', exact: true }).click();
  await ready(page);
  await expect(page.locator('canvas[data-urdf-model]')).toHaveAttribute('data-urdf-model', model.id);
  await expect(page.getByLabel('项目机器人型号', { exact: true })).toContainText(model.name);
  await openCode(page); await expect(moduleList(page).getByRole('option')).toHaveCount(0);
  await returnHome(page); await imageSource(page, id);
  await page.reload(); await enterProject(page, 'GP7 下载重试'); await ready(page);
  await expect(codeWorkspace(page)).toBeVisible();
  await expect(page.locator('canvas[data-urdf-model]')).toHaveAttribute('data-urdf-model', model.id);
  expect(errors).toEqual([]);
});
