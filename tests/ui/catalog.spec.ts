import { enterProject, openDefaultProject, waitForProjectScene } from './project-helpers';
import { expect, test, type Page, type Route } from '@playwright/test';
import type { RobotCatalog, RobotCatalogModel } from '../../src/catalog/types';
import type { Pose } from '../../src/robot/types';
import { prepareCatalogFixtures, readCatalog } from './catalog-fixtures';
import { bodyJoints, captureCurrent, expectNoPointMotionActions, openScene, pathPoints, selectPoint, selectRobot, setJoint } from './workspace-helpers';
import { chooseCompletion, codeEditor, generateSource, moduleList, newSource, openCode, readSource, replaceSource } from './programming-helpers';

test.setTimeout(90_000);
let catalog: RobotCatalog;
let fixtures: Map<string, Buffer>;
let downloads: RobotCatalogModel[];
test.beforeAll(async () => {
  test.setTimeout(180_000);
  catalog = await readCatalog();
  downloads = catalog.models.filter(model => !model.bundledMotionModelId);
  fixtures = await prepareCatalogFixtures(catalog);
});

const picker = (page: Page) => page.getByRole('dialog', { name: '机器人模型目录', exact: true });
const current = (page: Page) => page.getByLabel('项目机器人型号', { exact: true });
const genericCanvas = (page: Page) => page.locator('canvas[data-urdf-model]');
function observe(page: Page) {
  const errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  return errors;
}
async function open(page: Page) {
  await openDefaultProject(page);
  await waitForProjectScene(page, 'irb1100-4-0.58');
}
async function home(page: Page) {
  await page.getByRole('button', { name: '返回主页', exact: true }).click();
  await expect(page.getByRole('main', { name: '项目管理', exact: true })).toBeVisible();
}
async function reopen(page: Page, id: string) {
  await page.locator(`[data-project-card="${id}"] .project-card-open`).click();
  await waitForProjectScene(page);
}
async function realBytes(route: Route) {
  const bytes = fixtures.get(route.request().url());
  if (!bytes) throw new Error(`Unexpected asset URL: ${route.request().url()}`);
  await route.fulfill({ status: 200, contentType: 'application/octet-stream', headers: { 'access-control-allow-origin': '*' }, body: bytes });
}
async function routeAssets(page: Page, handler: (route: Route) => Promise<void> = realBytes) {
  await page.route(url => fixtures.has(url.href), handler);
}
async function expectModel(page: Page, model: RobotCatalogModel, count: number) {
  await waitForProjectScene(page, model.id);
  await expect(picker(page)).toBeHidden();
  await expect(current(page)).toContainText(model.name);
  await expect(genericCanvas(page)).toHaveAttribute('data-urdf-model', model.id);
  await expect(genericCanvas(page)).toHaveAttribute('data-urdf-visual-count', String(count));
  await expect(page.getByRole('status', { name: '型号能力', exact: true })).toContainText('直线运动仿真');
  await expect(page.getByRole('status', { name: '型号能力', exact: true })).toContainText('控制器文件导出尚未适配');
}
async function toolPose(page: Page): Promise<Pose> {
  return JSON.parse((await genericCanvas(page).getAttribute('data-urdf-tool-pose'))!);
}
async function noDocumentScroll(page: Page) {
  expect(await page.evaluate(() => Math.max(document.documentElement.scrollWidth - innerWidth, document.documentElement.scrollHeight - innerHeight, Math.abs(scrollX), Math.abs(scrollY)))).toBeLessThanOrEqual(1);
}

/** Keep both an immutable scene point and a separately generated source file. */
async function recordedWorkspace(page: Page) {
  const point = await captureCurrent(page);
  const source = await generateSource(page, point.pathId);
  await setJoint(page, 1, 3);
  await expect.poll(async () => (await bodyJoints(page))[0]).toBe(3);
  await selectPoint(page, point);
  return { point, source, target: await page.getByRole('textbox', { name: '目标 robtarget', exact: true }).inputValue() };
}

test('robots are chosen only when creating a project and the project header keeps its fixed identity', async ({ page }) => {
  const errors = observe(page);
  await page.setViewportSize({ width: 1000, height: 720 });
  await page.goto('/');
  await page.getByRole('main', { name: '项目管理', exact: true }).getByRole('button', { name: '新建项目', exact: true }).first().click();
  const dialog = page.getByRole('dialog', { name: '新建项目', exact: true });
  const options = dialog.getByRole('combobox', { name: '项目机器人', exact: true }).locator('option');
  expect(await options.evaluateAll(nodes => nodes.map(node => (node as HTMLOptionElement).value))).toEqual(catalog.models.map(model => model.id));
  expect(await options.allTextContents()).toEqual(catalog.models.map(model => model.name));
  expect((await options.allTextContents()).some(name => name.startsWith('KUKA'))).toBe(false);
  const bounds = (await dialog.boundingBox())!;
  expect(bounds.x).toBeGreaterThanOrEqual(0); expect(bounds.y).toBeGreaterThanOrEqual(0);
  expect(bounds.x + bounds.width).toBeLessThanOrEqual(1000); expect(bounds.y + bounds.height).toBeLessThanOrEqual(720);
  await noDocumentScroll(page);
  await page.screenshot({ path: '/tmp/emulation-catalog-picker.png' });
  await dialog.getByRole('textbox', { name: '项目名称', exact: true }).fill('固定型号项目');
  await dialog.getByRole('combobox', { name: '项目机器人', exact: true }).selectOption('irb1100-4-0.475');
  await dialog.getByRole('button', { name: '创建并打开', exact: true }).click();
  await waitForProjectScene(page, 'irb1100-4-0.475');
  await expect(current(page)).toContainText('IRB 1100-4/0.475');
  await expect(current(page)).toHaveAttribute('data-model-id', 'irb1100-4-0.475');
  await expect(page.getByRole('button', { name: '选择机器人型号', exact: true })).toHaveCount(0);
  await current(page).click();
  await expect(picker(page)).toHaveCount(0);
  await noDocumentScroll(page);
  expect(errors).toEqual([]);
});

test('cancelling a new project model download preserves the previous project robot and program', async ({ page }) => {
  const errors = observe(page), model = downloads.find(model => model.id === 'fanuc-crx10ia-l')!;
  const heldUrl = model.files[model.files.length - 1].url;
  let release!: () => void;
  const held = new Promise<void>(resolve => { release = resolve; });
  let reached = false, cancelled = false;
  await routeAssets(page, async route => {
    if (route.request().url() === heldUrl) {
      reached = true; await held;
      if (cancelled) { await route.abort('aborted').catch(() => {}); return; }
    }
    await realBytes(route);
  });
  try {
    await open(page);
    const originalProject = (await page.locator('[data-project-id]').getAttribute('data-project-id'))!;
    const { point, source, target } = await recordedWorkspace(page), originalJoints = await bodyJoints(page);
    await home(page);
    const cancelledProject = await enterProject(page, 'Cancelled download', model.id);
    const loading = page.getByRole('main', { name: '打开项目', exact: true });
    await expect.poll(() => reached).toBe(true);
    await expect(loading.getByRole('status', { name: '模型加载状态', exact: true })).toContainText('正在下载机器人模型');
    const progress = loading.getByRole('progressbar', { name: '模型下载进度', exact: true });
    expect(await progress.evaluate(element => (element as HTMLProgressElement).value)).toBeGreaterThan(0);
    expect(await progress.evaluate(element => (element as HTMLProgressElement).position)).toBeLessThan(1);
    await expect(page.locator('.robot-scene canvas')).toHaveCount(0);
    cancelled = true;
    await loading.getByRole('button', { name: '返回项目主页', exact: true }).click(); release();
    await expect(page.locator(`[data-project-card="${cancelledProject}"]`)).toBeVisible();
    await reopen(page, originalProject);
    await expect(current(page)).toContainText('IRB 1100-4/0.58');
    await expect.poll(() => bodyJoints(page)).toEqual(originalJoints);
    await selectPoint(page, point);
    await expect(page.getByRole('textbox', { name: '目标 robtarget', exact: true })).toHaveValue(target);
    await expectNoPointMotionActions(page);
    await openCode(page); expect(await readSource(page)).toBe(source);
    expect(errors).toEqual([]);
  } finally { cancelled = true; release(); }
});

test('corrupt project models cannot mount an editor, retry succeeds, and the verified cache reopens offline', async ({ page }) => {
  const errors = observe(page), model = downloads.find(model => model.id === 'fanuc-crx10ia-l')!;
  const brokenUrl = model.files.find(file => file.path.endsWith('.stl'))!.url;
  let corrupt = true, offline = false, networkRequests = 0;
  await routeAssets(page, async route => {
    networkRequests++;
    if (offline) { await route.abort('internetdisconnected'); return; }
    if (corrupt && route.request().url() === brokenUrl) {
      const bytes = Buffer.from(fixtures.get(brokenUrl)!); bytes[bytes.length - 1] ^= 1;
      await route.fulfill({ status: 200, contentType: 'application/octet-stream', headers: { 'access-control-allow-origin': '*' }, body: bytes }); return;
    }
    await realBytes(route);
  });
  await open(page);
  const originalProject = (await page.locator('[data-project-id]').getAttribute('data-project-id'))!;
  const { point, source, target } = await recordedWorkspace(page), originalJoints = await bodyJoints(page);
  await home(page);
  await enterProject(page, 'Cached CRX', model.id);
  const loading = page.getByRole('main', { name: '打开项目', exact: true });
  await expect(loading.getByRole('alert')).toContainText(/校验|SHA|完整/);
  await expect(page.locator('.robot-scene canvas')).toHaveCount(0);
  await expect(page.locator('.robot-code-workspace')).toHaveCount(0);
  corrupt = false;
  await loading.getByRole('button', { name: '重新加载模型', exact: true }).click();
  await expectModel(page, model, 7);
  const onlineRequests = networkRequests; expect(onlineRequests).toBeGreaterThan(model.files.length);
  await home(page); await reopen(page, originalProject);
  await expect.poll(() => bodyJoints(page)).toEqual(originalJoints);
  await selectPoint(page, point);
  await expect(page.getByRole('textbox', { name: '目标 robtarget', exact: true })).toHaveValue(target);
  await openCode(page); expect(await readSource(page)).toBe(source);
  await home(page);
  offline = true;
  await page.reload(); await enterProject(page, 'Cached CRX', model.id);
  await expectModel(page, model, 7);
  expect(networkRequests).toBe(onlineRequests);
  expect(errors).toEqual([]);
});

const anchors: Record<string, { xyz: number[]; sign: number; visuals: number }> = {
  'fanuc-crx10ia-l': { xyz: [719.5, -150, 955], sign: 1, visuals: 7 },
  'fanuc-r2000ib-165f': { xyz: [1847.5, 0, 1970], sign: -1, visuals: 9 },
  'yaskawa-gp7': { xyz: [560, 0, 815], sign: 1, visuals: 7 },
  'yaskawa-gp110': { xyz: [1540, 0, 1645], sign: 1, visuals: 7 },
};
for (const [modelId, expected] of Object.entries(anchors)) {
  test(`${modelId} renders its geometry, FK and brand-specific template without controller export and keeps the ABB project isolated`, async ({ page }) => {
    const errors = observe(page);
    await routeAssets(page);
    await open(page);
    expect(downloads.map(model => model.id)).toEqual(Object.keys(anchors));
    const model = downloads.find(item => item.id === modelId)!;
    await selectRobot(page, model.id);
    await expectModel(page, model, expected.visuals);
    await expect(page.getByRole('heading', { name: '机器人属性', exact: true })).toBeVisible();
    await expect(page.getByRole('spinbutton', { name: /^机器人关节 J/ })).toHaveCount(6);
    await expect.poll(async () => Math.max(...(await toolPose(page)).position.map((value, i) => Math.abs(value - expected.xyz[i])))).toBeLessThan(.001);
    await expect(page.getByRole('button', { name: '检查到此点的 MoveL', exact: true })).toHaveCount(0);
    await expect(page.getByRole('button', { name: '拖动法兰', exact: true })).toBeEnabled();
    await expectNoPointMotionActions(page);
    const before = await genericCanvas(page).screenshot();
    await page.getByRole('spinbutton', { name: '机器人关节 J1', exact: true }).fill('30');
    await expect.poll(async () => JSON.parse((await genericCanvas(page).getAttribute('data-urdf-joints'))!)[0]).toBe(30);
    const angle = expected.sign * Math.PI / 6, [x, y, z] = expected.xyz;
    const rotated = [x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle), z];
    await expect.poll(async () => Math.max(...(await toolPose(page)).position.map((value, i) => Math.abs(value - rotated[i])))).toBeLessThan(.001);
    for (const [index, axis] of ['X', 'Y', 'Z'].entries()) await expect(page.getByLabel('当前 TCP', { exact: true }).locator(`[data-axis="${axis}"] strong`)).toHaveText(rotated[index].toFixed(2));
    expect((await genericCanvas(page).screenshot()).equals(before)).toBe(false);
    await page.screenshot({ path: `/tmp/emulation-catalog-${model.id}.png` });
    const point = await captureCurrent(page);
    await expect(pathPoints(page, point.pathId)).toHaveCount(1);
    await expect(pathPoints(page, point.pathId)).toHaveAttribute('data-point-name', point.name);
    await expect(pathPoints(page, point.pathId)).toHaveAccessibleName('选择目标点 ' + point.name);
    await expect(page.getByLabel('目标 robconf', { exact: true })).toHaveCount(0);
    const code = await generateSource(page, point.pathId);
    await expect(page.getByRole('button', { name: '运行源码', exact: true })).toHaveCount(0);
    await expect(moduleList(page).getByRole('option')).toHaveCount(1);
    if (model.brand === 'FANUC') {
      await expect(moduleList(page).getByRole('option', { selected: true })).toHaveAccessibleName(/\.ls$/);
      expect(code).toContain('L P[1] 100mm/sec FINE');
      expect(code).toContain('P[1]{');
      expect(code).toContain('J1=30 deg');
      expect(code).toContain('{{UF}}');
      expect(code).toContain('{{UT}}');
      expect(code).not.toMatch(/MoveL|robconf|robtarget/);
    } else {
      await expect(moduleList(page).getByRole('option', { selected: true })).toHaveAccessibleName(/\.jbi$/);
      expect(code).toContain('MOVL C00000 V=100');
      expect(code).toContain('C00000=');
      expect(code).toContain('///TOOL {{TOOL}}');
      expect(code).toContain('///RCONF {{RCONF_P001}}');
      expect(code).not.toMatch(/MoveL|robconf|robtarget/);
    }
    await expect(page.getByRole('button', { name: '复制此 MoveL', exact: true })).toHaveCount(0);
    await expect(page.getByRole('button', { name: '导出此 MoveL', exact: true })).toHaveCount(0);
    if (model.id === 'fanuc-crx10ia-l') {
      await newSource(page, 'lexical.ls', `/PROG LEXICAL\n/ATTR\nOWNER=MNEDITOR;\nCOMMENT="QA";\n/MN\n: ! lexical note ;\n: -- second note ;\n1: J P[1] 10% FINE;\n/POS\nP[1]{\nGP1: UF:0,UT:0,CONFIG:'N U T,0,0,0',\nX=400mm,Y=0mm,Z=500mm,W=0deg,P=90deg,R=0deg\n};\n/END`);
      const editorContent = codeEditor(page);
      const doubleQuoted = editorContent.locator('span').filter({ hasText: /^"QA"$/ });
      const singleQuoted = editorContent.locator('span').filter({ hasText: /^'N U T,0,0,0'$/ });
      await expect(doubleQuoted).toBeVisible();
      await expect(singleQuoted).toBeVisible();
      expect(await singleQuoted.evaluate(element => getComputedStyle(element).color)).toBe(await doubleQuoted.evaluate(element => getComputedStyle(element).color));
      for (const text of ['! lexical note ;', '-- second note ;']) {
        const comment = editorContent.locator('span').filter({ hasText: new RegExp('^' + text + '$') });
        await expect(comment).toBeVisible();
        expect(await comment.evaluate(element => getComputedStyle(element).fontStyle)).toBe('italic');
      }
      expect(await editorContent.locator('span').filter({ hasText: /^J$/ }).evaluate(element => getComputedStyle(element).fontStyle)).not.toBe('italic');
      // Exercise a real multiword catalog entry, including replacement of the
      // full partially typed phrase and documentation from a middle token.
      await replaceSource(page, 'COL GUA');
      await editorContent.press('ControlOrMeta+End');
      await chooseCompletion(page, 'COL GUARD ADJUST');
      expect(await readSource(page)).toBe('COL GUARD ADJUST sensitivity');
      await editorContent.press('ArrowRight');
      await editorContent.locator('span').filter({ hasText: /GUARD/ }).first().hover();
      const hover = page.locator('.cm-tooltip-hover .robot-code-tooltip');
      await expect(hover.locator('strong')).toHaveText('COL GUARD ADJUST');
      await expect(hover.locator('dt').filter({ hasText: /sensitivity/ })).toBeVisible();
      await page.getByRole('button', { name: '程序', exact: true }).hover();
      await expect(hover).toBeHidden();
    }
    if (model.id === 'yaskawa-gp7') {
      await newSource(page, 'completion.jbi', '+SF');
      await codeEditor(page).press('ControlOrMeta+End');
      await chooseCompletion(page, '+SFTON3D');
      expect(await readSource(page)).toBe('+SFTON3D');
    }
    await openScene(page);
    await noDocumentScroll(page);
    await selectRobot(page, 'irb1100-4-0.58');
    await expect(page.getByRole('button', { name: '拖动法兰', exact: true })).toBeEnabled();
    const abbPoint = await captureCurrent(page);
    await expectNoPointMotionActions(page);
    const source = await generateSource(page, abbPoint.pathId);
    expect(source).toContain('MoveL ' + abbPoint.name + ',');
    expect(source).toContain('CONST robtarget ' + abbPoint.name);
    expect(errors).toEqual([]);
  });
}
