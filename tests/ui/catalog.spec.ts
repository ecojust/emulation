import { expect, test, type Page, type Route } from '@playwright/test';
import type { RobotCatalog, RobotCatalogModel } from '../../src/catalog/types';
import type { Pose } from '../../src/robot/types';
import { prepareCatalogFixtures, readCatalog } from './catalog-fixtures';
import { bodyJoints, captureCurrent, checkedSegment, expectCopyState, openScene, pathPoints, readRapid, selectRobot } from './workspace-helpers';
import { chooseCompletion, codeEditor, generateSource, moduleList, newSource, readSource, replaceSource } from './programming-helpers';

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
const entry = (page: Page, id: string) => picker(page).locator(`[data-catalog-model-id="${id}"]`);
const current = (page: Page) => page.getByRole('button', { name: '选择机器人型号', exact: true });
const genericCanvas = (page: Page) => page.locator('canvas[data-urdf-model]');
function observe(page: Page) {
  const errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  return errors;
}
async function open(page: Page) {
  await page.goto('/');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
}
async function showPicker(page: Page) {
  await current(page).click();
  await expect(picker(page)).toBeVisible();
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

test('the title opens a bounded searchable model catalog with honest brand availability', async ({ page }) => {
  const errors = observe(page);
  await page.setViewportSize({ width: 1000, height: 720 });
  await open(page);
  await showPicker(page);
  await expect(picker(page).locator('[data-catalog-model-id]')).toHaveCount(catalog.models.length);
  for (const brand of ['ABB', 'FANUC', 'KUKA', 'Yaskawa']) await expect(picker(page).getByRole('region', { name: `${brand} 机器人`, exact: true })).toBeVisible();
  const kuka = picker(page).getByRole('region', { name: 'KUKA 机器人', exact: true });
  await expect(kuka).toContainText(/暂无.*KUKA.*可下载资源|暂无可加载型号/);
  await expect(kuka.getByRole('button', { name: /^加载 / })).toHaveCount(0);
  for (const model of downloads) await expect(entry(page, model.id)).toContainText('URDF 正运动学 / 直线仿真');
  const bounds = (await picker(page).boundingBox())!;
  expect(bounds.x).toBeGreaterThanOrEqual(0);
  expect(bounds.y).toBeGreaterThanOrEqual(0);
  expect(bounds.x + bounds.width).toBeLessThanOrEqual(1000);
  expect(bounds.y + bounds.height).toBeLessThanOrEqual(720);
  expect(await picker(page).locator('.robot-catalog-list').evaluate(element => element.scrollHeight - element.clientHeight)).toBeGreaterThan(0);
  await noDocumentScroll(page);
  await page.screenshot({ path: '/tmp/emulation-catalog-picker.png' });
  await page.getByRole('searchbox', { name: '搜索机器人型号', exact: true }).fill('GP7');
  await expect(picker(page).locator('[data-catalog-model-id]')).toHaveCount(1);
  await expect(entry(page, 'yaskawa-gp7')).toBeVisible();
  await page.getByRole('button', { name: '关闭模型目录', exact: true }).click();
  await expect(current(page)).toContainText('IRB 1100-4/0.58');
  expect(errors).toEqual([]);
});

test('cancelling an in-progress verified asset download retains the current robot and valid program', async ({ page }) => {
  const errors = observe(page), model = downloads.find(model => model.id === 'fanuc-crx10ia-l')!;
  const heldUrl = model.files[model.files.length - 1].url;
  let release!: () => void;
  const held = new Promise<void>(resolve => { release = resolve; });
  let reached = false, cancelled = false;
  await routeAssets(page, async route => {
    if (route.request().url() === heldUrl) {
      reached = true;
      await held;
      if (cancelled) { await route.abort('aborted').catch(() => {}); return; }
    }
    await realBytes(route);
  });
  try {
    await open(page);
    const point = await checkedSegment(page);
    const program = await readRapid(page);
    const originalJoints = await bodyJoints(page);
    await showPicker(page);
    await entry(page, model.id).getByRole('button', { name: /^加载 / }).click();
    await expect.poll(() => reached).toBe(true);
    await expect(page.getByRole('status', { name: '模型加载进度', exact: true })).toContainText('正在下载模型');
    const progress = page.getByRole('progressbar', { name: '模型下载进度', exact: true });
    expect(await progress.evaluate(element => (element as HTMLProgressElement).value)).toBeGreaterThan(0);
    expect(await progress.evaluate(element => (element as HTMLProgressElement).position)).toBeLessThan(1);
    cancelled = true;
    await page.getByRole('button', { name: '取消加载', exact: true }).click();
    release();
    await expect(picker(page)).toContainText('已取消加载，当前机器人保持不变');
    await page.getByRole('button', { name: '关闭模型目录', exact: true }).click();
    await expect(current(page)).toContainText('IRB 1100-4/0.58');
    expect(await readRapid(page)).toBe(program);
    expect(await bodyJoints(page)).toEqual(originalJoints);
    await expect(page.locator('[data-target-point-id]')).toHaveAttribute('data-target-point-id', point.id);
    await expectCopyState(page, true);
    expect(errors).toEqual([]);
  } finally { cancelled = true; release(); }
});

test('a corrupt asset cannot replace the robot, retry succeeds, and verified browser cache works without its server', async ({ page }) => {
  const errors = observe(page), model = downloads.find(model => model.id === 'fanuc-crx10ia-l')!;
  const brokenUrl = model.files.find(file => file.path.endsWith('.stl'))!.url;
  let corrupt = true, offline = false, networkRequests = 0;
  await routeAssets(page, async route => {
    networkRequests++;
    if (offline) { await route.abort('internetdisconnected'); return; }
    if (corrupt && route.request().url() === brokenUrl) {
      const bytes = Buffer.from(fixtures.get(brokenUrl)!);
      bytes[bytes.length - 1] ^= 1;
      await route.fulfill({ status: 200, contentType: 'application/octet-stream', headers: { 'access-control-allow-origin': '*' }, body: bytes });
      return;
    }
    await realBytes(route);
  });
  await open(page);
  await checkedSegment(page);
  const program = await readRapid(page);
  await showPicker(page);
  await entry(page, model.id).getByRole('button', { name: /^加载 / }).click();
  await expect(picker(page).getByRole('alert')).toContainText(/校验|SHA|完整/);
  // The modal makes the old workspace inert, but its accepted state must remain.
  await expect(page.locator('button[aria-label="选择机器人型号"]')).toContainText('IRB 1100-4/0.58');
  expect(await page.locator('textarea[aria-label="RAPID MoveL 指令"]').inputValue()).toBe(program);
  corrupt = false;
  await page.getByRole('button', { name: '重试加载', exact: true }).click();
  await expectModel(page, model, 7);
  const onlineRequests = networkRequests;
  expect(onlineRequests).toBeGreaterThan(model.files.length);
  offline = true;
  await page.reload(); // Same origin/IndexedDB; the model host alone is offline.
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await showPicker(page);
  await expect(entry(page, model.id)).toContainText('已缓存');
  await entry(page, model.id).getByRole('button', { name: /^加载 / }).click();
  await expectModel(page, model, 7);
  expect(networkRequests).toBe(onlineRequests);
  expect(errors).toEqual([]);
});

test('all four downloadable robots render their own geometry, FK and brand-specific templates without controller export', async ({ page }) => {
  test.setTimeout(120_000);
  const errors = observe(page);
  await routeAssets(page);
  await open(page);
  const anchors: Record<string, { xyz: number[]; sign: number; visuals: number }> = {
    'fanuc-crx10ia-l': { xyz: [719.5, -150, 955], sign: 1, visuals: 7 },
    'fanuc-r2000ib-165f': { xyz: [1847.5, 0, 1970], sign: -1, visuals: 9 },
    'yaskawa-gp7': { xyz: [560, 0, 815], sign: 1, visuals: 7 },
    'yaskawa-gp110': { xyz: [1540, 0, 1645], sign: 1, visuals: 7 },
  };
  expect(downloads).toHaveLength(4);
  for (const model of downloads) {
    const expected = anchors[model.id];
    await selectRobot(page, model.id);
    await expectModel(page, model, expected.visuals);
    await expect(page.getByRole('heading', { name: '机器人属性', exact: true })).toBeVisible();
    await expect(page.getByRole('spinbutton', { name: /^机器人关节 J/ })).toHaveCount(6);
    await expect.poll(async () => Math.max(...(await toolPose(page)).position.map((value, i) => Math.abs(value - expected.xyz[i])))).toBeLessThan(.001);
    await expect(page.getByRole('button', { name: '检查到此点的 MoveL', exact: true })).toHaveCount(0);
    await expect(page.getByRole('button', { name: '拖动法兰', exact: true })).toHaveCount(0);
    await expect(page.getByRole('button', { name: '播放路径', exact: true })).toBeDisabled();
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
  }
  await selectRobot(page, 'irb1100-4-0.58');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await expect(page.getByRole('button', { name: '拖动法兰', exact: true })).toBeEnabled();
  await checkedSegment(page);
  await expectCopyState(page, true);
  expect(errors).toEqual([]);
});
