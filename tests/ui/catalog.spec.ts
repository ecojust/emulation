import { expect, test, type Page, type Route } from '@playwright/test';
import type { RobotCatalog, RobotCatalogModel } from '../../src/catalog/types';
import type { Pose } from '../../src/robot/types';
import { prepareCatalogFixtures, readCatalog } from './catalog-fixtures';
import { expectCopyState, readRapid, selectRobot } from './workspace-helpers';

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
  await expect(page.getByRole('status', { name: '型号能力', exact: true })).toContainText('尚未适配');
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
  for (const model of downloads) await expect(entry(page, model.id)).toContainText('URDF 关节调整与正运动学');
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
    await page.getByRole('button', { name: '求解 MoveL', exact: true }).click();
    await expect(page.getByRole('heading', { name: '运动学可达', exact: true })).toBeVisible();
    const program = await readRapid(page);
    const originalJoints = await page.getByLabel('当前关节姿态', { exact: true }).locator('strong').allTextContents();
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
    expect(await page.getByLabel('当前关节姿态', { exact: true }).locator('strong').allTextContents()).toEqual(originalJoints);
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
  await page.getByRole('button', { name: '求解 MoveL', exact: true }).click();
  await expect(page.getByRole('heading', { name: '运动学可达', exact: true })).toBeVisible();
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

test('all four real downloadable robots render their own geometry and FK while unsupported motion stays unavailable', async ({ page }) => {
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
    await expect(page.getByRole('spinbutton', { name: /^模型关节 / })).toHaveCount(6);
    await expect.poll(async () => Math.max(...(await toolPose(page)).position.map((value, i) => Math.abs(value - expected.xyz[i])))).toBeLessThan(.001);
    await expect(page.getByRole('button', { name: '求解 MoveL', exact: true })).toHaveCount(0);
    await expect(page.getByRole('button', { name: '拖动法兰', exact: true })).toHaveCount(0);
    await expect(page.getByRole('button', { name: '播放路径', exact: true })).toBeDisabled();
    const before = await genericCanvas(page).screenshot();
    await page.getByRole('spinbutton', { name: /^模型关节 / }).first().fill('30');
    await expect.poll(async () => JSON.parse((await genericCanvas(page).getAttribute('data-urdf-joints'))!)[0]).toBe(30);
    const angle = expected.sign * Math.PI / 6, [x, y, z] = expected.xyz;
    const rotated = [x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle), z];
    await expect.poll(async () => Math.max(...(await toolPose(page)).position.map((value, i) => Math.abs(value - rotated[i])))).toBeLessThan(.001);
    for (const [index, axis] of ['X', 'Y', 'Z'].entries()) await expect(page.getByLabel('当前 TCP', { exact: true }).locator(`[data-axis="${axis}"] strong`)).toHaveText(rotated[index].toFixed(2));
    expect((await genericCanvas(page).screenshot()).equals(before)).toBe(false);
    await page.screenshot({ path: `/tmp/emulation-catalog-${model.id}.png` });
    await page.getByRole('button', { name: '程序', exact: true }).click();
    await expect(page.getByRole('region', { name: '程序输出', exact: true })).toContainText('控制器程序尚未适配');
    await expect(page.getByRole('button', { name: '复制 MoveL', exact: true })).toHaveCount(0);
    await expect(page.getByRole('button', { name: /导出 .*模块/ })).toHaveCount(0);
    await page.getByRole('button', { name: '场景', exact: true }).click();
    await noDocumentScroll(page);
  }
  await selectRobot(page, 'irb1100-4-0.58');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await expect(page.getByRole('button', { name: '拖动法兰', exact: true })).toBeEnabled();
  await page.getByRole('button', { name: '求解 MoveL', exact: true }).click();
  await expect(page.getByRole('heading', { name: '运动学可达', exact: true })).toBeVisible();
  await expectCopyState(page, true);
  expect(errors).toEqual([]);
});
