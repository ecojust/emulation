import { expect, test, type Page } from '@playwright/test';
import { bodyJoints, captureCurrent, openProgram, openScene, programRows, selectRobotObject } from './workspace-helpers';

async function noDocumentScroll(page: Page) {
  await expect.poll(() => page.evaluate(() => {
    const root = document.documentElement, body = document.body;
    return Math.max(root.scrollWidth - innerWidth, root.scrollHeight - innerHeight, body.scrollWidth - innerWidth, body.scrollHeight - innerHeight, Math.abs(scrollX), Math.abs(scrollY));
  })).toBeLessThanOrEqual(1);
}
async function sceneSized(page: Page) {
  const scene = page.getByLabel('ABB IRB 1100 三维仿真场景');
  const host = page.getByLabel('机器人三维视图', { exact: true });
  await expect.poll(async () => {
    const canvasBox = (await scene.boundingBox())!, hostBox = (await host.boundingBox())!;
    return Math.max(Math.abs(canvasBox.width - hostBox.width), Math.abs(canvasBox.height - hostBox.height));
  }).toBeLessThanOrEqual(1);
  return (await scene.boundingBox())!;
}

test('resizing the fixed workspace preserves object-only navigation, local inspectors and program scrolling', async ({ page }) => {
  test.setTimeout(90_000);
  const errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  await page.goto('/');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await selectRobotObject(page);
  const objects = page.getByRole('complementary', { name: '场景对象', exact: true });
  const controls = page.getByRole('complementary', { name: '对象属性', exact: true });
  await expect(objects.getByRole('button', { name: '选择机器人', exact: true })).toBeVisible();
  await expect(objects.getByRole('button', { name: '选择危险区 危险区 1', exact: true })).toBeVisible();
  await expect(objects.getByLabel('当前 TCP', { exact: true })).toHaveCount(0);
  await expect(objects.getByLabel('当前关节姿态', { exact: true })).toHaveCount(0);
  await expect(objects.getByRole('spinbutton')).toHaveCount(0);
  await expect(controls.getByLabel('当前 TCP', { exact: true })).toBeVisible();
  await expect(controls.getByLabel('当前关节姿态', { exact: true })).toBeVisible();
  await expect(controls.getByRole('spinbutton', { name: /^机器人关节 J/ })).toHaveCount(6);
  const dh = controls.locator('details').filter({ has: page.locator('summary').filter({ hasText: 'DH 参数与关节范围' }) });
  await expect(dh.getByRole('table')).toBeHidden();
  await dh.locator('summary').click();
  await expect(dh.getByRole('table')).toBeVisible();
  await expect(dh.locator('tbody tr')).toHaveCount(6);
  await dh.locator('summary').click();
  await expect(dh.getByRole('table')).toBeHidden();
  for (const size of [{ width: 1440, height: 1000 }, { width: 1000, height: 720 }, { width: 1000, height: 1000 }]) {
    await page.setViewportSize(size);
    await noDocumentScroll(page);
    const scene = page.getByLabel('ABB IRB 1100 三维仿真场景');
    const expanded = await sceneSized(page);
    expect(expanded.width).toBeGreaterThan(250);
    expect(expanded.height).toBeGreaterThan(200);
    await page.getByRole('button', { name: '折叠左侧面板', exact: true }).click();
    await expect(page.locator('[data-left-collapsed]')).toHaveAttribute('data-left-collapsed', 'true');
    await sceneSized(page);
    await expect.poll(async () => (await scene.boundingBox())!.width).toBeGreaterThan(expanded.width + 80);
    await noDocumentScroll(page);
    await page.getByRole('button', { name: '展开左侧面板', exact: true }).click();
    await expect(page.locator('[data-left-collapsed]')).toHaveAttribute('data-left-collapsed', 'false');
    await sceneSized(page);
    await expect.poll(async () => Math.abs((await scene.boundingBox())!.width - expanded.width)).toBeLessThanOrEqual(1);
    await noDocumentScroll(page);
  }
  await page.setViewportSize({ width: 1000, height: 720 });
  await sceneSized(page);
  await dh.locator('summary').click();
  await expect(dh.getByRole('table')).toBeVisible();
  const box = (await controls.boundingBox())!;
  await page.mouse.move(box.x + box.width - 20, box.y + box.height / 2);
  await page.mouse.wheel(0, -10000);
  await expect.poll(() => controls.evaluate(element => element.scrollTop)).toBe(0);
  expect(await controls.evaluate(element => element.scrollHeight - element.clientHeight)).toBeGreaterThan(10);
  await page.mouse.wheel(0, 700);
  await expect.poll(() => controls.evaluate(element => element.scrollTop)).toBeGreaterThan(10);
  await noDocumentScroll(page);
  await page.screenshot({ path: '/tmp/emulation-workspace-1000x720.png' });
  const originalJoints = await bodyJoints(page);
  await captureCurrent(page);
  const rows = programRows(page);
  const insert = page.getByRole('button', { name: '＋ 插入当前姿态', exact: true });
  // Populate through real capture callbacks so overflow represents actual
  // saved MoveL objects, not injected DOM or mocked program state.
  for (let count = 2; count <= 18; count++) {
    await insert.click();
    await expect(rows).toHaveCount(count);
  }
  const rowIds = await rows.evaluateAll(elements => elements.map(element => element.getAttribute('data-program-point-id')));
  const list = page.getByRole('listbox', { name: 'MoveL 程序指令', exact: true });
  expect(await list.evaluate(element => element.scrollHeight - element.clientHeight)).toBeGreaterThan(10);
  const listBox = (await list.boundingBox())!;
  await page.mouse.move(listBox.x + listBox.width / 2, listBox.y + listBox.height / 2);
  await page.mouse.wheel(0, -10000);
  await expect.poll(() => list.evaluate(element => element.scrollTop)).toBe(0);
  await page.mouse.wheel(0, 700);
  await expect.poll(() => list.evaluate(element => element.scrollTop)).toBeGreaterThan(10);
  await rows.first().click();
  await list.press('ControlOrMeta+a');
  await page.getByRole('button', { name: '转为 Path', exact: true }).click();
  await expect(objects.getByRole('button', { name: '选择路径 Path 1', exact: true })).toHaveClass(/selected/);
  await expect(objects.locator('[data-object-kind="target"]')).toHaveCount(18);
  await expect(rows.locator('.program-path-tag')).toHaveText(Array(18).fill('Path 1'));
  expect(await rows.evaluateAll(elements => elements.map(element => element.getAttribute('data-program-point-id')))).toEqual(rowIds);
  const objectList = objects.locator('.object-panel-content');
  expect(await objectList.evaluate(element => element.scrollHeight - element.clientHeight)).toBeGreaterThan(10);
  const objectBox = (await objectList.boundingBox())!;
  await page.mouse.move(objectBox.x + objectBox.width / 2, objectBox.y + objectBox.height / 2);
  await page.mouse.wheel(0, 700);
  await expect.poll(() => objectList.evaluate(element => element.scrollTop)).toBeGreaterThan(10);
  await noDocumentScroll(page);
  await page.screenshot({ path: '/tmp/emulation-program-1000x720.png' });
  await openScene(page);
  await sceneSized(page);
  expect(await bodyJoints(page)).toEqual(originalJoints);
  await expect(objects.getByRole('button', { name: '选择路径 Path 1', exact: true })).toHaveClass(/selected/);
  await openProgram(page);
  await expect(rows).toHaveCount(18);
  await expect(list.locator('[aria-selected="true"]')).toHaveCount(18);
  expect(await rows.evaluateAll(elements => elements.map(element => element.getAttribute('data-program-point-id')))).toEqual(rowIds);
  await noDocumentScroll(page);
  expect(errors).toEqual([]);
});
