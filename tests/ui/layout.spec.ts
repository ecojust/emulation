import { openDefaultProject, waitForProjectScene } from './project-helpers';
import { expect, test, type Page } from '@playwright/test';
import { bodyJoints, captureCurrent, openProgram, openScene, pathPoints, selectPath, selectRobotObject } from './workspace-helpers';
import { codeWorkspace, generateSource, moduleList, newSource, readSource } from './programming-helpers';

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

test('resizing the fixed workspace preserves scene objects and locally scrolling Path and module panels', async ({ page }) => {
  test.setTimeout(150_000);
  const errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  await openDefaultProject(page);
  await waitForProjectScene(page, 'irb1100-4-0.58');
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
  const first = await captureCurrent(page);
  const points = pathPoints(page, first.pathId);
  // Populate through real Path context-menu callbacks, preserving actual saved
  // poses instead of injecting DOM rows or application state.
  for (let count = 2; count <= 18; count++) {
    await captureCurrent(page, first.pathId);
    await expect(points).toHaveCount(count);
  }
  const pointIds = await points.evaluateAll(elements => elements.map(element => element.getAttribute('data-object-id')));
  const objectList = objects.locator('.object-panel-content');
  expect(await objectList.evaluate(element => element.scrollHeight - element.clientHeight)).toBeGreaterThan(10);
  const objectBox = (await objectList.boundingBox())!;
  await page.mouse.move(objectBox.x + objectBox.width / 2, objectBox.y + objectBox.height / 2);
  await page.mouse.wheel(0, -10000);
  await expect.poll(() => objectList.evaluate(element => element.scrollTop)).toBe(0);
  await page.mouse.wheel(0, 700);
  await expect.poll(() => objectList.evaluate(element => element.scrollTop)).toBeGreaterThan(10);
  await noDocumentScroll(page);
  await selectPath(page, first.pathId);
  const generated = await generateSource(page, first.pathId);
  expect(generated.match(/MoveL P\d+,/g)).toHaveLength(18);
  const generatedName = (await moduleList(page).getByRole('option', { selected: true }).getAttribute('aria-label'))!;
  await expect(objects).toHaveCount(0);
  await expect(controls).toHaveCount(0);
  await expect(page.getByRole('tablist', { name: '源码文件', exact: true })).toHaveCount(0);
  // File creation is a user action too: a long list must scroll independently
  // without bringing the old scene object inspector into the source workspace.
  for (let count = 2; count <= 22; count++) await newSource(page, `layout_${count}.modx`);
  const modules = moduleList(page), files = modules.getByRole('option');
  await expect(files).toHaveCount(22);
  const fileIds = await files.evaluateAll(elements => elements.map(element => element.getAttribute('data-source-file-id')));
  expect(await modules.evaluate(element => element.scrollHeight - element.clientHeight)).toBeGreaterThan(10);
  const moduleBox = (await modules.boundingBox())!;
  expect(moduleBox.x).toBeLessThan((await codeWorkspace(page).boundingBox())!.x);
  await page.mouse.move(moduleBox.x + moduleBox.width / 2, moduleBox.y + moduleBox.height / 2);
  await page.mouse.wheel(0, -10000);
  await expect.poll(() => modules.evaluate(element => element.scrollTop)).toBe(0);
  await page.mouse.wheel(0, 700);
  await expect.poll(() => modules.evaluate(element => element.scrollTop)).toBeGreaterThan(10);
  await noDocumentScroll(page);
  await files.last().press('Home');
  await expect(files.first()).toHaveAttribute('aria-selected', 'true');
  // Keyboard navigation reveals the first option, which may align its border
  // with the list viewport and leave the list's own top padding scrolled away.
  const moduleTopPadding = await modules.evaluate(element => parseFloat(getComputedStyle(element).paddingTop));
  await expect.poll(() => modules.evaluate(element => element.scrollTop)).toBeLessThanOrEqual(moduleTopPadding + 1);
  await expect(files.first()).toBeInViewport({ ratio: 1 });
  await noDocumentScroll(page);
  expect(await readSource(page)).toBe(generated);
  const sourceScroller = codeWorkspace(page).locator('.cm-scroller');
  expect(await sourceScroller.evaluate(element => element.scrollHeight - element.clientHeight)).toBeGreaterThan(10);
  const sourceBox = (await sourceScroller.boundingBox())!;
  await page.mouse.move(sourceBox.x + sourceBox.width / 2, sourceBox.y + sourceBox.height / 2);
  await page.mouse.wheel(0, -10000);
  await expect.poll(() => sourceScroller.evaluate(element => element.scrollTop)).toBe(0);
  await page.mouse.wheel(0, 700);
  await expect.poll(() => sourceScroller.evaluate(element => element.scrollTop)).toBeGreaterThan(10);
  await noDocumentScroll(page);
  await page.screenshot({ path: '/tmp/emulation-program-1000x720.png' });
  await openScene(page);
  await sceneSized(page);
  expect(await bodyJoints(page)).toEqual(originalJoints);
  await expect(objects.getByRole('button', { name: '选择路径 Path 1', exact: true })).toHaveClass(/selected/);
  await expect(points).toHaveCount(18);
  expect(await points.evaluateAll(elements => elements.map(element => element.getAttribute('data-object-id')))).toEqual(pointIds);
  await openProgram(page);
  await expect(files).toHaveCount(22);
  await expect(files.filter({ hasText: generatedName })).toHaveAttribute('aria-selected', 'true');
  expect(await files.evaluateAll(elements => elements.map(element => element.getAttribute('data-source-file-id')))).toEqual(fileIds);
  expect(await readSource(page)).toBe(generated);
  await noDocumentScroll(page);
  expect(errors).toEqual([]);
});
