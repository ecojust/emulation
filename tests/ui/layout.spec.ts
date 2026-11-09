import { expect, test, type Page } from '@playwright/test';
import { openProgram, openScene } from './workspace-helpers';

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

test('resizing the fixed workspace preserves usable local panels, collapsible robot status and program scrolling', async ({ page }) => {
  test.setTimeout(90_000);
  const errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  await page.goto('/');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await page.getByRole('button', { name: '求解 MoveL', exact: true }).click();
  await expect(page.getByRole('heading', { name: '运动学可达', exact: true })).toBeVisible();
  const objects = page.getByRole('complementary', { name: '对象与状态', exact: true });
  for (const [summary, label] of [['当前位置 position', '当前 TCP'], ['当前姿态', '当前关节姿态']]) {
    const section = objects.locator('details').filter({ has: page.locator('summary').filter({ hasText: summary }) });
    await expect(objects.getByLabel(label, { exact: true })).toBeVisible();
    await section.locator('summary').click();
    await expect(objects.getByLabel(label, { exact: true })).toBeHidden();
    await section.locator('summary').click();
    await expect(objects.getByLabel(label, { exact: true })).toBeVisible();
  }
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
  const controls = page.locator('.control-panel');
  const box = (await controls.boundingBox())!;
  await page.mouse.move(box.x + box.width - 20, box.y + box.height / 2);
  await page.mouse.wheel(0, -10000);
  await expect.poll(() => controls.evaluate(element => element.scrollTop)).toBe(0);
  expect(await controls.evaluate(element => element.scrollHeight - element.clientHeight)).toBeGreaterThan(10);
  await page.mouse.wheel(0, 700);
  await expect.poll(() => controls.evaluate(element => element.scrollTop)).toBeGreaterThan(10);
  await noDocumentScroll(page);
  await page.screenshot({ path: '/tmp/emulation-workspace-1000x720.png' });
  await openProgram(page);
  const code = page.getByRole('textbox', { name: 'RAPID MoveL 指令', exact: true });
  await expect(code).toHaveValue(/MoveL/);
  expect(await code.evaluate(element => element.scrollWidth - element.clientWidth)).toBeGreaterThan(10);
  const codeBox = (await code.boundingBox())!;
  await page.mouse.move(codeBox.x + codeBox.width / 2, codeBox.y + codeBox.height / 2);
  await page.mouse.wheel(650, 0);
  await expect.poll(() => code.evaluate(element => element.scrollLeft)).toBeGreaterThan(10);
  await noDocumentScroll(page);
  await page.screenshot({ path: '/tmp/emulation-program-1000x720.png' });
  await openScene(page);
  await expect(page.getByRole('heading', { name: '运动学可达', exact: true })).toBeVisible();
  await noDocumentScroll(page);
  expect(errors).toEqual([]);
});
