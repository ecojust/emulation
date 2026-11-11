import { expect, test, type Page } from '@playwright/test';
import { enterProject, waitForProjectScene } from './project-helpers';
import { bodyCanvas, bodyJoints, observeBrowser, openScene, selectRobotObject } from './workspace-helpers';
import { newSource } from './programming-helpers';

async function noDocumentOverflow(page: Page) {
  expect(await page.evaluate(() => Math.max(document.documentElement.scrollWidth - innerWidth,
    document.documentElement.scrollHeight - innerHeight, document.body.scrollWidth - innerWidth,
    document.body.scrollHeight - innerHeight))).toBeLessThanOrEqual(1);
}
async function contrastAt(page: Page, selector: string) {
  return page.locator(selector).first().evaluate(element => {
    const rgba = (color: string) => color.match(/[\d.]+/g)!.map(Number);
    const luminance = (rgb: number[]) => rgb.slice(0, 3).map(value => value / 255)
      .map(value => value <= .04045 ? value / 12.92 : ((value + .055) / 1.055) ** 2.4)
      .reduce((sum, value, axis) => sum + value * [.2126, .7152, .0722][axis], 0);
    let node: Element | null = element, background = [255, 255, 255];
    while (node) {
      const candidate = rgba(getComputedStyle(node).backgroundColor);
      if (candidate.length === 3 || candidate[3] === 1) { background = candidate; break; }
      node = node.parentElement;
    }
    const a = luminance(rgba(getComputedStyle(element).color)), b = luminance(background);
    return (Math.max(a, b) + .05) / (Math.min(a, b) + .05);
  });
}

test('light and dark themes persist across home and editor without replacing the robot or overflowing the window', async ({ page }) => {
  test.setTimeout(120_000);
  const errors = observeBrowser(page);
  await page.goto('/');
  const toggle = () => page.getByRole('button', { name: '切换主题', exact: true });
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'light');
  await expect(toggle()).toHaveAttribute('aria-pressed', 'false');
  await toggle().click();
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'dark');
  await expect(toggle()).toHaveAttribute('aria-pressed', 'true');
  expect(await contrastAt(page, '.project-home-brand strong')).toBeGreaterThanOrEqual(4.5);
  expect(await contrastAt(page, '.project-list-heading p')).toBeGreaterThanOrEqual(4.5);
  await page.screenshot({ path: '/tmp/emulation-theme-home-dark.png' });
  await page.reload();
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'dark');
  await enterProject(page, 'Theme verification');
  await waitForProjectScene(page);
  await selectRobotObject(page);
  const canvas = bodyCanvas(page), originalCanvas = (await canvas.elementHandle())!;
  await expect(canvas).toHaveAttribute('data-theme', 'dark');
  const joints = await bodyJoints(page);
  for (const theme of ['light', 'dark'] as const) {
    if (await page.locator('html').getAttribute('data-theme') !== theme) await toggle().click();
    await expect(canvas).toHaveAttribute('data-theme', theme);
    expect(await originalCanvas.evaluate(element => element.isConnected)).toBe(true);
    expect(await bodyJoints(page)).toEqual(joints);
    for (const selector of ['.object-panel-header h2', '.field-label', '.field-hint', '.project-robot-model']) {
      expect(await contrastAt(page, selector), `${theme} ${selector}`).toBeGreaterThanOrEqual(4.5);
    }
    await page.setViewportSize({ width: 1000, height: 720 });
    await noDocumentOverflow(page);
    const tabs = (await page.getByLabel('工作区视图', { exact: true }).boundingBox())!;
    const model = (await page.getByLabel('项目机器人型号', { exact: true }).boundingBox())!;
    const themeButton = (await toggle().boundingBox())!;
    expect(Math.abs(tabs.x + tabs.width / 2 - 500)).toBeLessThanOrEqual(1);
    expect(themeButton.x).toBeGreaterThan(tabs.x + tabs.width);
    expect(model.x + model.width).toBeLessThanOrEqual(1000);
    await page.setViewportSize({ width: 1440, height: 1000 });
    await page.screenshot({ path: `/tmp/emulation-theme-${theme}.png` });
  }
  await newSource(page, 'Theme.mod', 'MODULE Theme\n  PROC main()\n    ! Visible source on both themes\n    WaitTime 0.1;\n  ENDPROC\nENDMODULE');
  expect(await contrastAt(page, '.cm-line')).toBeGreaterThanOrEqual(4.5);
  await page.screenshot({ path: '/tmp/emulation-theme-program-dark.png' });
  await openScene(page);
  await page.getByRole('button', { name: '返回主页', exact: true }).click();
  await expect(page.getByRole('main', { name: '项目管理', exact: true })).toBeVisible();
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'dark');
  await expect(toggle()).toHaveAttribute('aria-pressed', 'true');
  await noDocumentOverflow(page);
  expect(errors).toEqual([]);
});
