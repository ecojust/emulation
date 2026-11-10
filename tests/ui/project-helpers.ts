import { expect, type Page } from '@playwright/test';

export const DEFAULT_TEST_PROJECT = 'UI project';

/** Enter through the same project home and create dialog used by the desktop client. */
export async function enterProject(page: Page, name = DEFAULT_TEST_PROJECT, modelId = 'irb1100-4-0.58') {
  const home = page.getByRole('main', { name: '项目管理', exact: true });
  await expect(home).toBeVisible();
  const create = home.getByRole('button', { name: '新建项目', exact: true }).first();
  await expect(create).toBeEnabled();
  const existing = home.getByRole('button', { name: '打开项目 ' + name, exact: true });
  if (await existing.count()) await existing.click();
  else {
    await create.click();
    const dialog = page.getByRole('dialog', { name: '新建项目', exact: true });
    await dialog.getByRole('textbox', { name: '项目名称', exact: true }).fill(name);
    await dialog.getByRole('combobox', { name: '项目机器人', exact: true }).selectOption(modelId);
    await dialog.getByRole('button', { name: '创建并打开', exact: true }).click();
  }
  await expect(page.locator('[data-project-id]')).toBeVisible();
  return (await page.locator('[data-project-id]').getAttribute('data-project-id'))!;
}

export async function openDefaultProject(page: Page) {
  await page.goto('/');
  return enterProject(page);
}

/** Project entry may intentionally stop at the download screen. Tests that
 * need a rendered editor must also await model verification and mesh loading.
 * Reopened projects can restore the program view, which hides the scene. */
export async function waitForProjectScene(page: Page, expectedModelId?: string) {
  const model = page.getByLabel('项目机器人型号', { exact: true });
  await expect(model).toBeVisible({ timeout: 30_000 });
  if (expectedModelId) await expect(model).toHaveAttribute('data-model-id', expectedModelId);
  await page.getByRole('button', { name: '场景', exact: true }).click();
  await expect(page.getByRole('region', { name: '机器人视口', exact: true })).toBeVisible();
  await expect(page.locator('.scene-model-status')).toContainText('机器人模型已加载', { timeout: 30_000 });
}
