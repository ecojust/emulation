import { readFile } from 'node:fs/promises';
import { expect, type Page } from '@playwright/test';
import { openProgram, openScene } from './workspace-helpers';

export const codeWorkspace = (page: Page) => page.getByRole('region', { name: '机器人源码工作区', exact: true });
export const codeEditor = (page: Page) => codeWorkspace(page).locator('.cm-content');
export const moduleList = (page: Page) => page.getByRole('listbox', { name: '程序模块列表', exact: true });

export async function openCode(page: Page) {
  await openProgram(page);
  await expect(codeWorkspace(page)).toBeVisible();
}
export async function newSource(page: Page, name: string, text = '') {
  await openCode(page);
  await page.getByRole('button', { name: '新建文件', exact: true }).click();
  const dialog = page.getByRole('dialog', { name: '新建源码文件', exact: true });
  await dialog.getByRole('textbox', { name: '源码文件名称', exact: true }).fill(name);
  await dialog.getByRole('button', { name: '确定', exact: true }).click();
  await expect(moduleList(page).getByRole('option', { name, exact: true })).toHaveAttribute('aria-selected', 'true');
  await expect(codeEditor(page)).toHaveAttribute('contenteditable', 'true');
  if (text) await replaceSource(page, text);
}
export async function replaceSource(page: Page, text: string) {
  const editor = codeEditor(page);
  await editor.click();
  await editor.press('ControlOrMeta+a');
  await page.keyboard.insertText(text);
  await editor.press('Escape');
}
/** Read through the real Copy shortcut, including CodeMirror's virtualized lines. */
export async function readSource(page: Page): Promise<string> {
  await page.context().grantPermissions(['clipboard-read', 'clipboard-write']);
  const editor = codeEditor(page);
  await editor.click();
  await editor.press('ControlOrMeta+a');
  await editor.press('ControlOrMeta+c');
  return page.evaluate(() => navigator.clipboard.readText());
}
export async function downloadSource(page: Page) {
  const pending = page.waitForEvent('download');
  await page.getByRole('button', { name: '导出源码', exact: true }).click();
  const download = await pending;
  const path = await download.path();
  expect(path).not.toBeNull();
  return { name: download.suggestedFilename(), bytes: await readFile(path!) };
}
/** A Path becomes a new module; converting never replaces an edited file. */
export async function generateSource(page: Page, pathId?: string): Promise<string> {
  await openScene(page);
  const path = pathId ? page.locator(`[data-object-kind="path"][data-object-id="${pathId}"]`)
    : page.locator('[data-object-kind="path"]').first();
  await expect(path).toBeVisible();
  await path.click({ button: 'right' });
  await page.getByRole('menu', { name: 'Path 右键菜单', exact: true }).getByRole('menuitem', { name: '转换为程序', exact: true }).click();
  await expect(codeWorkspace(page)).toBeVisible();
  await expect(moduleList(page).getByRole('option', { selected: true })).toHaveCount(1);
  await expect.poll(() => codeEditor(page).textContent()).toMatch(/MODULE|\/PROG|\/JOB/);
  return readSource(page);
}

export async function chooseCompletion(page: Page, name: string, mouse = false) {
  await codeEditor(page).press('Control+Space');
  const list = page.locator('.cm-tooltip-autocomplete');
  await expect(list).toBeVisible();
  const exactName = new RegExp('^' + name.replace(/[.*+?^${}()|[\]\\]/g, '\\$&') + '$');
  const option = list.getByRole('option').filter({ has: page.locator('.cm-completionLabel').filter({ hasText: exactName }) });
  await expect(option).toBeVisible();
  // CodeMirror's default interactionDelay is 75 ms after the menu opens.
  // Visible/selected options are not yet keyboard-interactive during that
  // guard; an earlier Enter intentionally falls through to insert a newline.
  await page.waitForTimeout(100);
  if (mouse) await option.click();
  else {
    for (let index = 0; index < 30; index++) {
      if (await list.locator('[aria-selected="true"] .cm-completionLabel').textContent() === name) break;
      await codeEditor(page).press('ArrowDown');
    }
    await expect(list.locator('[aria-selected="true"] .cm-completionLabel')).toHaveText(name);
    await codeEditor(page).press('Enter');
  }
  await expect(list).toBeHidden();
}
