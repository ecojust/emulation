import { expect, type Page } from '@playwright/test';

export async function openProgram(page: Page) {
  await page.getByRole('button', { name: '程序', exact: true }).click();
  await expect(page.getByRole('region', { name: 'RAPID 输出', exact: true })).toBeVisible();
}

export async function openScene(page: Page) {
  await page.getByRole('button', { name: '场景', exact: true }).click();
  await expect(page.getByLabel('ABB IRB 1100 三维仿真场景')).toBeVisible();
}

// Exercise the real navigation: inactive program controls are deliberately inert.
export async function expectCopyState(page: Page, enabled: boolean) {
  await openProgram(page);
  const button = page.getByRole('button', { name: '复制 MoveL', exact: true });
  if (enabled) await expect(button).toBeEnabled();
  else await expect(button).toBeDisabled();
  await openScene(page);
}

export async function readRapid(page: Page) {
  await openProgram(page);
  const instruction = await page.getByRole('textbox', { name: 'RAPID MoveL 指令', exact: true }).inputValue();
  await openScene(page);
  return instruction;
}
