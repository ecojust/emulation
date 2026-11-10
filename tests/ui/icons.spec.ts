import { openDefaultProject, waitForProjectScene } from './project-helpers';
import { expect, test, type Locator, type Page } from '@playwright/test';
import { chooseCompletion, codeEditor, moduleList, newSource, readSource, replaceSource } from './programming-helpers';
import { captureCurrent, expectNoPointMotionActions, observeBrowser, openPathMenu } from './workspace-helpers';

test.setTimeout(90_000);

async function openWorkspace(page: Page) {
  await openDefaultProject(page);
  await waitForProjectScene(page, 'irb1100-4-0.58');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
}

/** Inspect rendered geometry, rather than accepting text wrapped in an SVG. */
async function expectSvg(svg: Locator) {
  await expect(svg).toBeVisible();
  expect(await svg.evaluate(element => element.namespaceURI)).toBe('http://www.w3.org/2000/svg');
  expect(await svg.locator('path, circle, ellipse, line, polyline, polygon, rect').count()).toBeGreaterThan(0);
  await expect(svg.locator('text, tspan, foreignObject')).toHaveCount(0);
  await expect(svg).toHaveAttribute('aria-hidden', 'true');
  await expect(svg).toHaveAttribute('focusable', 'false');
}

async function expectControlIcon(control: Locator) {
  await expectSvg(control.locator('svg').first());
}

async function expectSvgDisclosure(summary: Locator) {
  await expectControlIcon(summary);
  expect(await summary.evaluate(element => getComputedStyle(element).listStyleType)).toBe('none');
  expect(await summary.evaluate(element => getComputedStyle(element, '::before').content)).toMatch(/^(none|normal|"")$/);
}

async function noDocumentScroll(page: Page) {
  expect(await page.evaluate(() => Math.max(
    document.documentElement.scrollWidth - innerWidth,
    document.documentElement.scrollHeight - innerHeight,
    Math.abs(scrollX), Math.abs(scrollY),
  ))).toBeLessThanOrEqual(1);
}

test('scene icons use SVG while disclosures, object menus and fixed project navigation remain operable', async ({ page }) => {
  const errors = observeBrowser(page);
  await page.setViewportSize({ width: 1000, height: 720 });
  await openWorkspace(page);
  for (const name of ['返回主页', '折叠左侧面板', '选择机器人', '添加 Path', '添加危险区', '重置视角']) {
    await expectControlIcon(page.getByRole('button', { name, exact: true }));
  }
  await page.getByRole('button', { name: '折叠左侧面板', exact: true }).click();
  const expand = page.getByRole('button', { name: '展开左侧面板', exact: true });
  await expectControlIcon(expand);
  await expect(page.getByRole('button', { name: '添加 Path', exact: true })).toHaveCount(0);
  await noDocumentScroll(page);
  await expand.click();

  const dh = page.locator('.dh-details'), dhSummary = dh.locator('summary');
  await expectSvgDisclosure(dhSummary);
  await dhSummary.click();
  await expect(dh).toHaveAttribute('open', '');
  await expect(dh.getByRole('table')).toBeVisible();
  await expect(dh.getByRole('row')).toHaveCount(7);
  await noDocumentScroll(page);
  await dhSummary.click();
  await expect(dh.getByRole('table')).toBeHidden();

  const point = await captureCurrent(page);
  await expectNoPointMotionActions(page);
  const target = page.locator(`[data-object-kind="target"][data-object-id="${point.id}"]`);
  const path = page.locator(`[data-object-kind="path"][data-object-id="${point.pathId}"]`);
  await expectControlIcon(target);
  await expectControlIcon(path);
  const pathDetails = page.locator('details.object-path').filter({ has: path });
  const pathSummary = pathDetails.locator('summary');
  await expectSvgDisclosure(pathSummary);
  // Use the disclosure's actual hit area, outside its independently selectable Path button.
  const summaryBounds = (await pathSummary.boundingBox())!;
  const arrowBounds = (await pathSummary.locator(':scope > svg').boundingBox())!;
  const disclosurePosition = { x: arrowBounds.x + arrowBounds.width / 2 - summaryBounds.x, y: arrowBounds.y + arrowBounds.height / 2 - summaryBounds.y };
  await pathSummary.click({ position: disclosurePosition });
  await expect(pathDetails).not.toHaveAttribute('open');
  await expect(target).toBeHidden();
  await pathSummary.click({ position: disclosurePosition });
  await expect(target).toBeVisible();

  const menu = await openPathMenu(page, point.pathId);
  for (const name of ['运行', '添加当前点到 Path', '转换为程序', '删除 Path']) {
    await expectControlIcon(menu.getByRole('menuitem', { name, exact: true }));
  }
  await noDocumentScroll(page);
  await menu.press('Escape');
  await expect(menu).toHaveCount(0);
  await expect(path).toBeFocused();
  await target.press('Shift+F10');
  const pointMenu = page.getByRole('menu', { name: '目标点右键菜单', exact: true });
  await expectControlIcon(pointMenu.getByRole('menuitem', { name: '移动至此', exact: true }));
  await expectControlIcon(pointMenu.getByRole('menuitem', { name: '删除点', exact: true }));
  await pointMenu.press('Escape');
  await expect(target).toBeFocused();

  const zones = page.locator('[data-object-kind="zone"]'), zoneCount = await zones.count();
  await page.getByRole('button', { name: '添加危险区', exact: true }).click();
  await expect(zones).toHaveCount(zoneCount + 1);
  await expectControlIcon(zones.last());
  await expect(page.getByRole('heading', { name: '危险区属性', exact: true })).toBeVisible();
  await expect(page.getByRole('button', { name: '选择机器人型号', exact: true })).toHaveCount(0);
  await page.getByRole('button', { name: '返回主页', exact: true }).click();
  const home = page.getByRole('main', { name: '项目管理', exact: true });
  const create = home.getByRole('button', { name: '新建项目', exact: true }).first();
  await expectControlIcon(create);
  await create.click();
  const dialog = page.getByRole('dialog', { name: '新建项目', exact: true });
  const close = dialog.getByRole('button', { name: '关闭项目对话框', exact: true });
  await expectControlIcon(close);
  await noDocumentScroll(page);
  await close.click();
  await expect(dialog).toHaveCount(0);
  await noDocumentScroll(page);
  expect(errors).toEqual([]);
});

test('module and CodeMirror SVG icons retain folding, completion, search and diagnostic behavior', async ({ page }) => {
  const errors = observeBrowser(page);
  await page.setViewportSize({ width: 1000, height: 720 });
  await openWorkspace(page);
  const original = 'MODULE Icons\nPROC main()\nWaitTime 0.01;\nENDPROC\nENDMODULE';
  await newSource(page, 'icons.modx', original);
  const file = moduleList(page).getByRole('option', { name: 'icons.modx', exact: true });
  await expectSvg(file.locator('svg.module-file-icon'));
  await expectSvg(file.locator('svg.module-file-dirty'));
  await expectSvg(page.getByLabel('当前模块', { exact: true }).getByLabel('已修改', { exact: true }).locator('svg'));

  // The gutter begins with an invisible width-measurement marker; interact
  // with the real first source line rather than that non-interactive spacer.
  const marker = page.locator('.cm-foldGutter .robot-fold-marker:visible').first();
  await expectControlIcon(marker);
  await expect(marker.locator('svg')).toHaveAttribute('data-icon', 'chevron-down');
  await marker.click();
  const placeholder = codeEditor(page).locator('.cm-foldPlaceholder');
  await expectControlIcon(placeholder);
  await expect(marker.locator('svg')).toHaveAttribute('data-icon', 'chevron-right');
  await placeholder.click();
  await expect(placeholder).toHaveCount(0);
  expect(await readSource(page)).toBe(original);

  await page.getByRole('button', { name: '查找', exact: true }).click();
  const search = page.locator('.cm-search');
  await expect(search).toBeVisible();
  await search.locator('input[name="search"]').fill('WaitTime');
  const searchClose = search.locator('button[name="close"]');
  await expectControlIcon(searchClose);
  await noDocumentScroll(page);
  await searchClose.click();
  await expect(search).toHaveCount(0);
  expect(await readSource(page)).toBe(original);

  await page.getByRole('tab', { name: '指令参考', exact: true }).click();
  const notes = page.locator('.robot-code-language-notes');
  await expectSvgDisclosure(notes.locator('summary'));
  await notes.locator('summary').click();
  await expect(notes.locator('ul')).toBeVisible();
  await noDocumentScroll(page);
  await notes.locator('summary').click();
  await expect(notes.locator('ul')).toBeHidden();

  await replaceSource(page, 'Mov');
  await codeEditor(page).press('Control+Space');
  const completion = page.locator('.cm-tooltip-autocomplete');
  await expect(completion).toBeVisible();
  const moveL = completion.getByRole('option').filter({ has: page.locator('.cm-completionLabel').filter({ hasText: /^MoveL$/ }) });
  await expectControlIcon(moveL.locator('.robot-completion-icon'));
  await expect(completion.locator('.cm-completionIcon')).toHaveCount(0);
  await chooseCompletion(page, 'MoveL', true);
  expect(await readSource(page)).toMatch(/^MoveL\s+.*;/);

  await replaceSource(page, 'MODULE Broken\nPROC main()\nVAR num n:=;\nENDPROC\nENDMODULE');
  const diagnostic = page.locator('.cm-lint-marker-error').first();
  await expect(diagnostic).toBeVisible();
  // CodeMirror already renders diagnostic markers as SVG image content.
  expect(await diagnostic.evaluate(element => {
    const style = getComputedStyle(element);
    return [style.content, style.backgroundImage, style.maskImage].join(' ');
  })).toContain('image/svg+xml');
  // Physical KeyM emits Shift's uppercase key; a literal lowercase "m"
  // incorrectly reaches CodeMirror's separate Ctrl-m tab-focus command.
  await codeEditor(page).press('ControlOrMeta+Shift+KeyM');
  const lintPanel = page.locator('.cm-panel-lint');
  await expect(lintPanel).toBeVisible();
  const lintClose = lintPanel.locator('button[name="close"]');
  await expectControlIcon(lintClose);
  await noDocumentScroll(page);
  await lintClose.click();
  await expect(lintPanel).toHaveCount(0);
  await noDocumentScroll(page);
  expect(errors).toEqual([]);
});
