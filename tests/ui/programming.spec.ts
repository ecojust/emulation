import { expect, test, type Page } from '@playwright/test';
import { parseRobtarget } from '../../src/robot/rapid';
import { captureCurrent, observeBrowser, openScene, pathPoints, selectRobot, selectRobotObject, setJoint } from './workspace-helpers';
import { chooseCompletion, codeEditor, codeWorkspace, downloadSource, generateSource, newSource, openCode, readSource, replaceSource, moduleList } from './programming-helpers';

test.setTimeout(120_000);
test.use({ actionTimeout: 15_000 });
const clean = (name: string, value = 1) => `MODULE ${name}\nVAR num count:=${value};\nENDMODULE`;
const sourceStatus = (page: Page) => page.getByRole('status', { name: '源码状态', exact: true });
// The editor suite also runs against the release build. Use user-facing
// readouts here; the separate motion suite retains its DEV-only rig probes.
async function openWorkspace(page: Page) {
  await page.goto('/');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await expect(page.getByLabel('ABB IRB 1100 三维仿真场景')).toBeVisible();
  await selectRobotObject(page);
  await expect(page.getByLabel('当前关节姿态', { exact: true }).locator('strong')).toHaveCount(6);
}
async function bodyJoints(page: Page): Promise<number[]> {
  const wasProgram = await page.getByRole('button', { name: '程序', exact: true }).getAttribute('aria-pressed') === 'true';
  await openScene(page);
  await selectRobotObject(page);
  const joints = (await page.getByLabel('当前关节姿态', { exact: true }).locator('strong').allTextContents()).map(text => Number(text.replace('°', '')));
  if (wasProgram) await openCode(page);
  return joints;
}
async function bodyPose(page: Page) {
  const wasProgram = await page.getByRole('button', { name: '程序', exact: true }).getAttribute('aria-pressed') === 'true';
  await openScene(page);
  await selectRobotObject(page);
  const position = (await page.getByLabel('当前 TCP', { exact: true }).locator('strong').allTextContents()).map(Number);
  if (wasProgram) await openCode(page);
  return { position };
}
async function saveDraft(page: Page) {
  await page.getByRole('button', { name: '保存草稿', exact: true }).click();
  await expect(sourceStatus(page)).toContainText('草稿已保存');
  await expect(page.getByRole('button', { name: '保存草稿', exact: true })).toBeEnabled();
}
async function noDocumentScroll(page: Page) {
  await expect.poll(() => page.evaluate(() => Math.max(document.documentElement.scrollWidth - innerWidth,
    document.documentElement.scrollHeight - innerHeight, document.body.scrollWidth - innerWidth,
    document.body.scrollHeight - innerHeight, Math.abs(scrollX), Math.abs(scrollY)))).toBeLessThanOrEqual(1);
}


test('real code input highlights tokens, accepts keyboard and mouse completion, and shows parameter documentation', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const before = await bodyJoints(page);
  await openCode(page);
  await expect(moduleList(page).getByRole('option')).toHaveCount(0);
  await expect(page.getByRole('tablist', { name: '源码文件', exact: true })).toHaveCount(0);
  await expect(page.getByRole('button', { name: '选择机器人', exact: true })).toHaveCount(0);
  await newSource(page, 'keyboard.modx', 'Mov');
  await codeEditor(page).press('ControlOrMeta+End');
  await chooseCompletion(page, 'MoveL');
  await expect(codeEditor(page)).toContainText('ToPoint');
  await page.keyboard.insertText('p_check');
  await codeEditor(page).press('Tab');
  await page.keyboard.insertText('v100');
  expect(await readSource(page)).toMatch(/MoveL\s+p_check\s*,\s*v100/);
  await replaceSource(page, 'Mov');
  await codeEditor(page).press('ControlOrMeta+End');
  await chooseCompletion(page, 'MoveL', true);
  await expect(codeEditor(page)).toContainText('ToPoint');
  const source = 'MODULE Keyboard\nVAR num count:=37;\nPROC main()\n! 独立注释\nTPWrite "你好";\nMoveL p_check,v100,fine,tool0;\nENDPROC\nENDMODULE';
  await replaceSource(page, source);
  const tokens = ['MODULE', '37', '"你好"', '! 独立注释'];
  const colors: string[] = [];
  for (const text of tokens) {
    const token = codeEditor(page).locator('span').filter({ hasText: new RegExp('^' + text + '$') }).first();
    await expect(token).toBeVisible();
    colors.push(await token.evaluate(element => getComputedStyle(element).color));
  }
  expect(new Set(colors).size).toBe(4);
  await codeEditor(page).locator('span').filter({ hasText: /^MoveL$/ }).first().hover();
  const hover = page.locator('.cm-tooltip-hover .robot-code-tooltip');
  await expect(hover).toBeVisible();
  await expect(hover.locator('strong')).toHaveText('MoveL');
  await expect(hover.locator('dt').filter({ hasText: /ToPoint.*robtarget/ })).toBeVisible();
  await expect(hover.locator('dt').filter({ hasText: /Speed.*speeddata/ })).toBeVisible();
  await noDocumentScroll(page);
  await page.getByRole('button', { name: '程序', exact: true }).hover();
  await expect(hover).toBeHidden();
  await page.getByRole('tab', { name: '指令参考', exact: true }).click();
  await page.getByRole('textbox', { name: '查找指令参考', exact: true }).fill('MoveL');
  await page.getByRole('listbox', { name: '指令参考列表', exact: true }).getByRole('option', { name: 'MoveL', exact: true }).click();
  await expect(page.locator('.robot-code-reference-detail')).toContainText('ToPoint');
  await expect(page.locator('.robot-code-reference-detail a')).toHaveAttribute('href', /^https:\/\//);
  await page.screenshot({ path: '/tmp/emulation-source-editor-reference.png' });
  expect(await bodyJoints(page)).toEqual(before);
  expect(errors).toEqual([]);
});

test('diagnostics navigate to the actual file and selection and follow dialect and latest edits', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  // This case isolates grammar-version changes across the whole project.
  // A modern extension would correctly add its own RW6 compatibility warning.
  await newSource(page, 'EMULATE.mod', clean('EMULATE'));
  await newSource(page, 'bad.mod', 'MODULE Broken\nPROC main()\nVAR num n:=;\nENDPROC\nENDMODULE');
  const diagnostic = page.locator('.robot-code-diagnostics button').filter({ hasText: 'bad.mod:3:' });
  await expect(diagnostic.first()).toBeVisible();
  await newSource(page, 'good.mod', clean('Good'));
  await diagnostic.first().click();
  await expect(moduleList(page).getByRole('option', { name: 'bad.mod', exact: true })).toHaveAttribute('aria-selected', 'true');
  await expect(codeEditor(page)).toBeFocused();
  await expect.poll(() => page.evaluate(() => window.getSelection()?.toString())).toBe(';');
  await replaceSource(page, 'MODULE Broken\nVAR num Break:=1;\nENDMODULE');
  await expect(page.locator('.robot-code-diagnostics .severity-error')).not.toHaveCount(0);
  const dialect = page.getByRole('combobox', { name: '控制器语言版本', exact: true });
  await dialect.selectOption('rapid-rw6');
  await expect(page.locator('.robot-code-diagnostics .severity-error')).toHaveCount(0);
  await expect(page.locator('.robot-code-empty')).toContainText('当前检查未报告问题');
  await dialect.selectOption('rapid-rw7');
  await expect(page.locator('.robot-code-diagnostics .severity-error')).not.toHaveCount(0);
  // Two real edits replace an invalid revision before its analysis can arrive.
  await replaceSource(page, 'MODULE Broken\nVAR num n:=;\nENDMODULE');
  await replaceSource(page, clean('Repaired', 9));
  await expect(page.locator('.robot-code-diagnostics .severity-error')).toHaveCount(0);
  await expect(page.locator('.robot-code-empty')).toContainText('当前检查未报告问题');
  expect(await readSource(page)).toBe(clean('Repaired', 9));
  await moduleList(page).getByRole('option', { name: 'good.mod', exact: true }).click();
  expect(await readSource(page)).toBe(clean('Good'));
  expect(errors).toEqual([]);
});

test('real IndexedDB drafts isolate two robot projects and restore files and dialect with external networking blocked', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  await newSource(page, 'shared.mod', clean('LongArm', 58));
  await newSource(page, 'second.mod', clean('Second', 2));
  await page.getByRole('combobox', { name: '控制器语言版本', exact: true }).selectOption('rapid-rw6');
  await page.getByRole('button', { name: '重命名', exact: true }).click();
  const rename = page.getByRole('dialog', { name: '重命名源码文件', exact: true });
  await rename.getByRole('textbox', { name: '源码文件名称', exact: true }).fill('renamed.mod');
  await rename.getByRole('button', { name: '确定', exact: true }).click();
  await saveDraft(page);
  await expect(moduleList(page).getByRole('option', { name: 'renamed.mod', exact: true })).toHaveAttribute('data-source-dirty', 'true');
  await expect(moduleList(page).getByRole('option')).toHaveCount(2);
  await moduleList(page).getByRole('option', { name: 'renamed.mod', exact: true }).press('Home');
  await expect(moduleList(page).getByRole('option', { name: 'shared.mod', exact: true })).toHaveAttribute('aria-selected', 'true');
  expect(await readSource(page)).toBe(clean('LongArm', 58));
  await moduleList(page).getByRole('option', { name: 'shared.mod', exact: true }).press('ArrowDown');
  await expect(moduleList(page).getByRole('option', { name: 'renamed.mod', exact: true })).toHaveAttribute('aria-selected', 'true');
  expect(await readSource(page)).toBe(clean('Second', 2));
  await moduleList(page).getByRole('option', { name: 'renamed.mod', exact: true }).press('ArrowUp');
  await expect(moduleList(page).getByRole('option', { name: 'shared.mod', exact: true })).toHaveAttribute('aria-selected', 'true');
  await moduleList(page).getByRole('option', { name: 'shared.mod', exact: true }).press('End');
  await expect(moduleList(page).getByRole('option', { name: 'renamed.mod', exact: true })).toHaveAttribute('aria-selected', 'true');
  await saveDraft(page);
  await selectRobot(page, 'irb1100-4-0.475');
  await newSource(page, 'shared.mod', clean('ShortArm', 475));
  await newSource(page, 'delete.mod', clean('Temporary'));
  await page.getByRole('button', { name: '删除文件', exact: true }).click();
  await page.getByRole('dialog', { name: '删除源码文件', exact: true }).getByRole('button', { name: '删除', exact: true }).click();
  await expect(moduleList(page).getByRole('option', { name: 'delete.mod', exact: true })).toHaveCount(0);
  await moduleList(page).getByRole('option', { name: 'shared.mod', exact: true }).click();
  await saveDraft(page);
  const external: string[] = [];
  const origin = new URL(page.url()).origin;
  await page.route('**/*', async route => {
    if (new URL(route.request().url()).origin !== origin) { external.push(route.request().url()); await route.abort('internetdisconnected'); }
    else await route.continue();
  });
  await page.reload();
  await selectRobot(page, 'irb1100-4-0.475');
  await openCode(page);
  await expect(moduleList(page).getByRole('option', { name: 'shared.mod', exact: true })).toHaveAttribute('aria-selected', 'true');
  await expect(moduleList(page).getByRole('option', { name: 'renamed.mod', exact: true })).toHaveCount(0);
  await expect(moduleList(page).getByRole('option', { name: 'delete.mod', exact: true })).toHaveCount(0);
  expect(await readSource(page)).toBe(clean('ShortArm', 475));
  await selectRobot(page, 'irb1100-4-0.58');
  await openCode(page);
  await expect(moduleList(page).getByRole('option', { name: 'shared.mod', exact: true })).toHaveCount(1);
  await expect(moduleList(page).getByRole('option', { name: 'renamed.mod', exact: true })).toHaveAttribute('aria-selected', 'true');
  await expect(moduleList(page).getByRole('option', { name: 'renamed.mod', exact: true })).toHaveAttribute('data-source-dirty', 'true');
  await expect(page.getByRole('combobox', { name: '控制器语言版本', exact: true })).toHaveValue('rapid-rw6');
  expect(await readSource(page)).toBe(clean('Second', 2));
  await moduleList(page).getByRole('option', { name: 'shared.mod', exact: true }).click();
  expect(await readSource(page)).toBe(clean('LongArm', 58));
  await page.getByRole('tab', { name: '指令参考', exact: true }).click();
  await page.getByRole('textbox', { name: '查找指令参考', exact: true }).fill('MoveL');
  await expect(page.getByRole('listbox', { name: '指令参考列表', exact: true }).getByRole('option', { name: 'MoveL', exact: true })).toBeVisible();
  expect(external).toEqual([]);
  expect(errors).toEqual([]);
});

test('file import and downloads preserve raw bytes while Path conversion creates an independent module', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const point = await captureCurrent(page);
  await openCode(page);
  const original = '\ufeffMODULE Imported\r\nPROC main()\r\n! 保留原始编码与换行\r\nENDPROC\r\nENDMODULE\r\n';
  const chooser = page.waitForEvent('filechooser');
  await page.getByRole('button', { name: '导入源码', exact: true }).click();
  await (await chooser).setFiles({ name: 'imported.mod', mimeType: 'text/plain', buffer: Buffer.from(original) });
  const tab = moduleList(page).getByRole('option', { name: 'imported.mod', exact: true });
  await expect(tab).toHaveAttribute('aria-selected', 'true');
  await expect(tab).toHaveAttribute('data-source-dirty', 'false');
  expect((await downloadSource(page)).bytes.equals(Buffer.from(original))).toBe(true);
  await codeEditor(page).click();
  await codeEditor(page).press('ControlOrMeta+End');
  await page.keyboard.insertText('! edited');
  const edited = original + '! edited';
  const downloaded = await downloadSource(page);
  expect(downloaded.name).toBe('imported.mod');
  expect(downloaded.bytes.equals(Buffer.from(edited))).toBe(true);
  await expect(tab).toHaveAttribute('data-source-dirty', 'true');
  await expect(sourceStatus(page)).toContainText('浏览器未确认');
  const generated = await generateSource(page, point.pathId);
  expect(generated).toContain('MoveL P001');
  const generatedName = await moduleList(page).getByRole('option', { selected: true }).getAttribute('aria-label');
  expect(generatedName).toMatch(/\.modx$/);
  const editedGenerated = generated + '\n! Keep this module edit';
  await replaceSource(page, editedGenerated);
  const regenerated = await generateSource(page, point.pathId);
  expect(regenerated).toContain('MoveL P001');
  expect(regenerated).not.toContain('Keep this module edit');
  const regeneratedName = await moduleList(page).getByRole('option', { selected: true }).getAttribute('aria-label');
  expect(regeneratedName).not.toBe(generatedName);
  await expect(moduleList(page).getByRole('option')).toHaveCount(3);
  await moduleList(page).getByRole('option', { name: generatedName!, exact: true }).click();
  expect(await readSource(page)).toBe(editedGenerated);
  await tab.click();
  expect((await downloadSource(page)).bytes.equals(Buffer.from(edited))).toBe(true);
  await page.getByRole('button', { name: '撤销', exact: true }).click();
  expect((await downloadSource(page)).bytes.equals(Buffer.from(original))).toBe(true);
  await page.getByRole('button', { name: '重做', exact: true }).click();
  expect((await downloadSource(page)).bytes.equals(Buffer.from(edited))).toBe(true);
  const beforeNames = await moduleList(page).getByRole('option').allTextContents();
  const invalidChooser = page.waitForEvent('filechooser');
  await page.getByRole('button', { name: '导入源码', exact: true }).click();
  await (await invalidChooser).setFiles({ name: 'invalid.mod', mimeType: 'text/plain', buffer: Buffer.from([0xff, 0xfe, 0x80]) });
  await expect(sourceStatus(page)).toContainText('UTF-8');
  expect(await moduleList(page).getByRole('option').allTextContents()).toEqual(beforeNames);
  expect((await downloadSource(page)).bytes.equals(Buffer.from(edited))).toBe(true);
  // Modern RW7 files use UTF-8 without BOM. Diagnose an imported BOM without
  // silently rewriting the user's bytes, just as with the legacy extension.
  const modernChooser = page.waitForEvent('filechooser');
  await page.getByRole('button', { name: '导入源码', exact: true }).click();
  await (await modernChooser).setFiles({ name: 'modern.modx', mimeType: 'text/plain', buffer: Buffer.from(original) });
  await expect(moduleList(page).getByRole('option', { name: 'modern.modx', exact: true })).toHaveAttribute('aria-selected', 'true');
  await expect(page.locator('.robot-code-diagnostics button').filter({ hasText: /modern\.modx.*UTF-8 BOM/ })).toBeVisible();
  const modernDownload = await downloadSource(page);
  expect(modernDownload.name).toBe('modern.modx');
  expect(modernDownload.bytes.equals(Buffer.from(original))).toBe(true);
  expect(errors).toEqual([]);
});

test('module editing stays independent from Path simulation and context insertion captures the actual body in one undo step', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const initialImage = await page.getByLabel('ABB IRB 1100 三维仿真场景').screenshot();
  await captureCurrent(page);
  const oldPose = await bodyPose(page);
  await setJoint(page, 1, 3);
  await expect.poll(async () => (await bodyJoints(page))[0]).toBe(3);
  const currentPose = await bodyPose(page), joints = await bodyJoints(page);
  await openScene(page);
  expect((await page.getByLabel('ABB IRB 1100 三维仿真场景').screenshot()).equals(initialImage)).toBe(false);
  const initial = 'MODULE PoseDemo\nCONST num P001:=7;\nPROC main()\n\nENDPROC\nENDMODULE';
  await newSource(page, 'pose.mod', initial);
  await expect(page.getByRole('button', { name: '运行源码', exact: true })).toHaveCount(0);
  await expect(page.getByRole('tablist', { name: '程序编辑模式', exact: true })).toHaveCount(0);
  await codeEditor(page).press('ControlOrMeta+Home');
  for (let index = 0; index < 3; index++) await codeEditor(page).press('ArrowDown');
  await codeEditor(page).press('Home');
  const line = codeEditor(page).locator('.cm-line').nth(3);
  await line.click({ button: 'right', position: { x: 15, y: 7 } });
  await page.getByRole('menu', { name: '源码右键菜单', exact: true }).getByRole('menuitem', { name: '插入当前姿态', exact: true }).click();
  await expect(codeEditor(page)).toContainText('MoveL P002');
  const inserted = await readSource(page);
  expect(inserted).toContain('CONST num P001:=7;');
  expect(inserted).toMatch(/MoveL P002,/);
  const declaration = /CONST robtarget P002\s*:=\s*([^;]+);/.exec(inserted);
  expect(declaration).not.toBeNull();
  const target = parseRobtarget(declaration![1]);
  // Visible TCP values are rounded to .01 mm; source targets retain .001 mm.
  expect(Math.max(...target.pose.position.map((value, index) => Math.abs(value - currentPose.position[index])))).toBeLessThanOrEqual(.006);
  expect(Math.hypot(...target.pose.position.map((value, index) => value - oldPose.position[index]))).toBeGreaterThan(10);
  expect(await bodyJoints(page)).toEqual(joints);
  await codeEditor(page).press('ControlOrMeta+Home');
  await page.getByRole('tab', { name: '指令参考', exact: true }).click();
  await page.getByRole('textbox', { name: '查找指令参考', exact: true }).fill('MoveL');
  await page.getByRole('listbox', { name: '指令参考列表', exact: true }).getByRole('option', { name: 'MoveL', exact: true }).click();
  await expect(page.locator('.robot-code-reference-detail h3')).toHaveText('MoveL');
  await page.screenshot({ path: 'docs/images/offline-programming.png' });
  await page.getByRole('button', { name: '撤销', exact: true }).click();
  expect(await readSource(page)).toBe(initial);
  await replaceSource(page, 'MODULE Invalid\nMoveL missing;');
  await expect(page.getByRole('button', { name: '运行源码', exact: true })).toHaveCount(0);
  expect(await bodyJoints(page)).toEqual(joints);
  await openScene(page);
  await expect(pathPoints(page)).toHaveCount(1);
  await page.locator('[data-object-kind="path"]').first().click();
  await expect(page.getByRole('button', { name: '运行 Path', exact: true })).toBeEnabled();
  expect(errors).toEqual([]);
});

test('long code, search and diagnostic panels scroll locally across fixed desktop sizes', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const source = 'MODULE Scroll\n' + Array.from({ length: 120 }, (_, index) => '! line-' + index + ' ' + 'word '.repeat(90)).join('\n') + '\nENDMODULE';
  await newSource(page, 'scroll.mod', source);
  const scroller = codeWorkspace(page).locator('.cm-scroller');
  for (const viewport of [{ width: 1440, height: 1000 }, { width: 1000, height: 720 }, { width: 1000, height: 1000 }]) {
    await page.setViewportSize(viewport);
    await noDocumentScroll(page);
    const box = (await scroller.boundingBox())!;
    expect(box.height).toBeGreaterThan(110);
    expect(await scroller.evaluate(element => element.scrollHeight - element.clientHeight)).toBeGreaterThan(500);
    await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
    await page.mouse.wheel(0, 600);
    await expect.poll(() => scroller.evaluate(element => element.scrollTop)).toBeGreaterThan(80);
    await page.mouse.wheel(400, 0);
    await expect.poll(() => scroller.evaluate(element => element.scrollLeft)).toBeGreaterThan(80);
    await noDocumentScroll(page);
  }
  await page.getByRole('button', { name: '查找', exact: true }).click();
  const search = page.locator('.cm-search input[name="search"]');
  // CM's search panel commits on keyup/change, so enter the query with real
  // keyboard events before Enter runs findNext.
  await search.pressSequentially('line-119');
  await search.press('Enter');
  await expect(page.locator('.cm-searchMatch-selected')).toHaveText('line-119');
  await search.press('Escape');
  await newSource(page, 'errors.mod', 'MODULE Errors\nPROC main()\n' + 'value:=;\n'.repeat(50) + 'ENDPROC\nENDMODULE');
  await expect(page.locator('.robot-code-diagnostics .severity-error')).toHaveCount(50);
  const panel = page.locator('.robot-code-panel-body');
  expect(await panel.evaluate(element => element.scrollHeight - element.clientHeight)).toBeGreaterThan(200);
  const box = (await panel.boundingBox())!;
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.wheel(0, 400);
  await expect.poll(() => panel.evaluate(element => element.scrollTop)).toBeGreaterThan(40);
  await noDocumentScroll(page);
  await page.screenshot({ path: '/tmp/emulation-source-editor-1000.png' });
  expect(errors).toEqual([]);
});
