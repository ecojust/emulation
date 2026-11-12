import { expect, test, type Page } from '@playwright/test';
import { prepareCatalogFixtures, readCatalog } from './catalog-fixtures';
import { enterProject, waitForProjectScene } from './project-helpers';
import { codeEditor, generateSource, moduleList, readSource, replaceSource } from './programming-helpers';
import { captureCurrent, observeBrowser, setJoint } from './workspace-helpers';

test.setTimeout(120_000);
test.use({ actionTimeout: 15_000 });

let fixtures: Map<string, Buffer>;
test.beforeAll(async () => {
  test.setTimeout(180_000);
  fixtures = await prepareCatalogFixtures(await readCatalog());
});

const templateStatus = (page: Page) => page.getByRole('status', { name: '模板参数状态', exact: true });
const syntaxErrors = (page: Page) => page.locator('.robot-code-diagnostics .severity-error');
const parameters = (page: Page) => page.locator('.robot-code-diagnostics .severity-warning').filter({ hasText: '待配置' });

async function expectTemplateDiagnostics(page: Page) {
  await page.getByRole('tab', { name: /^问题 / }).click();
  // A positive result from the real analysis Worker must arrive before an
  // empty error list is evidence: a newly opened file is still being checked.
  await expect(templateStatus(page)).toBeVisible();
  await expect(parameters(page).first()).toBeVisible();
  await expect(syntaxErrors(page)).toHaveCount(0);
}

for (const model of [
  { id: 'fanuc-crx10ia-l', name: 'FANUC CRX-10iA/L', brand: 'FANUC', extension: '.ls', theme: 'light' },
  { id: 'fanuc-r2000ib-165f', name: 'FANUC R-2000iB/165F', brand: 'FANUC', extension: '.ls', theme: 'dark' },
  { id: 'yaskawa-gp7', name: 'Yaskawa GP7', brand: 'Yaskawa', extension: '.jbi', theme: 'light' },
  { id: 'yaskawa-gp110', name: 'Yaskawa GP110', brand: 'Yaskawa', extension: '.jbi', theme: 'dark' },
] as const) {
  test(`${model.name} ${model.theme} Path generation and current-pose insertion retain pending parameters without hiding syntax errors`, async ({ page }) => {
    const browserErrors = observeBrowser(page);
    // Transport uses the catalogue's real pinned/hash-checked mesh bytes;
    // project creation, capture, generation and Worker analysis stay real.
    await page.route(url => fixtures.has(url.href), async route => {
      const bytes = fixtures.get(route.request().url());
      if (!bytes) throw new Error(`Unexpected model asset: ${route.request().url()}`);
      await route.fulfill({ status: 200, contentType: 'application/octet-stream', headers: { 'access-control-allow-origin': '*' }, body: bytes });
    });
    await page.goto('/');
    await enterProject(page, `${model.name} generated program`, model.id);
    await waitForProjectScene(page, model.id);
    if (await page.locator('html').getAttribute('data-theme') !== model.theme) {
      await page.getByRole('button', { name: '切换主题', exact: true }).click();
    }
    await expect(page.locator('html')).toHaveAttribute('data-theme', model.theme);

    await setJoint(page, 1, 10);
    await expect(page.getByRole('spinbutton', { name: '机器人关节 J1', exact: true })).toHaveValue('10');
    const first = await captureCurrent(page);
    await setJoint(page, 1, 20);
    await expect(page.getByRole('spinbutton', { name: '机器人关节 J1', exact: true })).toHaveValue('20');
    await captureCurrent(page, first.pathId);

    const generated = await generateSource(page, first.pathId);
    await expect(moduleList(page).getByRole('option', { selected: true })).toContainText(model.extension);
    if (model.brand === 'FANUC') {
      expect(generated).toContain('{{UF}}');
      expect(generated).toContain('{{UT}}');
      expect(generated.match(/\bL P\[\d+\]/g)).toHaveLength(2);
      expect(generated).toMatch(/J1\s*=\s*10 deg/);
      expect(generated).toMatch(/J1\s*=\s*20 deg/);
    } else {
      expect(generated).toContain('{{TOOL}}');
      expect(generated.match(/\{\{RCONF_P\d+\}\}/g)).toHaveLength(2);
      expect(generated.match(/^MOVL C\d{5}/gm)).toHaveLength(2);
      expect(generated).toContain('///NPOS 2,0,0,0,0,0');
    }
    await expectTemplateDiagnostics(page);

    // readSource selects the document. Collapse that selection before using
    // the user's toolbar action so insertion cannot replace the whole file.
    await codeEditor(page).press('ControlOrMeta+End');
    await page.getByRole('button', { name: '插入当前姿态', exact: true }).click();
    await expect(page.getByRole('status', { name: '源码状态', exact: true })).toContainText('已插入当前仿真姿态');
    const inserted = await readSource(page);
    expect(inserted).not.toBe(generated);
    if (model.brand === 'FANUC') {
      expect(inserted.match(/\bL P\[\d+\]/g)).toHaveLength(3);
      expect(inserted.match(/\{\{UF\}\}/g)).toHaveLength(3);
      expect(inserted.match(/\{\{UT\}\}/g)).toHaveLength(3);
    } else {
      expect(inserted.match(/^MOVL C\d{5}/gm)).toHaveLength(3);
      expect(inserted.match(/\{\{RCONF_P\d+\}\}/g)).toHaveLength(3);
      expect(inserted).toContain('///NPOS 3,0,0,0,0,0');
    }

    // A recognized motion with a missing required speed is a genuine syntax
    // error (an unknown vendor macro could legitimately be only a warning).
    // Keep the template marker and unresolved parameters intact throughout.
    const broken = model.brand === 'FANUC'
      ? inserted.replace(/\bL (P\[\d+\]) [^\r\n;]+ ;/, 'L $1 ;')
      : inserted.replace(/\bMOVL (C\d{5}) V=[^\r\n]+/, 'MOVL $1 V=');
    expect(broken).not.toBe(inserted);
    await replaceSource(page, broken);
    await expect(syntaxErrors(page).first()).toBeVisible();
    await expect(parameters(page).first()).toBeVisible();
    await expect(templateStatus(page)).toBeVisible();

    // Waiting for an actual error and then its removal also proves that the
    // final insertion result was analysed, rather than accepting stale results.
    await replaceSource(page, inserted);
    await expectTemplateDiagnostics(page);
    expect(await readSource(page)).toBe(inserted);
    await expect(page.locator('html')).toHaveAttribute('data-theme', model.theme);
    await page.screenshot({ path: `/tmp/emulation-generated-programs-${model.id}.png` });
    expect(browserErrors).toEqual([]);
  });
}
