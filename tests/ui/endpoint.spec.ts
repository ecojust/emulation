import { expect, test, type Locator, type Page } from '@playwright/test';
import { expectCopyState, openProgram, openScene, readRapid, selectRobot } from './workspace-helpers';

test.setTimeout(60_000);
const rows = (page: Page) => page.locator('[data-endpoint-joints]');
const canvas = (page: Page) => page.getByLabel('ABB IRB 1100 三维仿真场景');
async function openSolutions(page: Page) {
  const errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
  await page.goto('/');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await page.getByRole('button', { name: '求解 MoveL', exact: true }).click();
  await expect(page.getByRole('heading', { name: '运动学可达', exact: true })).toBeVisible();
  const details = page.locator('details').filter({ has: page.locator('summary').filter({ hasText: '端点姿态与干涉检测' }) });
  if ((await details.getAttribute('open')) === null) await details.locator('summary').click();
  await expect.poll(() => rows(page).count()).toBeGreaterThan(1);
  return errors;
}
async function differentWrist(page: Page): Promise<{ row: Locator; joints: number[] }> {
  const values = await rows(page).evaluateAll(elements => elements.map(element => JSON.parse(element.getAttribute('data-endpoint-joints')!) as number[]));
  const index = values.findIndex(joints => joints[4] < -1);
  expect(index).toBeGreaterThanOrEqual(0);
  return { row: rows(page).nth(index), joints: values[index] };
}
async function expectJoints(page: Page, joints: number[]) {
  const actual = page.getByLabel('当前关节姿态', { exact: true }).locator('strong');
  for (let i = 0; i < 6; i++) await expect(actual.nth(i)).toHaveText(`${joints[i].toFixed(1)}°`);
}
async function states(page: Page): Promise<string[]> {
  return rows(page).evaluateAll(elements => elements.map(element => element.getAttribute('data-collision-state') || 'missing'));
}
async function allStates(page: Page, allowed: string[]) {
  const values = await states(page);
  return values.length > 0 && values.every(value => allowed.includes(value));
}
async function expectZone(page: Page, min: number[], max: number[]) {
  await expect.poll(async () => {
    const box = JSON.parse((await canvas(page).getAttribute('data-danger-zone'))!);
    if (!box.enabled) return Infinity;
    return Math.max(...box.minMm.map((value: number, i: number) => Math.abs(value - min[i])), ...box.maxMm.map((value: number, i: number) => Math.abs(value - max[i])));
  }).toBeLessThan(1e-6);
}
async function setZone(page: Page, center: number[], size: number[]) {
  for (const [index, axis] of ['X', 'Y', 'Z'].entries()) {
    await page.getByRole('spinbutton', { name: `危险区中心 ${axis}`, exact: true }).fill(String(center[index]));
    await page.getByRole('spinbutton', { name: `危险区尺寸 ${axis}`, exact: true }).fill(String(size[index]));
  }
}
async function openZoneSettings(page: Page) {
  const settings = page.locator('details').filter({ has: page.locator('summary').filter({ hasText: '干涉区域设置' }) });
  if ((await settings.getAttribute('open')) === null) await settings.locator('summary').click();
}
async function hitZoneIds(page: Page): Promise<string[][]> {
  return rows(page).evaluateAll(elements => elements.map(element => JSON.parse(element.getAttribute('data-collision-zone-ids') || '[]') as string[]));
}
async function sceneZones(page: Page): Promise<{ id: string; enabled: boolean; minMm?: number[]; maxMm?: number[] }[]> {
  return JSON.parse((await canvas(page).getAttribute('data-danger-zones'))!);
}
async function expectZoneInputs(page: Page, center: number[], size: number[]) {
  for (const [index, axis] of ['X', 'Y', 'Z'].entries()) {
    await expect(page.getByRole('spinbutton', { name: `危险区中心 ${axis}`, exact: true })).toHaveValue(String(center[index]));
    await expect(page.getByRole('spinbutton', { name: `危险区尺寸 ${axis}`, exact: true })).toHaveValue(String(size[index]));
  }
}
async function expectRenderedZone(page: Page, id: string, min: number[], max: number[]) {
  // Worker completion and a matching zone ID do not imply the next scene RAF
  // has applied every dimension. Read fresh bounds until the geometry matches.
  await expect.poll(async () => {
    const zone = (await sceneZones(page)).find(item => item.id === id);
    if (!zone?.enabled || zone.minMm?.length !== 3 || zone.maxMm?.length !== 3) return Infinity;
    return Math.max(...zone.minMm.map((value, i) => Math.abs(value - min[i])), ...zone.maxMm.map((value, i) => Math.abs(value - max[i])));
  }, { message: `Rendered bounds of ${id}: ${JSON.stringify({ min, max })}` }).toBeLessThan(5e-7);
}

test('applying a different wrist solution previews its actual joints and cannot reuse validated motion commands', async ({ page }) => {
  const errors = await openSolutions(page);
  const instruction = await readRapid(page);
  const candidate = await differentWrist(page);
  await candidate.row.getByRole('button', { name: /^应用姿态 / }).click();
  await expect(page.getByText('端点姿态预览', { exact: true })).toBeVisible();
  await expectJoints(page, candidate.joints);
  await expectCopyState(page, false);
  await expect(page.getByRole('button', { name: '播放路径', exact: true })).toBeDisabled();
  await openProgram(page);
  await expect(page.getByRole('button', { name: /导出 .*模块/ })).toBeDisabled();
  await openScene(page);
  await page.evaluate(() => window.scrollTo(0, 0));
  await page.screenshot({ path: '/tmp/emulation-endpoint-preview.png', fullPage: true });
  await page.getByRole('button', { name: '返回已验证路径', exact: true }).click();
  await expect(page.getByText('端点姿态预览', { exact: true })).toHaveCount(0);
  await expectCopyState(page, true);
  await expect(page.getByRole('button', { name: '播放路径', exact: true })).toBeEnabled();
  expect(await readRapid(page)).toBe(instruction);
  await candidate.row.getByRole('button', { name: /^应用姿态 / }).click();
  await selectRobot(page, 'irb1100-4-0.475');
  await expect(page.getByText('机器人模型已加载 · 7/7', { exact: true })).toBeVisible();
  await expect(page.getByText('端点姿态预览', { exact: true })).toHaveCount(0);
  await expectCopyState(page, false);
  await expectJoints(page, [0, 20, -30, 0, 40, 0]);
  await expect(rows(page)).toHaveCount(0);
  expect(errors).toEqual([]);
});

test('actual danger-box geometry marks overlaps, clears a distant box, and never labels invalid input as checked', async ({ page }) => {
  const errors = await openSolutions(page);
  await openZoneSettings(page);
  await page.getByRole('checkbox', { name: '显示干涉区域', exact: true }).check();
  await setZone(page, [0, 0, 300], [2500, 2500, 2500]);
  await expect.poll(() => allStates(page, ['hit'])).toBe(true);
  expect(await rows(page).count()).toBeGreaterThan(1);
  await expectZone(page, [-1250, -1250, -950], [1250, 1250, 1550]);
  await expect(rows(page).first().getByLabel('干涉', { exact: true })).toBeVisible();
  await page.evaluate(() => window.scrollTo(0, 0));
  await page.screenshot({ path: '/tmp/emulation-danger-overlap.png', fullPage: true });
  const observed: { state: string; text: string }[] = [];
  await page.exposeFunction('recordCollisionState', (value: { state: string; text: string }) => observed.push(value));
  await rows(page).first().evaluate(element => {
    const report = () => (window as unknown as { recordCollisionState: (value: { state: string; text: string }) => Promise<void> }).recordCollisionState({ state: element.getAttribute('data-collision-state') || '', text: element.textContent || '' });
    const observer = new MutationObserver(() => { void report(); });
    observer.observe(element, { attributes: true, attributeFilter: ['data-collision-state'], childList: true, subtree: true, characterData: true });
  });
  await setZone(page, [5000, 5000, 5000], [100, 100, 100]);
  await expect.poll(() => allStates(page, ['clear'])).toBe(true);
  await expectZone(page, [4950, 4950, 4950], [5050, 5050, 5050]);
  await expect(rows(page).getByLabel('干涉', { exact: true })).toHaveCount(0);
  expect(observed.some(value => value.state === 'pending')).toBe(true);
  for (const pending of observed.filter(value => value.state === 'pending')) expect(pending.text).not.toContain('未检测到干涉');
  expect((await rows(page).allTextContents()).join(' ')).not.toMatch(/路径安全|安全可执行|无碰撞/);
  await page.getByRole('spinbutton', { name: '危险区尺寸 X', exact: true }).fill('');
  await expect.poll(() => allStates(page, ['error', 'unchecked'])).toBe(true);
  await expect.poll(async () => JSON.parse((await canvas(page).getAttribute('data-danger-zone'))!)).toEqual({ enabled: false });
  expect((await rows(page).allTextContents()).join(' ')).not.toContain('未检测到干涉');
  await page.screenshot({ path: '/tmp/emulation-danger-invalid.png', fullPage: true });
  expect(errors).toEqual([]);
});

test('actual mesh detection removes an OBB false positive, finds surface and solid interference, and exposes uncertain geometry', async ({ page }) => {
  test.setTimeout(90_000);
  const errors = await openSolutions(page);
  const selected = rows(page).filter({ has: page.getByText('已验证路径解', { exact: true }) });
  await expect(selected).toHaveCount(1);
  await openZoneSettings(page);
  const zoneId = await page.getByRole('combobox', { name: '当前危险区', exact: true }).inputValue();
  // Pinned real base mesh: this corner overlaps its OBB, but no triangle and no
  // closed solid. The selected non-singular pose has no other candidate link.
  await setZone(page, [75, 75, 80], [4, 4, 4]);
  await expectRenderedZone(page, zoneId, [73, 73, 78], [77, 77, 82]);
  await expect(selected).toHaveAttribute('data-collision-state', 'clear');
  await expect(selected.getByLabel('未检测到干涉', { exact: true })).toBeVisible();
  await expect(selected).toHaveAttribute('data-collision-zone-ids', '[]');
  await expect(selected).toHaveAttribute('data-collision-uncertain-zone-ids', '[]');
  await page.screenshot({ path: '/tmp/emulation-mesh-empty-corner.png' });
  await setZone(page, [0, 70, 100], [4, 4, 4]);
  await expectRenderedZone(page, zoneId, [-2, 68, 98], [2, 72, 102]);
  await expect(selected).toHaveAttribute('data-collision-state', 'hit');
  await expect(selected.getByLabel('干涉', { exact: true })).toHaveAttribute('title', /基座/);
  await expect(selected).toHaveAttribute('data-collision-zone-ids', JSON.stringify([zoneId]));
  await expect(selected).toHaveAttribute('data-collision-uncertain-zone-ids', '[]');
  // Entirely inside the closed base: no triangle crosses this two-mm box.
  await setZone(page, [0, 0, 50], [2, 2, 2]);
  await expectRenderedZone(page, zoneId, [-1, -1, 49], [1, 1, 51]);
  await expect(selected).toHaveAttribute('data-collision-state', 'hit');
  await expect(selected.getByLabel('干涉', { exact: true })).toHaveAttribute('title', /基座/);
  // Source link_1 is non-manifold. No triangle crosses this box, so neither
  // interference nor separation can be asserted from the enclosing volume.
  await setZone(page, [0, 85, 180], [4, 4, 4]);
  await expectRenderedZone(page, zoneId, [-2, 83, 178], [2, 87, 182]);
  await expect(selected).toHaveAttribute('data-collision-state', 'uncertain');
  await expect(selected.getByLabel('无法确定', { exact: true })).toBeVisible();
  await expect(selected).toHaveAttribute('data-collision-zone-ids', '[]');
  await expect(selected).toHaveAttribute('data-collision-uncertain-zone-ids', JSON.stringify([zoneId]));
  await expect(selected.getByLabel('未检测到干涉', { exact: true })).toHaveCount(0);
  await expect(selected.getByLabel('干涉', { exact: true })).toHaveCount(0);
  await page.screenshot({ path: '/tmp/emulation-mesh-uncertain.png' });
  await page.getByRole('spinbutton', { name: '危险区尺寸 X', exact: true }).fill('');
  await expect(selected).toHaveAttribute('data-collision-state', 'error');
  await expect(selected.getByLabel('未检查', { exact: true })).toBeVisible();
  await expect(selected).toHaveAttribute('data-collision-zone-ids', '[]');
  await expect(selected).toHaveAttribute('data-collision-uncertain-zone-ids', '[]');
  await setZone(page, [5000, 5000, 5000], [100, 100, 100]);
  await expectRenderedZone(page, zoneId, [4950, 4950, 4950], [5050, 5050, 5050]);
  await expect(selected).toHaveAttribute('data-collision-state', 'clear');
  await expect(selected.getByLabel('未检测到干涉', { exact: true })).toBeVisible();
  expect(errors).toEqual([]);
});

test('multiple danger zones retain their identities and endpoint preview across views, edits, selection and deletion', async ({ page }) => {
  test.setTimeout(90_000);
  const errors = await openSolutions(page);
  const originalSolutions = await rows(page).evaluateAll(elements => elements.map(element => element.getAttribute('data-endpoint-joints')));
  const originalCount = originalSolutions.length;
  await openZoneSettings(page);
  const selector = page.getByRole('combobox', { name: '当前危险区', exact: true });
  const firstId = await selector.inputValue();
  expect(firstId).not.toBe('');
  await page.getByRole('textbox', { name: '危险区名称', exact: true }).fill('Near fixture');
  await setZone(page, [0, 0, 300], [2500, 2500, 2500]);
  await expectZoneInputs(page, [0, 0, 300], [2500, 2500, 2500]);
  await expect.poll(() => allStates(page, ['hit'])).toBe(true);
  await expectRenderedZone(page, firstId, [-1250, -1250, -950], [1250, 1250, 1550]);
  await page.getByRole('button', { name: '添加危险区', exact: true }).click();
  await expect(selector).not.toHaveValue(firstId);
  await expect(selector).not.toHaveValue('');
  const secondId = await selector.inputValue();
  expect(secondId).not.toBe(firstId);
  await page.getByRole('textbox', { name: '危险区名称', exact: true }).fill('Far fixture');
  await setZone(page, [5000, 5000, 5000], [100, 100, 100]);
  await expectZoneInputs(page, [5000, 5000, 5000], [100, 100, 100]);
  await expect.poll(async () => (await hitZoneIds(page)).every(ids => ids.length === 1 && ids[0] === firstId)).toBe(true);
  await expect(rows(page)).toHaveCount(originalCount);
  await expect(rows(page).first().getByLabel('干涉', { exact: true })).toHaveAttribute('title', /Near fixture/);
  expect(await rows(page).first().getByLabel('干涉', { exact: true }).getAttribute('title')).not.toContain('Far fixture');
  await expect.poll(async () => (await sceneZones(page)).map(zone => zone.id).sort()).toEqual([firstId, secondId].sort());
  await expectRenderedZone(page, secondId, [4950, 4950, 4950], [5050, 5050, 5050]);
  const candidate = await differentWrist(page);
  await openProgram(page);
  await candidate.row.getByRole('button', { name: /^应用姿态 / }).click();
  await expect(page.locator('[data-active-view]')).toHaveAttribute('data-active-view', 'scene');
  await expectJoints(page, candidate.joints);
  const sceneBox = (await canvas(page).boundingBox())!;
  const initialCamera = JSON.parse((await canvas(page).getAttribute('data-camera-position'))!) as number[];
  await page.mouse.move(sceneBox.x + sceneBox.width * .72, sceneBox.y + sceneBox.height * .67);
  // Move to a non-default camera with real input. Zoom converges by elapsed time;
  // OrbitControls' frame-based inertia is covered by the separate orbit regression.
  await page.mouse.wheel(0, -100);
  await expect.poll(async () => Math.abs(Number(await canvas(page).getAttribute('data-camera-distance')) - Number(await canvas(page).getAttribute('data-zoom-target')))).toBeLessThan(1e-8);
  const camera = JSON.parse((await canvas(page).getAttribute('data-camera-position'))!) as number[];
  expect(Math.hypot(...camera.map((value, i) => value - initialCamera[i]))).toBeGreaterThan(.01);
  await openProgram(page);
  await expect(page.getByRole('button', { name: '复制 MoveL', exact: true })).toBeDisabled();
  await page.getByRole('textbox', { name: '程序模块名称', exact: true }).fill('RetainedPreview');
  await expectJoints(page, candidate.joints);
  await openScene(page);
  await expect(page.getByText('端点姿态预览', { exact: true })).toBeVisible();
  await expect(selector).toHaveValue(secondId);
  await expectJoints(page, candidate.joints);
  await expectRenderedZone(page, firstId, [-1250, -1250, -950], [1250, 1250, 1550]);
  await expectRenderedZone(page, secondId, [4950, 4950, 4950], [5050, 5050, 5050]);
  await expect.poll(async () => {
    const current = JSON.parse((await canvas(page).getAttribute('data-camera-position'))!) as number[];
    return Math.hypot(...current.map((value, i) => value - camera[i]));
  }).toBeLessThan(1e-8);
  await page.screenshot({ path: '/tmp/emulation-multiple-danger-zones.png' });
  await page.getByRole('button', { name: '选择危险区 Near fixture', exact: true }).click();
  await expect(selector).toHaveValue(firstId);
  await expect(page.getByRole('textbox', { name: '危险区名称', exact: true })).toHaveValue('Near fixture');
  await setZone(page, [6000, 6000, 6000], [100, 100, 100]);
  await expectZoneInputs(page, [6000, 6000, 6000], [100, 100, 100]);
  await expect.poll(() => allStates(page, ['clear'])).toBe(true);
  await expectRenderedZone(page, firstId, [5950, 5950, 5950], [6050, 6050, 6050]);
  for (const ids of await hitZoneIds(page)) expect(ids).toEqual([]);
  await expectJoints(page, candidate.joints);
  await page.getByRole('button', { name: '删除危险区', exact: true }).click();
  await expect(selector).toHaveValue(secondId);
  await expect.poll(async () => (await sceneZones(page)).map(zone => zone.id)).toEqual([secondId]);
  await expectRenderedZone(page, secondId, [4950, 4950, 4950], [5050, 5050, 5050]);
  await expect(rows(page)).toHaveCount(originalCount);
  await expect.poll(() => allStates(page, ['clear'])).toBe(true);
  await page.getByRole('button', { name: '删除危险区', exact: true }).click();
  await expect.poll(async () => sceneZones(page)).toEqual([]);
  await expect.poll(() => allStates(page, ['off'])).toBe(true);
  expect((await rows(page).allTextContents()).join(' ')).not.toMatch(/未检测到干涉|路径安全|安全可执行|无碰撞/);
  await page.getByRole('button', { name: '添加危险区', exact: true }).click();
  await expect(selector).not.toHaveValue('');
  expect(await selector.inputValue()).not.toBe(firstId);
  expect(await selector.inputValue()).not.toBe(secondId);
  const disabledId = await selector.inputValue();
  await page.getByRole('checkbox', { name: '显示干涉区域', exact: true }).uncheck();
  await expect.poll(() => allStates(page, ['off'])).toBe(true);
  await expect.poll(async () => sceneZones(page)).toEqual([{ id: disabledId, enabled: false }]);
  expect((await rows(page).allTextContents()).join(' ')).not.toContain('未检测到干涉');
  await expectJoints(page, candidate.joints);
  await expect(rows(page)).toHaveCount(originalCount);
  await openProgram(page);
  await expect(page.getByRole('textbox', { name: '程序模块名称', exact: true })).toHaveValue('RetainedPreview');
  await expect(page.getByRole('button', { name: '复制 MoveL', exact: true })).toBeDisabled();
  expect(await rows(page).evaluateAll(elements => elements.map(element => element.getAttribute('data-endpoint-joints')))).toEqual(originalSolutions);
  expect(errors).toEqual([]);
});
