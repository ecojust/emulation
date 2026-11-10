import { expect, test, type Locator, type Page } from '@playwright/test';
import { bodyPose, captureCurrent, checkedSegment, expectBodyJoints, expectCopyState, makePath, observeBrowser, openProgram, openScene, openWorkspace, scene, selectPoint, selectRobot, type CapturedPoint } from './workspace-helpers';

test.setTimeout(90_000);
const rows = (page: Page) => page.locator('[data-endpoint-joints]');
async function openSolutions(page: Page) {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const point = await captureCurrent(page);
  await makePath(page, [point]);
  await selectPoint(page, point);
  await expect.poll(() => rows(page).count()).toBeGreaterThan(1);
  return { errors, point };
}
async function differentWrist(page: Page): Promise<{ row: Locator; joints: number[] }> {
  const values = await rows(page).evaluateAll(elements => elements.map(element => JSON.parse(element.getAttribute('data-endpoint-joints')!) as number[]));
  const index = values.findIndex(joints => joints[4] < -1);
  expect(index).toBeGreaterThanOrEqual(0);
  return { row: rows(page).nth(index), joints: values[index] };
}
async function allStates(page: Page, allowed: string[]) {
  const values = await rows(page).evaluateAll(elements => elements.map(element => element.getAttribute('data-collision-state') || 'missing'));
  return values.length > 0 && values.every(value => allowed.includes(value));
}
export async function setZone(page: Page, center: number[], size: number[]) {
  for (const [index, axis] of ['X', 'Y', 'Z'].entries()) {
    await page.getByRole('spinbutton', { name: `危险区中心 ${axis}`, exact: true }).fill(String(center[index]));
    await page.getByRole('spinbutton', { name: `危险区尺寸 ${axis}`, exact: true }).fill(String(size[index]));
  }
}
async function selectZone(page: Page, id?: string) {
  const object = id ? page.locator(`[data-object-kind="zone"][data-object-id="${id}"]`) : page.locator('[data-object-kind="zone"]').first();
  await object.click();
  await expect(page.getByRole('heading', { name: '危险区属性', exact: true })).toBeVisible();
  return (await page.locator('[data-selected-zone-id]').getAttribute('data-selected-zone-id'))!;
}
async function hitZoneIds(page: Page): Promise<string[][]> {
  return rows(page).evaluateAll(elements => elements.map(element => JSON.parse(element.getAttribute('data-collision-zone-ids') || '[]') as string[]));
}
async function sceneZones(page: Page): Promise<{ id: string; enabled: boolean; minMm?: number[]; maxMm?: number[] }[]> {
  return JSON.parse((await scene(page).getAttribute('data-danger-zones'))!);
}
async function expectRenderedZone(page: Page, id: string, min: number[], max: number[]) {
  // Input commits, collision workers and the actual rendered box are separate
  // updates. Poll fresh bounds instead of assuming an input fill awaited RAF.
  await expect.poll(async () => {
    const zone = (await sceneZones(page)).find(item => item.id === id);
    if (!zone?.enabled || zone.minMm?.length !== 3 || zone.maxMm?.length !== 3) return Infinity;
    return Math.max(...zone.minMm.map((value, i) => Math.abs(value - min[i])), ...zone.maxMm.map((value, i) => Math.abs(value - max[i])));
  }).toBeLessThan(5e-7);
}
async function editAndInspect(page: Page, point: CapturedPoint, id: string, center: number[], size: number[]) {
  await selectZone(page, id);
  await setZone(page, center, size);
  await expectRenderedZone(page, id, center.map((value, i) => value - size[i] / 2), center.map((value, i) => value + size[i] / 2));
  await selectPoint(page, point);
  await expect.poll(() => rows(page).count()).toBeGreaterThan(1);
}
async function readyRow(page: Page) {
  await expect.poll(() => rows(page).count()).toBeGreaterThan(1);
  const values = await rows(page).evaluateAll(elements => elements.map(element => JSON.parse(element.getAttribute('data-endpoint-joints')!) as number[]));
  const ready = [0, 20, -30, 0, 40, 0];
  const index = values.findIndex(joints => joints.every((value, i) => Math.abs(value - ready[i]) < .001));
  expect(index).toBeGreaterThanOrEqual(0);
  return rows(page).nth(index);
}

test('applying an independent wrist solution updates the stored point and actual robot while invalidating checked motion', async ({ page }) => {
  const errors = observeBrowser(page);
  await openWorkspace(page);
  const point = await checkedSegment(page);
  await expect.poll(() => rows(page).count()).toBeGreaterThan(1);
  const originalTarget = await page.getByRole('textbox', { name: '目标 robtarget', exact: true }).inputValue();
  const candidate = await differentWrist(page);
  await candidate.row.getByRole('button', { name: /^应用姿态 / }).click();
  await expectBodyJoints(page, candidate.joints);
  await expect(page.locator('[data-target-point-id]')).toHaveAttribute('data-target-point-id', point.id);
  await expect(page.getByRole('textbox', { name: '目标 robtarget', exact: true })).not.toHaveValue(originalTarget);
  await expect(page.locator('.workspace-message')).toContainText(/原运动验证已失效/);
  await expectCopyState(page, false);
  await expect(page.getByRole('button', { name: '播放路径', exact: true })).toBeDisabled();
  await expect(page.getByRole('button', { name: '导出此 MoveL', exact: true })).toHaveCount(0);
  await page.screenshot({ path: '/tmp/emulation-endpoint-applied.png' });
  await selectRobot(page, 'irb1100-4-0.475');
  await expectBodyJoints(page, [0, 20, -30, 0, 40, 0]);
  await expect(rows(page)).toHaveCount(0);
  await expectCopyState(page, false);
  expect(errors).toEqual([]);
});

test('danger boxes check every independent endpoint, clear distant geometry, and never label invalid input as checked', async ({ page }) => {
  const { errors, point } = await openSolutions(page);
  const id = await selectZone(page);
  await editAndInspect(page, point, id, [0, 0, 300], [2500, 2500, 2500]);
  await expect.poll(() => allStates(page, ['hit'])).toBe(true);
  await expect(rows(page).first().getByLabel('干涉', { exact: true })).toBeVisible();
  await page.screenshot({ path: '/tmp/emulation-danger-overlap.png' });
  await editAndInspect(page, point, id, [5000, 5000, 5000], [100, 100, 100]);
  await expect.poll(() => allStates(page, ['clear'])).toBe(true);
  await expect(rows(page).getByLabel('干涉', { exact: true })).toHaveCount(0);
  expect((await rows(page).allTextContents()).join(' ')).not.toMatch(/路径安全|安全可执行|无碰撞/);
  await selectZone(page, id);
  await page.getByRole('spinbutton', { name: '危险区尺寸 X', exact: true }).fill('');
  await selectPoint(page, point);
  await expect.poll(() => allStates(page, ['error', 'unchecked'])).toBe(true);
  await expect.poll(async () => (await sceneZones(page)).some(zone => zone.id === id && zone.enabled)).toBe(false);
  expect((await rows(page).allTextContents()).join(' ')).not.toContain('未检测到干涉');
  expect(errors).toEqual([]);
});

test('actual mesh detection removes an OBB false positive, finds surface and solid interference, and exposes uncertain geometry', async ({ page }) => {
  const { errors, point } = await openSolutions(page);
  const id = await selectZone(page);
  const cases = [
    { center: [75, 75, 80], size: [4, 4, 4], state: 'clear', label: '未检测到干涉' },
    { center: [0, 70, 100], size: [4, 4, 4], state: 'hit', label: '干涉' },
    { center: [0, 0, 50], size: [2, 2, 2], state: 'hit', label: '干涉' },
    { center: [0, 85, 180], size: [4, 4, 4], state: 'uncertain', label: '无法确定' },
  ];
  for (const fixture of cases) {
    await editAndInspect(page, point, id, fixture.center, fixture.size);
    const selected = await readyRow(page);
    await expect(selected).toHaveAttribute('data-collision-state', fixture.state);
    await expect(selected.getByLabel(fixture.label, { exact: true })).toBeVisible();
    await expect(selected).toHaveAttribute('data-collision-zone-ids', JSON.stringify(fixture.state === 'hit' ? [id] : []));
    await expect(selected).toHaveAttribute('data-collision-uncertain-zone-ids', JSON.stringify(fixture.state === 'uncertain' ? [id] : []));
    if (fixture.state === 'hit') await expect(selected.getByLabel('干涉', { exact: true })).toHaveAttribute('title', /基座/);
    if (fixture.state === 'clear' || fixture.state === 'uncertain') await page.screenshot({ path: `/tmp/emulation-mesh-${fixture.state}.png` });
  }
  await expectCopyState(page, false);
  expect(errors).toEqual([]);
});

test('multiple zone identities and stored endpoint survive view changes, edits, deletion and disabled detection', async ({ page }) => {
  const { errors, point } = await openSolutions(page);
  const initialBody = await bodyPose(page);
  const originalTarget = await page.getByRole('textbox', { name: '目标 robtarget', exact: true }).inputValue();
  const firstId = await selectZone(page);
  await page.getByRole('textbox', { name: '危险区名称', exact: true }).fill('Near fixture');
  await setZone(page, [0, 0, 300], [2500, 2500, 2500]);
  await expectRenderedZone(page, firstId, [-1250, -1250, -950], [1250, 1250, 1550]);
  await page.getByRole('button', { name: '添加危险区', exact: true }).click();
  const secondId = (await page.locator('[data-selected-zone-id]').getAttribute('data-selected-zone-id'))!;
  expect(secondId).not.toBe(firstId);
  await page.getByRole('textbox', { name: '危险区名称', exact: true }).fill('Far fixture');
  await setZone(page, [5000, 5000, 5000], [100, 100, 100]);
  await expectRenderedZone(page, secondId, [4950, 4950, 4950], [5050, 5050, 5050]);
  await selectPoint(page, point);
  await expect.poll(() => allStates(page, ['hit'])).toBe(true);
  for (const ids of await hitZoneIds(page)) expect(ids).toEqual([firstId]);
  await expect(rows(page).first().getByLabel('干涉', { exact: true })).toHaveAttribute('title', /Near fixture/);
  expect(await rows(page).first().getByLabel('干涉', { exact: true }).getAttribute('title')).not.toContain('Far fixture');
  const box = (await scene(page).boundingBox())!;
  await page.mouse.move(box.x + box.width * .72, box.y + box.height * .67);
  await page.mouse.wheel(0, -100);
  await expect.poll(async () => Math.abs(Number(await scene(page).getAttribute('data-camera-distance')) - Number(await scene(page).getAttribute('data-zoom-target')))).toBeLessThan(1e-8);
  const camera = JSON.parse((await scene(page).getAttribute('data-camera-position'))!) as number[];
  await openProgram(page);
  await expect(page.getByRole('textbox', { name: '目标 robtarget', exact: true })).toHaveValue(originalTarget);
  await openScene(page);
  await expect.poll(async () => {
    const actual = JSON.parse((await scene(page).getAttribute('data-camera-position'))!) as number[];
    return Math.hypot(...actual.map((value, i) => value - camera[i]));
  }).toBeLessThan(1e-8);
  await expectRenderedZone(page, firstId, [-1250, -1250, -950], [1250, 1250, 1550]);
  await expectRenderedZone(page, secondId, [4950, 4950, 4950], [5050, 5050, 5050]);
  await page.screenshot({ path: '/tmp/emulation-object-multiple-zones.png' });
  await editAndInspect(page, point, firstId, [6000, 6000, 6000], [100, 100, 100]);
  await expect.poll(() => allStates(page, ['clear'])).toBe(true);
  await selectZone(page, firstId);
  await page.getByRole('button', { name: '删除危险区', exact: true }).click();
  await expect.poll(async () => (await sceneZones(page)).map(zone => zone.id)).toEqual([secondId]);
  await expectRenderedZone(page, secondId, [4950, 4950, 4950], [5050, 5050, 5050]);
  await page.getByRole('button', { name: '删除危险区', exact: true }).click();
  await expect.poll(() => sceneZones(page)).toEqual([]);
  await selectPoint(page, point);
  await expect.poll(() => allStates(page, ['off'])).toBe(true);
  expect((await rows(page).allTextContents()).join(' ')).not.toContain('未检测到干涉');
  await page.getByRole('button', { name: '添加危险区', exact: true }).click();
  const disabledId = (await page.locator('[data-selected-zone-id]').getAttribute('data-selected-zone-id'))!;
  expect([firstId, secondId]).not.toContain(disabledId);
  await page.getByRole('checkbox', { name: '显示干涉区域', exact: true }).uncheck();
  await expect.poll(() => sceneZones(page)).toEqual([{ id: disabledId, enabled: false }]);
  await selectPoint(page, point);
  await expect.poll(() => allStates(page, ['off'])).toBe(true);
  expect(await bodyPose(page)).toEqual(initialBody);
  await expect(page.getByRole('textbox', { name: '目标 robtarget', exact: true })).toHaveValue(originalTarget);
  expect(errors).toEqual([]);
});
