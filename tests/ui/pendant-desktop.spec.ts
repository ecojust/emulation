import { expect, test, type Page } from '@playwright/test';
import { execFileSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { resolve } from 'node:path';
import type { JogRequest, JogResult } from '../../src/pendant/motion';
import type { PendantInput, PendantSession, PendantState } from '../../src/pendant/types';
import { bodyCanvas, bodyJoints, bodyPose, observeBrowser, openWorkspace, selectRobotObject } from './workspace-helpers';
import { enterProject, waitForProjectScene } from './project-helpers';

// Browser workspace + test connection transport + actual Rust core-cli. This
// checks React ownership/rendering; native socket and installed WebView tests
// separately verify the real transport. No global Tauri flag is introduced.
test.skip(process.env.EMULATION_UI_PREVIEW === '1', 'The isolated transport routes require Vite development modules.');
test.setTimeout(90_000);
const executable = process.env.EMULATION_NATIVE_CORE || resolve(process.env.CARGO_TARGET_DIR || 'target', 'debug', process.platform === 'win32' ? 'core-cli.exe' : 'core-cli');
interface Harness {
  session: PendantSession | null;
  state: PendantState | null;
  listener: ((input: PendantInput) => void) | null;
  starts: number;
  stops: number;
  sequence: number;
  timer: number | null;
  emit(active: boolean, mode: 'linear' | 'joint', sessionId?: string): void;
}
type TestWindow = Window & { __pendantHarness: Harness };

async function installBridge(page: Page) {
  if (!existsSync(executable)) throw new Error('Pendant desktop UI requires the actual Rust core-cli; build it or set EMULATION_NATIVE_CORE.');
  const calls: { requestId: string; request: JogRequest; result?: JogResult }[] = [], cancelled: string[] = [];
  let nextGate: Promise<void> | null = null, releaseGate: (() => void) | null = null;
  await page.exposeFunction('__pendantInvoke', async (command: string, args: { requestId: string; request: JogRequest }) => {
    if (command === 'cancel_motion') { cancelled.push(args.requestId); return; }
    if (command !== 'pendant_jog') throw new Error('Unexpected test bridge command: ' + command);
    const call = { requestId: args.requestId, request: args.request, result: undefined as JogResult | undefined };
    calls.push(call);
    const gate = nextGate; nextGate = null;
    if (gate) await gate;
    const output = execFileSync(executable, { input: JSON.stringify({ op: 'pendant_jog', request: args.request }) + '\n',
      encoding: 'utf8', maxBuffer: 4 * 1024 * 1024, timeout: 5000 });
    const response = JSON.parse(output.trim());
    if (response.error) throw new Error(response.error);
    call.result = response.result as JogResult;
    return call.result;
  });
  await page.addInitScript(() => {
    const harness: Harness = {
      session: null, state: null, listener: null, starts: 0, stops: 0, sequence: 0, timer: null,
      emit(active, mode, sessionId) {
        const id = sessionId ?? harness.session?.sessionId;
        if (!id) return;
        harness.listener?.({ sessionId: id, epoch: 1, sequence: ++harness.sequence, receivedAtMs: Date.now(),
          mode, space: 'world', axes: active ? [1, 0, 0, 0, 0, 0] : [0, 0, 0, 0, 0, 0], speed: .1, active });
      },
    };
    (window as unknown as TestWindow).__pendantHarness = harness;
  });
  await page.route(url => url.pathname === '/src/pendant/transport.ts', route => route.fulfill({ contentType: 'application/javascript', body: `
    const h = window.__pendantHarness;
    export const pendantTransport = {
      async start() {
        h.starts++; h.sequence = 0;
        h.session = {sessionId: 'ui-session-' + h.starts, connected: false, port: 12345,
          expiresAtUnixMs: Date.now() + 60000, urls: [{name: 'UI test network', url: 'http://192.0.2.10:12345/#token=ui-only-' + h.starts}]};
        return structuredClone(h.session);
      },
      async stop() { h.stops++; h.session = null; },
      async status() { return h.session ? structuredClone(h.session) : null; },
      async publish(id, state) { if (h.session?.sessionId === id) h.state = structuredClone(state); },
      async listen(listener) { h.listener = listener; return () => { if (h.listener === listener) h.listener = null; }; }
    };` }));
  await page.route(url => url.pathname === '/src/pendant/motion.ts', async route => {
    const response = await route.fetch(), source = await response.text();
    const boundary = /import\s*\{\s*invoke\s*,\s*isTauri\s*\}\s*from\s*["'][^"']+["'];?/;
    expect(source.match(boundary), 'Patch only the pendant native import; retain its actual scheduler and validation').toBeTruthy();
    await route.fulfill({ response, body: source.replace(boundary,
      'const invoke = (command, args) => window.__pendantInvoke(command, args); const isTauri = () => true;') });
  });
  return {
    calls, cancelled,
    delayNext() { nextGate = new Promise<void>(resolve => { releaseGate = resolve; }); },
    release() { releaseGate?.(); releaseGate = null; },
  };
}

async function prepare(page: Page) {
  await openWorkspace(page);
  // Keep these ownership examples away from the default demonstration zone;
  // dedicated native tests exercise collision rejection with enabled zones.
  const zones = page.locator('[data-object-kind="zone"]');
  for (let index = 0; index < await zones.count(); index++) {
    await zones.nth(index).click(); await page.getByRole('checkbox', { name: '显示干涉区域', exact: true }).uncheck();
  }
  await selectRobotObject(page);
}
async function pair(page: Page, rotate = false) {
  await page.getByRole('button', { name: '示教器', exact: true }).click();
  const dialog = page.getByRole('dialog', { name: '手机示教器', exact: true });
  const qr = dialog.getByRole('img', { name: '手机示教器配对二维码', exact: true });
  await expect(qr).toBeVisible();
  const firstId = await page.evaluate(() => (window as unknown as TestWindow).__pendantHarness.session!.sessionId);
  if (rotate) {
    const before = await qr.locator('path').getAttribute('d');
    await dialog.getByRole('button', { name: '重新生成二维码', exact: true }).click();
    await expect(qr.locator('path')).not.toHaveAttribute('d', before!);
    expect(await page.evaluate(() => (window as unknown as TestWindow).__pendantHarness.session!.sessionId)).not.toBe(firstId);
  }
  await page.evaluate(() => { (window as unknown as TestWindow).__pendantHarness.session!.connected = true; });
  await expect(dialog.getByText('手机已连接', { exact: true })).toBeVisible();
  await dialog.getByRole('button', { name: '返回场景', exact: true }).click();
  await expect(dialog).toHaveCount(0);
  await expect(page.getByRole('status', { name: '手机示教状态', exact: true })).toBeVisible();
  await expect.poll(() => page.evaluate(() => (window as unknown as TestWindow).__pendantHarness.state?.available)).toBe(true);
  return page.evaluate(() => (window as unknown as TestWindow).__pendantHarness.session!.sessionId);
}
async function hold(page: Page, mode: 'joint' | 'linear', rearm = true) {
  await page.evaluate(({ mode, rearm }) => {
    const h = (window as unknown as TestWindow).__pendantHarness;
    if (h.timer !== null) clearInterval(h.timer);
    if (rearm) h.emit(false, mode);
    h.emit(true, mode);
    h.timer = window.setInterval(() => h.emit(true, mode), 50);
  }, { mode, rearm });
}
async function release(page: Page, mode: 'joint' | 'linear' = 'joint') {
  await page.evaluate(mode => {
    const h = (window as unknown as TestWindow).__pendantHarness;
    if (h.timer !== null) clearInterval(h.timer);
    h.timer = null; h.emit(false, mode);
  }, mode);
}
async function renderedAccepted(page: Page) {
  const accepted = await page.getByRole('spinbutton', { name: /^机器人关节 J[1-6]$/ })
    .evaluateAll(inputs => inputs.map(input => Number((input as HTMLInputElement).value)));
  expect(accepted).toHaveLength(6);
  await expect.poll(async () => (await bodyJoints(page)).map(value => Number(value.toFixed(6)))).toEqual(accepted);
  return bodyJoints(page);
}
async function fixedBody(page: Page, joints: number[]) {
  // Observe beyond the 350 ms input lifetime and several 50 ms motion ticks.
  const samples = await bodyCanvas(page).evaluate(canvas => new Promise<number[][]>(resolve => {
    const values: number[][] = [], began = performance.now();
    const timer = setInterval(() => {
      values.push(JSON.parse(canvas.getAttribute('data-robot-joints')!));
      if (performance.now() - began >= 450) { clearInterval(timer); resolve(values); }
    }, 25);
  }));
  expect(samples.length).toBeGreaterThan(1);
  for (const sample of samples) expect(sample).toEqual(joints);
}

test('desktop pendant test transport drives native axis and linear Jog, rotates QR and releases ownership on disconnect', async ({ page }) => {
  const bridge = await installBridge(page), errors = observeBrowser(page);
  await prepare(page); const initial = await bodyJoints(page);
  const sessionId = await pair(page, true);
  await expect(page.locator('.object-panel')).toHaveAttribute('inert', '');
  await expect(page.getByRole('spinbutton', { name: '机器人关节 J1', exact: true })).toBeDisabled();
  await expect(bodyCanvas(page)).toHaveAttribute('data-gizmo-target', 'none');
  await hold(page, 'joint');
  await expect.poll(async () => (await bodyJoints(page))[0] - initial[0]).toBeGreaterThan(.2);
  await release(page); const axisEnd = await renderedAccepted(page);
  await fixedBody(page, axisEnd);
  expect(axisEnd.slice(1)).toEqual(initial.slice(1));
  const beforeLinear = await bodyPose(page);
  await hold(page, 'linear');
  await expect.poll(async () => (await bodyPose(page)).position[0] - beforeLinear.position[0]).toBeGreaterThan(.5);
  await release(page, 'linear'); const lineEnd = await renderedAccepted(page), afterLinear = await bodyPose(page);
  await fixedBody(page, lineEnd);
  expect(Math.hypot(afterLinear.position[1] - beforeLinear.position[1], afterLinear.position[2] - beforeLinear.position[2])).toBeLessThan(.002);
  expect(1 - Math.abs(afterLinear.orientation.reduce((sum, value, index) => sum + value * beforeLinear.orientation[index], 0))).toBeLessThan(1e-8);
  expect(bridge.calls.some(call => call.request.mode === 'joint' && call.result?.status === 'success')).toBe(true);
  expect(bridge.calls.some(call => call.request.mode === 'linear' && call.result?.status === 'success')).toBe(true);
  await page.getByRole('status', { name: '手机示教状态', exact: true }).getByRole('button', { name: '断开', exact: true }).click();
  await expect(page.locator('.object-panel')).not.toHaveAttribute('inert', '');
  await expect(page.getByRole('spinbutton', { name: '机器人关节 J1', exact: true })).toBeEnabled();
  await page.evaluate(id => { const h = (window as unknown as TestWindow).__pendantHarness; h.emit(false, 'joint', id); h.emit(true, 'joint', id); }, sessionId);
  await fixedBody(page, lineEnd);
  expect(await page.evaluate(() => (window as unknown as TestWindow).__pendantHarness.session)).toBeNull();
  expect(errors).toEqual([]);
});

test('desktop pendant drops a late native result on view change, retains dark theme and disconnects before saving home', async ({ page }) => {
  const bridge = await installBridge(page), errors = observeBrowser(page);
  try {
    await prepare(page); await pair(page);
    await page.getByRole('button', { name: '切换主题', exact: true }).click();
    await expect(bodyCanvas(page)).toHaveAttribute('data-theme', 'dark');
    const before = await bodyJoints(page), count = bridge.calls.length;
    bridge.delayNext(); await hold(page, 'joint');
    await expect.poll(() => bridge.calls.length).toBe(count + 1);
    await page.getByRole('button', { name: '程序', exact: true }).click();
    await expect.poll(() => bridge.cancelled.includes(bridge.calls[count].requestId)).toBe(true);
    bridge.release();
    await expect.poll(() => bridge.calls[count].result?.status).toBe('success');
    await expect.poll(() => page.evaluate(() => (window as unknown as TestWindow).__pendantHarness.state?.available)).toBe(false);
    await page.getByRole('button', { name: '场景', exact: true }).click();
    await expect.poll(() => page.evaluate(() => (window as unknown as TestWindow).__pendantHarness.state?.available)).toBe(true);
    // The old held packets continue, but must not rearm after ownership changed.
    await fixedBody(page, before);
    expect(bridge.calls).toHaveLength(count + 1);
    await release(page); await hold(page, 'joint');
    await expect.poll(async () => (await bodyJoints(page))[0] - before[0]).toBeGreaterThan(.2);
    await release(page); const accepted = await renderedAccepted(page);
    await fixedBody(page, accepted);
    const pendingCount = bridge.calls.length;
    bridge.delayNext(); await hold(page, 'joint');
    await expect.poll(() => bridge.calls.length).toBe(pendingCount + 1);
    await page.getByRole('button', { name: '返回主页', exact: true }).click();
    await expect(page.getByRole('main', { name: '项目管理', exact: true })).toBeVisible();
    expect(await page.evaluate(() => (window as unknown as TestWindow).__pendantHarness.session)).toBeNull();
    bridge.release();
    await expect.poll(() => bridge.calls[pendingCount].result?.status).toBe('success');
    await release(page);
    await enterProject(page); await waitForProjectScene(page); await selectRobotObject(page);
    await expect.poll(() => bodyJoints(page)).toEqual(accepted);
    await expect(bodyCanvas(page)).toHaveAttribute('data-theme', 'dark');
    await expect(page.getByRole('status', { name: '手机示教状态', exact: true })).toHaveCount(0);
    expect(errors).toEqual([]);
  } finally { bridge.release(); }
});
