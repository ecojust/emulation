import { expect, test } from '@playwright/test';
import { spawn } from 'node:child_process';
import { mkdtempSync, readFileSync, existsSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';

// Explicit integration harness only. Pairing capabilities must not enter traces.
test.use({ trace: 'off' });
const executable = process.env.EMULATION_PENDANT_SERVER;
test.skip(!executable, 'Set EMULATION_PENDANT_SERVER to the built pendant-check example for real native socket verification.');
test.setTimeout(40_000);

interface NativeInput { active: boolean; axes: number[]; epoch: number; sequence: number; mode: string; reason: string | null }

test('actual phone assets pair over the native WebSocket and stop on desktop heartbeat loss and disconnect', async ({ page }) => {
  const directory = mkdtempSync(join(tmpdir(), 'emulation-pendant-phone-'));
  const sessionFile = join(directory, 'session.json');
  const child = spawn(resolve(executable!), [sessionFile], { stdio: ['pipe', 'pipe', 'pipe'] });
  let ready = false, output = '', stderr = '', exitCode: number | null | undefined;
  const received: NativeInput[] = [];
  child.on('exit', code => { exitCode = code; });
  child.on('error', () => { exitCode = -1; });
  child.stderr.on('data', chunk => { stderr += String(chunk); });
  child.stdout.on('data', chunk => {
    output += String(chunk);
    let newline;
    while ((newline = output.indexOf('\n')) >= 0) {
      const line = output.slice(0, newline); output = output.slice(newline + 1);
      const value = JSON.parse(line);
      if (value.type === 'ready') ready = true;
      if (value.type === 'input') received.push(value.input);
    }
  });
  let heartbeat: ReturnType<typeof setInterval> | undefined;
  try {
    await expect.poll(() => ready || exitCode !== undefined, { timeout: 12_000 }).toBe(true);
    expect(ready, 'Native pendant harness must start').toBe(true);
    const session = JSON.parse(readFileSync(sessionFile, 'utf8')) as { sessionId: string; port: number; urls: { name: string; url: string }[] };
    // Keep the real capability private; only use its fragment with loopback.
    const pairing = new URL(session.urls[0].url); pairing.hostname = '127.0.0.1';
    const state = { robotName: 'Native integration robot', modelId: 'irb1100-4-0.58', joints: [0, 20, -30, 0, 40, 0],
      tcp: { position: [364, 0, 617], orientation: [1, 0, 0, 0] }, available: true, reason: null, moving: false };
    const publish = () => child.stdin.write(JSON.stringify({ type: 'state', sessionId: session.sessionId, state }) + '\n');
    publish(); heartbeat = setInterval(publish, 150);
    await page.setViewportSize({ width: 390, height: 844 });
    try { await page.goto(pairing.toString()); } catch { throw new Error('The native pendant page could not be loaded.'); }
    await expect(page.getByRole('application', { name: '虚拟摇杆' })).toHaveAttribute('aria-disabled', 'false');
    expect(new URL(page.url()).hash).toBe('');
    await expect(page.getByText('Native integration robot', { exact: true })).toBeVisible();
    const stick = page.getByRole('application', { name: '虚拟摇杆' });
    const hold = async (x: number, y: number) => {
      const box = (await stick.boundingBox())!;
      await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2); await page.mouse.down();
      await page.mouse.move(box.x + box.width / 2 + x * box.width * .25, box.y + box.height / 2 - y * box.height * .25);
      await expect(stick).toHaveAttribute('data-held', 'true');
    };
    await hold(1, 0);
    await expect.poll(() => received.at(-1)?.active === true && received.at(-1)!.axes[0] > .5).toBe(true);
    await page.mouse.up(); await expect.poll(() => received.at(-1)?.active).toBe(false);
    await page.getByRole('button', { name: '轴模式', exact: true }).click();
    await page.getByRole('button', { name: 'J3 / J4', exact: true }).click();
    await hold(0, 1);
    await expect.poll(() => received.at(-1)?.mode === 'joint' && received.at(-1)!.axes[3] > .5).toBe(true);
    const previousEpoch = received.at(-1)!.epoch;
    clearInterval(heartbeat); heartbeat = undefined;
    await expect.poll(() => received.at(-1)?.active === false && received.at(-1)!.epoch > previousEpoch).toBe(true);
    await expect(stick).toHaveAttribute('data-held', 'false');
    await expect(stick).toHaveAttribute('aria-disabled', 'true');
    await page.mouse.up(); publish(); heartbeat = setInterval(publish, 150);
    await expect(stick).toHaveAttribute('aria-disabled', 'false');
    expect(received.at(-1)?.active).toBe(false);
    state.tcp.position[0] = 365.25; publish();
    await expect(page.locator('#tcp-x')).toHaveText('365.25');
    await hold(1, 0);
    await expect.poll(() => received.at(-1)?.active).toBe(true);
    await page.close(); await expect.poll(() => received.at(-1)?.active).toBe(false);
    expect(stderr).toBe('');
  } finally {
    clearInterval(heartbeat);
    if (exitCode === undefined) {
      child.stdin.write('{"type":"stop"}\n');
      await Promise.race([new Promise<void>(done => child.once('exit', () => done())), new Promise<void>(done => setTimeout(done, 2000))]);
      if (exitCode === undefined) child.kill();
    }
    if (existsSync(directory)) rmSync(directory, { recursive: true });
  }
});
