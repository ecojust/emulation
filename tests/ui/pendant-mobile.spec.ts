import { expect, test, type Page } from '@playwright/test';
import { readFileSync } from 'node:fs';

interface SentInput { type: string; active?: boolean; axes?: number[]; sequence?: number; epoch?: number }
interface MobileTestWindow extends Window {
  __pendantSocket: {
    sent: SentInput[]; stop(): void; close(): void; setAvailable(value: boolean): void;
    pauseState(): void; bufferedAmount: number;
  };
}
async function openPhone(page: Page, token = 'phone-test-token-that-is-not-a-real-secret') {
  await page.setViewportSize({ width: 390, height: 844 });
  const assets = new Map(['index.html', 'controller.js', 'styles.css'].map(name => [name,
    readFileSync(new URL(`../../src/pendant/mobile/${name}`, import.meta.url), 'utf8')]));
  await page.route('**/__pendant_mobile__/**', route => {
    const name = new URL(route.request().url()).pathname.split('/').at(-1) || 'index.html';
    return route.fulfill({ status: assets.has(name) ? 200 : 404, body: assets.get(name) || '',
      contentType: name.endsWith('.js') ? 'text/javascript' : name.endsWith('.css') ? 'text/css' : 'text/html' });
  });
  await page.addInitScript(() => {
    class Socket extends EventTarget {
      readyState = 0; bufferedAmount = 0; sent: SentInput[] = []; epoch = 1; timer: ReturnType<typeof setInterval> | undefined;
      state = { robotName: 'ABB IRB1100-4/0.58', modelId: 'irb1100-4-0.58', joints: [0, 20, -30, 0, 40, 0],
        tcp: { position: [364, 0, 617], orientation: [1, 0, 0, 0] }, available: true, reason: null as string | null, moving: false };
      constructor(public url: string) {
        super(); (window as unknown as MobileTestWindow).__pendantSocket = this;
        setTimeout(() => { this.readyState = 1; this.dispatchEvent(new Event('open')); }, 0);
      }
      message(value: unknown) { this.dispatchEvent(new MessageEvent('message', { data: JSON.stringify(value) })); }
      send(data: string) {
        const packet = JSON.parse(data); this.sent.push(packet);
        if (packet.type === 'auth') setTimeout(() => {
          this.message({ type: 'ready', epoch: this.epoch, sessionId: 'browser-test-session' });
          this.message({ type: 'state', state: this.state });
          this.timer = setInterval(() => { this.message({ type: 'state', state: this.state }); }, 200);
        }, 0);
        else if (packet.type === 'input') this.state.moving = packet.active;
      }
      stop() { this.state.moving = false; this.message({ type: 'stop', epoch: ++this.epoch, message: '信号超时，已停止 Jog。' }); }
      close() { if (this.readyState === 3) return; this.readyState = 3; clearInterval(this.timer); this.dispatchEvent(new Event('close')); }
      setAvailable(value: boolean) { this.state.available = value; this.state.reason = value ? null : '电脑正在运行程序'; this.message({ type: 'state', state: this.state }); }
      pauseState() { clearInterval(this.timer); }
    }
    Object.defineProperty(window, 'WebSocket', { value: Socket });
  });
  await page.goto('/__pendant_mobile__/' + (token ? '#token=' + token : ''));
  await expect(page.getByRole('heading', { name: '示教器', exact: true })).toBeVisible();
  if (token) await expect(page.getByRole('application', { name: '虚拟摇杆' })).toHaveAttribute('aria-disabled', 'false');
}
const packets = (page: Page) => page.evaluate(() => (window as unknown as MobileTestWindow).__pendantSocket.sent.filter(packet => packet.type === 'input'));
async function hold(page: Page, x: number, y: number) {
  const box = (await page.getByRole('application', { name: '虚拟摇杆' }).boundingBox())!;
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2); await page.mouse.down();
  await page.mouse.move(box.x + box.width / 2 + box.width * .25 * x, box.y + box.height / 2 - box.height * .25 * y);
  await expect(page.getByRole('application', { name: '虚拟摇杆' })).toHaveAttribute('data-held', 'true');
}

test('phone joystick maps linear and all joint pairs and requires a fresh gesture after a server stop', async ({ page }) => {
  await openPhone(page);
  expect(new URL(page.url()).hash).toBe('');
  expect((await packets(page))[0]).toMatchObject({ active: false, epoch: 1, sequence: 1 });
  await hold(page, 1, 0);
  await expect.poll(async () => (await packets(page)).at(-1)?.axes?.[0] ?? 0).toBeGreaterThan(.5);
  await page.mouse.up();
  await expect.poll(async () => (await packets(page)).at(-1)?.active).toBe(false);
  await expect(page.getByText('已停止，拖动摇杆可继续。', { exact: true })).toBeVisible();
  await page.getByRole('button', { name: '轴模式', exact: true }).click();
  await page.getByRole('button', { name: 'J5 / J6', exact: true }).click();
  await hold(page, 0, 1);
  await expect.poll(async () => (await packets(page)).at(-1)?.axes?.[5] ?? 0).toBeGreaterThan(.5);
  const last = (await packets(page)).at(-1)!; expect(last.axes?.slice(0, 5)).toEqual([0, 0, 0, 0, 0]);
  await page.evaluate(() => (window as unknown as MobileTestWindow).__pendantSocket.stop());
  await expect(page.getByRole('application', { name: '虚拟摇杆' })).toHaveAttribute('data-held', 'false');
  await expect.poll(async () => (await packets(page)).at(-1)).toMatchObject({ active: false, epoch: 2 });
  const stoppedCount = (await packets(page)).length;
  await page.mouse.move(50, 50); await page.waitForTimeout(100);
  expect(await packets(page)).toHaveLength(stoppedCount);
  await page.mouse.up(); await hold(page, 1, 0);
  await expect.poll(async () => (await packets(page)).at(-1)?.axes?.[4] ?? 0).toBeGreaterThan(.5);
  await page.mouse.up();
  expect(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
  await page.screenshot({ path: '/tmp/emulation-pendant-mobile-light.png', fullPage: true });
  await page.emulateMedia({ colorScheme: 'dark' });
  await page.screenshot({ path: '/tmp/emulation-pendant-mobile-dark.png', fullPage: true });
});

test('phone loses its hold on blur, desktop unavailability, stale state and disconnect without resuming on reconnect', async ({ page }) => {
  await openPhone(page); await hold(page, 1, 0);
  await page.evaluate(() => window.dispatchEvent(new Event('blur')));
  await expect(page.getByRole('application', { name: '虚拟摇杆' })).toHaveAttribute('data-held', 'false');
  expect((await packets(page)).at(-1)?.active).toBe(false); await page.mouse.up();
  await hold(page, 1, 0);
  await page.evaluate(() => (window as unknown as MobileTestWindow).__pendantSocket.setAvailable(false));
  await expect(page.getByRole('application', { name: '虚拟摇杆' })).toHaveAttribute('aria-disabled', 'true');
  await expect(page.getByText('电脑正在运行程序', { exact: true })).toBeVisible();
  await page.mouse.up();
  await page.evaluate(() => (window as unknown as MobileTestWindow).__pendantSocket.setAvailable(true));
  await hold(page, 1, 0); await page.evaluate(() => (window as unknown as MobileTestWindow).__pendantSocket.pauseState());
  await expect(page.getByRole('application', { name: '虚拟摇杆' })).toHaveAttribute('data-held', 'false');
  await expect(page.getByText('电脑状态更新中断，已停止 Jog。', { exact: true })).toBeVisible();
  expect((await packets(page)).at(-1)?.active).toBe(false); await page.mouse.up();
  await page.evaluate(() => (window as unknown as MobileTestWindow).__pendantSocket.close());
  await page.getByRole('button', { name: '重新连接', exact: true }).click();
  await expect(page.getByRole('application', { name: '虚拟摇杆' })).toHaveAttribute('aria-disabled', 'false');
  expect((await packets(page))[0]).toMatchObject({ active: false, sequence: 1 });
  expect((await packets(page)).every(packet => !packet.active)).toBe(true);
});

test('phone without a pairing token stays disabled and has readable compact and landscape layouts', async ({ page }) => {
  await openPhone(page, '');
  await expect(page.getByText('配对信息已失效，请重新扫描电脑端二维码。', { exact: true })).toBeVisible();
  await expect(page.getByRole('application', { name: '虚拟摇杆' })).toHaveAttribute('aria-disabled', 'true');
  await expect(page.getByRole('button', { name: '重新连接', exact: true })).toBeHidden();
  for (const size of [{ width: 320, height: 650 }, { width: 844, height: 390 }]) {
    await page.setViewportSize(size);
    expect(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
    await expect(page.getByRole('button', { name: '停止 Jog', exact: true })).toBeVisible();
  }
});


test('primary touch cancellation and screen rotation consume a held gesture', async ({ page }) => {
  await openPhone(page);
  const protocol = await page.context().newCDPSession(page);
  await protocol.send('Emulation.setTouchEmulationEnabled', { enabled: true, maxTouchPoints: 1 });
  const stick = page.getByRole('application', { name: '虚拟摇杆' });
  const box = (await stick.boundingBox())!, x = box.x + box.width / 2, y = box.y + box.height / 2;
  await protocol.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x, y, id: 7 }] });
  await protocol.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x: x + box.width * .25, y, id: 7 }] });
  await expect.poll(async () => (await packets(page)).at(-1)?.axes?.[0] ?? 0).toBeGreaterThan(.5);
  await protocol.send('Input.dispatchTouchEvent', { type: 'touchCancel', touchPoints: [] });
  await expect(stick).toHaveAttribute('data-held', 'false');
  expect((await packets(page)).at(-1)?.active).toBe(false);
  await protocol.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x, y, id: 8 }] });
  await protocol.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x, y: y - box.height * .25, id: 8 }] });
  await expect.poll(async () => (await packets(page)).at(-1)?.axes?.[1] ?? 0).toBeGreaterThan(.5);
  await page.setViewportSize({ width: 844, height: 390 });
  await expect(stick).toHaveAttribute('data-held', 'false');
  expect((await packets(page)).at(-1)?.active).toBe(false);
  const count = (await packets(page)).length;
  await protocol.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x: x + 10, y, id: 8 }] });
  await page.waitForTimeout(100); expect(await packets(page)).toHaveLength(count);
  await protocol.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
});
