import { invoke, isTauri } from '@tauri-apps/api/core';
import { listen } from '@tauri-apps/api/event';
import type { PendantInput, PendantSession, PendantTransport } from './types';

export const pendantTransport: PendantTransport = {
  async start() {
    if (!isTauri()) throw new Error('手机示教器需要桌面客户端。请在 macOS 或 Windows 客户端中打开项目后连接。');
    return invoke<PendantSession>('start_pendant');
  },
  async stop() { if (isTauri()) await invoke('stop_pendant'); },
  async status() { return isTauri() ? invoke<PendantSession | null>('pendant_status') : null; },
  async publish(sessionId, state) { if (isTauri()) await invoke('publish_pendant_state', { sessionId, state }); },
  async listen(callback) {
    if (!isTauri()) return () => {};
    return listen<PendantInput>('pendant-input', event => callback(event.payload));
  },
};
