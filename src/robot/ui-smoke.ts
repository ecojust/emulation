import { invoke } from '@tauri-apps/api/core';
import { nativeDesktop, solveMotion } from './client';
import type { PlanRequest, PlanResult } from './types';

let started = false;
export function isNativeUiSmoke(): boolean {
  return nativeDesktop && Boolean((window as Window & { __EMULATION_UI_SMOKE__?: boolean }).__EMULATION_UI_SMOKE__);
}

/** Only an explicit --ui-smoke-test launch enables this installed-app check. */
export async function runNativeUiSmoke(request: PlanRequest, show: (result: PlanResult) => void): Promise<void> {
  if (!isNativeUiSmoke() || started) return;
  started = true;
  const errors: string[] = [];
  window.addEventListener('error', event => errors.push(event.message));
  window.addEventListener('unhandledrejection', event => errors.push(String(event.reason)));
  try {
    const result = await solveMotion(request);
    if (result.status !== 'success' || result.path.length < 7 || Math.abs(result.selected.pose.position[1] - 30) > 0.001) throw new Error('Native IPC returned an invalid motion path');
    show(result);
    const file = await invoke<{ fileName: string; content: string }>('preview_program', { requestId: crypto.randomUUID(), request: { programName: 'UiSmoke', controller: 'abb_rapid', motion: request } });
    if (!file.fileName.endsWith('.mod') || !file.content.includes('MODULE UiSmoke') || !file.content.includes('CJointT') || !file.content.includes('MoveL')) throw new Error('Native module IPC returned an invalid module');
    const until = performance.now() + 20000;
    while (!document.querySelector('[data-loaded-parts="7"]') || !document.querySelector('.result-success')) {
      if (performance.now() > until) throw new Error('Bundled robot meshes or rendered result did not load');
      if (errors.length) throw new Error(errors.join('; '));
      await new Promise(resolve => setTimeout(resolve, 100));
    }
    const canvas = document.querySelector<HTMLCanvasElement>('canvas[aria-label="ABB IRB 1100 三维仿真场景"]');
    const gl = canvas?.getContext('webgl2');
    if (!gl || gl.isContextLost()) throw new Error('Installed WebView has no working WebGL2 context');
    for (const model of ['irb1100-4-0.58', 'irb1100-4-0.475']) for (const name of ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6']) {
      const response = await fetch(`${import.meta.env.BASE_URL}models/${model}/${name}.stl`);
      if (!response.ok || (await response.arrayBuffer()).byteLength < 84) throw new Error(`Missing packaged robot part: ${model}/${name}`);
    }
    if (errors.length) throw new Error(errors.join('; '));
    await invoke('ui_smoke_report', { success: true, detail: 'Installed WebView, Rust planner IPC, module IPC, WebGL2 and all 14 packaged meshes passed' });
  } catch (cause) {
    await invoke('ui_smoke_report', { success: false, detail: cause instanceof Error ? cause.message : String(cause) });
  }
}
