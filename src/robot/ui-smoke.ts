import { invoke } from '@tauri-apps/api/core';
import { nativeDesktop, solveMotion } from './client';
import type { PlanRequest, PlanResult } from './types';

let started = false;
type DiagnosticsWindow = Window & {
  __EMULATION_UI_SMOKE__?: boolean;
  __EMULATION_UI_DIAGNOSTICS__?: { consoleErrors: string[]; events: string[] };
};
interface ResourceCheck { resource: string; status?: number; bytes?: number; error?: string }
export function isNativeUiSmoke(): boolean {
  return nativeDesktop && Boolean((window as DiagnosticsWindow).__EMULATION_UI_SMOKE__);
}

function snapshot(resources: ResourceCheck[]) {
  const canvas = document.querySelector<HTMLCanvasElement>('canvas[aria-label="ABB IRB 1100 三维仿真场景"]');
  const gl = canvas?.getContext('webgl2');
  const debug = gl?.getExtension('WEBGL_debug_renderer_info');
  return {
    userAgent: navigator.userAgent,
    platform: navigator.platform,
    readyState: document.readyState,
    viewport: [innerWidth, innerHeight, devicePixelRatio],
    loadedParts: document.querySelector('[data-loaded-parts]')?.getAttribute('data-loaded-parts') ?? null,
    modelStatus: document.querySelector('.scene-model-status')?.textContent ?? null,
    graphicsError: document.querySelector('.scene-graphics-error')?.textContent ?? null,
    loadError: document.querySelector('.scene-load-error')?.textContent ?? null,
    renderedResult: Boolean(document.querySelector('.result-success')),
    alerts: [...document.querySelectorAll('[role="alert"]')].map(element => element.textContent),
    canvas: canvas ? { width: canvas.width, height: canvas.height } : null,
    webgl2: gl ? {
      contextLost: gl.isContextLost(),
      version: gl.getParameter(gl.VERSION),
      vendor: gl.getParameter(gl.VENDOR),
      renderer: gl.getParameter(gl.RENDERER),
      gpu: debug ? gl.getParameter(debug.UNMASKED_RENDERER_WEBGL) : null,
    } : null,
    earlyErrors: (window as DiagnosticsWindow).__EMULATION_UI_DIAGNOSTICS__ ?? null,
    resources,
  };
}

/** Only an explicit --ui-smoke-test launch enables this installed-app check. */
export async function runNativeUiSmoke(request: PlanRequest, show: (result: PlanResult) => void): Promise<void> {
  if (!isNativeUiSmoke() || started) return;
  started = true;
  const errors: string[] = [];
  const resources: ResourceCheck[] = [];
  window.addEventListener('error', event => errors.push(event.message));
  window.addEventListener('unhandledrejection', event => errors.push(String(event.reason)));
  try {
    const result = await solveMotion(request);
    if (result.status !== 'success' || result.path.length < 7 || Math.abs(result.selected.pose.position[1] - 30) > 0.001) throw new Error('Native IPC returned an invalid motion path');
    show(result);
    const file = await invoke<{ fileName: string; content: string }>('preview_program', { requestId: crypto.randomUUID(), request: { programName: 'UiSmoke', controller: 'abb_rapid', motion: request } });
    if (!file.fileName.endsWith('.mod') || !file.content.includes('MODULE UiSmoke') || !file.content.includes('CJointT') || !file.content.includes('MoveL')) throw new Error('Native module IPC returned an invalid module');
    const interference = await invoke<{ id: string; collidingLinks: number[]; zones: { zoneId: string; collidingLinks: number[] }[] }[]>('check_collisions', {
      requestId: crypto.randomUUID(),
      request: { modelId: request.modelId, zones: [{ id: 'smoke-zone', enabled: true, centerMm: [0, 0, 400], sizeMm: [4000, 4000, 4000] }], solutions: [{ id: 'ui-smoke', joints: request.startJoints }] },
    });
    if (interference.length !== 1 || interference[0].id !== 'ui-smoke' || interference[0].collidingLinks.length !== 7 || interference[0].zones[0]?.zoneId !== 'smoke-zone') throw new Error('Native bounding-box inspection IPC returned an invalid result');
    // Check packaged files independently from rendering, so a missing GPU cannot
    // be misreported as a missing STL. Preserve each response in failure evidence.
    await Promise.all(['irb1100-4-0.58', 'irb1100-4-0.475'].flatMap(model => ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'].map(async name => {
      const entry: ResourceCheck = { resource: `${model}/${name}.stl` };
      resources.push(entry);
      try {
        const response = await fetch(`${import.meta.env.BASE_URL}models/${entry.resource}`);
        entry.status = response.status;
        entry.bytes = (await response.arrayBuffer()).byteLength;
        if (!response.ok || entry.bytes < 84) entry.error = 'Missing or invalid packaged STL';
      } catch (cause) { entry.error = String(cause); }
    })));
    if (resources.some(resource => resource.error)) throw new Error('Packaged STL resource verification failed');
    const until = performance.now() + 20000;
    while (!document.querySelector('[data-loaded-parts="7"]') || !document.querySelector('.result-success')) {
      const graphicsError = document.querySelector('.scene-graphics-error')?.textContent;
      if (graphicsError) throw new Error(`Installed WebView cannot render the scene: ${graphicsError}`);
      const loadError = document.querySelector('.scene-load-error')?.textContent;
      if (loadError) throw new Error(`Robot mesh loader failed: ${loadError}`);
      if (performance.now() > until) throw new Error('Bundled robot meshes or rendered result did not load');
      if (errors.length) throw new Error(errors.join('; '));
      await new Promise(resolve => setTimeout(resolve, 100));
    }
    const canvas = document.querySelector<HTMLCanvasElement>('canvas[aria-label="ABB IRB 1100 三维仿真场景"]');
    const gl = canvas?.getContext('webgl2');
    if (!gl || gl.isContextLost()) throw new Error('Installed WebView has no working WebGL2 context');
    const earlyErrors = (window as DiagnosticsWindow).__EMULATION_UI_DIAGNOSTICS__;
    if (earlyErrors?.consoleErrors.length || earlyErrors?.events.length) throw new Error('Installed WebView reported a console or runtime error');
    if (errors.length) throw new Error(errors.join('; '));
    await invoke('ui_smoke_report', { success: true, detail: JSON.stringify({ message: 'Installed WebView, Rust planner/module/interference IPC, WebGL2 and all 14 packaged meshes passed', ...snapshot(resources) }) });
  } catch (cause) {
    await invoke('ui_smoke_report', { success: false, detail: JSON.stringify({ message: cause instanceof Error ? cause.message : String(cause), ...snapshot(resources), errors }) });
  }
}
