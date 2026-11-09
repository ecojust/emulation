import { invoke } from '@tauri-apps/api/core';
import { nativeDesktop, solveMotion } from './client';
import type { PlanRequest, PlanResult } from './types';
import type { CollisionResult } from './collision-geometry';
import { inspectEndpoint } from './endpoint-inspection';
import { buildGenericChain } from './generic-motion-chain';
import { solveGenericMotion } from './generic-motion';
import { listModelCache, loadCatalogModel, MODEL_CATALOG, type LoadedCatalogModel } from '../catalog/client';

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
  const canvas = document.querySelector<HTMLCanvasElement>('.robot-scene canvas');
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
export async function runNativeUiSmoke(request: PlanRequest, show: (result: PlanResult) => void, selectCatalogModel?: (model: LoadedCatalogModel) => void): Promise<void> {
  if (!isNativeUiSmoke() || started) return;
  started = true;
  const errors: string[] = [];
  const resources: ResourceCheck[] = [];
  let stage = 'bundled-model';
  const catalogEvidence: Record<string, unknown> = {};
  const nativeEvidence: Record<string, unknown> = {};
  window.addEventListener('error', event => errors.push(event.message));
  window.addEventListener('unhandledrejection', event => errors.push(String(event.reason)));
  try {
    stage = 'offline-source-worker';
    // Execute the production inline worker from the installed custom protocol,
    // not a mocked parser or Vite-only URL. All language data must be bundled.
    const { analyzeSourceProject } = await import('../programming/analysis-client');
    const sources = [
      { modelId: 'irb1100-4-0.58', dialectId: 'rapid-rw7' as const, name: 'SMOKE.modx', text: 'MODULE Smoke\nPROC main()\nTPWrite "你好";\nENDPROC\nENDMODULE' },
      { modelId: 'fanuc-crx10ia-l', dialectId: 'fanuc-r30ib' as const, name: 'SMOKE.ls', text: '/PROG SMOKE\n/ATTR\nCOMMENT = "Offline";\n/MN\n1:R[1]=2;\n/POS\n/END' },
      { modelId: 'yaskawa-gp7', dialectId: 'inform-yrc1000' as const, name: 'SMOKE.jbi', text: '/JOB\n//NAME SMOKE\n//POS\n///NPOS 0,0,0,0,0,0\n//INST\n///ATTR SC,RW\n///GROUP1 RB1\nNOP\nSET I000 2\nEND' },
    ];
    for (const sample of sources) {
      const analysis = await analyzeSourceProject({ modelId: sample.modelId, sourceRevision: 1, dialectId: sample.dialectId, files: [{ id: 'smoke', name: sample.name, text: sample.text }] });
      if (!analysis.byFile.smoke || analysis.byFile.smoke.diagnostics.some(item => item.severity === 'error')) throw new Error(`Installed source worker rejected ${sample.dialectId}: ${JSON.stringify(analysis.byFile.smoke?.diagnostics)}`);
    }
    const invalid = await analyzeSourceProject({ modelId: 'irb1100-4-0.58', sourceRevision: 2, dialectId: 'rapid-rw7', files: [{ id: 'bad', name: 'BAD.modx', text: 'MODULE Bad PROC main() VAR num a := ; ENDPROC ENDMODULE' }] });
    if (!invalid.byFile.bad.diagnostics.some(item => item.severity === 'error')) throw new Error('Installed source worker accepted malformed RAPID');
    nativeEvidence.sourceWorker = { languages: sources.map(sample => sample.dialectId), invalidSourceRejected: true };
    stage = 'offline-editor-view';
    const programButton = [...document.querySelectorAll<HTMLButtonElement>('.workspace-view-switch button')].find(button => button.textContent === '程序');
    if (!programButton) throw new Error('Installed program view is unavailable');
    programButton.click();
    await new Promise(resolve => setTimeout(resolve, 0));
    const codeButton = [...document.querySelectorAll<HTMLButtonElement>('.program-mode-switch button')].find(button => button.textContent === '代码');
    if (!codeButton) throw new Error('Installed source mode is unavailable');
    codeButton.click();
    const editorDeadline = performance.now() + 15_000;
    while (!document.querySelector('.robot-code-workspace[data-language-id="rapid"] .cm-content')) {
      if (performance.now() > editorDeadline) throw new Error('Installed local source editor did not mount');
      if (errors.length) throw new Error(errors.join('; '));
      await new Promise(resolve => setTimeout(resolve, 100));
    }
    if (!document.querySelector('[aria-label="源码运行状态"]')?.textContent?.includes('示教')) throw new Error('Installed source run state did not separate authored code from teach simulation');
    nativeEvidence.sourceEditorMounted = true;
    [...document.querySelectorAll<HTMLButtonElement>('.workspace-view-switch button')].find(button => button.textContent === '场景')?.click();
    await new Promise(resolve => setTimeout(resolve, 0));
    stage = 'bundled-model';
    const result = await solveMotion(request);
    if (result.status !== 'success' || result.path.length < 7 || Math.abs(result.selected.pose.position[1] - 30) > 0.001) throw new Error('Native IPC returned an invalid motion path');
    const file = await invoke<{ fileName: string; content: string }>('preview_program', { requestId: crypto.randomUUID(), request: { programName: 'UiSmoke', controller: 'abb_rapid', motion: request } });
    if (!file.fileName.endsWith('.mod') || !file.content.includes('MODULE UiSmoke') || !file.content.includes('CJointT') || !file.content.includes('MoveL')) throw new Error('Native module IPC returned an invalid module');
    const alternatives = await inspectEndpoint({ modelId: request.modelId, pose: result.selected.pose, seedJoints: result.selected.joints });
    if (!alternatives.length || !alternatives.some(solution => solution.configuration.every((value, index) => value === result.selected.configuration[index])
      && solution.joints.every((value, index) => Math.abs(value - result.selected.joints[index]) < 1e-4))) throw new Error('Native endpoint inspection did not recover the recorded posture');
    nativeEvidence.endpointAlternatives = alternatives.length;
    const interference = await invoke<CollisionResult[]>('check_collisions', {
      requestId: crypto.randomUUID(),
      request: { modelId: request.modelId, zones: [{ id: 'smoke-zone', enabled: true, centerMm: [0, 0, 400], sizeMm: [4000, 4000, 4000] }], solutions: [{ id: 'ui-smoke', joints: request.startJoints }] },
    });
    if (interference.length !== 1 || interference[0].id !== 'ui-smoke' || interference[0].collidingLinks.length !== 7 || interference[0].uncertainLinks.length || interference[0].zones[0]?.zoneId !== 'smoke-zone') throw new Error('Native mesh inspection IPC returned an invalid result');
    const meshCases = await invoke<CollisionResult[]>('check_collisions', {
      requestId: crypto.randomUUID(),
      request: {
        modelId: request.modelId,
        solutions: [{ id: 'base-mesh', joints: [0, 0, 0, 0, 0, 0] }],
        zones: [
          { id: 'empty-corner', enabled: true, centerMm: [75, 75, 80], sizeMm: [4, 4, 4] },
          { id: 'surface', enabled: true, centerMm: [0, 70, 100], sizeMm: [4, 4, 4] },
          { id: 'contained', enabled: true, centerMm: [0, 0, 50], sizeMm: [2, 2, 2] },
        ],
      },
    });
    const meshResult = meshCases[0];
    if (meshCases.length !== 1 || meshResult?.id !== 'base-mesh' || meshResult.uncertainLinks.length
      || meshResult.zones.length !== 2 || meshResult.zones[0].zoneId !== 'surface' || meshResult.zones[1].zoneId !== 'contained'
      || meshResult.zones.some(zone => zone.collidingLinks.length !== 1 || zone.collidingLinks[0] !== 0 || zone.uncertainLinks.length)) throw new Error('Native triangle/containment inspection IPC failed real STL cases');
    nativeEvidence.meshCases = ['empty-corner-clear', 'surface-hit', 'contained-hit'];
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
    // Publish the point after direct native checks finish, so background inspector
    // requests triggered by selection cannot be preempted by the smoke export.
    show(result);
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
    const canvas = document.querySelector<HTMLCanvasElement>('.robot-scene canvas');
    const gl = canvas?.getContext('webgl2');
    if (!gl || gl.isContextLost()) throw new Error('Installed WebView has no working WebGL2 context');
    const earlyErrors = (window as DiagnosticsWindow).__EMULATION_UI_DIAGNOSTICS__;
    if (earlyErrors?.consoleErrors.length || earlyErrors?.events.length) throw new Error('Installed WebView reported a console or runtime error');
    if (errors.length) throw new Error(errors.join('; '));
    stage = 'catalog-download';
    if (!selectCatalogModel) throw new Error('Catalog selection callback is unavailable');
    const entry = MODEL_CATALOG.find(model => model.brand === 'Yaskawa' && model.name.includes('GP7'));
    if (!entry) throw new Error('GP7 catalog entry is unavailable');
    const controller = new AbortController();
    const deadline = setTimeout(() => controller.abort(), 60_000);
    try {
      const loaded = await loadCatalogModel(entry.id, { signal: controller.signal, onProgress: progress => { catalogEvidence.progress = progress; } });
      if (loaded.kind !== 'urdf' || loaded.model.geometries.size !== 7) throw new Error('Downloaded GP7 did not prepare all seven real meshes');
      selectCatalogModel(loaded);
      stage = 'catalog-render';
      const renderUntil = performance.now() + 10_000;
      const expectedPosition = [560, 0, 815];
      while (true) {
        const modelTitle = document.querySelector('[aria-label="选择机器人型号"]')?.textContent;
        const positions = [...document.querySelectorAll('[aria-label="当前 TCP"] [data-axis] strong')].map(node => Number(node.textContent));
        const mounted = document.querySelector('canvas')?.getAttribute('aria-label')?.includes(entry.name);
        const parts = document.querySelector('[data-loaded-parts="7"]');
        if (modelTitle?.includes(entry.name) && mounted && parts && positions.length === 3 && positions.every((value, i) => Math.abs(value - expectedPosition[i]) < 0.02)) {
          catalogEvidence.modelId = entry.id; catalogEvidence.toolPositionMm = positions; catalogEvidence.parts = 7;
          break;
        }
        if (performance.now() > renderUntil) throw new Error('Downloaded model did not render with its own GP7 tool coordinates');
        await new Promise(resolve => setTimeout(resolve, 100));
      }
      // A second load proves reuse in this app process. Offline behavior is checked
      // independently by transport tests that make every network request fail.
      stage = 'catalog-cache';
      const cache = await listModelCache(controller.signal);
      if (!cache[entry.id]) throw new Error('Native model cache did not retain the verified download');
      let downloadedAgain = false;
      const cached = await loadCatalogModel(entry.id, { signal: controller.signal, onProgress: progress => { if (progress.phase === 'downloading') downloadedAgain = true; } });
      if (cached.kind !== 'urdf') throw new Error('Native cached model returned the wrong kind');
      cached.dispose();
      if (downloadedAgain) throw new Error('Complete native model cache unexpectedly downloaded again');
      catalogEvidence.cacheReused = true;
      stage = 'generic-native-motion';
      const genericRequest = {
        chain: buildGenericChain(loaded.model),
        startJoints: [20, 15, -25, 25, 35, 10] as [number, number, number, number, number, number],
        targetJoints: [21, 14.5, -24.25, 25.5, 34.25, 11] as [number, number, number, number, number, number],
        speedMmPerSecond: 100,
      };
      const genericMotion = await solveGenericMotion(genericRequest, controller.signal);
      if (genericMotion.status !== 'success' || genericMotion.path.length < 2
        || genericMotion.path.at(-1)!.joints.some((angle, index) => Math.abs(angle - genericRequest.targetJoints[index]) > 1e-6)) throw new Error('Native GP7 straight motion did not reach the recorded joints');
      nativeEvidence.genericMotionSamples = genericMotion.path.length;
      const activeGl = document.querySelector<HTMLCanvasElement>('.robot-scene canvas')?.getContext('webgl2');
      if (!activeGl || activeGl.isContextLost()) throw new Error('WebGL2 was lost after the model switch');
      if (errors.length || earlyErrors?.consoleErrors.length || earlyErrors?.events.length) throw new Error('Runtime error occurred while loading the downloaded model');
    } finally { clearTimeout(deadline); }
    await invoke('ui_smoke_report', { success: true, detail: JSON.stringify({ message: 'Installed offline language worker, WebView, native ABB/GP7 motion, endpoint/mesh IPC, packaged meshes, verified GP7 download, asset protocol, URDF rendering and cache reuse passed', stage, nativeEvidence, catalogEvidence, ...snapshot(resources) }) });
  } catch (cause) {
    await invoke('ui_smoke_report', { success: false, detail: JSON.stringify({ message: cause instanceof Error ? cause.message : String(cause), stage, nativeEvidence, catalogEvidence, ...snapshot(resources), errors }) });
  }
}
