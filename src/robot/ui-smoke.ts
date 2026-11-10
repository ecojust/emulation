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
    renderedTargetId: document.querySelector<HTMLElement>('[data-target-point-id]')?.dataset.targetPointId ?? null,
    pathRunPhase: document.querySelector<HTMLElement>('[data-run-phase]')?.dataset.runPhase ?? null,
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
  async function waitForUi(ready: () => boolean, message: string, timeout = 15_000) {
    const deadline = performance.now() + timeout;
    while (!ready()) {
      if (performance.now() > deadline) throw new Error(message);
      if (errors.length) throw new Error(errors.join('; '));
      await new Promise(resolve => setTimeout(resolve, 100));
    }
  }
  async function changeView(name: '程序' | '场景') {
    const button = [...document.querySelectorAll<HTMLButtonElement>('.workspace-view-switch button')].find(item => item.textContent?.trim() === name);
    if (!button) throw new Error(`Installed ${name} view is unavailable`);
    button.click();
    await waitForUi(() => button.getAttribute('aria-pressed') === 'true', `Installed ${name} view did not open`);
  }
  const moduleOptions = () => [...document.querySelectorAll<HTMLButtonElement>('[aria-label="程序模块列表"] [role="option"]')];
  function assertSourceDoesNotRun() {
    const canRun = [...document.querySelectorAll<HTMLButtonElement>('.robot-code-tools button')].some(button => !button.disabled && /^(?:运行|执行)/.test(button.textContent?.trim() ?? ''));
    if (canRun) throw new Error('Installed source editor exposes execution without a source runner');
  }
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
    await changeView('程序');
    await waitForUi(() => Boolean(document.querySelector('.robot-code-workspace[data-language-id="rapid"] .cm-content')
      && document.querySelector('[aria-label="程序模块列表"]')), 'Installed local module editor did not mount');
    const sourceTools = [...document.querySelectorAll<HTMLButtonElement>('.robot-code-tools button')];
    for (const label of ['新建文件', '导入源码']) {
      const button = sourceTools.find(item => item.textContent?.trim() === label);
      if (!button || button.disabled) throw new Error(`Installed module editor cannot ${label}`);
    }
    const initialModules = moduleOptions();
    if (!initialModules.length && (!document.querySelector('.robot-code-no-file')?.textContent?.includes('Path')
      || document.querySelector('.robot-code-workspace .cm-content')?.textContent?.trim())) throw new Error('Installed empty module workspace did not offer Path conversion');
    assertSourceDoesNotRun();
    nativeEvidence.sourceEditorMounted = true;
    nativeEvidence.initialModuleCount = initialModules.length;
    await changeView('场景');
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
    const previousPathIds = new Set([...document.querySelectorAll<HTMLElement>('[data-object-kind="path"]')].map(element => element.dataset.objectId));
    show(result);
    const until = performance.now() + 20000;
    while (!document.querySelector('[data-loaded-parts="7"]') || !document.querySelector('[data-target-point-id]')
      || !document.querySelector('[data-endpoint-joints]')) {
      const graphicsError = document.querySelector('.scene-graphics-error')?.textContent;
      if (graphicsError) throw new Error(`Installed WebView cannot render the scene: ${graphicsError}`);
      const loadError = document.querySelector('.scene-load-error')?.textContent;
      if (loadError) throw new Error(`Robot mesh loader failed: ${loadError}`);
      if (performance.now() > until) throw new Error('Bundled robot meshes, recorded target or endpoint alternatives did not load');
      if (errors.length) throw new Error(errors.join('; '));
      await new Promise(resolve => setTimeout(resolve, 100));
    }
    const canvas = document.querySelector<HTMLCanvasElement>('.robot-scene canvas');
    const gl = canvas?.getContext('webgl2');
    if (!gl || gl.isContextLost()) throw new Error('Installed WebView has no working WebGL2 context');
    const earlyErrors = (window as DiagnosticsWindow).__EMULATION_UI_DIAGNOSTICS__;
    if (earlyErrors?.consoleErrors.length || earlyErrors?.events.length) throw new Error('Installed WebView reported a console or runtime error');
    if (errors.length) throw new Error(errors.join('; '));
    const nearTuple = (actual: unknown, expected: readonly number[], tolerance: number): actual is number[] =>
      Array.isArray(actual) && actual.length === expected.length
      && actual.every((value, index) => typeof value === 'number' && Number.isFinite(value) && Math.abs(value - expected[index]) <= tolerance);
    const targetField = document.querySelector<HTMLTextAreaElement>('[aria-label="目标 robtarget"]');
    const targetValue: unknown = targetField ? JSON.parse(targetField.value) : null;
    if (!Array.isArray(targetValue) || !nearTuple(targetValue[0], result.selected.pose.position, 0.001)
      || !nearTuple(targetValue[1], result.selected.pose.orientation, 1e-8)
      || !nearTuple(targetValue[2], result.selected.configuration, 0)) throw new Error('Installed target inspector did not preserve the native robot target');
    if ([...document.querySelectorAll<HTMLButtonElement>('.target-inspector button')]
      .some(button => /^检查到此点的/.test(button.textContent?.trim() ?? ''))) throw new Error('Installed target inspector still exposes the removed single-segment check');
    const smokePointId = document.querySelector<HTMLElement>('[data-target-point-id]')?.dataset.targetPointId;
    const smokePathId = [...document.querySelectorAll<HTMLButtonElement>('[data-object-kind="path"]')]
      .find(button => !previousPathIds.has(button.dataset.objectId) && button.getAttribute('aria-label') === '选择路径 Smoke Path')?.dataset.objectId;
    if (!smokePathId || !smokePointId) throw new Error('Installed native result did not create the smoke Path and target');
    const recordedTarget = targetField!.value;
    nativeEvidence.recordedTarget = { pointId: smokePointId, pathId: smokePathId, matchesNativeResult: true, endpointAlternatives: document.querySelectorAll('[data-endpoint-joints]').length };
    async function openSmokePathMenu() {
      const path = [...document.querySelectorAll<HTMLButtonElement>('[data-object-kind="path"]')]
        .find(button => button.dataset.objectId === smokePathId);
      if (!path) throw new Error('Installed smoke Path is unavailable');
      const bounds = path.getBoundingClientRect();
      path.dispatchEvent(new MouseEvent('contextmenu', { bubbles: true, cancelable: true, button: 2, clientX: bounds.left + 8, clientY: bounds.top + 8 }));
      await waitForUi(() => Boolean(document.querySelector('[role="menu"][aria-label="Path 右键菜单"]')), 'Installed Path context menu did not open');
    }
    async function selectRobot() {
      const button = document.querySelector<HTMLButtonElement>('[data-object-kind="robot"]');
      if (!button || button.disabled) throw new Error('Installed robot properties are unavailable');
      button.click();
      await waitForUi(() => document.querySelectorAll('.robot-inspector input[aria-label^="机器人关节 J"]').length === 6, 'Installed robot properties did not open');
    }
    const currentJoints = () => [...document.querySelectorAll<HTMLInputElement>('.robot-inspector input[aria-label^="机器人关节 J"]')].map(input => Number(input.value));
    const currentPosition = () => [...document.querySelectorAll('[aria-label="当前 TCP"] [data-axis] strong')].map(node => Number(node.textContent));
    stage = 'path-context-run';
    // The callback records the native target but leaves the body at startJoints.
    // Running must perform this nonzero motion through the actual context menu.
    await selectRobot();
    if (!nearTuple(currentJoints(), request.startJoints, 1e-5)
      || nearTuple(request.startJoints, result.selected.joints, 0.01)) throw new Error('Installed Path run does not start from the expected distinct robot pose');
    await openSmokePathMenu();
    const run = [...document.querySelectorAll<HTMLButtonElement>('[role="menu"][aria-label="Path 右键菜单"] [role="menuitem"]')]
      .find(button => button.textContent?.trim() === '运行');
    if (!run || run.disabled) throw new Error('Installed Path context menu cannot run the recorded target');
    run.click();
    await waitForUi(() => {
      const state = document.querySelector<HTMLElement>('[data-run-phase]');
      if (state?.dataset.runPathId !== smokePathId) return false;
      if (state.dataset.runPhase === 'error') throw new Error('Installed Path context run failed: ' + state.textContent);
      return state.dataset.runPhase === 'complete' && state.dataset.runPointId === smokePointId;
    }, 'Installed Path context run did not complete the recorded target');
    await selectRobot();
    await waitForUi(() => nearTuple(currentJoints(), result.selected.joints, 1e-4)
      && nearTuple(currentPosition(), result.selected.pose.position, 0.02), 'Installed Path run did not display the final native joints and TCP');
    nativeEvidence.pathContextRun = { pathId: smokePathId, pointId: smokePointId, completed: true, joints: currentJoints(), toolPositionMm: currentPosition() };
    const point = [...document.querySelectorAll<HTMLButtonElement>('[data-object-kind="target"]')].find(button => button.dataset.objectId === smokePointId);
    if (!point) throw new Error('Installed Path run removed its recorded target');
    point.click();
    await waitForUi(() => document.querySelector<HTMLTextAreaElement>('[aria-label="目标 robtarget"]')?.value === recordedTarget, 'Installed Path run changed the recorded target');
    stage = 'offline-path-module';
    // Exercise the same Path context menu as the user; source conversion must
    // add and select a new module without replacing any existing source file.
    await changeView('程序');
    const previousModules = new Map(moduleOptions().map(option => [option.dataset.sourceFileId, option.getAttribute('aria-label')]));
    await changeView('场景');
    await openSmokePathMenu();
    const convert = [...document.querySelectorAll<HTMLButtonElement>('[role="menu"][aria-label="Path 右键菜单"] [role="menuitem"]')]
      .find(button => button.textContent?.trim() === '转换为程序');
    if (!convert || convert.disabled) throw new Error('Installed Path cannot convert its recorded pose into a module');
    convert.click();
    await waitForUi(() => moduleOptions().some(option => option.getAttribute('aria-selected') === 'true' && !previousModules.has(option.dataset.sourceFileId)), 'Installed Path conversion did not select a new module');
    const currentModules = moduleOptions(), selectedModule = currentModules.find(option => option.getAttribute('aria-selected') === 'true')!;
    const moduleName = selectedModule.getAttribute('aria-label') ?? '', moduleId = selectedModule.dataset.sourceFileId;
    if (!moduleId || !/\.modx?$/i.test(moduleName) || selectedModule.dataset.sourceDirty !== 'true' || currentModules.length !== previousModules.size + 1
      || [...previousModules].some(([id, name]) => !currentModules.some(option => option.dataset.sourceFileId === id && option.getAttribute('aria-label') === name))) throw new Error('Installed Path conversion replaced existing modules or created an invalid source file');
    await waitForUi(() => document.querySelector('[aria-label="当前模块"]')?.getAttribute('data-source-file-id') === moduleId
      && /\bMoveL\b/.test(document.querySelector('.robot-code-workspace .cm-content')?.textContent ?? ''), 'Installed editor did not load the converted Path module');
    const generatedSource = [...document.querySelectorAll('.robot-code-workspace .cm-line')].map(line => line.textContent).join('\n');
    if (!/\bMODULE\b/.test(generatedSource) || !/\brobtarget\b/.test(generatedSource) || !/\bENDMODULE\b/.test(generatedSource)) throw new Error('Installed Path conversion did not render a complete RAPID module');
    await waitForUi(() => Boolean(document.querySelector('[aria-label="源码辅助面板"] .robot-code-diagnostics'))
      && !document.querySelector('.robot-code-empty')?.textContent?.includes('正在检查源码'), 'Installed converted module analysis did not finish');
    if (document.querySelector('.robot-code-empty[role="alert"]')) throw new Error('Installed converted module analysis failed');
    const moduleDiagnostics = [...document.querySelectorAll<HTMLLIElement>('.robot-code-diagnostics li')]
      .filter(item => item.querySelector('.diagnostic-location')?.textContent?.startsWith(moduleName + ':'));
    if (moduleDiagnostics.some(item => item.querySelector('.severity-error'))) throw new Error('Installed source editor rejected the converted Path module');
    assertSourceDoesNotRun();
    nativeEvidence.pathModule = { name: moduleName, selected: true, sourceContainsMoveL: true, previousModulesPreserved: true, diagnostics: moduleDiagnostics.length };
    await changeView('场景');
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
    await invoke('ui_smoke_report', { success: true, detail: JSON.stringify({ message: 'Installed offline language worker, Path context-run and module workflow, WebView, native ABB/GP7 motion, endpoint/mesh IPC, packaged meshes, verified GP7 download, asset protocol, URDF rendering and cache reuse passed', stage, nativeEvidence, catalogEvidence, ...snapshot(resources) }) });
  } catch (cause) {
    await invoke('ui_smoke_report', { success: false, detail: JSON.stringify({ message: cause instanceof Error ? cause.message : String(cause), stage, nativeEvidence, catalogEvidence, ...snapshot(resources), errors }) });
  }
}
