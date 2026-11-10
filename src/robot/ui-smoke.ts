import { invoke } from '@tauri-apps/api/core';
import { nativeDesktop, solveMotion } from './client';
import type { JointAngles, PlanRequest, PlanResult, Pose } from './types';
import type { GrindingRequest, GrindingResult } from './grinding-types';
import { validateGrindingResult } from './grinding-validation';
import { forwardKinematics, normalizeQuaternion, quaternionDistanceDeg } from './kinematics';
import { getRobotModel } from './model';
import { composePose } from '../workspace/frame-graph';
import type { CollisionResult } from './collision-geometry';
import { inspectEndpoint } from './endpoint-inspection';
import { buildGenericChain } from './generic-motion-chain';
import { solveGenericMotion } from './generic-motion';
import { forwardGenericKinematics } from './generic-kinematics';
import { inspectGenericEndpoint } from './generic-endpoint';
import { solveGenericPoseMotion } from './generic-pose-motion';
import { checkGenericCollisions, genericCollisionParts } from './generic-collision';
import { listModelCache, loadCatalogModel, MODEL_CATALOG } from '../catalog/client';

let started = false;
type DiagnosticsWindow = Window & {
  __EMULATION_UI_SMOKE__?: boolean;
  __EMULATION_UI_DIAGNOSTICS__?: { consoleErrors: string[]; events: string[] };
};
interface ResourceCheck { resource: string; status?: number; bytes?: number; error?: string }
/** Compare physical rotations directly; acos(dot) loses precision near equality. */
function samePhysicalOrientation(actual: Pose['orientation'], expected: Pose['orientation']): boolean {
  const a = normalizeQuaternion(actual), b = normalizeQuaternion(expected);
  return Math.min(Math.hypot(...a.map((value, index) => value - b[index])),
    Math.hypot(...a.map((value, index) => value + b[index]))) <= 1e-12;
}

export function isNativeUiSmoke(): boolean {
  return nativeDesktop && Boolean((window as DiagnosticsWindow).__EMULATION_UI_SMOKE__);
}

let projectBootstrapStarted = false;
/** Exercise the real project-home entry when an installed app is explicitly tested. */
export function startNativeUiSmokeFromProjectHome(): void {
  if (!isNativeUiSmoke() || projectBootstrapStarted) return;
  projectBootstrapStarted = true;
  void (async () => {
    async function waitFor(check: () => boolean, message: string) {
      const until = performance.now() + 30_000;
      while (!check()) {
        if (performance.now() > until) throw new Error(message);
        await new Promise(resolve => setTimeout(resolve, 100));
      }
    }
    const create = () => [...document.querySelectorAll<HTMLButtonElement>('[aria-label="项目管理"] button')]
      .find(button => button.textContent?.trim() === '新建项目' && !button.disabled);
    await waitFor(() => Boolean(create()), 'Installed project home did not become ready');
    create()!.click();
    await waitFor(() => Boolean(document.querySelector('input[aria-label="项目名称"]')), 'Installed project creation dialog did not open');
    const name = document.querySelector<HTMLInputElement>('input[aria-label="项目名称"]')!;
    Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value')!.set!.call(name, 'Installed UI verification');
    name.dispatchEvent(new Event('input', { bubbles: true }));
    const model = document.querySelector<HTMLSelectElement>('select[aria-label="项目机器人"]');
    if (!model) throw new Error('Installed project dialog has no robot selection');
    model.value = 'irb1100-4-0.58';
    model.dispatchEvent(new Event('change', { bubbles: true }));
    const submit = () => [...document.querySelectorAll<HTMLButtonElement>('[role="dialog"] button')]
      .find(button => button.textContent?.trim() === '创建并打开' && !button.disabled);
    await waitFor(() => Boolean(submit()), 'Installed project form did not accept its name and robot');
    submit()!.click();
    await waitFor(() => Boolean(document.querySelector('[data-project-id] .workspace')), 'Installed new project did not open the workspace');
  })().catch(async cause => {
    await invoke('ui_smoke_report', { success: false, detail: JSON.stringify({ stage: 'project-home', message: cause instanceof Error ? cause.message : String(cause), ...snapshot([]) }) });
  });
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
export async function runNativeUiSmoke(request: PlanRequest, show: (result: PlanResult) => void): Promise<void> {
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
  async function returnToProjectHome() {
    const home = document.querySelector<HTMLButtonElement>('[aria-label="项目导航"] button[aria-label="返回主页"]');
    if (!home || home.disabled) throw new Error('Installed workspace has no available return-home action');
    home.click();
    await waitForUi(() => Boolean(document.querySelector('main[aria-label="项目管理"]')), 'Installed project did not return home');
  }
  async function openProject(id: string) {
    const card = [...document.querySelectorAll<HTMLElement>('[data-project-card]')].find(item => item.dataset.projectCard === id);
    const open = card?.querySelector<HTMLButtonElement>('.project-card-open');
    if (!open || open.disabled) throw new Error('Installed project home lost a saved project');
    open.click();
    await waitForUi(() => document.querySelector<HTMLElement>('[data-project-id]')?.dataset.projectId === id
      && document.querySelector('[data-loaded-parts]')?.getAttribute('data-loaded-parts') === '7', 'Installed project failed to reopen', 30_000);
  }
  async function createRobotProject(name: string, modelId: string) {
    const home = document.querySelector('main[aria-label="项目管理"]');
    const create = [...(home?.querySelectorAll<HTMLButtonElement>('button') ?? [])].find(button => button.textContent?.trim() === '新建项目');
    if (!create || create.disabled) throw new Error('Installed project home has no available create action');
    create.click();
    await waitForUi(() => document.querySelector('.project-modal[role="dialog"] h2')?.textContent === '新建项目', 'Installed project create dialog did not open');
    const dialog = document.querySelector('.project-modal[role="dialog"]')!;
    const nameInput = dialog.querySelector<HTMLInputElement>('[aria-label="项目名称"]');
    const robotSelect = dialog.querySelector<HTMLSelectElement>('[aria-label="项目机器人"]');
    if (!nameInput || !robotSelect || ![...robotSelect.options].some(option => option.value === modelId)) throw new Error('Installed new project dialog lost the requested robot');
    Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value')!.set!.call(nameInput, name);
    nameInput.dispatchEvent(new Event('input', { bubbles: true }));
    Object.getOwnPropertyDescriptor(HTMLSelectElement.prototype, 'value')!.set!.call(robotSelect, modelId);
    robotSelect.dispatchEvent(new Event('change', { bubbles: true }));
    await waitForUi(() => dialog.querySelector<HTMLButtonElement>('button[type="submit"]')?.disabled === false, 'Installed project create form did not accept its inputs');
    dialog.querySelector<HTMLButtonElement>('button[type="submit"]')!.click();
    await waitForUi(() => Boolean(document.querySelector('[data-project-id]')), 'Installed project was not created');
    const id = document.querySelector<HTMLElement>('[data-project-id]')!.dataset.projectId!;
    if (document.querySelector('[aria-label="选择机器人型号"]')) throw new Error('Installed project still exposes in-place robot replacement');
    return id;
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
    stage = 'grinding-planner';
    // Exercise the real desktop command with independent world and tool frames.
    // These calls must not move the visible robot or replace the recorded target.
    const grindingBase: Pose = { position: [250, -170, 80], orientation: [Math.cos(Math.PI / 8), 0, 0, Math.sin(Math.PI / 8)] };
    const grindingTool: Pose = { position: [25, 5, 40], orientation: [Math.cos(Math.PI / 12), Math.sin(Math.PI / 12), 0, 0] };
    const grindingStart = composePose(grindingBase, composePose(forwardKinematics(getRobotModel(request.modelId), request.startJoints).pose, grindingTool));
    const grindingRequest: GrindingRequest = {
      robot: { kind: 'abb', modelId: request.modelId }, basePose: grindingBase, toolPose: grindingTool, toolRadiusMm: 2,
      startJoints: [...request.startJoints], orientation: null, speedMmPerSecond: 100, clearanceMm: 1,
      allowDetours: false, zones: [], targets: [1, 2].map(scale => {
        const joints = request.startJoints.map((q, axis) => q + (axis === 0 ? .4 * scale : axis === 5 ? scale : 0)) as JointAngles;
        return composePose(grindingBase, composePose(forwardKinematics(getRobotModel(request.modelId), joints).pose, grindingTool));
      }),
    };
    const grindingSnapshot = JSON.stringify(grindingRequest);
    const grinding = validateGrindingResult(await invoke<GrindingResult>('plan_grinding_path', { requestId: crypto.randomUUID(), request: grindingRequest }), grindingRequest);
    if (grinding.status !== 'success' || grinding.waypoints.length !== 2 || grinding.path.length < 3
      || grinding.path[0].joints.some((q, i) => Math.abs(q - request.startJoints[i]) > 1e-7)
      || !samePhysicalOrientation(grinding.path[0].pose.orientation, grindingStart.orientation)
      || quaternionDistanceDeg(grindingRequest.targets[0].orientation, grindingRequest.targets[1].orientation) < .1
      || grinding.waypoints.some((point, index) => point.targetIndex !== index
        || point.pose.position.some((value, axis) => value !== grindingRequest.targets[index].position[axis])
        || !samePhysicalOrientation(point.pose.orientation, grindingRequest.targets[index].orientation)
        || point.configuration?.[3] !== grinding.waypoints[0].configuration?.[3])
      || grinding.path.some((sample, index) => index > 0 && sample.joints.some((q, axis) => Math.abs(q - grinding.path[index - 1].joints[axis]) > 15.00001))
      || grinding.waypoints[0].kind !== 'transfer' || grinding.waypoints[1].kind !== 'process'
      || JSON.stringify(grindingRequest) !== grindingSnapshot) throw new Error('Native grinding IPC did not preserve distinct target poses, continuous configurations, TCP frames or start');
    const blocked = await invoke<GrindingResult>('plan_grinding_path', { requestId: crypto.randomUUID(), request: { ...grindingRequest,
      zones: [{ id: 'grinding-blocked', enabled: true, centerMm: grindingBase.position, sizeMm: [4000, 4000, 4000] }],
    } });
    if (blocked.status !== 'failure' || blocked.code !== 'collision' || 'path' in blocked) throw new Error('Native grinding IPC accepted an obstructed approach or returned a partial path');
    nativeEvidence.grinding = { samples: grinding.path.length, waypoints: grinding.waypoints.length, checkedIntervals: grinding.checkedIntervals,
      worldBase: grindingBase, flangeToTcp: grindingTool, targetPosesPreserved: true, continuousConfigurations: true, startPreserved: true, blockedApproachRejected: true };
    stage = 'bundled-model';

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
    const abbProjectId = document.querySelector<HTMLElement>('[data-project-id]')?.dataset.projectId;
    if (!abbProjectId) throw new Error('Installed ABB workspace has no persisted project identity');
    const entry = MODEL_CATALOG.find(model => model.brand === 'Yaskawa' && model.name.includes('GP7'));
    if (!entry) throw new Error('GP7 catalog entry is unavailable');
    const controller = new AbortController();
    const deadline = setTimeout(() => controller.abort(), 60_000);
    let solverModel: Awaited<ReturnType<typeof loadCatalogModel>> | null = null;
    try {
      await returnToProjectHome();
      catalogEvidence.projectId = await createRobotProject('Installed GP7 verification', entry.id);
      stage = 'catalog-render';
      const renderUntil = performance.now() + 60_000;
      const expectedPosition = [560, 0, 815];
      while (true) {
        const modelTitle = document.querySelector('[aria-label="项目机器人型号"]')?.textContent;
        const positions = [...document.querySelectorAll('[aria-label="当前 TCP"] [data-axis] strong')].map(node => Number(node.textContent));
        const mounted = document.querySelector('canvas')?.getAttribute('aria-label')?.includes(entry.name);
        const parts = document.querySelector('[data-loaded-parts="7"]');
        if (modelTitle?.includes(entry.name) && mounted && parts && positions.length === 3 && positions.every((value, i) => Math.abs(value - expectedPosition[i]) < 0.02)) {
          catalogEvidence.modelId = entry.id; catalogEvidence.toolPositionMm = positions; catalogEvidence.parts = 7;
          break;
        }
        if (performance.now() > renderUntil || controller.signal.aborted) throw new Error('Downloaded project did not render with its own GP7 tool coordinates');
        await new Promise(resolve => setTimeout(resolve, 100));
      }
      // The project loader performed the actual download and rendering. A
      // second independent prepared model verifies cache reuse and native IK.
      stage = 'catalog-cache';
      const cache = await listModelCache(controller.signal);
      if (!cache[entry.id]) throw new Error('Native model cache did not retain the verified download');
      let downloadedAgain = false;
      const loaded = solverModel = await loadCatalogModel(entry.id, { signal: controller.signal, onProgress: progress => { catalogEvidence.progress = progress; if (progress.phase === 'downloading') downloadedAgain = true; } });
      if (loaded.kind !== 'urdf' || loaded.model.geometries.size !== 7) throw new Error('Native cached model did not prepare all seven GP7 meshes');
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
      stage = 'generic-native-endpoint';
      const candidates = await inspectGenericEndpoint({ chain: genericRequest.chain, seedJoints: genericRequest.startJoints,
        pose: forwardGenericKinematics(genericRequest.chain, genericRequest.startJoints).pose }, controller.signal);
      if (candidates.length < 2) throw new Error('Native GP7 endpoint inspection did not return independent joint branches');
      nativeEvidence.genericEndpointCandidates = candidates.length;
      stage = 'generic-native-world-tcp';
      const basePose: Pose = { position: [250, -170, 80], orientation: [Math.cos(Math.PI / 8), 0, 0, Math.sin(Math.PI / 8)] };
      const toolPose: Pose = { position: [25, 5, 40], orientation: [Math.cos(Math.PI / 12), Math.sin(Math.PI / 12), 0, 0] };
      const worldStart = composePose(composePose(basePose, forwardGenericKinematics(genericRequest.chain, genericRequest.startJoints).pose), toolPose);
      const worldTarget: Pose = { position: [worldStart.position[0] + 3, worldStart.position[1] + 1, worldStart.position[2]], orientation: worldStart.orientation };
      // The client independently checks every returned world TCP against URDF FK.
      const poseMotion = await solveGenericPoseMotion({ chain: genericRequest.chain, startJoints: genericRequest.startJoints,
        target: worldTarget, basePose, toolPose, speedMmPerSecond: 100 }, controller.signal);
      if (poseMotion.status !== 'success' || poseMotion.path.length < 2
        || !nearTuple(poseMotion.path[0].joints, genericRequest.startJoints, 1e-6)
        || !nearTuple(poseMotion.selected.pose.position, worldTarget.position, 0.01)
        || poseMotion.path.some(sample => quaternionDistanceDeg(sample.pose.orientation, worldStart.orientation) > 0.001)) throw new Error('Native GP7 world TCP Jog did not preserve the requested framed direction');
      nativeEvidence.genericWorldTcp = { samples: poseMotion.path.length, basePose, toolPose, end: poseMotion.selected.pose };
      stage = 'generic-native-mesh';
      const collisionRequest = { modelId: entry.id, basePose,
        solutions: [{ id: 'gp7-verified', joints: genericRequest.startJoints }],
        zones: [{ id: 'containing', enabled: true, centerMm: [0, 0, 0] as [number, number, number], sizeMm: [10000, 10000, 10000] as [number, number, number] }] };
      const meshHit = await checkGenericCollisions(collisionRequest, controller.signal);
      const meshClear = await checkGenericCollisions({ ...collisionRequest, zones: [{ ...collisionRequest.zones[0], id: 'distant', centerMm: [100000, 100000, 100000] }] }, controller.signal);
      if (meshHit[0]?.collidingLinks.length !== genericCollisionParts(entry.id).length || meshHit[0].uncertainLinks.length
        || meshClear[0]?.collidingLinks.length || meshClear[0]?.uncertainLinks.length || meshClear[0]?.zones.length) throw new Error('Native GP7 verified mesh collision did not distinguish enclosed and distant zones');
      nativeEvidence.genericMesh = { hitParts: meshHit[0].collidingLinks.length, distantClear: true };

      const activeGl = document.querySelector<HTMLCanvasElement>('.robot-scene canvas')?.getContext('webgl2');
      if (!activeGl || activeGl.isContextLost()) throw new Error('WebGL2 was lost after opening the GP7 project');
      if (errors.length || earlyErrors?.consoleErrors.length || earlyErrors?.events.length) throw new Error('Runtime error occurred while loading the downloaded model');
    } finally { clearTimeout(deadline); if (solverModel?.kind === 'urdf') solverModel.dispose(); }
    stage = 'project-cover-and-persistence';
    const projectId = document.querySelector<HTMLElement>('[data-project-id]')?.dataset.projectId;
    if (!projectId) throw new Error('Installed workspace has no persisted project identity');
    await selectRobot();
    const jointInput = document.querySelector<HTMLInputElement>('[aria-label="机器人关节 J1"]');
    if (!jointInput) throw new Error('Installed GP7 joint editor is unavailable');
    Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value')!.set!.call(jointInput, '5');
    jointInput.dispatchEvent(new Event('input', { bubbles: true }));
    await waitForUi(() => currentJoints()[0] === 5 && Math.abs(currentPosition()[1]) > 1, 'Installed GP7 pose edit did not reach the scene');
    const savedJoints = currentJoints(), savedPosition = currentPosition();
    // Returning from a hidden scene must still render its real, current pixels.
    await changeView('程序');
    const home = document.querySelector<HTMLButtonElement>('[aria-label="项目导航"] button[aria-label="返回主页"]');
    if (!home || home.disabled) throw new Error('Installed workspace has no available return-home action');
    home.click();
    const projectCard = () => [...document.querySelectorAll<HTMLElement>('[data-project-card]')].find(card => card.dataset.projectCard === projectId);
    await waitForUi(() => Boolean(projectCard()?.querySelector<HTMLImageElement>('img')?.complete), 'Installed project did not save its actual cover');
    const cover = projectCard()!.querySelector<HTMLImageElement>('img')!;
    if (cover.naturalWidth < 100 || cover.naturalHeight < 100 || !cover.src.startsWith('data:image/jpeg;base64,')) throw new Error('Installed project cover is not a rendered image');
    const pixels = document.createElement('canvas'); pixels.width = 32; pixels.height = 32;
    const context = pixels.getContext('2d'); if (!context) throw new Error('Cannot inspect installed project cover');
    context.drawImage(cover, 0, 0, 32, 32);
    const rgba = context.getImageData(0, 0, 32, 32).data, colors = new Set<string>();
    for (let i = 0; i < rgba.length; i += 4) colors.add(`${rgba[i]},${rgba[i + 1]},${rgba[i + 2]}`);
    if (colors.size < 16) throw new Error('Installed project cover contains no rendered scene detail');
    projectCard()!.querySelector<HTMLButtonElement>('.project-card-open')!.click();
    await waitForUi(() => document.querySelector<HTMLElement>('[data-project-id]')?.dataset.projectId === projectId
      && document.querySelector('[data-loaded-parts]')?.getAttribute('data-loaded-parts') === '7', 'Installed persisted project failed to reopen', 30_000);
    await changeView('场景');
    await selectRobot();
    await waitForUi(() => document.querySelector('[aria-label="项目机器人型号"]')?.textContent?.includes('GP7') === true
      && nearTuple(currentJoints(), savedJoints, 1e-5) && nearTuple(currentPosition(), savedPosition, 0.02), 'Installed project did not restore the selected robot and pose');
    // Robot identity is fixed per project. Reopen the original ABB project
    // through actual navigation, rather than replacing the GP7 scene in place.
    await returnToProjectHome();
    await openProject(abbProjectId);
    if (document.querySelector('[aria-label="项目机器人型号"]')?.getAttribute('data-model-id') !== 'irb1100-4-0.58'
      || document.querySelector('[aria-label="选择机器人型号"]')) throw new Error('Installed project robot identity is not fixed');
    await changeView('程序');
    await waitForUi(() => moduleOptions().some(option => option.dataset.sourceFileId === moduleId), 'Installed ABB project lost its source module while saving the separate GP7 project');
    moduleOptions().find(option => option.dataset.sourceFileId === moduleId)!.click();
    await waitForUi(() => /\bMoveL\b/.test(document.querySelector('.robot-code-workspace .cm-content')?.textContent ?? ''), 'Installed project source content was not restored');
    nativeEvidence.project = { id: projectId, previousProjectId: abbProjectId, actualCover: [cover.naturalWidth, cover.naturalHeight], coverColors: colors.size, restoredModel: 'yaskawa-gp7', restoredJoints: savedJoints, previousModelSourcePreserved: true };
    await invoke('ui_smoke_report', { success: true, detail: JSON.stringify({ message: 'Installed offline language worker, Path context-run and module workflow, WebView, native ABB/GP7 motion, framed TCP Jog, endpoint/mesh IPC, project cover and persistence, packaged meshes, verified GP7 download, asset protocol, URDF rendering and cache reuse passed', stage, nativeEvidence, catalogEvidence, ...snapshot(resources) }) });
  } catch (cause) {
    await invoke('ui_smoke_report', { success: false, detail: JSON.stringify({ message: cause instanceof Error ? cause.message : String(cause), stage, nativeEvidence, catalogEvidence, ...snapshot(resources), errors }) });
  }
}
