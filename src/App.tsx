import { useEffect, useMemo, useRef, useState } from 'react';
import RobotScene, { type RobotSceneHandle } from './scene/RobotScene';
import Icon from './ui/Icon';
import './catalog/catalog.css';
import RobotInspector from './workspace/RobotInspector';
import TargetInspector from './workspace/TargetInspector';
import ProgramPanel, { type ProgramCodeWorkspace } from './workspace/ProgramPanel';
import ModuleList from './workspace/ModuleList';
import SceneObjectTree, { type SceneTreeMenu } from './workspace/SceneObjectTree';
import { detachRobotTools, sceneRemovalLabel, type SceneRemovalKind } from './workspace/scene-removal';
import type { ProjectWorkspaceProps } from './projects/ProjectApp';
import ProjectModelLoader from './projects/ProjectModelLoader';
import { validateProjectModelWorld, type ProjectScene, type ProjectModelScene } from './projects/schema';
import SceneFrameInspector, { AttachmentSelector, PoseEditor, PoseReadout } from './workspace/SceneFrameInspector';
import ToolInspector from './workspace/ToolInspector';
import PathPlanningDialog from './workspace/PathPlanningDialog';
import { useFrameGraph } from './workspace/useFrameGraph';
import { activeToolTcpPose, baseFlangeToWorldTcp, worldTcpToBaseFlange, robotBaseWorldPose, referenceWorldPose, pathAttachment, pathWorldPose, pathPointToWorld, worldPointToPath, toolWorldPose, toolTcpWorldPose, composePose, relativePose, identityPose, type FrameObjectRef, type FrameParent } from './workspace/frame-graph';
import { planGrindingPath } from './robot/grinding';
import type { GrindingRequest, GrindingInput, GrindingSuccess } from './robot/grinding-types';
import { buildGenericCollisionBodies } from './scene/robot-collision-bounds';
import SceneObjectMenu from './workspace/SceneObjectMenu';
import ObjectRemovalDialog from './workspace/ObjectRemovalDialog';
import { createPathModule } from './workspace/path-module';
import { useProgram } from './workspace/useProgram';
import { useWorkspace, type DangerZoneDraft, type WorkspaceSelection, type WorkspaceView } from './workspace/useWorkspace';
import { surfacePathDisabledReason, type ProgramPoint, type ProgramSnapshot } from './workspace/program';
import type { SceneSurface, SurfacePick } from './surface/types';
import { validateSceneSurfaces, validateSceneSurface, evaluateSurfaceAnchor, validateSurfacePickReference } from './surface/validation';
import { surfacePickPose } from './surface/pose';
import { planSurfacePath, surfacePathGeometryReason, type SurfacePathPlan } from './surface/path-planning';
import SurfaceImportDialog from './surface/SurfaceImportDialog';
import SurfaceInspector from './surface/SurfaceInspector';
import SurfaceRegionInspector from './surface/SurfaceRegionInspector';
import SurfaceUvDialog, { type SurfaceUvInput } from './surface/SurfaceUvDialog';
import { getProgramProfile } from './workspace/program-templates';
import { useDocuments } from './programming/useDocuments';
import { useSourceAnalysis } from './programming/useSourceAnalysis';
import { loadLanguageDefinition } from './programming/registry';
import { PROGRAM_DIALECTS, getModelProgrammingProfile } from './programming/profiles';
import type { RobotLanguageDefinition } from './programming/types';
import type { SourceInsertion } from './programming/RobotCodeEditor';
import { planProgramRun, sampleProgramRun, sampleJointPositioning, ProgramRunError, type PlannedLeg } from './workspace/program-runner';
import { buildGenericChain } from './robot/generic-motion-chain';
import { solveGenericMotion } from './robot/generic-motion';
import type { GenericPlanRequest } from './robot/generic-motion-types';
import type { LoadedCatalogModel } from './catalog/client';
import { inspectGenericEndpoint } from './robot/generic-endpoint';
import { solveGenericPoseMotion } from './robot/generic-pose-motion';
import { checkGenericCollisions, genericCollisionParts } from './robot/generic-collision';
import { forwardUrdfKinematics } from './catalog/kinematics';
import { nativeDesktop, solveMotion } from './robot/client';
import { inspectEndpoint } from './robot/endpoint-inspection';
import { isNativeUiSmoke, runNativeUiSmoke } from './robot/ui-smoke';
import { forwardKinematics, quaternionDistanceDeg } from './robot/kinematics';
import { getConfiguration } from './robot/configuration';
import { checkCollisions, type CollisionResult } from './robot/collision';
import { createPoseJogController, type PoseJogController, type PoseJogSuccess, type JogStatus } from './robot/jog-controller';
import { getRobotModel } from './robot/model';
import { normalizeOrientation } from './robot/rapid';
import type { IKSolution, JointAngles, MotionSample, PlanRequest, Pose, Quaternion, Vec3 } from './robot/types';
import './styles.css';
import './App.css';

type EndpointSolution = Omit<IKSolution, 'configuration'> & { configuration?: IKSolution['configuration'] };
const GENERIC_READY: JointAngles = [20, 15, -25, 25, 35, 10];
type View = 'iso' | 'front' | 'top';
type CollisionStatus = 'pending' | 'hit' | 'uncertain' | 'clear' | 'error' | 'off';
interface CollisionView { key: string; status: 'pending' | 'ready' | 'error'; items: CollisionResult[]; error?: string }
interface ExecutionStatus { pathId?: string; phase: 'idle' | 'planning' | 'running' | 'positioning' | 'error' | 'complete'; message: string; pointId: string | null; line: number; cycle: number }
const axes = ['X', 'Y', 'Z'] as const;
const EMPTY_PATH: MotionSample[] = [];
const numberText = (value: number, digits = 6) => Number(value.toFixed(digits)).toString();
const strings = (values: readonly number[]) => values.map(value => numberText(value));
function finite(values: readonly string[]): number[] {
  if (values.some(value => !value.trim() || !Number.isFinite(Number(value)))) throw new Error('请填写完整的有限关节角；当前保留上一个有效姿态。');
  return values.map(Number);
}
function quaternionToEulerZYX(value: Quaternion): Vec3 {
  const [w, x, y, z] = normalizeOrientation(value);
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const locked = Math.abs(Math.cos(pitch)) < 1e-7;
  const yaw = locked ? Math.atan2(-2 * (x * y - w * z), 1 - 2 * (x * x + z * z)) : Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const roll = locked ? 0 : Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  return [yaw, pitch, roll].map(angle => angle * 180 / Math.PI) as Vec3;
}
/** Mount no workspace/source hooks until the project's intended model is ready. */
export default function App(props: Partial<ProjectWorkspaceProps> = {}) {
  return <ProjectModelLoader project={props.project} onCancel={props.onCancelOpen}>{loaded => <Workspace {...props} initialModel={loaded}/>}</ProjectModelLoader>;
}
function Workspace({ project, sourceRepository, onReturnHome, onSaveScene, initialModel }: Partial<ProjectWorkspaceProps> & { initialModel: LoadedCatalogModel }) {
  const initialScene = project?.scene;
  const initialId = initialModel.entry.id;
  const [surfaces, setSurfaces] = useState<SceneSurface[]>(() => initialScene?.surfaces ?? []);
  const surfacesRef = useRef(surfaces); surfacesRef.current = surfaces;
  const [surfaceImportOpen, setSurfaceImportOpen] = useState(false);
  const [surfacePickingPathId, setSurfacePickingPathId] = useState<string | null>(null);
  const [surfaceUvPathId, setSurfaceUvPathId] = useState<string | null>(null);
  const [surfacePreview, setSurfacePreview] = useState<SurfacePathPlan | null>(null);
  const [surfaceBusy, setSurfaceBusy] = useState(false);
  const surfaceOperation = useRef<AbortController | null>(null);
  const surfacePlanContexts = useRef(new WeakMap<SurfacePathPlan, string>());
  const savedBody = initialScene?.models[initialId];
  const sceneHandle = useRef<RobotSceneHandle | null>(null);
  const bodyStates = useRef<Record<string, Pick<ProjectModelScene, 'robotPresent' | 'joints'>>>(Object.fromEntries(Object.entries(initialScene?.models ?? {}).map(([id, saved]) => [id, { robotPresent: saved.robotPresent, joints: saved.joints }])));
  const [robotPresent, setRobotPresent] = useState(savedBody?.robotPresent ?? true);
  const [homeBusy, setHomeBusy] = useState(false), [homeError, setHomeError] = useState('');
  const homeOperation = useRef(false);
  const saveSceneBeforeClose = useRef<(() => Promise<void>) | null>(null);
  const modelId = initialModel.kind === 'bundled' ? initialModel.motionModelId : 'irb1100-4-0.58';
  const model = getRobotModel(modelId);
  const catalogModel = initialModel.kind === 'urdf' ? initialModel : null;
  const genericModel = catalogModel?.model ?? null;
  const activeName = catalogModel?.entry.name ?? model.name;
  const activeBrand = catalogModel?.entry.brand ?? 'ABB';
  const activeModelId = catalogModel?.entry.id ?? modelId;
  useEffect(() => () => catalogModel?.dispose(), [catalogModel]);
  const [nativeJoints, setNativeJoints] = useState<JointAngles>([...(savedBody?.joints ?? model.readyJoints)]);
  const [jointDraft, setJointDraft] = useState<string[]>(strings(savedBody?.joints ?? model.readyJoints));
  const [catalogJoints, setCatalogJoints] = useState<number[]>(initialModel.kind === 'urdf' ? [...(savedBody?.joints ?? initialModel.model.description.homeJoints)] : []);
  const [catalogDraft, setCatalogDraft] = useState<string[]>(initialModel.kind === 'urdf' ? strings(savedBody?.joints ?? initialModel.model.description.homeJoints) : []);
  const [speed, setSpeed] = useState(initialScene?.speed ?? 100);
  const program = useProgram(activeModelId, Object.fromEntries(Object.entries(initialScene?.models ?? {}).map(([id, saved]) => [id, saved.program])));
  const frameState = useFrameGraph(activeModelId, Object.fromEntries(Object.entries(initialScene?.models ?? {}).map(([id, saved]) => [id, saved.graph]))), graph = frameState.graph;
  const documents = useDocuments(activeModelId, sourceRepository ? { repository: sourceRepository } : undefined);
  const documentsRef = useRef(documents); documentsRef.current = documents;
  const activeModelRef = useRef(activeModelId); activeModelRef.current = activeModelId;
  const [languageState, setLanguageState] = useState<{ key: string; loadedKey: string; definition: RobotLanguageDefinition | null; error: string }>({ key: '', loadedKey: '', definition: null, error: '' });
  const [languageRetry, setLanguageRetry] = useState(0);
  const languageKey = activeModelId + ':' + documents.project.dialectId;
  const sourceLanguage = languageState.loadedKey.startsWith(activeModelId + ':') ? languageState.definition : null;
  const sourceLanguagePending = languageState.loadedKey !== languageKey;
  const [sourceBusy, setSourceBusy] = useState(false), [sourceMessage, setSourceMessage] = useState('');
  const sourceOperation = useRef<{ modelId: string; controller?: AbortController } | null>(null);
  const [analysisPaused, setAnalysisPaused] = useState(false);
  const [closeState, setCloseState] = useState<'saving' | 'error' | null>(null);
  const [closeError, setCloseError] = useState('');
  const closeAttempt = useRef({ busy: false, allow: false });
  const alive = useRef(true);
  const profile = getProgramProfile(activeModelId);
  const genericChain = useMemo(() => {
    try { return genericModel ? { chain: buildGenericChain(genericModel), error: '' } : null; }
    catch (error) { return { chain: null, error: error instanceof Error ? error.message : '当前 URDF 不能用于直线运动' }; }
  }, [genericModel]);
  const workspace = useWorkspace({ modelKey: activeModelId, pathIds: program.paths.map(path => path.id), targetIds: program.points.map(point => point.id), frameIds: graph.frames.map(frame => frame.id), toolIds: graph.tools.map(tool => tool.id), surfaceIds: surfaces.map(surface => surface.id), surfacePointIds: program.paths.flatMap(path => path.surface?.points.map(point => point.id) ?? []), initial: initialScene });
  const { activeView, leftCollapsed, selection, zones, selectedZone, selectedZoneId, zoneConfiguration } = workspace;
  const sourceAnalysis = useSourceAnalysis(documents.project, activeView === 'program' && documents.ready && !analysisPaused);
  const selectedSurface = selection.kind === 'surface' ? surfaces.find(surface => surface.id === selection.id) ?? null : null;
  const selectedSurfacePath = selection.kind === 'surface-point' ? program.paths.find(path => path.surface?.points.some(point => point.id === selection.id)) ?? null : selection.kind === 'path' ? program.paths.find(path => path.id === selection.id && path.surface) ?? null : null;
  const selectedSurfacePoint = selection.kind === 'surface-point' ? selectedSurfacePath?.surface?.points.find(point => point.id === selection.id) ?? null : null;
  const selectedPoint = selection.kind === 'target' ? program.points.find(point => point.id === selection.id) ?? null : selectedSurfacePoint && selectedSurfacePath?.surface?.kind === 'uv' ? program.points.find(point => point.id === selectedSurfacePath.pointIds[selectedSurfacePath.surface!.points.indexOf(selectedSurfacePoint)]) ?? null : null;
  const selectedPath = selection.kind === 'path' ? program.paths.find(path => path.id === selection.id) ?? null : null;
  const selectedFrame = selection.kind === 'frame' ? graph.frames.find(frame => frame.id === selection.id) ?? null : null;
  const selectedTool = selection.kind === 'tool' ? graph.tools.find(tool => tool.id === selection.id) ?? null : null;
  const [planningPathId, setPlanningPathId] = useState<string | null>(null);
  const [planningPreview, setPlanningPreview] = useState<GrindingSuccess | null>(null);
  const [message, setMessage] = useState('');
  const [view, setView] = useState<View>(initialScene?.view ?? 'iso');
  const [viewRevision, setViewRevision] = useState(0);
  const [showFrames, setShowFrames] = useState(initialScene?.showFrames ?? false);
  const [jogEnabled, setJogEnabled] = useState(false);
  const [jogMode, setJogMode] = useState<'translate' | 'rotate'>('translate');
  const [jogSpace, setJogSpace] = useState<'world' | 'local'>('world');
  const [jogStatus, setJogStatus] = useState<JogStatus>({ pending: false, error: null });
  const [jogDragging, setJogDragging] = useState(false);
  const [jogResetRevision, setJogResetRevision] = useState(0);
  const jogController = useRef<PoseJogController | null>(null);
  const [zoneDragging, setZoneDragging] = useState(false);
  const inspectionRun = useRef<AbortController | null>(null);
  const collisionRun = useRef<AbortController | null>(null);
  const [inspectionRevision, setInspectionRevision] = useState(0);
  const [inspection, setInspection] = useState<{ key: string; pending: boolean; solutions: EndpointSolution[]; error: string }>({ key: '', pending: false, solutions: [], error: '' });
  const [endpointPreview, setEndpointPreview] = useState<{ modelId: string; pointId: string; revision: number; graphRevision: number; solution: EndpointSolution } | null>(null);
  const [collisionView, setCollisionView] = useState<CollisionView>({ key: '', status: 'pending', items: [] });
  const [loop, setLoop] = useState(initialScene?.loop ?? false);
  const [execution, setExecution] = useState<ExecutionStatus>({ phase: 'idle', message: '', pointId: null, line: 0, cycle: 0 });
  const executionRun = useRef<{ controller: AbortController; frame: number } | null>(null);
  const executing = surfaceBusy || execution.phase === 'planning' || execution.phase === 'running' || execution.phase === 'positioning';
  const [objectMenu, setObjectMenu] = useState<SceneTreeMenu | null>(null);
  const [removeObject, setRemoveObject] = useState<{ kind: SceneRemovalKind; id: string; modelId: string } | null>(null);
  const displayJoints = (genericModel ? catalogJoints : nativeJoints) as JointAngles;
  // Preview only changes rendering until the user explicitly updates this point.
  const previewSolution = endpointPreview?.modelId === activeModelId && endpointPreview.pointId === selectedPoint?.id && endpointPreview.revision === selectedPoint.revision && endpointPreview.graphRevision === graph.revision ? endpointPreview.solution : null;
  const visibleJoints = previewSolution?.joints ?? displayJoints;
  const currentFk = useMemo(() => genericModel ? null : forwardKinematics(model, displayJoints), [genericModel, model, displayJoints]);
  const currentFlangePose = useMemo(() => genericModel ? forwardUrdfKinematics(genericModel.description, displayJoints).toolPose : currentFk!.pose, [genericModel, displayJoints, currentFk]);
  const basePose = useMemo(() => robotBaseWorldPose(graph), [graph]);
  const toolPose = useMemo(() => activeToolTcpPose(graph), [graph]);
  const currentPose = useMemo(() => currentFlangePose ? baseFlangeToWorldTcp(graph, currentFlangePose) : null, [graph, currentFlangePose]);
  const selectedPointWorld = useMemo(() => selectedPoint ? pathPointToWorld(graph, selectedPoint.pathId, selectedPoint.pose) : null, [selectedPoint, graph]);
  const genericBodies = useMemo(() => {
    try { return genericModel ? { bodies: buildGenericCollisionBodies(genericModel), error: '' } : null; }
    catch (error) { return { bodies: null, error: error instanceof Error ? error.message : '机器人几何包络不可用' }; }
  }, [genericModel]);
  const sceneFrames = useMemo(() => graph.frames.map(frame => ({ id: frame.id, name: frame.name, pose: referenceWorldPose(graph, frame.id), selected: selection.kind === 'frame' && selection.id === frame.id })), [graph, selection]);
  const visibleFlangePose = previewSolution ? genericModel ? forwardUrdfKinematics(genericModel.description, visibleJoints).toolPose : forwardKinematics(model, visibleJoints).pose : currentFlangePose;
  const sceneTools = useMemo(() => visibleFlangePose ? graph.tools.map(tool => ({ id: tool.id, name: tool.name, pose: toolWorldPose(graph, tool.id, visibleFlangePose), tcpPose: toolTcpWorldPose(graph, tool.id, visibleFlangePose), radiusMm: tool.radiusMm, selected: selection.kind === 'tool' && selection.id === tool.id })) : [], [graph, visibleFlangePose, selection]);
  const configuration = useMemo(() => genericModel ? null : getConfiguration(model, displayJoints, currentFk!), [genericModel, model, displayJoints, currentFk]);
  const limits = useMemo(() => genericModel ? genericModel.description.actuatedJoints : model.limits.map((limit, index) => ({ ...limit, name: 'J' + (index + 1) })), [genericModel, model]);
  const poseError = useMemo(() => {
    try {
      const values = finite(genericModel ? catalogDraft : jointDraft);
      if (values.length !== limits.length) return '关节角数量与型号不匹配。';
      for (const [index, limit] of limits.entries()) if (values[index] < limit.minDeg || values[index] > limit.maxDeg) return limit.name + ' 超出关节范围；当前保留上一个有效姿态。';
      return '';
    } catch (error) { return error instanceof Error ? error.message : '关节输入无效'; }
  }, [genericModel, catalogDraft, jointDraft, limits]);
  const poseOperationDisabledReason = !robotPresent ? '请先添加机器人本体。' : homeBusy || closeState !== null ? '正在保存项目。' : executing || sourceBusy || jogStatus.pending || jogDragging ? '请等待当前操作完成后再记录姿态。' : null;
  const poseCaptureDisabledReason = poseOperationDisabledReason || poseError || (!currentPose || displayJoints.length !== 6 ? '当前模型缺少有效的六轴工具姿态。' : genericModel ? genericChain?.error || null : configuration?.singular || !configuration?.configuration ? (configuration?.reason || '当前姿态没有有效 robconf') + '；请调整关节或使用示例姿态。' : null);
  const captureDisabledReason = previewSolution ? '请先退出端点预览，再记录姿态或检查运动。' : poseCaptureDisabledReason;
  const pointUpdateDisabledReason = previewSolution ? poseOperationDisabledReason || (!genericModel && !previewSolution.configuration ? '预览姿态没有有效 robconf，请重新检查端点。' : null) : poseCaptureDisabledReason;
  const moduleGenerationDisabled = !documents.ready || sourceBusy || executing || closeState !== null;
  const sceneZones = useMemo(() => zoneConfiguration.validZones.map(zone => ({ ...zone, name: zones.find(draft => draft.id === zone.id)?.name.trim() || '未命名危险区' })), [zoneConfiguration.validZones, zones]);
  const scenePaths = useMemo(() => program.paths.map(path => ({ id: path.id, name: path.name, selected: selection.kind === 'path' && selection.id === path.id || selectedSurfacePath?.id === path.id,
    points: path.surface ? path.surface.points.map(point => ({ id: point.id, name: point.name, pose: pathPointToWorld(graph, path.id, point.pose) })) : path.pointIds.flatMap(id => {
      const point = program.points.find(item => item.id === id);
      return point ? [{ id: point.id, name: point.pointName, pose: pathPointToWorld(graph, path.id, point.pose) }] : [];
    }) })), [program.paths, program.points, selection, selectedSurfacePath, graph]);
  const displayedScenePaths = surfacePreview ? [...scenePaths, { id: 'surface-preview', name: 'UV 预览', selected: true, points: surfacePreview.geometry.points.map((point, index) => ({ id: 'preview-' + index, name: String(index + 1), pose: point.pose })) }] : scenePaths;
  const pickingPath = program.paths.find(path => path.id === surfacePickingPathId) ?? null;
  const shownRegion = pickingPath?.surface ?? selectedSurfacePath?.surface;
  const sceneSurfaceRegion = shownRegion?.kind === 'region' ? { points: shownRegion.points.map(point => point.pick), controlIndices: shownRegion.points.flatMap((point, index) => shownRegion.controlPointIds.includes(point.id) ? [index] : []), closed: !surfacePickingPathId } : undefined;
  const zoneNames = useMemo(() => new Map(zones.map(zone => [zone.id, zone.name.trim() || '未命名危险区'])), [zones]);
  const pointKey = selectedPoint ? activeModelId + ':' + selectedPoint.id + ':' + selectedPoint.revision + ':' + graph.revision : '';
  const solutions = inspection.key === pointKey && !inspection.pending ? inspection.solutions : [];
  const collisionRequest = useMemo(() => robotPresent && !zoneConfiguration.errors.length && solutions.length ? { modelId: activeModelId, basePose, zones: zoneConfiguration.validZones, solutions: solutions.map(solution => ({ id: solution.id, joints: solution.joints })) } : null, [robotPresent, activeModelId, basePose, zoneConfiguration, solutions]);
  const collisionKey = collisionRequest ? JSON.stringify(collisionRequest) : '';
  const collisionById = useMemo(() => new Map(collisionView.items.map(item => [item.id, item])), [collisionView.items]);
  const planningContextKey = JSON.stringify([activeModelId, robotPresent, graph.revision, program.revision, zones, displayJoints, speed, surfaces.map(surface => [surface.id, surface.revision, surface.visible])]);
  const sceneContextRef = useRef(planningContextKey); sceneContextRef.current = planningContextKey;
  const sourceReadOnly = sourceBusy || executing || closeState !== null || !documents.ready || sourceLanguagePending;
  const draftStatusText = documents.draftStatus === 'saved' ? '草稿已保存' : documents.draftStatus === 'saving' ? '正在保存草稿…' : documents.draftStatus === 'pending' ? '草稿等待保存…' : documents.draftStatus === 'loading' ? '正在恢复草稿…' : '草稿保存失败';
  const sourceStatus = [...new Set([languageState.key === languageKey && languageState.error ? '语言版本加载失败：' + languageState.error : sourceLanguagePending ? '正在加载本地语言版本…' : '', sourceMessage, documents.error, draftStatusText, documents.dirty ? '源码已修改' : ''].filter(Boolean))].join(' · ');

  function sourceWritable() { return !surfaceOperation.current && documents.ready && !sourceLanguagePending && !sourceOperation.current && !executionRun.current && !closeAttempt.current.busy && !homeOperation.current; }
  function beginSourceOperation(message: string) {
    if (!sourceWritable()) return null;
    const operation = { modelId: activeModelId }; sourceOperation.current = operation;
    setSourceBusy(true); setSourceMessage(message); return operation;
  }
  function currentSourceOperation(operation: { modelId: string }) { return alive.current && sourceOperation.current === operation && activeModelRef.current === operation.modelId && !closeAttempt.current.busy; }
  function finishSourceOperation(operation: { modelId: string }) {
    if (sourceOperation.current === operation) { sourceOperation.current = null; if (alive.current) setSourceBusy(false); }
  }
  async function importSources() {
    const operation = beginSourceOperation('正在选择源文件…'); if (!operation) return;
    const owner = documents, sourceRevision = documents.project.sourceRevision, languageId = documents.project.languageId;
    try {
      const files = await (await import('./programming/source-io')).openSourceFiles(languageId);
      if (!currentSourceOperation(operation)) return;
      if (documentsRef.current.project.sourceRevision !== sourceRevision) { setSourceMessage('源码工程已改变，请重新导入。'); return; }
      if (!files?.length) { setSourceMessage('已取消导入'); return; }
      if (owner.importFiles(files)) setSourceMessage('已导入 ' + files.length + ' 个源文件。');
    } catch (error) { if (currentSourceOperation(operation)) { owner.reportError(error); setSourceMessage('源文件导入失败'); } }
    finally { finishSourceOperation(operation); }
  }
  async function exportSource() {
    const file = documents.activeFile; if (!file) return;
    const operation = beginSourceOperation('正在导出源文件…'); if (!operation) return;
    const owner = documents, fileId = file.id, snapshot = { name: file.name, text: file.text }, languageId = documents.project.languageId;
    try {
      const result = await (await import('./programming/source-io')).saveSourceFile(languageId, snapshot);
      if (!currentSourceOperation(operation)) return;
      if (!result) { setSourceMessage('已取消导出'); return; }
      if (result.outcome === 'saved') {
        owner.markFileSaved(fileId, { name: result.name, text: snapshot.text });
        setSourceMessage('源文件已保存：' + (result.path ?? result.name));
      } else setSourceMessage('已请求下载 ' + result.name + '；浏览器未确认文件保存，源码修改标记保持原状。');
    } catch (error) { if (currentSourceOperation(operation)) { owner.reportError(error); setSourceMessage('源文件导出失败'); } }
    finally { finishSourceOperation(operation); }
  }
  async function saveSourceDraft() {
    const operation = beginSourceOperation('正在保存草稿…'); if (!operation) return;
    const owner = documents;
    try { const saved = await owner.save(); if (currentSourceOperation(operation)) setSourceMessage(saved ? '草稿已保存' : '草稿保存失败；正文仍保留在当前窗口。'); }
    catch (error) { if (currentSourceOperation(operation)) owner.reportError(error); }
    finally { finishSourceOperation(operation); }
  }
  async function generatePathModule(pathId: string) {
    if (moduleGenerationDisabled || sourceOperation.current) return;
    const path = program.paths.find(item => item.id === pathId);
    if (!path) return;
    const reason = surfaceMotionReason(pathId); if (reason) { setMessage(reason); return; }
    const operation = { modelId: activeModelId, controller: new AbortController() }, owner = documents;
    sourceOperation.current = operation; setSourceBusy(true); setObjectMenu(null); clearPreview(); cancelInspections();
    const capturedContext = sceneContextRef.current;
    try {
      let points = [...program.pointsForPath(pathId)];
      if (needsFramedPlanning(pathId) && points.length) {
        setMessage('正在按当前坐标系与工具解析路径目标…');
        let start = [...displayJoints] as JointAngles;
        const resolved: ProgramPoint[] = [];
        const worldPoints = points.map(point => ({ ...point, pose: pathPointToWorld(graph, pathId, point.pose) }));
        for (const [index, point] of worldPoints.entries()) {
          if (!currentSourceOperation(operation) || sceneContextRef.current !== capturedContext) return;
          const leg = await planPointSegment(start, point, operation.controller.signal);
          start = [...leg.path.at(-1)!.joints];
          const conf = genericModel ? null : getConfiguration(model, start, forwardKinematics(model, start)).configuration;
          if (!genericModel && !conf) throw new Error('变换后的目标没有有效轴配置。');
          resolved.push({ ...points[index], joints: start, configuration: conf });
        }
        points = resolved;
      }
      if (!currentSourceOperation(operation) || sceneContextRef.current !== capturedContext) return;
      const generated = createPathModule(activeModelId, path, points, documents.project.dialectId, documents.files.map(file => file.name),
        { toolPose, referencePose: relativePose(basePose, pathWorldPose(graph, pathId)) });
      const file = owner.addFile(generated.name, generated.text);
      if (!file) { setMessage('模块创建失败，路径和已有文件已保留。'); return; }
      setSourceMessage('已从 ' + path.name + ' 创建模块 ' + file.name + '。'); setMessage(''); changeWorkspaceView('program');
    } catch (error) { if (currentSourceOperation(operation)) setMessage(error instanceof Error ? error.message : '路径转换失败'); }
    finally { finishSourceOperation(operation); }
  }
  async function insertSourcePose(input: { fileId: string; source: string; from: number; to: number }): Promise<SourceInsertion> {
    if (!sourceWritable() || captureDisabledReason || !currentPose) throw new Error(captureDisabledReason || '当前不能插入机器人姿态。');
    const modelKey = activeModelId, owner = documents, file = owner.files.find(item => item.id === input.fileId), revision = owner.project.sourceRevision;
    if (!file || file.text !== input.source) throw new Error('源码文件已改变，请重试。');
    const captured: ProgramSnapshot = { pose: { position: [...currentFlangePose!.position], orientation: [...currentFlangePose!.orientation] }, joints: [...displayJoints], configuration: genericModel ? null : [...configuration!.configuration!] as ProgramSnapshot['configuration'], speedMmPerSecond: speed };
    const module = await import('./programming/source-insertion');
    if (!alive.current || activeModelRef.current !== modelKey || documentsRef.current.project.sourceRevision !== revision || sourceOperation.current || executionRun.current || closeAttempt.current.busy) throw new Error('工程或机器人状态已改变，请重新插入姿态。');
    const insertion = module.insertCurrentSourcePose(modelKey, input.source, input.from, input.to, captured);
    return graph.activeToolId ? { ...insertion, message: (insertion.message ?? '') + ' 此处记录机器人基座坐标下的法兰姿态，使用程序默认零偏置工具。' } : insertion;
  }
  function retrySourceAnalysis() { setAnalysisPaused(true); }
  async function finishNativeClose(discard = false) {
    if (!nativeDesktop || closeAttempt.current.busy) return;
    surfaceOperation.current?.abort(); setSurfaceImportOpen(false); setSurfacePickingPathId(null); setSurfaceUvPathId(null); setSurfacePreview(null);
    closeAttempt.current.busy = true; stopExecution(false); stopJog(); clearPreview(); cancelInspections(); sourceOperation.current?.controller?.abort(); sourceOperation.current = null; setSourceBusy(false); setPlanningPathId(null); setPlanningPreview(null); setObjectMenu(null); setRemoveObject(null); setCloseState('saving'); setCloseError('');
    try {
      if (!discard && (!await documentsRef.current.flushAll() || documentsRef.current.hasPendingDrafts())) {
        setCloseError('部分型号的草稿未能保存。正文仍在当前窗口，可重试保存，或继续编辑并导出源文件。'); setCloseState('error'); return;
      }
      if (!discard) await saveSceneBeforeClose.current?.();
      const window = (await import('@tauri-apps/api/window')).getCurrentWindow();
      closeAttempt.current.allow = true;
      await window.close();
    } catch (error) {
      closeAttempt.current.allow = false; setCloseError(error instanceof Error ? error.message : '关闭窗口失败'); setCloseState('error');
    } finally { closeAttempt.current.busy = false; }
  }
  const codeWorkspace: ProgramCodeWorkspace | undefined = sourceLanguage && documents.ready ? {
    language: sourceLanguage, files: documents.files, activeFileId: documents.project.activeFileId,
    onActiveFileChange: id => documents.selectFile(id),
    onTextChange: (id, text) => sourceWritable() && documents.updateText(id, text),
    analysisByFile: sourceAnalysis.result.byFile, pendingFileIds: sourceAnalysis.pending || analysisPaused ? documents.files.map(file => file.id) : [],
    analysisError: sourceAnalysis.error, onRetryAnalysis: retrySourceAnalysis,
    readOnly: sourceReadOnly, status: sourceStatus, dirty: documents.dirty,
    onNewFile: name => { if (sourceWritable()) documents.addFile(name); },
    onRenameFile: (id, name) => { if (sourceWritable()) documents.renameFile(id, name); },
    onRemoveFile: id => { if (sourceWritable()) documents.removeFile(id); },
    onImportFiles: () => void importSources(), onExport: () => void exportSource(), onSave: () => void saveSourceDraft(),
    onInsertCurrent: insertSourcePose,
    captureDisabledReason: sourceReadOnly ? '请等待当前文件操作完成。' : captureDisabledReason,
    toolbar: <><label className="source-language-select">语言版本<select aria-label="控制器语言版本" value={documents.project.dialectId} disabled={sourceBusy || executing || closeState !== null || !documents.ready} onChange={event => { if (!sourceOperation.current && !executionRun.current && !closeAttempt.current.busy) documents.setDialect(event.target.value); }}>{PROGRAM_DIALECTS.filter(dialect => getModelProgrammingProfile(activeModelId).dialectIds.includes(dialect.id)).map(dialect => <option key={dialect.id} value={dialect.id}>{dialect.name} · {dialect.referenceVersion}</option>)}</select></label>{documents.draftStatus === 'error' && <button type="button" disabled={sourceBusy} onClick={() => void saveSourceDraft()}>重试保存草稿</button>}{languageState.key === languageKey && languageState.error && <button type="button" onClick={() => setLanguageRetry(value => value + 1)}>重试语言加载</button>}</>,
  } : undefined;

  useEffect(() => {
    if (activeView !== 'program' || !documents.ready) return;
    if (languageState.loadedKey === languageKey && languageState.definition) return;
    let cancelled = false;
    setLanguageState(current => ({ key: languageKey, loadedKey: current.loadedKey, definition: current.definition, error: '' }));
    void loadLanguageDefinition(documents.project.dialectId).then(definition => {
      if (!cancelled) setLanguageState({ key: languageKey, loadedKey: languageKey, definition, error: '' });
    }, error => { if (!cancelled) setLanguageState(current => ({ ...current, key: languageKey, error: error instanceof Error ? error.message : '本地语言支持加载失败' })); });
    return () => { cancelled = true; };
  }, [activeView, documents.ready, languageKey, languageRetry]);
  useEffect(() => {
    if (!analysisPaused) return;
    const timer = window.setTimeout(() => setAnalysisPaused(false), 0); return () => window.clearTimeout(timer);
  }, [analysisPaused]);
  useEffect(() => {
    if (!nativeDesktop) return;
    let cancelled = false, unlisten: (() => void) | undefined;
    void import('@tauri-apps/api/window').then(async module => {
      const detach = await module.getCurrentWindow().onCloseRequested(async event => {
        if (closeAttempt.current.allow) { closeAttempt.current.allow = false; return; }
        if (homeOperation.current) { event.preventDefault(); return; }
        if (!documentsRef.current.hasPendingDrafts() && !saveSceneBeforeClose.current) return;
        event.preventDefault();
        if (!closeAttempt.current.busy) await finishNativeClose();
      });
      if (cancelled) detach(); else unlisten = detach;
    }).catch(error => { if (!cancelled) setSourceMessage('未能启用关闭前项目和模块草稿保存：' + (error instanceof Error ? error.message : String(error))); });
    return () => { cancelled = true; unlisten?.(); };
  }, []);
  useEffect(() => { alive.current = true; return () => { alive.current = false; sourceOperation.current?.controller?.abort(); sourceOperation.current = null; }; }, []);

  function collectScene(camera = sceneHandle.current?.camera() ?? initialScene?.camera ?? null, workspaceView = activeView): ProjectScene {
    bodyStates.current[activeModelId] = { robotPresent, joints: [...displayJoints] };
    const frames = frameState.getStore(), programs = program.getStore();
    const models: ProjectScene['models'] = {};
    for (const [id, body] of Object.entries(bodyStates.current)) {
      const frame = frames[id] ?? (id === activeModelId ? graph : undefined), storedProgram = programs[id];
      if (!frame || !storedProgram) throw new Error('型号场景尚未保存完整：' + id);
      models[id] = { ...body, graph: frame, program: storedProgram };
    }
    if (!currentFlangePose) throw new Error('当前型号缺少有效法兰位姿，项目尚未保存。');
    validateProjectModelWorld(models[activeModelId], currentFlangePose);
    return { activeModelId, models, surfaces, zones: structuredClone(zones), activeView: workspaceView, leftCollapsed, view, showFrames, loop, speed, camera };
  }
  saveSceneBeforeClose.current = onSaveScene ? async () => { await onSaveScene(collectScene()); } : null;
  async function returnHome() {
    if (!onReturnHome || homeOperation.current || sourceOperation.current || closeState !== null) return;
    homeOperation.current = true; setHomeBusy(true); setHomeError('');
    const savedView = activeView;
    stopExecution(false); stopJog(); clearPreview(); cancelInspections();
    surfaceOperation.current?.abort(); setSurfaceImportOpen(false); setSurfacePickingPathId(null); setSurfaceUvPathId(null); setSurfacePreview(null);
    setPlanningPathId(null); setPlanningPreview(null); setObjectMenu(null); setZoneDragging(false);
    workspace.setActiveView('scene');
    try {
      if (!await documentsRef.current.flushAll() || documentsRef.current.hasPendingDrafts()) throw new Error('模块草稿尚未保存，请重试；当前项目仍保持打开。');
      // Let React commit the accepted body pose and remove every transient preview.
      await new Promise<void>(resolve => requestAnimationFrame(() => requestAnimationFrame(() => resolve())));
      if (!sceneHandle.current) throw new Error('三维场景尚未就绪，请稍后重试。');
      const captured = await sceneHandle.current.capture();
      const scene = collectScene(captured.camera, savedView);
      await onReturnHome(scene, captured.thumbnail);
    } catch (error) {
      setHomeError(error instanceof Error ? error.message : '项目保存失败，请重试。');
      workspace.setActiveView(savedView);
    } finally { homeOperation.current = false; if (alive.current) setHomeBusy(false); }
  }
  function surfaceEdit(action: () => void) {
    if (executing || sourceOperation.current || closeAttempt.current.busy || homeOperation.current) return;
    try { clearPreview(); stopJog(); cancelInspections(); setSurfacePreview(null); action(); }
    catch (error) { setMessage(error instanceof Error ? error.message : '表面对象更新失败。'); }
  }
  function importSceneSurface(value: SceneSurface) {
    if (executing || sourceOperation.current || homeOperation.current || closeAttempt.current.busy) throw new Error('请等待当前操作完成后导入模型。');
    const next = validateSceneSurfaces([...surfacesRef.current, value]);
    // Reserve space for source drafts and scene metadata rather than failing only on exit.
    if (new TextEncoder().encode(JSON.stringify(next)).length > 48 * 1024 * 1024) throw new Error('当前项目的模型数据超过48 MiB，请精简网格后再导入。');
    setSurfaces(next); surfacesRef.current = next;
    chooseObject({ kind: 'surface', id: value.id }); setSurfaceImportOpen(false);
    setMessage('已导入 ' + value.name + '。右键 Path 可开始点选表面区域。');
  }
  function editSurface(id: string, patch: Partial<Pick<SceneSurface, 'name' | 'visible' | 'pose'>>) {
    surfaceEdit(() => {
      const current = surfacesRef.current.find(surface => surface.id === id); if (!current) throw new Error('模型已不存在。');
      const updated = validateSceneSurface({ ...current, ...patch, revision: current.revision + (patch.pose ? 1 : 0) });
      const next = surfacesRef.current.map(surface => surface.id === id ? updated : surface); setSurfaces(next); surfacesRef.current = next;
      if (patch.pose) { program.invalidateSurfacePaths(id); setSurfacePickingPathId(null); }
    });
  }
  function surfaceMotionReason(pathId: string): string | null {
    const path = program.paths.find(item => item.id === pathId); if (!path) return 'Path 已不存在。';
    return surfacePathDisabledReason(path, { paths: program.paths, surfaceRevision: surfaces.find(surface => surface.id === path.surface?.surfaceId)?.revision ?? null }) || surfacePathGeometryReason(path, graph, surfaces);
  }
  function resolvedSurfacePoint(pointId: string) {
    const owner = program.paths.find(path => path.surface?.points.some(point => point.id === pointId));
    const index = owner?.surface?.points.findIndex(point => point.id === pointId) ?? -1;
    return owner && !surfaceMotionReason(owner.id) && index >= 0 ? program.points.find(point => point.id === owner.pointIds[index]) : undefined;
  }
  function capturePathReason(pathId: string): string | null {
    return program.paths.find(path => path.id === pathId)?.surface ? '此 Path 保存表面区域；请在普通 Path 中示教机器人点。' : captureDisabledReason;
  }
  function beginSurfacePicking(pathId: string) {
    if (executing || sourceBusy || closeState !== null) return;
    const path = program.paths.find(item => item.id === pathId); if (!path) return;
    if (path.surface?.kind === 'uv') { setMessage('UV 路径保留其表面目标，请在原区域 Path 中点选。'); return; }
    if (!surfaces.some(surface => surface.visible)) { setMessage('请先导入并显示工件模型。'); setSurfaceImportOpen(true); return; }
    const reason = surfacePathGeometryReason(path, graph, surfaces);
    if (reason) { setMessage(reason); return; }
    chooseObject({ kind: 'path', id: path.id }); workspace.setActiveView('scene');
    setObjectMenu(null); setSurfacePickingPathId(path.id); setMessage('沿模型表面按顺序点选区域边界；拖动视角不会添加点，Esc 结束。');
  }
  function addSurfacePick(pick: SurfacePick) {
    if (!surfacePickingPathId || executing || sourceBusy || homeOperation.current || closeAttempt.current.busy) return;
    try {
      const path = program.paths.find(item => item.id === surfacePickingPathId), surface = surfacesRef.current.find(item => item.id === pick.surfaceId);
      if (!path || !surface || !surface.visible || pick.surfaceRevision !== surface.revision) throw new Error('模型或区域已改变，请重新点选。');
      validateSurfacePickReference(pick, surface);
      const previous = path.surface?.points.at(-1);
      if (previous && Math.hypot(...previous.pick.position.map((value, i) => value - pick.position[i])) < .001) return;
      const pose = surfacePickPose(pick, previous ? pathPointToWorld(graph, path.id, previous.pose).orientation : currentPose?.orientation);
      program.appendSurfacePoints(path.id, [{ pose: worldPointToPath(graph, path.id, pose), pick }]);
      setMessage('已添加表面点。沿区域边界继续点选，完成后标记4个角控制点。');
    } catch (error) { setMessage(error instanceof Error ? error.message : '表面点添加失败。'); }
  }
  function setSurfaceControl(pathId: string, pointId: string, enabled: boolean) {
    surfaceEdit(() => { program.setSurfaceControl(pathId, pointId, enabled); });
  }
  function resetSurfaceRegion(pathId: string) {
    surfaceEdit(() => {
      program.clearSurfacePath(pathId);
      chooseObject({ kind: 'path', id: pathId }); workspace.setActiveView('scene');
      setSurfacePickingPathId(pathId);
      setMessage('原区域已清空，关联 UV 路径需重新生成。请在模型上重新点选边界。');
    });
  }
  function openSurfaceUv(pathId: string) {
    if (!robotPresent || executing || sourceBusy || closeState !== null) return;
    const path = program.paths.find(item => item.id === pathId);
    if (!path?.surface || path.surface.kind !== 'region' || path.surface.controlPointIds.length !== 4) { setMessage('请先点选区域，并按边界顺序设置4个角控制点。'); return; }
    const reason = surfacePathGeometryReason(path, graph, surfaces); if (reason) { setMessage(reason); return; }
    chooseObject({ kind: 'path', id: path.id }); setSurfacePickingPathId(null); setObjectMenu(null);
    setSurfacePreview(null); setSurfaceUvPathId(path.id);
  }
  async function generateSurfaceUv(settings: SurfaceUvInput, signal: AbortSignal): Promise<SurfacePathPlan> {
    const path = program.paths.find(item => item.id === surfaceUvPathId), surface = surfacesRef.current.find(item => item.id === path?.surface?.surfaceId);
    if (!path || !surface || !robotPresent || surfaceOperation.current || executionRun.current || sourceOperation.current || homeOperation.current || closeAttempt.current.busy) throw new Error('当前场景尚不能生成表面路径。');
    const controller = new AbortController(), abort = () => controller.abort();
    signal.addEventListener('abort', abort, { once: true }); if (signal.aborted) controller.abort();
    const context = sceneContextRef.current; surfaceOperation.current = controller; setSurfaceBusy(true); clearPreview(); stopJog(); cancelInspections();
    try {
      const result = await planSurfacePath({ surface, path, graph, settings,
        request: makeGrindingRequest({ targets: [], startJoints: [...displayJoints], clearanceMm: 0, speedMmPerSecond: speed }) }, controller.signal);
      if (controller.signal.aborted || !alive.current || context !== sceneContextRef.current) throw new Error('场景或区域已改变，生成结果已作废。');
      surfacePlanContexts.current.set(result, context); return result;
    } finally { signal.removeEventListener('abort', abort); if (surfaceOperation.current === controller) { surfaceOperation.current = null; if (alive.current) setSurfaceBusy(false); } }
  }
  function applySurfaceUv(result: SurfacePathPlan, settings: SurfaceUvInput) {
    if (surfacePlanContexts.current.get(result) !== sceneContextRef.current || executing || sourceOperation.current || homeOperation.current || closeAttempt.current.busy) throw new Error('场景、区域或机器人姿态已变化，请重新生成。');
    const source = program.paths.find(path => path.id === surfaceUvPathId), surface = surfacesRef.current.find(item => item.id === source?.surface?.surfaceId);
    if (!source?.surface || !surface) throw new Error('原区域或模型已不存在。');
    const reason = surfacePathGeometryReason(source, graph, surfaces); if (reason) throw new Error(reason);
    const samples = result.geometry.points.map(point => ({ pose: point.pose, pick: { surfaceId: surface.id, surfaceRevision: surface.revision, ...point.anchor, ...evaluateSurfaceAnchor(surface, point.anchor) } }));
    const snapshots = result.motion.waypoints.map(point => ({ pose: point.pose, joints: point.joints, configuration: point.configuration, speedMmPerSecond: speed, motionRole: 'process' as const }));
    // A new Path starts in world coordinates, so its saved poses equal the verified mesh targets exactly.
    const path = program.createUvPath(source.id, source.surface.revision, samples, settings, { resolvedSnapshots: snapshots });
    setSurfacePreview(null); setSurfaceUvPathId(null); workspace.selectObject({ kind: 'path', id: path.id });
    setMessage('已创建 ' + path.name + '：全部目标来自模型表面，机器人姿态连续性检查通过。');
  }
  useEffect(() => {
    if (!surfacePickingPathId) return;
    const finish = (event: KeyboardEvent) => { if (event.key === 'Escape') { setSurfacePickingPathId(null); setMessage('已结束区域点选，请在右侧标记4个角控制点。'); } };
    window.addEventListener('keydown', finish); return () => window.removeEventListener('keydown', finish);
  }, [surfacePickingPathId]);
  useEffect(() => () => { surfaceOperation.current?.abort(); }, []);
  function writeBodyPose(joints: JointAngles) {
    if (genericModel) { setCatalogJoints([...joints]); setCatalogDraft(strings(joints)); }
    else { setNativeJoints([...joints]); setJointDraft(strings(joints)); }
  }
  function clearPreview() { setEndpointPreview(null); }
  function stopJog() {
    jogController.current?.dispose(); jogController.current = null;
    setJogStatus({ pending: false, error: null }); setJogDragging(false);
    setJogResetRevision(value => value + 1);
  }
  function cancelInspections() {
    inspectionRun.current?.abort(); collisionRun.current?.abort();
  }
  function chooseObject(next: WorkspaceSelection) {
    if (JSON.stringify(selection) === JSON.stringify(next)) return;
    clearPreview(); stopJog(); cancelInspections(); setMessage('');
    if (surfacePickingPathId && !(next.kind === 'path' && next.id === surfacePickingPathId) && !(next.kind === 'surface-point' && program.paths.find(path => path.id === surfacePickingPathId)?.surface?.points.some(point => point.id === next.id))) setSurfacePickingPathId(null);
    workspace.selectObject(next);
  }
  function changeWorkspaceView(next: WorkspaceView) {
    if (executionRun.current && next === 'program') return;
    if (next === activeView) return;
    setEndpointPreview(null); stopJog(); setZoneDragging(false); setSurfacePickingPathId(null);
    workspace.setActiveView(next);
  }
  function changeView(next: View) { stopJog(); setZoneDragging(false); setView(next); setViewRevision(value => value + 1); }
  function editJoint(index: number, value: string) {
    if (!robotPresent || homeOperation.current || executionRun.current || sourceOperation.current) return;
    stopJog(); clearPreview();
    const drafts = [...(genericModel ? catalogDraft : jointDraft)];
    drafts[index] = value;
    if (genericModel) setCatalogDraft(drafts); else setJointDraft(drafts);
    try {
      const values = finite(drafts);
      if (!values.every((angle, i) => angle >= limits[i].minDeg && angle <= limits[i].maxDeg)) return;
      if (genericModel) setCatalogJoints(values); else setNativeJoints(values as JointAngles);
    } catch { /* Numeric drafts retain the previous rendered pose. */ }
  }
  function restoreProjectRobot() {
    if (robotPresent || executing || surfaceOperation.current || sourceBusy || sourceOperation.current || executionRun.current || homeBusy || homeOperation.current || homeError || closeState !== null || closeAttempt.current.busy || removeObject !== null) return;
    // Restore this project's existing body without changing its base or joints.
    chooseObject({ kind: 'robot' });
    setRobotPresent(true);
    bodyStates.current[activeModelId] = { robotPresent: true, joints: [...displayJoints] };
    setMessage('已恢复本项目的机器人本体。');
  }
  function setRobotPose(kind: 'ready' | 'zero') {
    if (!robotPresent || homeOperation.current || executionRun.current || sourceOperation.current) return;
    stopJog(); clearPreview();
    if (genericModel) { const joints = kind === 'ready' ? GENERIC_READY : genericModel.description.homeJoints; setCatalogJoints([...joints]); setCatalogDraft(strings(joints)); }
    else { const joints = kind === 'ready' ? model.readyJoints : model.homeJoints; setNativeJoints([...joints]); setJointDraft(strings(joints)); }
  }
  function snapshot(speedValue = speed, pathId?: string): ProgramSnapshot {
    if (captureDisabledReason || !currentPose) throw new Error(captureDisabledReason || '当前姿态没有有效工具坐标。');
    return { pose: pathId ? worldPointToPath(graph, pathId, currentPose) : currentPose, joints: [...displayJoints], configuration: genericModel ? null : configuration!.configuration!, speedMmPerSecond: speedValue };
  }
  function createPath() {
    if (executing || sourceBusy) return;
    try {
      const path = program.createPath();
      chooseObject({ kind: 'path', id: path.id });
      setMessage('已创建 ' + path.name + '，右键添加当前点。');
    } catch (error) { setMessage(error instanceof Error ? error.message : 'Path 创建失败'); }
  }
  function captureToPath(pathId: string) {
    if (closeState !== null || closeAttempt.current.busy) return;
    try {
      const point = program.captureToPath(pathId, snapshot(speed, pathId));
      // Keep the active robot/Jog selection so repeated teaching stays fluid.
      if (!(selection.kind === 'robot' && jogEnabled)) chooseObject({ kind: 'target', id: point.id });
      setMessage('已将当前姿态记录为 ' + point.pointName + '。');
    } catch (error) { setMessage(error instanceof Error ? error.message : '姿态记录失败'); }
  }
  function updatePointFromCurrent() {
    if (!selectedPoint || homeOperation.current || closeAttempt.current.busy || sourceOperation.current || executionRun.current) return;
    try {
      if (pointUpdateDisabledReason) throw new Error(pointUpdateDisabledReason);
      // Endpoint inspection already validates FK, limits and configuration for
      // this point and graph revision. Keep the exact recorded target, including
      // its Path-local frame and process role, instead of round-tripping IK FK.
      const next: ProgramSnapshot = previewSolution ? { ...selectedPoint, joints: [...previewSolution.joints], configuration: genericModel ? null : [...previewSolution.configuration!] } : snapshot(selectedPoint.speedMmPerSecond, selectedPoint.pathId);
      program.updatePoint(selectedPoint.id, next);
      clearPreview(); cancelInspections();
      setMessage('已用' + (previewSolution ? '预览姿态' : '当前姿态') + '更新 ' + selectedPoint.pointName + '。');
    }
    catch (error) { setMessage(error instanceof Error ? error.message : '目标点更新失败'); }
  }
  function confirmRemoveObject() {
    const requested = removeObject;
    setRemoveObject(null);
    if (!requested || requested.modelId !== activeModelId || executionRun.current || executing || sourceOperation.current || homeOperation.current || closeState !== null || closeAttempt.current.busy) return;
    clearPreview(); stopJog(); cancelInspections(); setPlanningPathId(null); setPlanningPreview(null); setSurfacePickingPathId(null); setSurfacePreview(null);
    try {
      if (requested.kind === 'robot') {
        stopExecution(false); setJogEnabled(false);
        frameState.replaceGraph(detachRobotTools(graph, currentFlangePose ?? undefined));
        program.invalidateSurfaceSolutions(); setRobotPresent(false);
        bodyStates.current[activeModelId] = { robotPresent: false, joints: [...displayJoints] };
        workspace.selectObject({ kind: 'robot' });
      } else if (requested.kind === 'surface') {
        const next = surfacesRef.current.filter(surface => surface.id !== requested.id); setSurfaces(next); surfacesRef.current = next; program.invalidateSurfacePaths(requested.id); workspace.selectObject({ kind: 'robot' });
      } else if (requested.kind === 'surface-point') {
        const owner = program.paths.find(path => path.surface?.points.some(point => point.id === requested.id));
        if (owner) { program.removeSurfacePoint(owner.id, requested.id); workspace.selectObject({ kind: 'path', id: owner.id }); }
      } else if (requested.kind === 'frame') { frameState.removeFrame(requested.id); program.invalidateSurfaceSolutions(); workspace.selectObject({ kind: 'robot' }); }
      else if (requested.kind === 'tool') { frameState.removeTool(requested.id); program.invalidateSurfaceSolutions(); workspace.selectObject({ kind: 'robot' }); }
      else if (requested.kind === 'zone') { workspace.setZones(current => current.filter(zone => zone.id !== requested.id)); workspace.selectObject({ kind: 'robot' }); }
      else if (requested.kind === 'path') {
        program.removePath(requested.id); frameState.removePath(requested.id);
        workspace.selectObject({ kind: 'robot' });
      } else {
        const point = program.points.find(item => item.id === requested.id);
        program.removePoint(requested.id);
        workspace.selectObject(point ? { kind: 'path', id: point.pathId } : { kind: 'robot' });
      }
    } catch (error) { setMessage(error instanceof Error ? error.message : '对象删除失败'); return; }
    setMessage('已删除场景对象，生成的模块文件保持不变。');
  }
  function previewEndpoint(solution: EndpointSolution) {
    if (!selectedPoint || !robotPresent || homeOperation.current || closeAttempt.current.busy || executionRun.current || sourceOperation.current || jogStatus.pending || jogDragging) return;
    setEndpointPreview({ modelId: activeModelId, pointId: selectedPoint.id, revision: selectedPoint.revision, graphRevision: graph.revision, solution });
    workspace.setActiveView('scene');
  }
  function graphEdit(action: () => void) {
    if (executionRun.current || executing || sourceOperation.current || sourceBusy || closeState !== null) throw new Error('请先停止当前操作，再修改坐标对象。');
    clearPreview(); stopJog(); cancelInspections(); setPlanningPreview(null);
    action(); program.invalidateSurfaceSolutions(); setSurfacePreview(null); setMessage('');
  }
  function editSceneGraph(action: () => void) {
    try { graphEdit(action); } catch (error) { setMessage(error instanceof Error ? error.message : '坐标对象更新失败'); }
  }
  function reparentObject(object: FrameObjectRef, parent: FrameParent) {
    if (parent.kind === 'flange' && !robotPresent) throw new Error('请先添加机器人本体。');
    if (object.kind === 'tool' && parent.kind === 'flange' && graph.tools.some(tool => tool.id !== object.id && tool.parent.kind === 'flange')) {
      throw new Error('当前机器人已挂载一个工具，请先将原工具移回世界坐标。');
    }
    graphEdit(() => frameState.reparent(object, parent, currentFlangePose ?? undefined));
  }
  function addReferenceFrame() {
    editSceneGraph(() => { const frame = frameState.addFrame(); workspace.selectObject({ kind: 'frame', id: frame.id }); });
  }
  function addTool() {
    editSceneGraph(() => {
      const tool = frameState.addTool({ localPose: currentFlangePose ? composePose(basePose, currentFlangePose) : identityPose(), tcpPose: { position: [0, 0, 50], orientation: [1, 0, 0, 0] }, radiusMm: 10 });
      workspace.selectObject({ kind: 'tool', id: tool.id });
    });
  }
  function isIdentity(pose: Pose) { return Math.hypot(...pose.position) < 1e-7 && quaternionDistanceDeg(pose.orientation, [1, 0, 0, 0]) < 1e-7; }
  function needsFramedPlanning(pathId?: string) {
    if (!isIdentity(basePose) || !isIdentity(toolPose) || Boolean(pathId && (!isIdentity(pathWorldPose(graph, pathId)) || program.paths.find(path => path.id === pathId)?.planning || program.paths.find(path => path.id === pathId)?.surface))) return true;
    // Returning a frame/tool to identity does not make snapshots recorded under
    // its previous transform valid again. Check actual FK against each target.
    return Boolean(pathId && program.pointsForPath(pathId).some(point => {
      const flange = genericModel ? forwardUrdfKinematics(genericModel.description, point.joints).toolPose : forwardKinematics(model, point.joints).pose;
      if (!flange) return true;
      const actual = baseFlangeToWorldTcp(graph, flange), expected = pathPointToWorld(graph, pathId, point.pose);
      return Math.hypot(...actual.position.map((n, i) => n - expected.position[i])) > .01 || quaternionDistanceDeg(actual.orientation, expected.orientation) > .001;
    }));
  }
  function makeGrindingRequest(input: Omit<GrindingRequest, 'robot' | 'basePose' | 'toolPose' | 'toolRadiusMm' | 'zones'>): GrindingRequest {
    if (zoneConfiguration.errors.length) throw new Error('禁入区数据无效，请修正位置与尺寸后规划。');
    const mounted = graph.tools.filter(tool => tool.parent.kind === 'flange');
    if (mounted.length > 1) throw new Error('当前规划仅支持一个已挂载工具，请先卸下其他工具。');
    const tool = mounted[0];
    if (tool && graph.activeToolId !== tool.id) throw new Error('挂载工具与当前TCP不一致，请重新挂接工具。');
    const robot: GrindingRequest['robot'] = genericModel
      ? genericChain?.chain && genericBodies?.bodies ? { kind: 'generic', chain: genericChain.chain, bodies: genericBodies.bodies } : (() => { throw new Error(genericBodies?.error || genericChain?.error || '当前型号缺少完整运动链或几何包络'); })()
      : { kind: 'abb', modelId };
    // A shifted mount capsule is enclosed by the flange→TCP capsule after this
    // conservative inflation; no part of an offset mounted tool is omitted.
    const toolRadiusMm = tool ? tool.radiusMm + Math.hypot(...tool.localPose.position) : 0;
    return { ...input, robot, basePose, toolPose, toolRadiusMm, zones: zoneConfiguration.validZones };
  }
  function openPathPlanning(pathId: string) {
    if (!robotPresent || executing || sourceBusy || closeState !== null || !program.paths.some(path => path.id === pathId && !path.surface)) return;
    clearPreview(); stopJog(); cancelInspections(); setObjectMenu(null); workspace.selectObject({ kind: 'path', id: pathId });
    setPlanningPreview(null); setPlanningPathId(pathId);
  }
  function applyGrindingPath(result: GrindingSuccess, input: GrindingInput) {
    if (!robotPresent || !planningPathId || executing || sourceBusy || sourceOperation.current || closeAttempt.current.busy || !program.paths.some(path => path.id === planningPathId)) throw new Error('当前正在执行其他操作，请结束后重新规划。');
    const snapshots = result.waypoints.map(point => ({ pose: worldPointToPath(graph, planningPathId, point.pose), joints: point.joints,
      configuration: point.configuration, speedMmPerSecond: input.speedMmPerSecond, motionRole: point.kind }));
    program.replacePathPoints(planningPathId, snapshots, { kind: 'grinding', clearanceMm: input.clearanceMm });
    workspace.selectObject({ kind: 'path', id: planningPathId }); setPlanningPreview(null);
    setMessage('已按各目标点的位置和姿态应用连续路径。运行时将按当前基座、工具和禁入区重新验证整条运动。');
  }
  async function planPointSegment(start: JointAngles, point: ProgramPoint, signal: AbortSignal): Promise<PlannedLeg> {
    if (needsFramedPlanning(point.pathId)) {
      const path = program.paths.find(item => item.id === point.pathId);
      const result = await planGrindingPath(makeGrindingRequest({ startJoints: start, targets: [point.pose],
        orientation: null, allowDetours: false,
        clearanceMm: path?.planning?.clearanceMm ?? 0, speedMmPerSecond: point.speedMmPerSecond }), signal);
      if (result.status !== 'success') throw new Error(result.message);
      const target = result.waypoints.at(-1)!;
      return { path: result.path, durationSeconds: result.durationSeconds, resolvedTarget: { joints: target.joints, pose: target.pose } };
    }
    if (genericModel) {
      if (!genericChain?.chain) throw new Error(genericChain?.error || '当前模型缺少运动学链');
      const request: GenericPlanRequest = { chain: genericChain.chain, startJoints: start, targetJoints: point.joints, speedMmPerSecond: point.speedMmPerSecond };
      const result = await solveGenericMotion(request, signal);
      if (result.status !== 'success') throw new Error(result.message);
      return result;
    }
    const request: PlanRequest = { modelId, startJoints: start, target: { position: point.pose.position, orientation: point.pose.orientation }, speedMmPerSecond: point.speedMmPerSecond };
    const result = await solveMotion(request, signal);
    if (result.status !== 'success') throw new Error(result.message);
    if (!point.configuration || result.selected.configuration.some((value, i) => value !== point.configuration![i])) throw new Error('可达运动的 robconf 与已记录目标点不一致，请调整机器人起点，或调姿后重新记录目标点。');
    return result;
  }
  function stopExecution(report = true) {
    const active = executionRun.current;
    if (active) { active.controller.abort(); cancelAnimationFrame(active.frame); executionRun.current = null; }
    if (report && active) setExecution(value => ({ ...value, phase: 'idle', message: '已停止，机器人保留当前显示姿态。' }));
    else if (!report) setExecution({ phase: 'idle', message: '', pointId: null, line: 0, cycle: 0 });
  }
  function pathRunDisabledReason(pathId: string): string | null {
    return surfaceMotionReason(pathId) || captureDisabledReason || (sourceBusy || closeState !== null ? '请等待当前操作完成。' : null)
      || (!program.paths.find(path => path.id === pathId)?.pointIds.length ? '请先向此 Path 添加目标点。' : null);
  }
  async function runPath(pathId: string) {
    const path = program.paths.find(item => item.id === pathId);
    if (!path || pathRunDisabledReason(pathId) || executionRun.current || closeAttempt.current.busy) return;
    const points = program.pointsForPath(path.id).map(point => ({ ...point, pose: pathPointToWorld(graph, path.id, point.pose) }));
    const start = [...displayJoints] as JointAngles;
    clearPreview(); stopJog(); cancelInspections(); setObjectMenu(null); setZoneDragging(false);
    workspace.selectObject({ kind: 'path', id: path.id }); workspace.setActiveView('scene');
    const active = { controller: new AbortController(), frame: 0 }; executionRun.current = active;
    setExecution({ pathId: path.id, phase: 'planning', message: '正在检查 ' + path.name + '…', pointId: null, line: 0, cycle: 0 });
    try {
      const plan = await planProgramRun({ points, startJoints: start, loop, signal: active.controller.signal,
        planLeg: planPointSegment,
        onProgress: status => {
          if (executionRun.current === active) setExecution({ pathId: path.id, phase: 'planning', message: path.name + ' · 正在检查第 ' + status.lineNumber + ' 条指令' + (status.closing ? '（循环闭合）' : '') + '…', pointId: status.pointId, line: status.lineNumber, cycle: 0 });
        } });
      if (executionRun.current !== active || active.controller.signal.aborted) return;
      const started = performance.now();
      const tick = (now: number) => {
        if (executionRun.current !== active || active.controller.signal.aborted) return;
        const frame = sampleProgramRun(plan, (now - started) / 1000);
        writeBodyPose(frame.joints);
        const point = program.points.find(item => item.id === frame.pointId);
        setExecution({ pathId: path.id, phase: frame.finished ? 'complete' : 'running', message: frame.finished ? path.name + ' 仿真完成。' : path.name + ' · 正在执行第 ' + frame.lineNumber + ' 条 · ' + (point?.pointName ?? '') + (frame.cycle ? ' · 循环 ' + frame.cycle : ''), pointId: frame.pointId, line: frame.lineNumber, cycle: frame.cycle });
        if (frame.finished) executionRun.current = null;
        else active.frame = requestAnimationFrame(tick);
      };
      active.frame = requestAnimationFrame(tick);
    } catch (error) {
      if (executionRun.current !== active || active.controller.signal.aborted) return;
      executionRun.current = null;
      setExecution({ pathId: path.id, phase: 'error', message: error instanceof Error ? error.message : '程序检查失败', pointId: error instanceof ProgramRunError ? error.pointId : null, line: error instanceof ProgramRunError ? error.lineNumber : 0, cycle: 0 });
    }
  }
  function moveToPoint(id: string) {
    const point = program.points.find(item => item.id === id);
    setObjectMenu(null);
    if (!robotPresent || !point || executionRun.current || sourceOperation.current || closeAttempt.current.busy || jogStatus.pending || jogDragging) return;
    if (needsFramedPlanning(point.pathId)) { void moveFramedPoint(point); return; }
    const from = [...displayJoints] as JointAngles;
    clearPreview(); stopJog(); cancelInspections(); setZoneDragging(false);
    workspace.selectObject({ kind: 'target', id }); workspace.setActiveView('scene');
    const active = { controller: new AbortController(), frame: 0 }; executionRun.current = active;
    const duration = Math.max(0.4, ...from.map((angle, i) => 1.5 * Math.abs(point.joints[i] - angle) / (limits[i].maxSpeedDeg || 60)));
    const started = performance.now();
    setExecution({ phase: 'positioning', message: '正在定位到 ' + point.pointName + '；此操作不是直线运动验证。', pointId: id, line: 0, cycle: 0 });
    const tick = (now: number) => {
      if (executionRun.current !== active || active.controller.signal.aborted) return;
      const fraction = Math.min(1, (now - started) / (duration * 1000));
      writeBodyPose(sampleJointPositioning(from, point.joints, fraction));
      if (fraction >= 1) { executionRun.current = null; setExecution({ phase: 'complete', message: '已定位到 ' + point.pointName + ' 的记录姿态；未验证移动路径。', pointId: id, line: 0, cycle: 0 }); }
      else active.frame = requestAnimationFrame(tick);
    };
    active.frame = requestAnimationFrame(tick);
  }
  async function moveFramedPoint(point: ProgramPoint) {
    const start = [...displayJoints] as JointAngles, target = { ...point, pose: pathPointToWorld(graph, point.pathId, point.pose) };
    clearPreview(); stopJog(); cancelInspections(); setZoneDragging(false); workspace.selectObject({ kind: 'target', id: point.id }); workspace.setActiveView('scene');
    const active = { controller: new AbortController(), frame: 0 }; executionRun.current = active;
    setExecution({ phase: 'planning', message: '正在按当前参考坐标系与工具检查定位轨迹…', pointId: point.id, line: 1, cycle: 0 });
    try {
      const plan = await planProgramRun({ points: [target], startJoints: start, loop: false, signal: active.controller.signal, planLeg: planPointSegment });
      if (active.controller.signal.aborted || executionRun.current !== active) return;
      const began = performance.now();
      const tick = (now: number) => {
        if (active.controller.signal.aborted || executionRun.current !== active) return;
        const frame = sampleProgramRun(plan, (now - began) / 1000); writeBodyPose(frame.joints);
        setExecution({ phase: frame.finished ? 'complete' : 'positioning', message: frame.finished ? '已定位到当前参考坐标系下的 ' + point.pointName + '。' : '正在按当前工具定位到 ' + point.pointName + '…', pointId: point.id, line: 1, cycle: 0 });
        if (frame.finished) executionRun.current = null; else active.frame = requestAnimationFrame(tick);
      };
      active.frame = requestAnimationFrame(tick);
    } catch (error) {
      if (active.controller.signal.aborted || executionRun.current !== active) return;
      executionRun.current = null; setExecution({ phase: 'error', message: error instanceof Error ? error.message : '目标定位失败', pointId: point.id, line: 1, cycle: 0 });
    }
  }
  function beginJog() {
    if (!robotPresent || selection.kind !== 'robot' || !jogEnabled || poseError || !currentPose || executionRun.current || sourceOperation.current || homeOperation.current) return;
    const joints = [...displayJoints] as JointAngles;
    jogController.current?.dispose(); clearPreview();
    writeBodyPose(joints); setJogDragging(true); setJogStatus({ pending: false, error: null });
    jogController.current = createPoseJogController<PoseJogSuccess>({ initial: { modelId: activeModelId, joints, pose: currentPose, speedMmPerSecond: speed },
      solve: async (request, signal) => {
        if (genericModel) {
          if (!genericChain?.chain) throw new Error(genericChain?.error || '运动学链不可用');
          return solveGenericPoseMotion({ chain: genericChain.chain, startJoints: request.startJoints, target: { position: request.target.position, orientation: jogMode === 'translate' ? null : request.target.orientation }, basePose, toolPose, speedMmPerSecond: request.speedMmPerSecond }, signal);
        }
        if (!isIdentity(basePose) || !isIdentity(toolPose)) {
          const result = await planGrindingPath(makeGrindingRequest({ startJoints: request.startJoints, targets: [request.target], orientation: null, allowDetours: false, clearanceMm: 0, speedMmPerSecond: request.speedMmPerSecond }), signal);
          if (result.status !== 'success') return result;
          const last = result.path.at(-1)!;
          return { status: 'success', selected: { joints: last.joints, pose: last.pose } };
        }
        return solveMotion({ ...request, modelId }, signal);
      },
      onAccept: accepted => writeBodyPose(accepted.selected.joints), onStatus: setJogStatus });
  }
  function editZone(patch: Partial<Omit<DangerZoneDraft, 'id'>>) {
    if (!selectedZoneId || executionRun.current || sourceOperation.current) return;
    collisionRun.current?.abort(); workspace.editZone(selectedZoneId, patch);
  }
  useEffect(() => { setEndpointPreview(null); }, [activeModelId, selectedPoint?.id, selectedPoint?.revision, graph.revision]);
  useEffect(() => {
    const controller = new AbortController(); inspectionRun.current?.abort(); inspectionRun.current = controller;
    if (homeBusy || !robotPresent || !selectedPoint || executing || jogStatus.pending || jogDragging) return () => controller.abort();
    setInspection({ key: pointKey, pending: true, solutions: [], error: '' });
    const pose = worldTcpToBaseFlange(graph, selectedPointWorld!);
    const pending = genericModel
      ? genericChain?.chain ? inspectGenericEndpoint({ chain: genericChain.chain, pose, seedJoints: selectedPoint.joints }, controller.signal) : Promise.reject(new Error(genericChain?.error || '运动学链不可用'))
      : inspectEndpoint({ modelId, pose, seedJoints: selectedPoint.joints }, controller.signal);
    void pending.then(found => {
      if (!controller.signal.aborted && inspectionRun.current === controller) setInspection({ key: pointKey, pending: false, solutions: found, error: '' });
    }, error => {
      if (!controller.signal.aborted && inspectionRun.current === controller) setInspection({ key: pointKey, pending: false, solutions: [], error: error instanceof Error ? error.message : '端点检查失败' });
    });
    return () => { controller.abort(); if (inspectionRun.current === controller) inspectionRun.current = null; };
  }, [homeBusy, robotPresent, pointKey, selectedPoint, selectedPointWorld, graph, genericModel, genericChain, modelId, inspectionRevision, executing, jogStatus.pending, jogDragging]);
  useEffect(() => {
    const controller = new AbortController(); collisionRun.current?.abort(); collisionRun.current = controller;
    if (homeBusy || !collisionRequest || !zones.some(zone => zone.enabled) || executing || zoneDragging) return () => controller.abort();
    setCollisionView({ key: collisionKey, status: 'pending', items: [] });
    const timer = setTimeout(() => {
      const pending = genericModel ? checkGenericCollisions(collisionRequest, controller.signal) : checkCollisions({ ...collisionRequest, modelId }, controller.signal);
      void pending.then(items => {
        if (!controller.signal.aborted && collisionRun.current === controller) setCollisionView({ key: collisionKey, status: 'ready', items });
      }, error => {
        if (!controller.signal.aborted && collisionRun.current === controller) setCollisionView({ key: collisionKey, status: 'error', items: [], error: error instanceof Error ? error.message : '干涉检测失败' });
      });
    }, 120);
    return () => { clearTimeout(timer); controller.abort(); if (collisionRun.current === controller) collisionRun.current = null; };
  }, [homeBusy, collisionRequest, collisionKey, genericModel, modelId, executing, zoneDragging, zones]);
  useEffect(() => () => { jogController.current?.dispose(); inspectionRun.current?.abort(); collisionRun.current?.abort(); executionRun.current?.controller.abort(); if (executionRun.current) cancelAnimationFrame(executionRun.current.frame); }, []);
  useEffect(() => { setObjectMenu(null); setRemoveObject(null); }, [activeModelId, activeView]);
  useEffect(() => {
    if (!isNativeUiSmoke()) return;
    const pose = forwardKinematics(model, model.readyJoints).pose;
    const request: PlanRequest = { modelId, startJoints: model.readyJoints, target: { position: [pose.position[0], 30, pose.position[2]], orientation: pose.orientation }, speedMmPerSecond: 100 };
    void runNativeUiSmoke(request, result => {
      if (result.status !== 'success') return;
      const path = program.createPath('Smoke Path');
      const point = program.captureToPath(path.id, { pose: result.selected.pose, joints: result.selected.joints, configuration: result.selected.configuration, speedMmPerSecond: 100 });
      workspace.selectObject({ kind: 'target', id: point.id });
      writeBodyPose(request.startJoints);
    });
  }, []);

  function collisionInfo(id: string): { state: CollisionStatus; label: string; title: string } {
    if (zoneConfiguration.errors.length) return { state: 'error', label: '未检查', title: zoneConfiguration.errors.map(error => error.message).join('；') };
    if (!zones.some(zone => zone.enabled)) return { state: 'off', label: '检测已关闭', title: '未设置已启用的危险区' };
    if (collisionView.key !== collisionKey || collisionView.status === 'pending' || zoneDragging) return { state: 'pending', label: '检测中', title: '正在检查端点机器人模型与危险区是否干涉' };
    const checked = collisionById.get(id);
    if (collisionView.status === 'error' || !checked) return { state: 'error', label: '检测失败', title: collisionView.error || '未取得完整检测结果' };
    const describe = (kind: 'collidingLinks' | 'uncertainLinks') => checked.zones.filter(zone => zone[kind].length).map(zone => (zoneNames.get(zone.zoneId) || '未命名危险区') + '（' + zone[kind].map(link => genericModel ? genericCollisionParts(activeModelId)[link]?.name ?? '部件 ' + link : link === 0 ? '基座' : '连杆 ' + link).join('、') + '）').join('；');
    const unknown = checked.uncertainLinks.length ? '无法确定：' + describe('uncertainLinks') + '。当前模型无法可靠确定该区域是否位于实体内部。' : '';
    const scope = '仅检查端点与已启用危险区，未检查整段路径或自碰撞。';
    if (checked.collidingLinks.length) return { state: 'hit', label: '干涉', title: '确认干涉：' + describe('collidingLinks') + '。' + unknown + scope };
    if (checked.uncertainLinks.length) return { state: 'uncertain', label: '无法确定', title: unknown + scope };
    return { state: 'clear', label: '未检测到干涉', title: scope };
  }
  function collisionBadge(id: string) {
    const info = collisionInfo(id);
    return <span className={'collision-badge collision-' + info.state} role="status" aria-label={info.label} title={info.title}>{(info.state === 'hit' || info.state === 'uncertain') && <Icon name={info.state === 'hit' ? 'warning' : 'question'} size={13}/>}{info.label}</span>;
  }
  function collisionIds(id: string, uncertain = false): string[] {
    const state = collisionInfo(id).state;
    if (state !== 'hit' && state !== 'uncertain') return [];
    return collisionById.get(id)?.zones.filter(zone => (uncertain ? zone.uncertainLinks : zone.collidingLinks).length).map(zone => zone.zoneId) ?? [];
  }
  return <div className="app-shell">
    <header className="topbar" inert={homeBusy || Boolean(homeError) || closeState !== null || removeObject !== null}>
      <a className="brand" href="./" aria-label="Emulation 首页" onClick={event => { if (onReturnHome) { event.preventDefault(); void returnHome(); } }}><span className="brand-symbol"><Icon name="cube" size={25}/></span><span>emulation<span className="brand-caption">机器人运动仿真</span></span></a>
      <div className="header-divider"/><nav className="project-label project-breadcrumbs" aria-label="项目导航">{onReturnHome && <><button type="button" className="breadcrumb-home return-home-button" aria-label="返回主页" title="返回项目主页" disabled={homeBusy || sourceBusy || closeState !== null} onClick={() => void returnHome()}><Icon name="home" size={14}/><span>主页</span></button><Icon name="chevron-right" size={13} className="breadcrumb-separator"/></>}<span className="project-context" title={project?.name} aria-current="page">{project?.name ?? '运动工作台'}</span></nav>
      <div className="workspace-view-switch" aria-label="工作区视图">{(['scene', 'program'] as const).map(item => <button key={item} type="button" aria-pressed={activeView === item} disabled={executing && item === 'program'} className={activeView === item ? 'selected' : ''} onClick={() => changeWorkspaceView(item)}>{item === 'scene' ? '场景' : '程序'}</button>)}</div>
      <div className="header-right"><span className="project-robot-model" aria-label="项目机器人型号" data-model-id={activeModelId} title={activeName}><Icon name="robot" size={15}/><span>{activeName}</span></span><span className="version">v0.4.1</span></div>
    </header>
    <main className="workspace" data-active-view={activeView} data-left-collapsed={leftCollapsed} inert={homeBusy || Boolean(homeError) || closeState !== null || removeObject !== null}>
      <aside className="object-panel" aria-label={activeView === 'program' ? '程序模块' : '场景对象'}>
        <div className="object-panel-header"><h2>{activeView === 'program' ? '模块文件' : '场景对象'}</h2><button type="button" className="icon-button" aria-label={leftCollapsed ? '展开左侧面板' : '折叠左侧面板'} onClick={() => workspace.setLeftCollapsed(value => !value)}><Icon name={leftCollapsed ? 'chevron-right' : 'chevron-left'} size={16}/></button></div>
        <div className="object-panel-content" aria-hidden={leftCollapsed} inert={leftCollapsed}>
          {activeView === 'program' ? <ModuleList files={documents.files} activeFileId={documents.project.activeFileId} disabled={!documents.ready || sourceBusy || executing} onSelectFile={id => documents.selectFile(id)}/> : <SceneObjectTree robotPresent={robotPresent} robotName={activeName} joints={displayJoints} robotParent={graph.robotBase.parent} frames={graph.frames} tools={graph.tools}
            paths={program.paths.map(path => ({ ...path, parent: pathAttachment(graph, path.id).parent }))} points={program.points} zones={zones} surfaces={surfaces} onImportSurface={() => { setSurfacePickingPathId(null); setSurfaceImportOpen(true); }}
            invalidZoneIds={zoneConfiguration.errors.map(error => error.id)} selection={selection} executingPointId={execution.phase === 'running' ? execution.pointId : null}
            disabled={executing || sourceBusy} canAddZone={workspace.canAddZone} onSelect={chooseObject} onContextMenu={setObjectMenu} onReparent={reparentObject}
            onRestoreRobot={restoreProjectRobot} onAddFrame={addReferenceFrame} onAddTool={addTool} onAddPath={createPath}
            onAddZone={() => { clearPreview(); stopJog(); cancelInspections(); workspace.addZone(); }}/>}
        </div>
      </aside>

      <section className="scene-panel" aria-label="机器人视口" aria-hidden={activeView !== 'scene'} inert={activeView !== 'scene'}>
        <div className="scene-heading"><div><span className="eyebrow">ROBOT WORKSPACE</span><h1>机器人工作区</h1><p>{genericModel ? 'URDF 模型 · 关节姿态 · 工具坐标' : '对象调姿 · 姿态记录 · 端点检查'}</p></div><span className="robot-tag"><span className={activeBrand === 'ABB' ? 'abb-mark' : 'robot-brand-mark'}>{activeBrand}</span>{activeName}<span className="tag-divider"/>{limits.length} 轴</span></div>
        <div className="viewport">
          <RobotScene ref={sceneHandle} initialCamera={initialScene?.camera} robotPresent={robotPresent} model={model} joints={visibleJoints} urdfModel={genericModel ?? undefined} urdfJoints={visibleJoints}
            targetPose={selectedSurfacePoint && selectedSurfacePath ? pathPointToWorld(graph, selectedSurfacePath.id, selectedSurfacePoint.pose) : selectedPointWorld} path={surfacePreview?.motion.path ?? planningPreview?.path ?? EMPTY_PATH}
            showFrames={showFrames} view={view} viewRevision={viewRevision}
            sceneSurfaces={surfaces} selectedSurfaceId={selectedSurface?.id} surfacePickingEnabled={Boolean(surfacePickingPathId) && !executing && !homeBusy && activeView === 'scene'} surfacePickSurfaceId={pickingPath?.surface?.surfaceId} surfaceRegion={sceneSurfaceRegion} onSurfacePick={addSurfacePick} onSelectSurface={id => chooseObject({ kind: 'surface', id })}
            jogEnabled={robotPresent && !homeBusy && selection.kind === 'robot' && activeView === 'scene' && jogEnabled && !executing && !sourceBusy && !poseError}
            jogMode={jogMode} jogSpace={jogSpace} jogPending={jogStatus.pending} jogResetRevision={jogResetRevision}
            onJogStart={beginJog} onJogMove={pose => jogController.current?.update(pose)} onJogEnd={() => setJogDragging(false)}
            dangerZones={sceneZones} selectedDangerZoneId={selectedZoneId ?? undefined}
            zoneManipulationEnabled={!homeBusy && selection.kind === 'zone' && activeView === 'scene' && !executing && !sourceBusy}
            onDangerZoneMove={(id, centerMm) => { collisionRun.current?.abort(); workspace.editZone(id, { center: centerMm.map(String) }); }}
            onDangerZoneDrag={setZoneDragging} scenePaths={displayedScenePaths} selectedPointId={selectedSurfacePoint?.id ?? selectedPoint?.id ?? null}
            basePose={basePose} toolPose={toolPose} sceneFrames={sceneFrames} sceneTools={sceneTools} selectedJointIndex={selection.kind === 'joint' ? selection.index : null}/>
          <div className="viewport-toolbar"><div className="view-options" aria-label="观察视角">{([['iso', '透视'], ['front', '正视'], ['top', '俯视']] as const).map(([id, label]) => <button key={id} className={view === id ? 'selected' : ''} onClick={() => changeView(id)}>{label}</button>)}</div><button className="icon-button" title="重置视角" aria-label="重置视角" onClick={() => changeView('iso')}><Icon name="reset" size={15}/></button></div>
          {robotPresent && selection.kind === 'robot' && <div className="object-jog-toolbar"><div className="view-options"><button type="button" aria-pressed={jogEnabled} className={jogEnabled ? 'selected' : ''} disabled={executing || sourceBusy || Boolean(poseError)} onClick={() => { stopJog(); clearPreview(); setJogEnabled(value => !value); }}>拖动法兰</button>{jogEnabled && <>{([['translate', '平移'], ['rotate', '旋转']] as const).map(([id, label]) => <button key={id} type="button" disabled={executing || sourceBusy} aria-pressed={jogMode === id} className={jogMode === id ? 'selected' : ''} onClick={() => { stopJog(); setJogMode(id); }}>{label}</button>)}{([['world', '世界坐标'], ['local', '工具坐标']] as const).map(([id, label]) => <button key={id} type="button" disabled={executing || sourceBusy} aria-pressed={jogSpace === id} className={jogSpace === id ? 'selected' : ''} onClick={() => { stopJog(); setJogSpace(id); }}>{label}</button>)}</>}</div></div>}
          {robotPresent && jogEnabled && selection.kind === 'robot' && <div className="object-jog-status" role="status" aria-label="拖动状态">{jogStatus.error ? '已保持上一个有效姿态：' + jogStatus.error : configuration?.singular ? '当前姿态接近奇异位置，请调整关节或使用示例姿态。' : jogStatus.pending ? '正在检查拖动路径…' : '拖动控制柄调整当前 TCP；无效目标不会移动机器人。'}</div>}
          {selection.kind === 'zone' && <div className="object-jog-status" role="status">拖动世界坐标箭头移动危险区，或在右侧填写位置与尺寸。</div>}
          <div className="viewport-caption"><Icon name="circle" size={9} className="origin-icon"/>基座世界坐标 <code>[{basePose.position.map(value => numberText(value, 2)).join(', ')}]</code></div>
          <div className="scene-legend"><span><Icon name="dot" size={11} className="legend-current"/>当前 TCP</span>{selectedPoint && <span><Icon name="dot" size={11} className="legend-target"/>已记录目标</span>}</div>
        </div>
        {surfacePickingPathId && <div className="endpoint-preview-notice surface-picking-notice" role="status" aria-label="区域点选状态"><div><strong>点选区域 · {pickingPath?.name}</strong><span>沿同一模型表面依次点击边界，拖动鼠标可旋转视角；按 Esc 结束。</span></div><button type="button" className="copy-button" onClick={() => setSurfacePickingPathId(null)}>结束点选</button></div>}
        {previewSolution && <div className="endpoint-preview-notice" role="status" aria-label="端点预览"><div><strong>正在预览端点姿态</strong><span>预览暂不改写数据；点击“用当前姿态更新点”可保存此候选解。退出预览恢复原机器人姿态。</span><code>{previewSolution.configuration ? 'robconf [' + previewSolution.configuration.join(', ') + ']' : '关节 [' + previewSolution.joints.map(value => numberText(value, 1)).join(', ') + ']°'}</code></div><button type="button" className="copy-button" onClick={() => setEndpointPreview(null)}>退出预览</button></div>}
        <div className="scene-display-options"><label className="frame-toggle"><input type="checkbox" checked={showFrames} onChange={event => setShowFrames(event.target.checked)}/>显示关节坐标系</label></div>
        {execution.message && <div className={'execution-status execution-' + execution.phase} role="status" aria-label="程序运行状态" data-run-path-id={execution.pathId ?? ''} data-run-phase={execution.phase} data-run-point-id={execution.pointId ?? ''} data-run-line={execution.line} data-run-cycle={execution.cycle}><span>{execution.message}</span>{executing && <button type="button" className="copy-button" onClick={() => stopExecution()}><Icon name="stop" size={13}/>停止运行</button>}</div>}
      </section>

      <aside className="control-panel" aria-label="对象属性" aria-hidden={activeView !== 'scene'} inert={activeView !== 'scene'}><fieldset disabled={executing || sourceBusy} className="inspector-fieldset">
        {selectedSurface && <SurfaceInspector surface={selectedSurface} disabled={executing || sourceBusy} onNameChange={name => editSurface(selectedSurface.id, { name })} onVisibleChange={visible => editSurface(selectedSurface.id, { visible })} onPoseChange={pose => editSurface(selectedSurface.id, { pose })} onFocus={() => sceneHandle.current?.focusSurface(selectedSurface.id)} onRemove={() => setRemoveObject({ kind: 'surface', id: selectedSurface.id, modelId: activeModelId })}/>}
        {selection.kind === 'robot' && !robotPresent && <div className="robot-absent"><h2>机器人本体已删除</h2><p>参考坐标系、工具、路径与模块文件仍保留。</p><button type="button" className="copy-button" onClick={restoreProjectRobot}>恢复本项目机器人</button></div>}
        {selection.kind === 'robot' && robotPresent && <><RobotInspector name={activeName} model={genericModel ? null : model} urdf={genericModel?.description} pose={currentPose} joints={displayJoints}
          drafts={genericModel ? catalogDraft : jointDraft} limits={limits} euler={currentPose ? quaternionToEulerZYX(currentPose.orientation) : null}
          configuration={configuration?.configuration ?? null} poseError={poseError} captureDisabledReason={captureDisabledReason} speed={speed}
          onSpeed={setSpeed} onJoint={editJoint} onInitial={() => setRobotPose('ready')} onZero={() => setRobotPose('zero')}/>
          <AttachmentSelector object={{ kind: 'robot' }} parent={graph.robotBase.parent} frames={graph.frames} onReparent={reparentObject}/>
          <p className="field-hint">当前工具：{graph.tools.find(tool => tool.id === graph.activeToolId)?.name ?? '法兰原点'}。当前TCP显示世界坐标。</p></>}
        {robotPresent && selection.kind === 'base' && <div className="base-inspector" data-local-pose={JSON.stringify(graph.robotBase.localPose)} data-world-pose={JSON.stringify(basePose)}>
          <div className="panel-title"><Icon name="base"/><h2>机器人基座</h2></div>
          <AttachmentSelector object={{ kind: 'robot' }} parent={graph.robotBase.parent} frames={graph.frames} onReparent={reparentObject}/>
          <PoseEditor label="机器人基座" value={graph.robotBase.localPose} onChange={pose => graphEdit(() => frameState.editBase(pose))}/>
          <PoseReadout label="基座世界位姿" pose={basePose}/></div>}
        {robotPresent && selection.kind === 'flange' && currentFlangePose && <div className="flange-inspector"><div className="panel-title"><Icon name="flange"/><h2>末端法兰</h2></div>
          <PoseReadout label="法兰世界位姿" pose={composePose(basePose, currentFlangePose)}/><PoseReadout label="法兰基座位姿" pose={currentFlangePose}/>
          <p className="field-hint">将工具拖到左侧法兰节点可以挂载。工具中心点由安装位姿和TCP偏置共同确定。</p></div>}
        {robotPresent && selection.kind === 'joint' && <div className="joint-inspector" data-selected-joint-index={selection.index}>
          <div className="panel-title"><Icon name="joint"/><h2>关节轴 {selection.index + 1}</h2></div>
          <label className="field-label">关节角 <span>deg</span><input type="number" step="any" aria-label={'机器人关节 J' + (selection.index + 1)} value={(genericModel ? catalogDraft : jointDraft)[selection.index] ?? ''} onChange={event => editJoint(selection.index, event.target.value)}/></label>
          <p className="field-hint">范围 {limits[selection.index]?.minDeg}° 至 {limits[selection.index]?.maxDeg}° · 额定最大速度 {limits[selection.index]?.maxSpeedDeg}°/s</p>{poseError && <p role="alert">{poseError}</p>}</div>}
        {selectedFrame && <SceneFrameInspector frame={selectedFrame} frames={graph.frames} worldPose={referenceWorldPose(graph, selectedFrame.id)}
          onNameChange={name => editSceneGraph(() => frameState.editFrame(selectedFrame.id, { name }))} onPoseChange={localPose => graphEdit(() => frameState.editFrame(selectedFrame.id, { localPose }))}
          onReparent={reparentObject} onRemove={() => editSceneGraph(() => { frameState.removeFrame(selectedFrame.id); workspace.selectObject({ kind: 'robot' }); })}/>}
        {selectedTool && currentFlangePose && <ToolInspector tool={selectedTool} frames={graph.frames} worldPose={toolWorldPose(graph, selectedTool.id, currentFlangePose)} tcpWorldPose={toolTcpWorldPose(graph, selectedTool.id, currentFlangePose)}
          onNameChange={name => editSceneGraph(() => frameState.editTool(selectedTool.id, { name }))} onPoseChange={localPose => graphEdit(() => frameState.editTool(selectedTool.id, { localPose }))}
          onTcpChange={tcpPose => graphEdit(() => frameState.editTool(selectedTool.id, { tcpPose }))} onRadiusChange={radiusMm => graphEdit(() => frameState.editTool(selectedTool.id, { radiusMm }))}
          onReparent={reparentObject} onAlignFlange={() => editSceneGraph(() => frameState.editTool(selectedTool.id, { localPose: identityPose() }))}
          onRemove={() => editSceneGraph(() => { frameState.removeTool(selectedTool.id); workspace.selectObject({ kind: 'robot' }); })}/>}
        {selectedZone && <div className="zone-inspector" data-selected-zone-id={selectedZone.id}>
          <div className="panel-title"><div className="section-number"><Icon name="zone" size={14}/></div><h2>危险区属性</h2></div>
          <label className="field-label danger-zone-name">名称<input aria-label="危险区名称" maxLength={80} value={selectedZone.name} onChange={event => editZone({ name: event.target.value })}/></label>
          <div className="danger-zone-actions"><label className="danger-zone-toggle"><input type="checkbox" aria-label="显示干涉区域" checked={selectedZone.enabled} onChange={event => editZone({ enabled: event.target.checked })}/>显示并检测</label><button type="button" className="text-button" onClick={() => { collisionRun.current?.abort(); workspace.removeSelectedZone(); }}>删除危险区</button></div>
          {(['center', 'size'] as const).map(field => <section key={field}><label className="field-label danger-zone-size-label">{field === 'center' ? '世界坐标中心' : '区域尺寸'} <span>mm</span></label><div className="position-grid">{axes.map((axis, index) => <label key={axis}><span>{axis}</span><input type="number" aria-label={'危险区' + (field === 'center' ? '中心 ' : '尺寸 ') + axis} step="any" min={field === 'size' ? 0.001 : -1e6} max={1e6} value={selectedZone[field][index]} onChange={event => editZone({ [field]: selectedZone[field].map((value, i) => i === index ? event.target.value : value) })}/></label>)}</div></section>)}
          <button type="button" className="text-button danger-zone-reset" onClick={() => { collisionRun.current?.abort(); workspace.resetSelectedZone(); }}>恢复默认区域</button>
          {zoneConfiguration.errors.map(error => <p className="danger-zone-error" role="alert" key={error.id}>{error.message} 当前不能完成端点检测。</p>)}
          <p className="field-hint">路径规划会检查机器人和工具包络与禁入区的整段间距。目标点另提供机器人本体端点网格检测。</p>
        </div>}
        {selectedPath && <div className="path-inspector" data-local-pose={JSON.stringify(pathAttachment(graph, selectedPath.id).localPose)} data-world-pose={JSON.stringify(pathWorldPose(graph, selectedPath.id))}><div className="panel-title"><div className="section-number"><Icon name="path" size={14}/></div><h2>{selectedPath.name}</h2></div>
          <p className="field-hint">{selectedPath.surface ? '表面区域与加工目标保存在此 Path。完成机器人求解后可转换为独立模块。' : '调整机器人姿态后向 Path 添加点。转换为程序会新建独立模块；之后修改 Path 不会覆盖模块。'}</p>
          <AttachmentSelector object={{ kind: 'path', id: selectedPath.id }} parent={pathAttachment(graph, selectedPath.id).parent} frames={graph.frames} onReparent={reparentObject}/>
          {!selectedPath.surface && <button type="button" className="copy-button" disabled={!robotPresent} onClick={() => openPathPlanning(selectedPath.id)}><Icon name="route" size={14}/>路径规划</button>}
          <details className="path-frame-details"><summary>路径坐标与姿态<Icon name="chevron-right" size={13}/></summary><PoseEditor label="路径坐标" value={pathAttachment(graph, selectedPath.id).localPose} onChange={pose => graphEdit(() => frameState.editPath(selectedPath.id, pose))}/><PoseReadout label="路径世界位姿" pose={pathWorldPose(graph, selectedPath.id)}/></details>
          {selectedPath.surface?.kind !== 'region' && <div className="path-actions">{!selectedPath.surface && <button type="button" className="copy-button" disabled={Boolean(capturePathReason(selectedPath.id))} title={capturePathReason(selectedPath.id) ?? undefined} onClick={() => captureToPath(selectedPath.id)}>添加当前点到 Path</button>}<button type="button" className="copy-button" disabled={moduleGenerationDisabled || !selectedPath.pointIds.length || Boolean(surfaceMotionReason(selectedPath.id))} onClick={() => generatePathModule(selectedPath.id)}>转换为程序</button></div>}
          {!selectedPath.surface && <ol className="path-point-list">{selectedPath.pointIds.map(id => {
            const point = program.points.find(item => item.id === id);
            return point && <li key={id}><button type="button" className="copy-button" onClick={() => chooseObject({ kind: 'target', id })}>{point.pointName}</button><span>{point.motionRole === 'transfer' ? '转移' : point.motionRole === 'process' ? '加工' : profile.linearInstruction} · {point.speedMmPerSecond} mm/s</span></li>;
          })}</ol>}
          {selectedPath.surface?.kind !== 'region' && <div className="path-run-controls"><button type="button" className="copy-button" disabled={Boolean(pathRunDisabledReason(selectedPath.id))} title={pathRunDisabledReason(selectedPath.id) ?? undefined} onClick={() => void runPath(selectedPath.id)}><Icon name="play" size={13}/>运行 Path</button><label><input type="checkbox" checked={loop} onChange={event => setLoop(event.target.checked)}/>循环运行</label></div>}
          <p className="field-hint">{selectedPath.surface ? (selectedPath.surface.kind === 'uv' ? '运行前重新检查每段运动；曲面上的相邻目标由直线连接，连接段可能偏离表面。' : '完成区域点选并标记4个角控制点后，生成新的UV加工路径。') : selectedPath.planning ? '规划路径保留各目标点的位置和姿态；运行前重新验证每段运动与禁入区，转移段不代表贴合工件加工。' : '只运行此路径，开始前逐段检查运动。虚线表示点顺序；使用路径规划可进行整段禁入区检查。'}</p>
          <button type="button" className="text-button" onClick={() => setRemoveObject({ kind: 'path', id: selectedPath.id, modelId: activeModelId })}>删除 Path</button>
        </div>}
        {(selectedSurfacePath || selectedPath && surfacePickingPathId === selectedPath.id) && <SurfaceRegionInspector path={(selectedSurfacePath ?? selectedPath)!} surfaceName={surfaces.find(surface => surface.id === selectedSurfacePath?.surface?.surfaceId)?.name} selectedPointId={selectedSurfacePoint?.id} picking={surfacePickingPathId === (selectedSurfacePath ?? selectedPath)!.id} disabled={executing || sourceBusy} staleReason={selectedSurfacePath ? surfacePathGeometryReason(selectedSurfacePath, graph, surfaces) : null}
          onSelectPoint={id => chooseObject({ kind: 'surface-point', id })} onSetControl={(id, enabled) => setSurfaceControl((selectedSurfacePath ?? selectedPath)!.id, id, enabled)} onRemovePoint={id => setRemoveObject({ kind: 'surface-point', id, modelId: activeModelId })} onStartPicking={() => beginSurfacePicking((selectedSurfacePath ?? selectedPath)!.id)} onReset={() => resetSurfaceRegion((selectedSurfacePath ?? selectedPath)!.id)} onFinishPicking={() => setSurfacePickingPathId(null)} onGenerate={() => openSurfaceUv((selectedSurfacePath ?? selectedPath)!.id)}/>}
        {selectedPoint && <TargetInspector modelKey={activeModelId} point={selectedPoint} worldPose={selectedPointWorld!} referenceName={program.paths.find(path => path.id === selectedPoint.pathId)?.name ?? '路径'} updateDisabledReason={pointUpdateDisabledReason} previewing={Boolean(previewSolution)} onUpdate={updatePointFromCurrent}>
          {robotPresent && <section className="endpoint-details"><div className="field-heading"><h3>端点姿态与干涉检测</h3><button type="button" className="text-button" disabled={executing} onClick={() => { cancelInspections(); setInspectionRevision(value => value + 1); }}>重新检查端点解</button></div>
            <p className="field-hint" role="status" aria-label="端点检查状态">{inspection.key !== pointKey || inspection.pending ? '正在检查端点逆解…' : inspection.error || (solutions.length ? '预览仅显示候选姿态，不修改目标点或原机器人姿态。' : '没有找到有效的端点逆解。')}</p>
            <div className="endpoint-list">{solutions.map((solution, index) => <div key={solution.id} className={'endpoint-solution' + (previewSolution?.id === solution.id ? ' endpoint-previewed' : '')} role="group" aria-label={'端点解 ' + (index + 1)} data-endpoint-joints={JSON.stringify(solution.joints)} data-collision-state={collisionInfo(solution.id).state} data-collision-zone-ids={JSON.stringify(collisionIds(solution.id))} data-collision-uncertain-zone-ids={JSON.stringify(collisionIds(solution.id, true))}>
              <div className="endpoint-solution-heading"><code>{solution.configuration ? '[' + solution.configuration.join(', ') + ']' : '候选 ' + (index + 1)}</code>{(solution.configuration ? solution.configuration.every((value, i) => value === selectedPoint.configuration?.[i]) : solution.joints.every((value, i) => Math.abs(value - selectedPoint.joints[i]) < .01)) && <small>已记录配置</small>}<button type="button" className="copy-button endpoint-preview" aria-label={'预览姿态 ' + (index + 1)} aria-pressed={previewSolution?.id === solution.id} disabled={executing} onClick={() => previewEndpoint(solution)}>预览</button></div>
              <span className="endpoint-joints">{solution.joints.map(value => numberText(value, 1)).join(' / ')}°</span>{collisionBadge(solution.id)}
            </div>)}</div>
            <p className="field-hint collision-legend"><span><Icon name="warning" size={13}/>确认干涉</span><span><Icon name="question" size={13}/>无法可靠判断内部</span>。检测只覆盖端点与危险区，未检查整段路径或自碰撞。</p>
          </section>}
        </TargetInspector>}
      </fieldset>{message && <p className="workspace-message" role="status">{message}</p>}</aside>

      <section className="code-panel" aria-label="程序工作区" aria-hidden={activeView !== 'program'} inert={activeView !== 'program'}>
        <ProgramPanel modelKey={activeModelId} active={activeView === 'program'} codeWorkspace={codeWorkspace}
          codeLoadingError={!documents.ready && documents.error ? documents.error : languageState.key === languageKey ? languageState.error : ''}
          onRetryCodeLoading={() => { if (!documents.ready) void documents.retry(); else setLanguageRetry(value => value + 1); }}/>
      </section>
    </main>
    {surfaceImportOpen && <SurfaceImportDialog onImport={importSceneSurface} onClose={() => setSurfaceImportOpen(false)}/>}
    {surfaceUvPathId && program.paths.some(path => path.id === surfaceUvPathId) && <SurfaceUvDialog<SurfacePathPlan> key={surfaceUvPathId} pathName={program.paths.find(path => path.id === surfaceUvPathId)!.name} contextKey={planningContextKey} controlCount={program.paths.find(path => path.id === surfaceUvPathId)?.surface?.controlPointIds.length ?? 0}
      onGenerate={generateSurfaceUv} summarize={result => ({ pointCount: result.geometry.points.length, rowCount: result.geometry.rowOffsets.length, lengthMm: result.motion.lengthMm, maxRegularSpacingErrorMm: result.geometry.maxRegularSpacingErrorMm, maxOrientationStepDeg: result.geometry.maxOrientationStepDeg, warnings: ['已完成连续逆解和当前禁入区检查；工具方向随表面法向连续变化。', '边界末段可以短于设定间距。曲面点之间使用直线 MoveL 连接，连接段可能离面；尚未检查与工件的接触碰撞。'] })} onPreview={setSurfacePreview} onApply={applySurfaceUv} onClose={() => { setSurfaceUvPathId(null); setSurfacePreview(null); }}/>}
    {planningPathId && program.paths.some(path => path.id === planningPathId) && <PathPlanningDialog key={activeModelId + ':' + planningPathId}
      pathName={program.paths.find(path => path.id === planningPathId)!.name} modelName={activeName}
      initialTargets={program.pointsForPath(planningPathId).map(point => pathPointToWorld(graph, planningPathId, point.pose))}
      contextKey={planningContextKey}
      onPlan={(input, signal) => planGrindingPath(makeGrindingRequest({ ...input, startJoints: [...displayJoints] as JointAngles, allowDetours: true }), signal)}
      onPreview={setPlanningPreview} onApply={applyGrindingPath} onClose={() => { setPlanningPathId(null); setPlanningPreview(null); }}/>}
    {closeState === null && objectMenu && <SceneObjectMenu x={objectMenu.x} y={objectMenu.y} trigger={objectMenu.trigger} label={objectMenu.kind === 'path' ? 'Path 右键菜单' : objectMenu.kind === 'target' ? '目标点右键菜单' : sceneRemovalLabel(objectMenu.kind).replace(/^删除/, '') + '右键菜单'} onClose={() => setObjectMenu(null)} actions={objectMenu.kind === 'path' ? [
      { label: '路径规划', icon: 'route', disabled: !robotPresent || executing || sourceBusy || Boolean(program.paths.find(path => path.id === objectMenu.id)?.surface), onSelect: () => openPathPlanning(objectMenu.id) },
      { label: '点选区域', icon: 'target', disabled: executing || sourceBusy || program.paths.find(path => path.id === objectMenu.id)?.surface?.kind === 'uv', onSelect: () => beginSurfacePicking(objectMenu.id) },
      { label: '生成 UV 加工路径', icon: 'path', disabled: !robotPresent || executing || sourceBusy || program.paths.find(path => path.id === objectMenu.id)?.surface?.kind !== 'region', onSelect: () => openSurfaceUv(objectMenu.id) },
      { label: '运行', icon: 'play', disabled: Boolean(pathRunDisabledReason(objectMenu.id)), title: pathRunDisabledReason(objectMenu.id) ?? undefined, onSelect: () => void runPath(objectMenu.id) },
      { label: '添加当前点到 Path', icon: 'plus', disabled: Boolean(capturePathReason(objectMenu.id)), title: capturePathReason(objectMenu.id) ?? undefined, onSelect: () => captureToPath(objectMenu.id) },
      { label: '转换为程序', icon: 'code', disabled: moduleGenerationDisabled || !program.paths.find(path => path.id === objectMenu.id)?.pointIds.length || Boolean(surfaceMotionReason(objectMenu.id)), onSelect: () => generatePathModule(objectMenu.id) },
      { label: '删除 Path', icon: 'trash', disabled: executing || sourceBusy, onSelect: () => setRemoveObject({ kind: 'path', id: objectMenu.id, modelId: activeModelId }) },
    ] : objectMenu.kind === 'surface-point' ? [
      { label: '移动至此', icon: 'target', disabled: !robotPresent || executing || sourceBusy || jogStatus.pending || jogDragging || !resolvedSurfacePoint(objectMenu.id), title: '定位到已求解的记录姿态；不验证移动路径', onSelect: () => { const point = resolvedSurfacePoint(objectMenu.id); if (point) moveToPoint(point.id); } },
      { label: program.paths.find(path => path.surface?.points.some(point => point.id === objectMenu.id))?.surface?.controlPointIds.includes(objectMenu.id) ? '取消控制点' : '设为控制点', icon: 'frame', disabled: executing || sourceBusy || program.paths.find(path => path.surface?.points.some(point => point.id === objectMenu.id))?.surface?.kind !== 'region', onSelect: () => { const owner = program.paths.find(path => path.surface?.points.some(point => point.id === objectMenu.id)); if (owner) setSurfaceControl(owner.id, objectMenu.id, !owner.surface!.controlPointIds.includes(objectMenu.id)); } },
      { label: '删除表面点', icon: 'trash', disabled: executing || sourceBusy, onSelect: () => setRemoveObject({ kind: 'surface-point', id: objectMenu.id, modelId: activeModelId }) },
    ] : objectMenu.kind === 'target' ? [
      { label: '移动至此', icon: 'target', disabled: !robotPresent || executing || sourceBusy || jogStatus.pending || jogDragging, title: '定位到记录姿态；不验证移动路径', onSelect: () => moveToPoint(objectMenu.id) },
      { label: '删除点', icon: 'trash', disabled: executing || sourceBusy, onSelect: () => setRemoveObject({ kind: 'target', id: objectMenu.id, modelId: activeModelId }) },
    ] : [{ label: sceneRemovalLabel(objectMenu.kind), icon: 'trash', disabled: executing || sourceBusy, onSelect: () => setRemoveObject({ kind: objectMenu.kind, id: objectMenu.id, modelId: activeModelId }) }]}/>}
    {closeState === null && removeObject && <ObjectRemovalDialog kind={removeObject.kind} name={removeObject.kind === 'surface' ? surfaces.find(surface => surface.id === removeObject.id)?.name ?? '工件模型' : removeObject.kind === 'surface-point' ? program.paths.flatMap(path => path.surface?.points ?? []).find(point => point.id === removeObject.id)?.name ?? '表面点' : removeObject.kind === 'robot' ? activeName : removeObject.kind === 'frame' ? graph.frames.find(frame => frame.id === removeObject.id)?.name ?? '参考坐标系' : removeObject.kind === 'tool' ? graph.tools.find(tool => tool.id === removeObject.id)?.name ?? '工具' : removeObject.kind === 'zone' ? zones.find(zone => zone.id === removeObject.id)?.name ?? '危险区' : removeObject.kind === 'path' ? program.paths.find(path => path.id === removeObject.id)?.name ?? 'Path' : program.points.find(point => point.id === removeObject.id)?.pointName ?? '目标点'} count={program.paths.find(path => path.id === removeObject.id)?.pointIds.length ?? 0} onCancel={() => setRemoveObject(null)} onConfirm={confirmRemoveObject}/>}
    {closeState && <div className="source-close-backdrop"><section className="source-close-dialog" role="dialog" aria-modal="true" aria-label="关闭前保存项目和模块草稿" onKeyDown={event => {
      if (event.key === 'Escape' && closeState === 'error') { event.preventDefault(); setCloseState(null); }
      if (event.key === 'Tab') { const buttons = [...event.currentTarget.querySelectorAll<HTMLButtonElement>('button:not(:disabled)')], first = buttons[0], last = buttons.at(-1); if (event.shiftKey && document.activeElement === first) { event.preventDefault(); last?.focus(); } else if (!event.shiftKey && document.activeElement === last) { event.preventDefault(); first?.focus(); } }
    }}><h2>{closeState === 'saving' ? '正在保存项目和模块草稿' : '项目和模块草稿尚未保存'}</h2><p role="status">{closeState === 'saving' ? '正在保存项目场景与各型号的本地模块草稿，完成后关闭窗口。' : closeError}</p>{closeState === 'error' && <div className="source-close-actions"><button autoFocus type="button" onClick={() => setCloseState(null)}>继续编辑</button><button type="button" onClick={() => void finishNativeClose()}>重试保存并关闭</button><button type="button" className="source-discard" onClick={() => void finishNativeClose(true)}>放弃未保存的项目和模块草稿并关闭</button></div>}</section></div>}
    {homeBusy && <div className="source-close-backdrop"><section className="source-close-dialog" role="status"><h2>正在保存项目</h2><p>保存模块与场景，更新三维封面…</p></section></div>}
    {homeError && <div className="source-close-backdrop"><section className="source-close-dialog" role="alertdialog" aria-label="项目保存失败"><h2>项目尚未保存</h2><p>{homeError}</p><button type="button" onClick={() => setHomeError('')}>继续编辑</button><button type="button" onClick={() => void returnHome()}>重试返回主页</button></section></div>}
    <footer className="app-footer"><span><Icon name="info" size={13}/>{genericModel ? '基于源 URDF 的直线运动仿真；品牌模板未完成控制器验证。' : '路径运行前逐段检查运动；打磨路径另检查机器人与工具的禁入区间距。'}</span><span>世界坐标 Z 向上 · 长度 mm · 角度 deg</span></footer>
  </div>;
}
