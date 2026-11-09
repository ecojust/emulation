import { useEffect, useMemo, useRef, useState, type ReactNode } from 'react';
import RobotScene from './scene/RobotScene';
import RobotCatalogDialog from './catalog/RobotCatalogDialog';
import RobotInspector from './workspace/RobotInspector';
import TargetInspector from './workspace/TargetInspector';
import ProgramPanel, { type ProgramCodeWorkspace } from './workspace/ProgramPanel';
import { useProgram } from './workspace/useProgram';
import { useWorkspace, type DangerZoneDraft, type WorkspaceSelection, type WorkspaceView } from './workspace/useWorkspace';
import type { ProgramPoint, ProgramSnapshot } from './workspace/program';
import { getProgramProfile, renderProgramTemplate } from './workspace/program-templates';
import { useDocuments } from './programming/useDocuments';
import { useSourceAnalysis } from './programming/useSourceAnalysis';
import { loadLanguageDefinition } from './programming/registry';
import { PROGRAM_DIALECTS, PROGRAMMING_MODEL_IDS, getModelProgrammingProfile, getProgramDialect } from './programming/profiles';
import type { RobotLanguageDefinition } from './programming/types';
import type { SourceInsertion } from './programming/RobotCodeEditor';
import { planProgramRun, sampleProgramRun, sampleJointPositioning, ProgramRunError } from './workspace/program-runner';
import { buildGenericChain } from './robot/generic-motion-chain';
import { solveGenericMotion } from './robot/generic-motion';
import type { GenericPlanRequest, GenericPlanResult } from './robot/generic-motion-types';
import type { LoadedCatalogModel } from './catalog/client';
import { forwardUrdfKinematics } from './catalog/kinematics';
import { copyRapid, nativeDesktop, saveProgram, solveMotion } from './robot/client';
import { inspectEndpoint } from './robot/endpoint-inspection';
import { isNativeUiSmoke, runNativeUiSmoke } from './robot/ui-smoke';
import { forwardKinematics } from './robot/kinematics';
import { getConfiguration } from './robot/configuration';
import { checkCollisions, type CollisionResult } from './robot/collision';
import { createJogController, type JogController, type JogStatus } from './robot/jog-controller';
import { getRobotModel } from './robot/model';
import { formatMoveL, normalizeOrientation } from './robot/rapid';
import type { IKSolution, JointAngles, MotionSample, PlanRequest, PlanSuccess, Pose, Quaternion, RobotId, Vec3 } from './robot/types';
import './styles.css';
import './App.css';

type View = 'iso' | 'front' | 'top';
type CollisionStatus = 'pending' | 'hit' | 'uncertain' | 'clear' | 'error' | 'off';
interface CollisionView { key: string; status: 'pending' | 'ready' | 'error'; items: CollisionResult[]; error?: string }
type Segment = { pointId: string; pointRevision: number; modelId: string } & (
  { kind: 'abb'; request: PlanRequest; result: PlanSuccess } |
  { kind: 'generic'; request: GenericPlanRequest; result: Extract<GenericPlanResult, { status: 'success' }> });
interface ExecutionStatus { phase: 'idle' | 'planning' | 'running' | 'positioning' | 'error' | 'complete'; message: string; pointId: string | null; line: number; cycle: number }
const axes = ['X', 'Y', 'Z'] as const;
const EMPTY_PATH: MotionSample[] = [];
const PROGRAM_MODE_KEY = 'emulation:program-modes:v1';
function readProgramModes(): Record<string, 'teach' | 'code'> {
  try {
    const stored: unknown = JSON.parse(localStorage.getItem(PROGRAM_MODE_KEY) ?? '{}');
    if (!stored || typeof stored !== 'object' || Array.isArray(stored)) return {};
    return Object.fromEntries(Object.entries(stored).filter(([key, value]) => PROGRAMMING_MODEL_IDS.includes(key) && (value === 'teach' || value === 'code')));
  } catch { return {}; }
}
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
function Icon({ name, size = 18 }: { name: 'cube' | 'arrow' | 'play' | 'pause' | 'copy' | 'reset' | 'check' | 'target' | 'code' | 'info'; size?: number }) {
  const paths: Record<string, ReactNode> = {
    cube: <><path d="m12 3 8 4.5v9L12 21l-8-4.5v-9Z"/><path d="m4 7.5 8 4.5 8-4.5M12 12v9M8 5.3l8 4.5"/></>,
    arrow: <><path d="M5 12h14m-6-6 6 6-6 6"/></>, play: <path d="m8 5 11 7-11 7Z"/>, pause: <><path d="M8 5v14M16 5v14"/></>,
    copy: <><rect x="8" y="8" width="12" height="13" rx="2"/><path d="M16 8V5a2 2 0 0 0-2-2H5a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h3"/></>,
    reset: <><path d="M3 10a9 9 0 1 1 2 8M3 4v6h6"/></>, check: <path d="m5 12 4 4L19 6"/>,
    target: <><circle cx="12" cy="12" r="7"/><circle cx="12" cy="12" r="2"/><path d="M12 2v3m0 14v3M2 12h3m14 0h3"/></>,
    code: <><path d="m8 6-6 6 6 6m8-12 6 6-6 6M14 3l-4 18"/></>, info: <><circle cx="12" cy="12" r="9"/><path d="M12 11v6m0-10v.01"/></>,
  };
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{paths[name]}</svg>;
}
export default function App() {
  const [modelId, setModelId] = useState<RobotId>('irb1100-4-0.58');
  const model = getRobotModel(modelId);
  const [catalogOpen, setCatalogOpen] = useState(false);
  const [catalogModel, setCatalogModel] = useState<Extract<LoadedCatalogModel, { kind: 'urdf' }> | null>(null);
  const genericModel = catalogModel?.model ?? null;
  const activeName = catalogModel?.entry.name ?? model.name;
  const activeBrand = catalogModel?.entry.brand ?? 'ABB';
  const activeModelId = catalogModel?.entry.id ?? modelId;
  useEffect(() => () => catalogModel?.dispose(), [catalogModel]);
  const [nativeJoints, setNativeJoints] = useState<JointAngles>([...model.readyJoints]);
  const [jointDraft, setJointDraft] = useState<string[]>(strings(model.readyJoints));
  const [catalogJoints, setCatalogJoints] = useState<number[]>([]);
  const [catalogDraft, setCatalogDraft] = useState<string[]>([]);
  const [speed, setSpeed] = useState(100);
  const program = useProgram(activeModelId);
  const documents = useDocuments(activeModelId);
  const documentsRef = useRef(documents); documentsRef.current = documents;
  const activeModelRef = useRef(activeModelId); activeModelRef.current = activeModelId;
  const [programModes, setProgramModes] = useState(readProgramModes);
  const programMode = programModes[activeModelId] ?? 'teach';
  const [languageState, setLanguageState] = useState<{ key: string; loadedKey: string; definition: RobotLanguageDefinition | null; error: string }>({ key: '', loadedKey: '', definition: null, error: '' });
  const [languageRetry, setLanguageRetry] = useState(0);
  const languageKey = activeModelId + ':' + documents.project.dialectId;
  const sourceLanguage = languageState.loadedKey.startsWith(activeModelId + ':') ? languageState.definition : null;
  const sourceLanguagePending = languageState.loadedKey !== languageKey;
  const [sourceBusy, setSourceBusy] = useState(false), [sourceMessage, setSourceMessage] = useState('');
  const sourceOperation = useRef<{ modelId: string } | null>(null);
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
  const workspace = useWorkspace({ modelKey: activeModelId, pathIds: program.paths.map(path => path.id), targetIds: program.points.map(point => point.id) });
  const { activeView, leftCollapsed, selection, zones, selectedZone, selectedZoneId, zoneConfiguration } = workspace;
  const sourceAnalysis = useSourceAnalysis(documents.project, programMode === 'code' && activeView === 'program' && documents.ready && !analysisPaused);
  const selectedPoint = selection.kind === 'target' ? program.points.find(point => point.id === selection.id) ?? null : null;
  const selectedPath = selection.kind === 'path' ? program.paths.find(path => path.id === selection.id) ?? null : null;
  const [selectedRows, setSelectedRows] = useState<string[]>([]);
  const [message, setMessage] = useState('');
  const [view, setView] = useState<View>('iso');
  const [viewRevision, setViewRevision] = useState(0);
  const [showFrames, setShowFrames] = useState(false);
  const [jogEnabled, setJogEnabled] = useState(false);
  const [jogMode, setJogMode] = useState<'translate' | 'rotate'>('translate');
  const [jogSpace, setJogSpace] = useState<'world' | 'local'>('world');
  const [jogStatus, setJogStatus] = useState<JogStatus>({ pending: false, error: null });
  const [jogDragging, setJogDragging] = useState(false);
  const [jogResetRevision, setJogResetRevision] = useState(0);
  const jogController = useRef<JogController | null>(null);
  const [zoneDragging, setZoneDragging] = useState(false);
  const [segment, setSegment] = useState<Segment | null>(null);
  const [segmentError, setSegmentError] = useState('');
  const [busy, setBusy] = useState(false);
  const [playing, setPlaying] = useState(false);
  const [progress, setProgress] = useState(0);
  const [moduleName, setModuleName] = useState('EmulationMotion');
  const [exporting, setExporting] = useState(false);
  const [exportMessage, setExportMessage] = useState('');
  const [copyState, setCopyState] = useState('');
  const motionRun = useRef<AbortController | null>(null);
  const exportRun = useRef<AbortController | null>(null);
  const inspectionRun = useRef<AbortController | null>(null);
  const collisionRun = useRef<AbortController | null>(null);
  const [inspectionRevision, setInspectionRevision] = useState(0);
  const [inspection, setInspection] = useState<{ key: string; pending: boolean; solutions: IKSolution[]; error: string }>({ key: '', pending: false, solutions: [], error: '' });
  const [endpointPreview, setEndpointPreview] = useState<{ modelId: string; pointId: string; revision: number; solution: IKSolution } | null>(null);
  const [collisionView, setCollisionView] = useState<CollisionView>({ key: '', status: 'pending', items: [] });
  const [loop, setLoop] = useState(false);
  const [execution, setExecution] = useState<ExecutionStatus>({ phase: 'idle', message: '', pointId: null, line: 0, cycle: 0 });
  const executionRun = useRef<{ controller: AbortController; frame: number } | null>(null);
  const executing = execution.phase === 'planning' || execution.phase === 'running' || execution.phase === 'positioning';
  const [pointMenu, setPointMenu] = useState<{ id: string; x: number; y: number } | null>(null);
  const activeSegment = segment && segment.modelId === activeModelId && selectedPoint?.id === segment.pointId && selectedPoint.revision === segment.pointRevision ? segment : null;
  const displayJoints = useMemo<JointAngles>(() => {
    const samples = activeSegment?.result.path;
    if (!samples?.length) return genericModel ? catalogJoints as JointAngles : nativeJoints;
    if (progress <= 0) return samples[0].joints;
    if (progress >= 1) return samples[samples.length - 1].joints;
    const index = samples.findIndex(sample => sample.t >= progress);
    const before = samples[Math.max(0, index - 1)], after = samples[index < 0 ? samples.length - 1 : index];
    const t = after.t === before.t ? 0 : (progress - before.t) / (after.t - before.t);
    return before.joints.map((value, i) => value + (after.joints[i] - value) * t) as JointAngles;
  }, [nativeJoints, catalogJoints, genericModel, activeSegment, progress]);
  // A candidate is a rendering override, never the source for teaching or motion.
  const previewSolution = endpointPreview?.modelId === activeModelId && endpointPreview.pointId === selectedPoint?.id && endpointPreview.revision === selectedPoint.revision ? endpointPreview.solution : null;
  const visibleJoints = previewSolution?.joints ?? displayJoints;
  const currentFk = useMemo(() => genericModel ? null : forwardKinematics(model, displayJoints), [genericModel, model, displayJoints]);
  const currentPose = useMemo(() => genericModel ? forwardUrdfKinematics(genericModel.description, displayJoints).toolPose : currentFk!.pose, [genericModel, displayJoints, currentFk]);
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
  const poseCaptureDisabledReason = executing || busy || exporting || jogStatus.pending || jogDragging ? '请等待当前操作完成后再记录姿态。' : poseError || (!currentPose || displayJoints.length !== 6 ? '当前模型缺少有效的六轴工具姿态。' : genericModel ? genericChain?.error || null : configuration?.singular || !configuration?.configuration ? (configuration?.reason || '当前姿态没有有效 robconf') + '；请调整关节或使用示例姿态。' : null);
  const captureDisabledReason = previewSolution ? '请先退出端点预览，再记录姿态或检查运动。' : poseCaptureDisabledReason;
  const runDisabledReason = captureDisabledReason || (!program.points.length ? '请先在程序中记录目标点。' : null);
  const sceneZones = useMemo(() => zoneConfiguration.validZones.map(zone => ({ ...zone, name: zones.find(draft => draft.id === zone.id)?.name.trim() || '未命名危险区' })), [zoneConfiguration.validZones, zones]);
  const scenePaths = useMemo(() => program.paths.map(path => ({ id: path.id, name: path.name, selected: selection.kind === 'path' && selection.id === path.id, points: path.pointIds.flatMap(id => {
    const point = program.points.find(item => item.id === id);
    return point ? [{ id: point.id, name: point.pointName, pose: point.pose }] : [];
  }) })), [program.paths, program.points, selection]);
  const zoneNames = useMemo(() => new Map(zones.map(zone => [zone.id, zone.name.trim() || '未命名危险区'])), [zones]);
  const pointKey = selectedPoint ? activeModelId + ':' + selectedPoint.id + ':' + selectedPoint.revision : '';
  const solutions = inspection.key === pointKey && !inspection.pending ? inspection.solutions : [];
  const collisionRequest = useMemo(() => !genericModel && !zoneConfiguration.errors.length && solutions.length ? { modelId, zones: zoneConfiguration.validZones, solutions: solutions.map(solution => ({ id: solution.id, joints: solution.joints })) } : null, [genericModel, modelId, zoneConfiguration, solutions]);
  const collisionKey = collisionRequest ? JSON.stringify(collisionRequest) : '';
  const collisionById = useMemo(() => new Map(collisionView.items.map(item => [item.id, item])), [collisionView.items]);
  const code = activeSegment?.kind === 'abb' ? formatMoveL(activeSegment.result.selected.pose, activeSegment.result.selected.configuration, activeSegment.request.speedMmPerSecond) : '';
  const sourceReadOnly = sourceBusy || executing || closeState !== null || !documents.ready || sourceLanguagePending;
  const draftStatusText = documents.draftStatus === 'saved' ? '草稿已保存' : documents.draftStatus === 'saving' ? '正在保存草稿…' : documents.draftStatus === 'pending' ? '草稿等待保存…' : documents.draftStatus === 'loading' ? '正在恢复草稿…' : '草稿保存失败';
  const sourceStatus = [...new Set([languageState.key === languageKey && languageState.error ? '语言版本加载失败：' + languageState.error : sourceLanguagePending ? '正在加载本地语言版本…' : '', sourceMessage, documents.error, draftStatusText, documents.dirty ? '源码已修改' : ''].filter(Boolean))].join(' · ');

  function changeProgramMode(next: 'teach' | 'code') {
    if (sourceOperation.current || closeAttempt.current.busy) return;
    if (next === 'code') { stopExecution(); clearSegment(); stopJog(); setJogEnabled(false); cancelInspections(); }
    setProgramModes(current => ({ ...current, [activeModelId]: next }));
  }
  function sourceWritable() { return documents.ready && !sourceLanguagePending && !sourceOperation.current && !executionRun.current && !closeAttempt.current.busy; }
  function beginSourceOperation(message: string) {
    if (!sourceWritable()) return null;
    const operation = { modelId: activeModelId }; sourceOperation.current = operation;
    setSourceBusy(true); setSourceMessage(message); return operation;
  }
  function currentSourceOperation(operation: { modelId: string }) { return alive.current && sourceOperation.current === operation && activeModelRef.current === operation.modelId; }
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
  function generateSourceFromTeach(): boolean {
    if (!sourceWritable() || !documents.activeFile) return false;
    try {
      const file = documents.activeFile, separator = /\r\n|\n|\r/.exec(file.text)?.[0] ?? '\n';
      const text = (file.text.startsWith('\ufeff') ? '\ufeff' : '') + renderProgramTemplate(activeModelId, program.points).code.replace(/\n/g, separator);
      if (text === file.text) { setSourceMessage('当前文件已与示教模板一致。'); return false; }
      const saved = documents.updateText(file.id, text); if (saved) setSourceMessage('已用示教点生成 ' + file.name + '；可在编辑器中撤销。'); return saved;
    } catch (error) { documents.reportError(error); return false; }
  }
  async function insertSourcePose(input: { fileId: string; source: string; from: number; to: number }): Promise<SourceInsertion> {
    if (!sourceWritable() || captureDisabledReason || !currentPose) throw new Error(captureDisabledReason || '当前不能插入机器人姿态。');
    const modelKey = activeModelId, owner = documents, file = owner.files.find(item => item.id === input.fileId), revision = owner.project.sourceRevision;
    if (!file || file.text !== input.source) throw new Error('源码文件已改变，请重试。');
    const captured: ProgramSnapshot = { pose: { position: [...currentPose.position], orientation: [...currentPose.orientation] }, joints: [...displayJoints], configuration: genericModel ? null : [...configuration!.configuration!] as ProgramSnapshot['configuration'], speedMmPerSecond: speed };
    const module = await import('./programming/source-insertion');
    if (!alive.current || activeModelRef.current !== modelKey || documentsRef.current.project.sourceRevision !== revision || sourceOperation.current || executionRun.current || closeAttempt.current.busy) throw new Error('工程或机器人状态已改变，请重新插入姿态。');
    return module.insertCurrentSourcePose(modelKey, input.source, input.from, input.to, captured);
  }
  function retrySourceAnalysis() { setAnalysisPaused(true); }
  async function finishNativeClose(discard = false) {
    if (!nativeDesktop || closeAttempt.current.busy) return;
    closeAttempt.current.busy = true; setCloseState('saving'); setCloseError('');
    try {
      if (!discard && (!await documentsRef.current.flushAll() || documentsRef.current.hasPendingDrafts())) {
        setCloseError('部分型号的草稿未能保存。正文仍在当前窗口，可重试保存，或继续编辑并导出源文件。'); setCloseState('error'); return;
      }
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
    onGenerateFromTeach: generateSourceFromTeach, onInsertCurrent: insertSourcePose,
    captureDisabledReason: sourceReadOnly ? '请等待当前文件操作完成。' : captureDisabledReason,
    runDisabledReason: '当前语言尚无源码执行器；示教模式可运行已记录轨迹。',
    runStatus: sourceAnalysis.error ? '源码检查失败：' + sourceAnalysis.error : undefined,
    toolbar: <><label className="source-language-select">语言版本<select aria-label="控制器语言版本" value={documents.project.dialectId} disabled={sourceBusy || executing || closeState !== null || !documents.ready} onChange={event => { if (!sourceOperation.current && !executionRun.current && !closeAttempt.current.busy) documents.setDialect(event.target.value); }}>{PROGRAM_DIALECTS.filter(dialect => getModelProgrammingProfile(activeModelId).dialectIds.includes(dialect.id)).map(dialect => <option key={dialect.id} value={dialect.id}>{dialect.name} · {dialect.referenceVersion}</option>)}</select></label>{documents.draftStatus === 'error' && <button type="button" disabled={sourceBusy} onClick={() => void saveSourceDraft()}>重试保存草稿</button>}{languageState.key === languageKey && languageState.error && <button type="button" onClick={() => setLanguageRetry(value => value + 1)}>重试语言加载</button>}</>,
  } : undefined;

  useEffect(() => {
    try { localStorage.setItem(PROGRAM_MODE_KEY, JSON.stringify(programModes)); } catch { /* A UI preference must not block source editing. */ }
  }, [programModes]);
  useEffect(() => {
    if (programMode !== 'code' || activeView !== 'program' || !documents.ready || documents.project.revision !== 0 || documents.files.length) return;
    try { documents.addFile('EMULATE.' + getProgramDialect(documents.project.dialectId).extensions[0], renderProgramTemplate(activeModelId, program.points).code); }
    catch (error) { documents.reportError(error); }
  }, [programMode, activeView, documents.ready, documents.project.revision, documents.files.length, activeModelId, program.points]);
  useEffect(() => {
    if (programMode !== 'code' || activeView !== 'program' || !documents.ready) return;
    if (languageState.loadedKey === languageKey && languageState.definition) return;
    let cancelled = false;
    setLanguageState(current => ({ key: languageKey, loadedKey: current.loadedKey, definition: current.definition, error: '' }));
    void loadLanguageDefinition(documents.project.dialectId).then(definition => {
      if (!cancelled) setLanguageState({ key: languageKey, loadedKey: languageKey, definition, error: '' });
    }, error => { if (!cancelled) setLanguageState(current => ({ ...current, key: languageKey, error: error instanceof Error ? error.message : '本地语言支持加载失败' })); });
    return () => { cancelled = true; };
  }, [programMode, activeView, documents.ready, languageKey, languageRetry]);
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
        if (!documentsRef.current.hasPendingDrafts()) return;
        event.preventDefault();
        if (!closeAttempt.current.busy) await finishNativeClose();
      });
      if (cancelled) detach(); else unlisten = detach;
    }).catch(error => { if (!cancelled) setSourceMessage('未能启用关闭前草稿保存：' + (error instanceof Error ? error.message : String(error))); });
    return () => { cancelled = true; unlisten?.(); };
  }, []);
  useEffect(() => { alive.current = true; return () => { alive.current = false; sourceOperation.current = null; }; }, []);

  function writeBodyPose(joints: JointAngles) {
    if (genericModel) { setCatalogJoints([...joints]); setCatalogDraft(strings(joints)); }
    else { setNativeJoints([...joints]); setJointDraft(strings(joints)); }
  }
  function preserveVisiblePose() {
    writeBodyPose(displayJoints);
  }
  function clearSegment(preserve = true) {
    setEndpointPreview(null);
    if (preserve) preserveVisiblePose();
    motionRun.current?.abort(); motionRun.current = null;
    exportRun.current?.abort(); exportRun.current = null;
    setSegment(null); setSegmentError(''); setBusy(false); setPlaying(false); setProgress(0);
    setExporting(false); setExportMessage(''); setCopyState('');
  }
  function stopJog() {
    jogController.current?.dispose(); jogController.current = null;
    setJogStatus({ pending: false, error: null }); setJogDragging(false);
    setJogResetRevision(value => value + 1);
  }
  function cancelInspections() {
    inspectionRun.current?.abort(); collisionRun.current?.abort();
  }
  function chooseObject(next: WorkspaceSelection) {
    if (selection.kind === next.kind && (next.kind === 'robot' || (selection.kind !== 'robot' && selection.id === next.id))) return;
    clearSegment(); stopJog(); cancelInspections(); setMessage('');
    workspace.selectObject(next);
  }
  function changeWorkspaceView(next: WorkspaceView) {
    if (next === activeView) return;
    setEndpointPreview(null); setPlaying(false); stopJog(); setZoneDragging(false);
    workspace.setActiveView(next);
  }
  function changeView(next: View) { stopJog(); setZoneDragging(false); setView(next); setViewRevision(value => value + 1); }
  function editJoint(index: number, value: string) {
    if (executionRun.current) return;
    stopJog(); clearSegment();
    const drafts = [...(activeSegment ? strings(displayJoints) : genericModel ? catalogDraft : jointDraft)];
    drafts[index] = value;
    if (genericModel) setCatalogDraft(drafts); else setJointDraft(drafts);
    try {
      const values = finite(drafts);
      if (!values.every((angle, i) => angle >= limits[i].minDeg && angle <= limits[i].maxDeg)) return;
      if (genericModel) setCatalogJoints(values); else setNativeJoints(values as JointAngles);
    } catch { /* Numeric drafts retain the previous rendered pose. */ }
  }
  function setRobotPose(kind: 'ready' | 'zero') {
    if (executionRun.current) return;
    stopJog(); clearSegment(false);
    if (genericModel) { setCatalogJoints([...genericModel.description.homeJoints]); setCatalogDraft(strings(genericModel.description.homeJoints)); }
    else { const joints = kind === 'ready' ? model.readyJoints : model.homeJoints; setNativeJoints([...joints]); setJointDraft(strings(joints)); }
  }
  function selectCatalogModel(loaded: LoadedCatalogModel) {
    if (sourceOperation.current || closeState !== null) { if (loaded.kind === 'urdf') loaded.dispose(); return; }
    activeModelRef.current = loaded.entry.id; setSourceMessage('');
    stopExecution(false); setPointMenu(null);
    stopJog(); clearSegment(false); cancelInspections(); setJogEnabled(false); setSelectedRows([]); setMessage('');
    if (loaded.kind === 'bundled') {
      const next = getRobotModel(loaded.motionModelId);
      setCatalogModel(null); setModelId(loaded.motionModelId); setNativeJoints([...next.readyJoints]); setJointDraft(strings(next.readyJoints));
    } else {
      setCatalogModel(loaded); setCatalogJoints([...loaded.model.description.homeJoints]); setCatalogDraft(strings(loaded.model.description.homeJoints));
    }
    workspace.selectObject({ kind: 'robot' }); workspace.setActiveView('scene'); setViewRevision(value => value + 1);
  }
  function snapshot(speedValue = speed): ProgramSnapshot {
    if (captureDisabledReason || !currentPose) throw new Error(captureDisabledReason || '当前姿态没有有效工具坐标。');
    return { pose: currentPose, joints: [...displayJoints], configuration: genericModel ? null : configuration!.configuration!, speedMmPerSecond: speedValue };
  }
  function captureCurrent(afterPointId: string | null) {
    try {
      const point = program.capture(snapshot(), afterPointId);
      setSelectedRows([point.id]); chooseObject({ kind: 'target', id: point.id }); setMessage('已记录 ' + point.pointName + '；尚未验证运动路径。');
    } catch (error) { setMessage(error instanceof Error ? error.message : '姿态记录失败'); }
  }
  function updatePointFromCurrent() {
    if (!selectedPoint) return;
    try { const next = snapshot(selectedPoint.speedMmPerSecond); clearSegment(); cancelInspections(); program.updatePoint(selectedPoint.id, next); setMessage('已用当前姿态更新 ' + selectedPoint.pointName + '。'); }
    catch (error) { setMessage(error instanceof Error ? error.message : '目标点更新失败'); }
  }
  function convertToPath(ids: string[]) {
    if (executionRun.current) return;
    try { const path = program.convertToPath(ids); chooseObject({ kind: 'path', id: path.id }); setMessage('已创建 ' + path.name + '，程序执行顺序保持不变。'); }
    catch (error) { setMessage(error instanceof Error ? error.message : 'Path 创建失败'); }
  }
  function removePoints(ids: string[]) { if (executionRun.current) return; clearSegment(); cancelInspections(); setInspectionRevision(value => value + 1); for (const id of ids) program.removePoint(id); setSelectedRows(rows => rows.filter(id => !ids.includes(id))); }
  function previewEndpoint(solution: IKSolution) {
    if (!selectedPoint || busy || exporting || executionRun.current || jogStatus.pending || jogDragging) return;
    setPlaying(false);
    setEndpointPreview({ modelId: activeModelId, pointId: selectedPoint.id, revision: selectedPoint.revision, solution });
    workspace.setActiveView('scene');
  }
  async function checkSegment() {
    if (!selectedPoint || captureDisabledReason) return;
    const point = selectedPoint;
    const start = [...displayJoints] as JointAngles;
    clearSegment(); stopJog(); cancelInspections();
    const controller = new AbortController(); motionRun.current = controller; setBusy(true);
    try {
      const checked = await planPointSegment(start, point, controller.signal);
      if (motionRun.current !== controller || controller.signal.aborted) return;
      setSegment(checked); setProgress(0);
    } catch (error) { if (motionRun.current === controller && !controller.signal.aborted) setSegmentError(error instanceof Error ? error.message : 'MoveL 检查失败'); }
    finally { if (motionRun.current === controller) { motionRun.current = null; setBusy(false); } }
  }
  async function planPointSegment(start: JointAngles, point: ProgramPoint, signal: AbortSignal): Promise<Segment> {
    const base = { pointId: point.id, pointRevision: point.revision, modelId: activeModelId };
    if (genericModel) {
      if (!genericChain?.chain) throw new Error(genericChain?.error || '当前模型缺少运动学链');
      const request: GenericPlanRequest = { chain: genericChain.chain, startJoints: start, targetJoints: point.joints, speedMmPerSecond: point.speedMmPerSecond };
      const result = await solveGenericMotion(request, signal);
      if (result.status !== 'success') throw new Error(result.message);
      return { ...base, kind: 'generic', request, result };
    }
    const request: PlanRequest = { modelId, startJoints: start, target: { position: point.pose.position, orientation: point.pose.orientation }, speedMmPerSecond: point.speedMmPerSecond };
    const result = await solveMotion(request, signal);
    if (result.status !== 'success') throw new Error(result.message);
    if (!point.configuration || result.selected.configuration.some((value, i) => value !== point.configuration![i])) throw new Error('可达运动的 robconf 与已记录目标点不一致，请调整机器人起点，或调姿后重新记录目标点。');
    return { ...base, kind: 'abb', request, result };
  }
  function stopExecution(report = true) {
    const active = executionRun.current;
    if (active) { active.controller.abort(); cancelAnimationFrame(active.frame); executionRun.current = null; }
    if (report && active) setExecution(value => ({ ...value, phase: 'idle', message: '已停止，机器人保留当前显示姿态。' }));
    else if (!report) setExecution({ phase: 'idle', message: '', pointId: null, line: 0, cycle: 0 });
  }
  async function runProgram() {
    if (programMode === 'code' || runDisabledReason || executionRun.current) return;
    const start = [...displayJoints] as JointAngles;
    clearSegment(); stopJog(); cancelInspections(); setPointMenu(null); setZoneDragging(false);
    workspace.setActiveView('scene');
    const active = { controller: new AbortController(), frame: 0 }; executionRun.current = active;
    setExecution({ phase: 'planning', message: '正在检查完整程序…', pointId: null, line: 0, cycle: 0 });
    try {
      const plan = await planProgramRun({ points: program.points, startJoints: start, loop, signal: active.controller.signal,
        planLeg: async (from, point, signal) => (await planPointSegment(from, point, signal)).result,
        onProgress: status => {
          if (executionRun.current === active) setExecution({ phase: 'planning', message: '正在检查第 ' + status.lineNumber + ' 条指令' + (status.closing ? '（循环闭合）' : '') + '…', pointId: status.pointId, line: status.lineNumber, cycle: 0 });
        } });
      if (executionRun.current !== active || active.controller.signal.aborted) return;
      const started = performance.now();
      const tick = (now: number) => {
        if (executionRun.current !== active || active.controller.signal.aborted) return;
        const frame = sampleProgramRun(plan, (now - started) / 1000);
        writeBodyPose(frame.joints);
        const point = program.points.find(item => item.id === frame.pointId);
        setExecution({ phase: frame.finished ? 'complete' : 'running', message: frame.finished ? '程序仿真完成。' : '正在执行第 ' + frame.lineNumber + ' 条 · ' + (point?.pointName ?? '') + (frame.cycle ? ' · 循环 ' + frame.cycle : ''), pointId: frame.pointId, line: frame.lineNumber, cycle: frame.cycle });
        if (frame.finished) executionRun.current = null;
        else active.frame = requestAnimationFrame(tick);
      };
      active.frame = requestAnimationFrame(tick);
    } catch (error) {
      if (executionRun.current !== active || active.controller.signal.aborted) return;
      executionRun.current = null;
      setExecution({ phase: 'error', message: error instanceof Error ? error.message : '程序检查失败', pointId: error instanceof ProgramRunError ? error.pointId : null, line: error instanceof ProgramRunError ? error.lineNumber : 0, cycle: 0 });
    }
  }
  function moveToPoint(id: string) {
    const point = program.points.find(item => item.id === id);
    setPointMenu(null);
    if (!point || executionRun.current || busy || exporting || jogStatus.pending || jogDragging) return;
    const from = [...displayJoints] as JointAngles;
    clearSegment(); stopJog(); cancelInspections(); setZoneDragging(false);
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
  function beginJog() {
    if (genericModel || selection.kind !== 'robot' || !jogEnabled || poseError || busy || exporting || executionRun.current) return;
    const joints = [...displayJoints] as JointAngles, pose = forwardKinematics(model, joints).pose;
    jogController.current?.dispose(); clearSegment(false);
    setNativeJoints(joints); setJointDraft(strings(joints)); setJogDragging(true); setJogStatus({ pending: false, error: null });
    jogController.current = createJogController({ initial: { modelId, joints, pose, speedMmPerSecond: speed }, solve: solveMotion,
      onAccept: accepted => { setNativeJoints([...accepted.selected.joints]); setJointDraft(strings(accepted.selected.joints)); },
      onStatus: setJogStatus });
  }
  function editZone(patch: Partial<Omit<DangerZoneDraft, 'id'>>) {
    if (!selectedZoneId || executionRun.current) return;
    collisionRun.current?.abort(); workspace.editZone(selectedZoneId, patch);
  }
  async function exportSegment() {
    if (!activeSegment || activeSegment.kind !== 'abb' || !nativeDesktop || exporting || executionRun.current) return;
    const controller = new AbortController(); exportRun.current = controller; setExporting(true); setExportMessage('');
    try {
      const path = await saveProgram({ programName: moduleName, controller: 'abb_rapid', motion: activeSegment.request }, controller.signal);
      if (exportRun.current === controller) setExportMessage(path ? '已保存此 MoveL：' + path : '已取消保存');
    } catch (error) { if (exportRun.current === controller && !controller.signal.aborted) setExportMessage(error instanceof Error ? error.message : '模块导出失败'); }
    finally { if (exportRun.current === controller) { exportRun.current = null; setExporting(false); } }
  }
  async function copySegment() { if (!code) return; try { await copyRapid(code); setCopyState('已复制'); } catch { setCopyState('请选中代码手动复制'); } }

  useEffect(() => { setEndpointPreview(null); }, [activeModelId, selectedPoint?.id, selectedPoint?.revision]);
  useEffect(() => {
    const controller = new AbortController(); inspectionRun.current?.abort(); inspectionRun.current = controller;
    if (!selectedPoint || genericModel || busy || exporting || executing || jogStatus.pending || jogDragging) return () => controller.abort();
    setInspection({ key: pointKey, pending: true, solutions: [], error: '' });
    void inspectEndpoint({ modelId, pose: selectedPoint.pose, seedJoints: selectedPoint.joints }, controller.signal).then(found => {
      if (!controller.signal.aborted && inspectionRun.current === controller) setInspection({ key: pointKey, pending: false, solutions: found, error: '' });
    }, error => {
      if (!controller.signal.aborted && inspectionRun.current === controller) setInspection({ key: pointKey, pending: false, solutions: [], error: error instanceof Error ? error.message : '端点检查失败' });
    });
    return () => { controller.abort(); if (inspectionRun.current === controller) inspectionRun.current = null; };
  }, [pointKey, selectedPoint, genericModel, modelId, inspectionRevision, busy, exporting, executing, jogStatus.pending, jogDragging]);
  useEffect(() => {
    const controller = new AbortController(); collisionRun.current?.abort(); collisionRun.current = controller;
    if (!collisionRequest || !zones.some(zone => zone.enabled) || busy || exporting || executing || zoneDragging) return () => controller.abort();
    setCollisionView({ key: collisionKey, status: 'pending', items: [] });
    const timer = setTimeout(() => {
      void checkCollisions(collisionRequest, controller.signal).then(items => {
        if (!controller.signal.aborted && collisionRun.current === controller) setCollisionView({ key: collisionKey, status: 'ready', items });
      }, error => {
        if (!controller.signal.aborted && collisionRun.current === controller) setCollisionView({ key: collisionKey, status: 'error', items: [], error: error instanceof Error ? error.message : '干涉检测失败' });
      });
    }, 120);
    return () => { clearTimeout(timer); controller.abort(); if (collisionRun.current === controller) collisionRun.current = null; };
  }, [collisionRequest, collisionKey, busy, exporting, executing, zoneDragging, zones]);
  useEffect(() => {
    if (!playing || !activeSegment) return;
    const from = progress, started = performance.now(), duration = Math.max(1.2, activeSegment.result.durationSeconds) * 1000;
    let frame = 0;
    const tick = (time: number) => { const next = Math.min(1, from + (time - started) / duration); setProgress(next); if (next >= 1) setPlaying(false); else frame = requestAnimationFrame(tick); };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, [playing, activeSegment]);
  useEffect(() => () => { jogController.current?.dispose(); motionRun.current?.abort(); exportRun.current?.abort(); inspectionRun.current?.abort(); collisionRun.current?.abort(); executionRun.current?.controller.abort(); if (executionRun.current) cancelAnimationFrame(executionRun.current.frame); }, []);
  useEffect(() => {
    if (!pointMenu) return;
    const close = () => setPointMenu(null);
    const key = (event: KeyboardEvent) => { if (event.key === 'Escape') close(); };
    window.addEventListener('pointerdown', close); window.addEventListener('keydown', key); window.addEventListener('resize', close);
    return () => { window.removeEventListener('pointerdown', close); window.removeEventListener('keydown', key); window.removeEventListener('resize', close); };
  }, [pointMenu]);
  useEffect(() => {
    if (!isNativeUiSmoke()) return;
    const pose = forwardKinematics(model, model.readyJoints).pose;
    const request: PlanRequest = { modelId, startJoints: model.readyJoints, target: { position: [pose.position[0], 30, pose.position[2]], orientation: pose.orientation }, speedMmPerSecond: 100 };
    void runNativeUiSmoke(request, result => {
      if (result.status !== 'success') return;
      const point = program.capture({ pose: result.selected.pose, joints: result.selected.joints, configuration: result.selected.configuration, speedMmPerSecond: 100 });
      workspace.selectObject({ kind: 'target', id: point.id });
      setSegment({ kind: 'abb', pointId: point.id, pointRevision: point.revision, modelId, request, result }); setProgress(1);
    }, selectCatalogModel);
  }, []);

  function collisionInfo(id: string): { state: CollisionStatus; label: string; title: string } {
    if (zoneConfiguration.errors.length) return { state: 'error', label: '未检查', title: zoneConfiguration.errors.map(error => error.message).join('；') };
    if (!zones.some(zone => zone.enabled)) return { state: 'off', label: '检测已关闭', title: '未设置已启用的危险区' };
    if (collisionView.key !== collisionKey || collisionView.status === 'pending' || busy || exporting || zoneDragging) return { state: 'pending', label: '检测中', title: '正在检查端点机器人模型与危险区是否干涉' };
    const checked = collisionById.get(id);
    if (collisionView.status === 'error' || !checked) return { state: 'error', label: '检测失败', title: collisionView.error || '未取得完整检测结果' };
    const describe = (kind: 'collidingLinks' | 'uncertainLinks') => checked.zones.filter(zone => zone[kind].length).map(zone => (zoneNames.get(zone.zoneId) || '未命名危险区') + '（' + zone[kind].map(link => link === 0 ? '基座' : '连杆 ' + link).join('、') + '）').join('；');
    const unknown = checked.uncertainLinks.length ? '无法确定：' + describe('uncertainLinks') + '。当前模型无法可靠确定该区域是否位于实体内部。' : '';
    const scope = '仅检查端点与已启用危险区，未检查整段路径或自碰撞。';
    if (checked.collidingLinks.length) return { state: 'hit', label: '干涉', title: '确认干涉：' + describe('collidingLinks') + '。' + unknown + scope };
    if (checked.uncertainLinks.length) return { state: 'uncertain', label: '无法确定', title: unknown + scope };
    return { state: 'clear', label: '未检测到干涉', title: scope };
  }
  function collisionBadge(id: string) {
    const info = collisionInfo(id);
    return <span className={'collision-badge collision-' + info.state} role="status" aria-label={info.label} title={info.title}>{(info.state === 'hit' || info.state === 'uncertain') && <b aria-hidden="true">{info.state === 'hit' ? '!' : '?'}</b>}{info.label}</span>;
  }
  function collisionIds(id: string, uncertain = false): string[] {
    const state = collisionInfo(id).state;
    if (state !== 'hit' && state !== 'uncertain') return [];
    return collisionById.get(id)?.zones.filter(zone => (uncertain ? zone.uncertainLinks : zone.collidingLinks).length).map(zone => zone.zoneId) ?? [];
  }
  return <div className="app-shell">
    <header className="topbar" inert={closeState !== null}>
      <a className="brand" href="./" aria-label="Emulation 首页"><span className="brand-symbol"><Icon name="cube" size={25}/></span><span>emulation<span className="brand-caption">机器人运动仿真</span></span></a>
      <div className="header-divider"/><div className="project-label"><span className="project-context">运动工作台</span><span className="project-separator">/</span><button type="button" className="robot-model-button" aria-label="选择机器人型号" title={activeName} disabled={exporting || executing || sourceBusy} onClick={() => setCatalogOpen(true)}><span className="robot-model-name">{activeName}</span></button></div>
      <div className="workspace-view-switch" aria-label="工作区视图">{(['scene', 'program'] as const).map(item => <button key={item} type="button" aria-pressed={activeView === item} className={activeView === item ? 'selected' : ''} onClick={() => changeWorkspaceView(item)}>{item === 'scene' ? '场景' : '程序'}</button>)}</div>
      <div className="header-right"><span className="version">v0.2.1</span></div>
    </header>
    <main className="workspace" data-active-view={activeView} data-left-collapsed={leftCollapsed} inert={closeState !== null}>
      <aside className="object-panel" aria-label="场景对象">
        <div className="object-panel-header"><h2>场景对象</h2><button type="button" className="icon-button" aria-label={leftCollapsed ? '展开左侧面板' : '折叠左侧面板'} onClick={() => workspace.setLeftCollapsed(value => !value)}>{leftCollapsed ? '›' : '‹'}</button></div>
        <div className="object-panel-content" aria-hidden={leftCollapsed} inert={leftCollapsed}>
          <div className="workspace-object-list">
            <button type="button" aria-label="选择机器人" data-object-kind="robot" className={selection.kind === 'robot' ? 'selected' : ''} onClick={() => chooseObject({ kind: 'robot' })}><Icon name="cube" size={14}/><span>{activeName}</span></button>
            <div className="object-section-label"><span>危险区域</span><button type="button" className="text-button" aria-label="添加危险区" disabled={!workspace.canAddZone || exporting || executing} onClick={() => { clearSegment(); stopJog(); cancelInspections(); workspace.addZone(); }}>＋</button></div>
            {zones.map(zone => <button key={zone.id} type="button" data-object-kind="zone" data-object-id={zone.id} data-zone-id={zone.id} aria-label={'选择危险区 ' + (zone.name.trim() || '未命名危险区')} className={selectedZoneId === zone.id ? 'selected' : ''} onClick={() => chooseObject({ kind: 'zone', id: zone.id })}><span className={'zone-object-dot ' + (zone.enabled ? '' : 'disabled')}/><span>{zone.name.trim() || '未命名危险区'}</span>{zoneConfiguration.errors.some(error => error.id === zone.id) && <b title="区域参数无效">!</b>}</button>)}
            <div className="object-section-label"><span>Paths</span><small>{program.paths.length}</small></div>
            {program.paths.map(path => <details key={path.id} className="object-path" open><summary><button type="button" aria-label={'选择路径 ' + path.name} data-object-kind="path" data-object-id={path.id} className={selectedPath?.id === path.id ? 'selected' : ''} onClick={event => { event.preventDefault(); chooseObject({ kind: 'path', id: path.id }); }}><Icon name="arrow" size={13}/><span>{path.name}</span><small>{path.pointIds.length}</small></button></summary><div className="object-path-points">{path.pointIds.map(id => {
              const point = program.points.find(item => item.id === id);
              return point && <button type="button" key={id} aria-label={'选择目标点 ' + point.pointName} data-object-kind="target" data-object-id={id} className={selectedPoint?.id === id ? 'selected' : ''} onContextMenu={event => { event.preventDefault(); setPointMenu({ id, x: Math.min(event.clientX, window.innerWidth - 190), y: Math.min(event.clientY, window.innerHeight - 80) }); }} onKeyDown={event => { if (event.key === 'ContextMenu' || event.key === 'F10' && event.shiftKey) { event.preventDefault(); const rect = event.currentTarget.getBoundingClientRect(); setPointMenu({ id, x: Math.min(rect.left, window.innerWidth - 190), y: Math.min(rect.bottom, window.innerHeight - 80) }); } }} onClick={() => chooseObject({ kind: 'target', id })}><Icon name="target" size={12}/><span>{point.pointName}</span></button>;
            })}</div></details>)}
            {!program.paths.length && <p className="field-hint object-empty">在程序中多选指令行，转换为 Path 后显示在这里。</p>}
          </div>
        </div>
      </aside>

      <section className="scene-panel" aria-label="机器人视口" aria-hidden={activeView !== 'scene'} inert={activeView !== 'scene'}>
        <div className="scene-heading"><div><span className="eyebrow">ROBOT WORKSPACE</span><h1>机器人工作区</h1><p>{genericModel ? 'URDF 模型 · 关节姿态 · 工具坐标' : '对象调姿 · 姿态记录 · 端点检查'}</p></div><span className="robot-tag"><span className={activeBrand === 'ABB' ? 'abb-mark' : 'robot-brand-mark'}>{activeBrand}</span>{activeName}<span className="tag-divider"/>{limits.length} 轴</span></div>
        <div className="viewport">
          <RobotScene model={model} joints={visibleJoints} urdfModel={genericModel ?? undefined} urdfJoints={visibleJoints}
            targetPose={selectedPoint?.pose ?? null} path={activeSegment?.result.path ?? EMPTY_PATH} endJoints={activeSegment?.result.path.at(-1)?.joints}
            showFrames={showFrames} view={view} viewRevision={viewRevision}
            jogEnabled={!genericModel && selection.kind === 'robot' && activeView === 'scene' && jogEnabled && !busy && !exporting && !executing && !poseError}
            jogMode={jogMode} jogSpace={jogSpace} jogPending={jogStatus.pending} jogResetRevision={jogResetRevision}
            onJogStart={beginJog} onJogMove={pose => jogController.current?.update(pose)} onJogEnd={() => setJogDragging(false)}
            dangerZones={sceneZones} selectedDangerZoneId={selectedZoneId ?? undefined}
            zoneManipulationEnabled={selection.kind === 'zone' && activeView === 'scene' && !busy && !exporting && !executing}
            onDangerZoneMove={(id, centerMm) => { collisionRun.current?.abort(); workspace.editZone(id, { center: centerMm.map(String) }); }}
            onDangerZoneDrag={setZoneDragging} scenePaths={scenePaths} selectedPointId={selectedPoint?.id ?? null}/>
          <div className="viewport-toolbar"><div className="view-options" aria-label="观察视角">{([['iso', '透视'], ['front', '正视'], ['top', '俯视']] as const).map(([id, label]) => <button key={id} className={view === id ? 'selected' : ''} onClick={() => changeView(id)}>{label}</button>)}</div><button className="icon-button" title="重置视角" aria-label="重置视角" onClick={() => changeView('iso')}><Icon name="reset" size={15}/></button></div>
          {!genericModel && selection.kind === 'robot' && <div className="object-jog-toolbar"><div className="view-options"><button type="button" aria-pressed={jogEnabled} className={jogEnabled ? 'selected' : ''} disabled={busy || exporting || executing || Boolean(poseError)} onClick={() => { stopJog(); clearSegment(); setJogEnabled(value => !value); }}>拖动法兰</button>{jogEnabled && <>{([['translate', '平移'], ['rotate', '旋转']] as const).map(([id, label]) => <button key={id} type="button" disabled={executing || busy || exporting} aria-pressed={jogMode === id} className={jogMode === id ? 'selected' : ''} onClick={() => { stopJog(); setJogMode(id); }}>{label}</button>)}{([['world', '世界坐标'], ['local', '工具坐标']] as const).map(([id, label]) => <button key={id} type="button" disabled={executing || busy || exporting} aria-pressed={jogSpace === id} className={jogSpace === id ? 'selected' : ''} onClick={() => { stopJog(); setJogSpace(id); }}>{label}</button>)}</>}</div></div>}
          {jogEnabled && selection.kind === 'robot' && !genericModel && <div className="object-jog-status" role="status" aria-label="拖动状态">{jogStatus.error ? '已保持上一个有效姿态：' + jogStatus.error : configuration?.singular ? '当前姿态接近奇异位置，请调整关节或使用示例姿态。' : jogStatus.pending ? '正在检查拖动路径…' : '拖动控制柄调整法兰；无效目标不会移动机器人。'}</div>}
          {selection.kind === 'zone' && <div className="object-jog-status" role="status">拖动世界坐标箭头移动危险区，或在右侧填写位置与尺寸。</div>}
          <div className="viewport-caption"><span className="axis-dot"/>基座位于世界原点 <code>[0, 0, 0]</code></div>
          <div className="scene-legend"><span><i className="legend-current"/>当前 TCP</span>{selectedPoint && <span><i className="legend-target"/>已记录目标</span>}{activeSegment && <span><i className="legend-path"/>已验证单段</span>}</div>
        </div>
        {previewSolution && <div className="endpoint-preview-notice" role="status" aria-label="端点预览"><div><strong>正在预览端点姿态</strong><span>仅改变三维显示，目标点、程序和原机器人姿态保持不变。</span><code>robconf [{previewSolution.configuration.join(', ')}]</code></div><button type="button" className="copy-button" onClick={() => setEndpointPreview(null)}>退出预览</button></div>}
        <div className="playback"><button className="play-button" aria-label={playing ? '暂停播放' : '播放路径'} disabled={!activeSegment || busy || Boolean(previewSolution)} onClick={() => { if (!playing && progress >= 1) setProgress(0); setPlaying(value => !value); }}><Icon name={playing ? 'pause' : 'play'} size={15}/></button><div className="playback-track"><input type="range" aria-label="路径进度" min="0" max="100" step="0.1" value={progress * 100} disabled={!activeSegment || busy || Boolean(previewSolution)} onChange={event => { setPlaying(false); setProgress(Number(event.target.value) / 100); }}/></div><span className="progress-value">{Math.round(progress * 100)}<small>%</small></span><label className="frame-toggle"><input type="checkbox" checked={showFrames} onChange={event => setShowFrames(event.target.checked)}/>显示关节坐标系</label></div>
        {execution.message && <div className={'execution-status execution-' + execution.phase} role="status" aria-label="程序运行状态" data-run-phase={execution.phase} data-run-point-id={execution.pointId ?? ''} data-run-line={execution.line} data-run-cycle={execution.cycle}><span>{execution.message}</span>{executing && <button type="button" className="copy-button" onClick={() => stopExecution()}>停止运行</button>}</div>}
      </section>

      <aside className="control-panel" aria-label="对象属性"><fieldset disabled={exporting || executing} className="inspector-fieldset">
        {selection.kind === 'robot' && <RobotInspector name={activeName} model={genericModel ? null : model} pose={currentPose} joints={displayJoints}
          drafts={genericModel ? catalogDraft : jointDraft} limits={limits} euler={currentPose ? quaternionToEulerZYX(currentPose.orientation) : null}
          configuration={configuration?.configuration ?? null} poseError={poseError} captureDisabledReason={captureDisabledReason} speed={speed}
          onSpeed={setSpeed} onJoint={editJoint} onInitial={() => setRobotPose('ready')} onZero={() => setRobotPose('zero')}/>}
        {selectedZone && <div className="zone-inspector" data-selected-zone-id={selectedZone.id}>
          <div className="panel-title"><div className="section-number">Z</div><h2>危险区属性</h2></div>
          <label className="field-label danger-zone-name">名称<input aria-label="危险区名称" maxLength={80} value={selectedZone.name} onChange={event => editZone({ name: event.target.value })}/></label>
          <div className="danger-zone-actions"><label className="danger-zone-toggle"><input type="checkbox" aria-label="显示干涉区域" checked={selectedZone.enabled} onChange={event => editZone({ enabled: event.target.checked })}/>{genericModel ? '显示区域' : '显示并检测'}</label><button type="button" className="text-button" onClick={() => { collisionRun.current?.abort(); workspace.removeSelectedZone(); }}>删除危险区</button></div>
          {(['center', 'size'] as const).map(field => <section key={field}><label className="field-label danger-zone-size-label">{field === 'center' ? '世界坐标中心' : '区域尺寸'} <span>mm</span></label><div className="position-grid">{axes.map((axis, index) => <label key={axis}><span>{axis}</span><input type="number" aria-label={'危险区' + (field === 'center' ? '中心 ' : '尺寸 ') + axis} step="any" min={field === 'size' ? 0.001 : -1e6} max={1e6} value={selectedZone[field][index]} onChange={event => editZone({ [field]: selectedZone[field].map((value, i) => i === index ? event.target.value : value) })}/></label>)}</div></section>)}
          <button type="button" className="text-button danger-zone-reset" onClick={() => { collisionRun.current?.abort(); workspace.resetSelectedZone(); }}>恢复默认区域</button>
          {zoneConfiguration.errors.map(error => <p className="danger-zone-error" role="alert" key={error.id}>{error.message} 当前不能完成端点检测。</p>)}
          <p className="field-hint">{genericModel ? '当前型号仅显示区域，不执行机器人干涉检查。' : '检测已记录端点的实际模型与区域是否干涉。整段运动与自碰撞尚未检查。'}</p>
        </div>}
        {selectedPath && <div className="path-inspector"><div className="panel-title"><div className="section-number">L</div><h2>{selectedPath.name}</h2></div><p className="field-hint">Path 是程序点的编组，虚线只表示点的顺序。编组不会自动验证运动；运行前逐段检查。</p><ol className="path-point-list">{selectedPath.pointIds.map(id => {
          const point = program.points.find(item => item.id === id);
          return point && <li key={id}><button type="button" className="copy-button" onClick={() => chooseObject({ kind: 'target', id })}>{point.pointName}</button><span>{profile.linearInstruction} · {point.speedMmPerSecond} mm/s</span></li>;
        })}</ol><button type="button" className="text-button" onClick={() => { program.removePath(selectedPath.id); workspace.selectObject({ kind: 'robot' }); }}>删除 Path 编组</button><p className="field-hint">删除编组保留原程序点和执行顺序。</p></div>}
        {selectedPoint && <TargetInspector modelKey={activeModelId} point={selectedPoint} updateDisabledReason={poseCaptureDisabledReason} previewing={Boolean(previewSolution)} checking={busy} onUpdate={updatePointFromCurrent} onCheck={() => void checkSegment()}>
          {busy && <button type="button" className="text-button" onClick={() => clearSegment()}>取消 {profile.linearInstruction} 检查</button>}
          {segmentError && <div className="result-error" role="alert"><div><strong>此 {profile.linearInstruction} 未通过</strong><p>{segmentError}</p></div></div>}
          {activeSegment && <div className="result-success">
            <div className="success-heading"><span><Icon name="check" size={15}/></span><div><h3>此 {profile.linearInstruction} 已通过运动学检查</h3><p>仅当前起点到 {selectedPoint.pointName} · {activeSegment.result.path.length} 个采样点</p></div></div>
            <p className="field-hint">未验证整段碰撞、其他程序段或控制器动态。</p>
            {activeSegment.kind === 'abb' && <textarea className="segment-code" readOnly spellCheck={false} aria-label="RAPID MoveL 指令" value={code}/>}
            <div className="segment-actions">{activeSegment.kind === 'abb' && <button className="copy-button" type="button" onClick={() => void copySegment()}>{copyState || '复制此 MoveL'}</button>}<button className="copy-button" type="button" disabled={Boolean(previewSolution)} onClick={() => { setProgress(0); setPlaying(true); workspace.setActiveView('scene'); }}>播放此 {profile.linearInstruction}</button></div>
            {activeSegment.kind === 'abb' ? <><label className="field-label segment-module-label">模块名称<input aria-label="程序模块名称" maxLength={32} value={moduleName} onChange={event => setModuleName(event.target.value)}/></label><button className="copy-button" type="button" disabled={!nativeDesktop || exporting} onClick={() => void exportSegment()}>{exporting ? '正在导出…' : '导出此 MoveL'}</button><p className="field-hint" role="status">{exportMessage || (nativeDesktop ? '仅导出此已验证运动段；模块会检查实际起始关节，不自动移动到起点。' : '完整模块请使用桌面客户端导出。')}</p></> : <p className="field-hint">此型号支持运动仿真；控制器文件导出尚未适配。</p>}
          </div>}
          {!genericModel && <section className="endpoint-details"><div className="field-heading"><h3>端点姿态与干涉检测</h3><button type="button" className="text-button" disabled={busy || exporting} onClick={() => { cancelInspections(); setInspectionRevision(value => value + 1); }}>重新检查端点解</button></div>
            <p className="field-hint" role="status" aria-label="端点检查状态">{inspection.key !== pointKey || inspection.pending ? '正在检查端点逆解…' : inspection.error || (solutions.length ? '预览仅显示候选姿态，不修改目标点、原机器人姿态或已验证路径。' : '没有找到有效的端点逆解。')}</p>
            <div className="endpoint-list">{solutions.map((solution, index) => <div key={solution.id} className={'endpoint-solution' + (previewSolution?.id === solution.id ? ' endpoint-previewed' : '')} role="group" aria-label={'端点解 ' + (index + 1)} data-endpoint-joints={JSON.stringify(solution.joints)} data-collision-state={collisionInfo(solution.id).state} data-collision-zone-ids={JSON.stringify(collisionIds(solution.id))} data-collision-uncertain-zone-ids={JSON.stringify(collisionIds(solution.id, true))}>
              <div className="endpoint-solution-heading"><code>{'[' + solution.configuration.join(', ') + ']'}</code>{solution.configuration.every((value, i) => value === selectedPoint.configuration?.[i]) && <small>已记录配置</small>}<button type="button" className="copy-button endpoint-preview" aria-label={'预览姿态 ' + (index + 1)} aria-pressed={previewSolution?.id === solution.id} disabled={busy || exporting} onClick={() => previewEndpoint(solution)}>预览</button></div>
              <span className="endpoint-joints">{solution.joints.map(value => numberText(value, 1)).join(' / ')}°</span>{collisionBadge(solution.id)}
            </div>)}</div>
            <p className="field-hint">! 确认干涉 · ? 无法可靠判断内部。检测只覆盖端点与危险区，未检查整段路径或自碰撞。</p>
          </section>}
        </TargetInspector>}
      </fieldset>{message && <p className="workspace-message" role="status">{message}</p>}</aside>

      <section className="code-panel" aria-label="程序工作区" aria-hidden={activeView !== 'program'} inert={activeView !== 'program'}>
        <ProgramPanel modelKey={activeModelId} active={activeView === 'program'} points={program.points} paths={program.paths} selectedPointIds={selectedRows}
          captureDisabledReason={captureDisabledReason} onInsertCurrent={captureCurrent} onSelectionChange={setSelectedRows}
          onSelectTarget={id => chooseObject({ kind: 'target', id })} onConvertToPath={convertToPath} onRemovePoints={removePoints}
          running={executing} runDisabledReason={runDisabledReason} runStatus={execution.message} onRun={() => void runProgram()} onStop={() => stopExecution()}
          loop={loop} onLoopChange={setLoop} executingPointId={execution.phase === 'running' ? execution.pointId : null}
          mode={programMode} onModeChange={changeProgramMode} codeWorkspace={codeWorkspace}
          codeLoadingError={!documents.ready && documents.error ? documents.error : languageState.key === languageKey ? languageState.error : ''}
          onRetryCodeLoading={() => { if (!documents.ready) void documents.retry(); else setLanguageRetry(value => value + 1); }}/>
      </section>
    </main>
    {pointMenu && <div className="point-action-menu" role="menu" aria-label="目标点右键菜单" style={{ left: pointMenu.x, top: pointMenu.y }} onPointerDown={event => event.stopPropagation()}><button autoFocus type="button" role="menuitem" disabled={executing || busy || exporting || jogStatus.pending || jogDragging} title="定位到记录姿态" onClick={() => moveToPoint(pointMenu.id)}>移动至此</button><small>仿真定位 · 不验证移动路径</small></div>}
    {catalogOpen && <RobotCatalogDialog currentModelId={activeModelId} onSelect={selectCatalogModel} onClose={() => setCatalogOpen(false)}/>}
    {closeState && <div className="source-close-backdrop"><section className="source-close-dialog" role="dialog" aria-modal="true" aria-label="关闭前保存草稿" onKeyDown={event => {
      if (event.key === 'Escape' && closeState === 'error') { event.preventDefault(); setCloseState(null); }
      if (event.key === 'Tab') { const buttons = [...event.currentTarget.querySelectorAll<HTMLButtonElement>('button:not(:disabled)')], first = buttons[0], last = buttons.at(-1); if (event.shiftKey && document.activeElement === first) { event.preventDefault(); last?.focus(); } else if (!event.shiftKey && document.activeElement === last) { event.preventDefault(); first?.focus(); } }
    }}><h2>{closeState === 'saving' ? '正在保存草稿' : '草稿尚未保存'}</h2><p role="status">{closeState === 'saving' ? '正在保存各型号的本地草稿，完成后关闭窗口。' : closeError}</p>{closeState === 'error' && <div className="source-close-actions"><button autoFocus type="button" onClick={() => setCloseState(null)}>继续编辑</button><button type="button" onClick={() => void finishNativeClose()}>重试保存并关闭</button><button type="button" className="source-discard" onClick={() => void finishNativeClose(true)}>放弃未保存草稿并关闭</button></div>}</section></div>}
    <footer className="app-footer"><span><Icon name="info" size={13}/>{genericModel ? '基于源 URDF 的直线运动仿真；品牌模板未完成控制器验证。' : '程序运行前检查全部运动段；尚未检查整段碰撞。'}</span><span>世界坐标 Z 向上 · 长度 mm · 角度 deg</span></footer>
  </div>;
}
