import { useEffect, useMemo, useRef, useState, type FormEvent, type ReactNode } from 'react';
import RobotScene from './scene/RobotScene';
import { copyRapid, nativeDesktop, saveProgram, solveMotion } from './robot/client';
import { isNativeUiSmoke, runNativeUiSmoke } from './robot/ui-smoke';
import { forwardKinematics } from './robot/kinematics';
import { getConfiguration } from './robot/configuration';
import { checkCollisions, type CollisionResult } from './robot/collision';
import { useWorkspace, type DangerZoneDraft, type WorkspaceView } from './workspace/useWorkspace';
import { createJogController, type JogController, type JogStatus } from './robot/jog-controller';
import { getRobotModel } from './robot/model';
import RobotCatalogDialog from './catalog/RobotCatalogDialog';
import type { LoadedCatalogModel } from './catalog/client';
import { forwardUrdfKinematics } from './catalog/kinematics';
import { eulerZYXToQuaternion, formatMoveL, normalizeOrientation } from './robot/rapid';
import type { JointAngles, MotionSample, PlanRequest, PlanResult, Pose, Quaternion, RobotId, Vec3 } from './robot/types';
import './styles.css';

type Mode = 'auto' | 'orientation';
type View = 'iso' | 'front' | 'top';
const axes = ['X', 'Y', 'Z'] as const;
const EMPTY_PATH: MotionSample[] = [];
type CollisionStatus = 'pending' | 'hit' | 'uncertain' | 'clear' | 'error' | 'off';
interface CollisionView { key: string; status: 'pending' | 'ready' | 'error'; items: CollisionResult[]; error?: string }
const numberText = (v: number, digits = 2) => Number(v.toFixed(digits)).toString();
const strings = (v: number[]) => v.map(n => numberText(n, 6));
function finite(values: string[], label: string): number[] {
  if (values.some(v => !v.trim() || !Number.isFinite(Number(v)))) throw new Error(`${label}需要填写完整的有限数值。`);
  return values.map(Number);
}
function quaternionToEulerZYX(value: Quaternion): Vec3 {
  const [w, x, y, z] = normalizeOrientation(value);
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const locked = Math.abs(Math.cos(pitch)) < 1e-7;
  const yaw = locked ? Math.atan2(-2 * (x * y - w * z), 1 - 2 * (x * x + z * z)) : Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const roll = locked ? 0 : Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  return [yaw, pitch, roll].map(v => v * 180 / Math.PI) as Vec3;
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
function initialTarget(id: RobotId): string[] {
  const m = getRobotModel(id);
  const p = forwardKinematics(m, m.readyJoints).pose.position;
  return strings([p[0], p[1] + 30, p[2]]);
}

export default function App() {
  const [modelId, setModelId] = useState<RobotId>('irb1100-4-0.58');
  const model = getRobotModel(modelId);
  const [catalogOpen, setCatalogOpen] = useState(false);
  const [catalogModel, setCatalogModel] = useState<Extract<LoadedCatalogModel, { kind: 'urdf' }> | null>(null);
  const [catalogJointDraft, setCatalogJointDraft] = useState<string[]>([]);
  const [catalogJoints, setCatalogJoints] = useState<number[]>([]);
  const genericModel = catalogModel?.model ?? null;
  const activeName = catalogModel?.entry.name ?? model.name;
  const activeBrand = catalogModel?.entry.brand ?? 'ABB';
  const activeModelId = catalogModel?.entry.id ?? modelId;
  useEffect(() => () => { catalogModel?.dispose(); }, [catalogModel]);

  const [jointDraft, setJointDraft] = useState<string[]>(strings(model.readyJoints));
  const [positionDraft, setPositionDraft] = useState<string[]>(() => initialTarget(modelId));
  const [orientationDraft, setOrientationDraft] = useState<string[]>(() => strings(forwardKinematics(model, model.readyJoints).pose.orientation));
  const [eulerDraft, setEulerDraft] = useState<string[]>(() => strings(quaternionToEulerZYX(forwardKinematics(model, model.readyJoints).pose.orientation)));
  const [orientationFormat, setOrientationFormat] = useState<'quaternion' | 'euler'>('quaternion');
  const [mode, setMode] = useState<Mode>('auto');
  const [jogEnabled, setJogEnabled] = useState(false);
  const [jogMode, setJogMode] = useState<'translate' | 'rotate'>('translate');
  const [jogSpace, setJogSpace] = useState<'world' | 'local'>('world');
  const [jogStatus, setJogStatus] = useState<JogStatus>({ pending: false, error: null });
  const [jogDragging, setJogDragging] = useState(false);
  const [jogResetRevision, setJogResetRevision] = useState(0);
  const [jogJoints, setJogJoints] = useState<JointAngles | null>(null);
  const jogController = useRef<JogController | null>(null);
  const [speed, setSpeed] = useState(100);
  const [result, setResult] = useState<PlanResult | null>(null);
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  const [progress, setProgress] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [view, setView] = useState<View>('iso');
  const [viewRevision, setViewRevision] = useState(0);
  const [showFrames, setShowFrames] = useState(false);
  const [copyState, setCopyState] = useState('');
  const [moduleName, setModuleName] = useState('EmulationMotion');
  const [exporting, setExporting] = useState(false);
  const [exportMessage, setExportMessage] = useState('');
  const [validatedRequest, setValidatedRequest] = useState<PlanRequest | null>(null);
  const [endpointPreviewId, setEndpointPreviewId] = useState<string | null>(null);
  const workspace = useWorkspace();
  const { activeView, selectedZone, selectedZoneId, zones, zoneConfiguration, leftCollapsed } = workspace;
  const hasEnabledZones = zones.some(zone => zone.enabled);
  const zoneNames = useMemo(() => new Map(zones.map(zone => [zone.id, zone.name.trim() || '未命名危险区'])), [zones]);
  const sceneZones = useMemo(() => zoneConfiguration.validZones.map(zone => ({ ...zone, name: zones.find(draft => draft.id === zone.id)?.name.trim() || '未命名危险区' })), [zoneConfiguration.validZones, zones]);
  const [collisionView, setCollisionView] = useState<CollisionView>({ key: '', status: 'pending', items: [] });
  const collisionRun = useRef<AbortController | null>(null);
  const exportRequest = useRef<AbortController | null>(null);
  const currentRequest = useRef<AbortController | null>(null);
  const copyTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const success = result?.status === 'success' ? result : null;
  const allSolutions = useMemo(() => {
    if (genericModel) return [];
    const solutions = result?.solutions ?? [];
    if (result?.status === 'success' && !solutions.some(item => item.id === result.selected.id)) return [result.selected, ...solutions];
    return solutions;
  }, [result, genericModel]);
  const endpointPreview = allSolutions.find(item => item.id === endpointPreviewId) ?? null;
  const collisionRequest = useMemo(() => !genericModel && !zoneConfiguration.errors.length && allSolutions.length ? {
    modelId, zones: zoneConfiguration.validZones, solutions: allSolutions.map(item => ({ id: item.id, joints: item.joints })),
  } : null, [modelId, genericModel, zoneConfiguration, allSolutions]);
  const collisionKey = collisionRequest ? JSON.stringify(collisionRequest) : '';
  const collisionById = useMemo(() => new Map(collisionView.items.map(item => [item.id, item])), [collisionView.items]);
  useEffect(() => {
    const controller = new AbortController();
    collisionRun.current?.abort(); collisionRun.current = controller;
    if (!collisionRequest || !hasEnabledZones || busy || exporting || jogStatus.pending || jogDragging) return () => controller.abort();
    setCollisionView({ key: collisionKey, status: 'pending', items: [] });
    const timer = setTimeout(() => {
      if (controller.signal.aborted) return;
      void checkCollisions(collisionRequest, controller.signal).then(items => {
        if (!controller.signal.aborted && collisionRun.current === controller) setCollisionView({ key: collisionKey, status: 'ready', items });
      }, cause => {
        if (!controller.signal.aborted && collisionRun.current === controller) setCollisionView({ key: collisionKey, status: 'error', items: [], error: cause instanceof Error ? cause.message : '干涉检测失败' });
      });
    }, 120);
    return () => { clearTimeout(timer); controller.abort(); if (collisionRun.current === controller) collisionRun.current = null; };
  }, [collisionRequest, collisionKey, hasEnabledZones, busy, exporting, jogStatus.pending, jogDragging]);
  function collisionInfo(id: string): { state: CollisionStatus; label: string; title: string } {
    if (genericModel) return { state: 'off', label: '尚未适配', title: '该型号尚未适配端点求解与干涉检测' };
    if (zoneConfiguration.errors.length) return { state: 'error', label: '未检查', title: zoneConfiguration.errors.map(error => error.message).join('；') };
    if (!hasEnabledZones) return { state: 'off', label: '检测已关闭', title: '未设置已启用的危险区' };
    if (collisionView.key !== collisionKey || collisionView.status === 'pending' || busy || exporting || jogStatus.pending || jogDragging) return { state: 'pending', label: '检测中', title: '正在检查端点机器人模型与危险区是否干涉' };
    if (collisionView.status === 'error' || !collisionById.has(id)) return { state: 'error', label: '检测失败', title: collisionView.error || '未取得该端点的完整检测结果' };
    const checked = collisionById.get(id)!;
    const describeZones = (kind: 'collidingLinks' | 'uncertainLinks') => checked.zones.filter(zone => zone[kind].length).map(zone => `${zoneNames.get(zone.zoneId) || '未命名危险区'}（${zone[kind].map(link => link === 0 ? '基座' : `连杆 ${link}`).join('、')}）`).join('；');
    const uncertain = checked.uncertainLinks.length ? `无法确定：${describeZones('uncertainLinks')}。当前模型无法可靠确定该区域是否位于实体内部。` : '';
    const scope = '仅检查当前端点与已启用的危险区，未检查整段路径或自碰撞。';
    if (checked.collidingLinks.length) return { state: 'hit', label: '干涉', title: `确认干涉：${describeZones('collidingLinks')}。${uncertain}${scope}` };
    if (checked.uncertainLinks.length) return { state: 'uncertain', label: '无法确定', title: `${uncertain}${scope}` };
    return { state: 'clear', label: '未检测到干涉', title: `本次端点检测未发现机器人模型与已启用危险区发生干涉。${scope}` };
  }
  function collisionZoneIds(id: string): string[] {
    return collisionInfo(id).state === 'hit' ? (collisionById.get(id)?.zones.filter(zone => zone.collidingLinks.length).map(zone => zone.zoneId) ?? []) : [];
  }
  function uncertainZoneIds(id: string): string[] {
    const state = collisionInfo(id).state;
    return state === 'hit' || state === 'uncertain' ? (collisionById.get(id)?.zones.filter(zone => zone.uncertainLinks.length).map(zone => zone.zoneId) ?? []) : [];
  }

  function collisionBadge(id: string) {
    const info = collisionInfo(id);
    return <span className={`collision-badge collision-${info.state}`} role="status" aria-label={info.label} title={info.title}>{(info.state === 'hit' || info.state === 'uncertain') && <b aria-hidden="true">{info.state === 'hit' ? '!' : '?'}</b>}{info.label}</span>;
  }
  function applyEndpoint(id: string) {
    jogController.current?.dispose(); jogController.current = null;
    setJogStatus({ pending: false, error: null }); setJogDragging(false);
    setPlaying(false); setEndpointPreviewId(id); setCopyState('');
    workspace.setActiveView('scene');
    setJogResetRevision(revision => revision + 1);
  }
  function restoreValidatedPath() {
    setEndpointPreviewId(null); setProgress(1); setPlaying(false); setCopyState('');
    setJogResetRevision(revision => revision + 1);
  }
  function editSelectedZone(patch: Partial<Omit<DangerZoneDraft, 'id'>>) {
    if (!selectedZoneId) return;
    collisionRun.current?.abort();
    workspace.editZone(selectedZoneId, patch);
  }
  function editZoneVector(field: 'center' | 'size', index: number, value: string) {
    if (!selectedZone) return;
    editSelectedZone({ [field]: selectedZone[field].map((item, i) => i === index ? value : item) });
  }
  function changeWorkspaceView(next: WorkspaceView) {
    if (next === activeView) return;
    setPlaying(false);
    if (jogStatus.pending || jogDragging) stopJog();
    workspace.setActiveView(next);
  }
  const jointInputValid = useMemo(() => {
    if (genericModel) return false;
    try { return finite(jointDraft, '起始关节').every((value, i) => value >= model.limits[i].minDeg && value <= model.limits[i].maxDeg); } catch { return false; }
  }, [jointDraft, model, genericModel]);
  const startJoints = useMemo(() => {
    try {
      const values = finite(jointDraft, '起始关节');
      if (values.some((v, i) => v < model.limits[i].minDeg || v > model.limits[i].maxDeg)) return model.homeJoints;
      return values as JointAngles;
    } catch { return model.homeJoints; }
  }, [jointDraft, model]);
  const displayJoints = useMemo(() => {
    if (endpointPreview) return endpointPreview.joints;
    if (!success?.path.length) return jogJoints ?? startJoints;
    const samples = success.path;
    if (progress <= 0) return samples[0].joints;
    if (progress >= 1) return samples[samples.length - 1].joints;
    const next = samples.findIndex(sample => sample.t >= progress);
    const before = samples[Math.max(0, next - 1)], after = samples[next < 0 ? samples.length - 1 : next];
    const t = after.t === before.t ? 0 : (progress - before.t) / (after.t - before.t);
    return before.joints.map((v, i) => v + (after.joints[i] - v) * t) as JointAngles;
  }, [endpointPreview, success, progress, startJoints, jogJoints]);
  const visibleJoints = genericModel ? catalogJoints : displayJoints;
  const currentPose = useMemo(() => genericModel ? forwardUrdfKinematics(genericModel.description, catalogJoints).toolPose : forwardKinematics(model, displayJoints).pose, [genericModel, catalogJoints, model, displayJoints]);
  const catalogJointError = useMemo(() => {
    if (!genericModel) return '';
    try {
      const values = finite(catalogJointDraft, '模型关节');
      for (const [index, joint] of genericModel.description.actuatedJoints.entries()) {
        if (values[index] < joint.minDeg || values[index] > joint.maxDeg) return `${joint.name} 须在 ${numberText(joint.minDeg)}° 至 ${numberText(joint.maxDeg)}° 以内；当前保留上一个有效姿态。`;
      }
      return '';
    } catch (cause) { return cause instanceof Error ? cause.message : '关节角无效'; }
  }, [genericModel, catalogJointDraft]);

  const startConfiguration = useMemo(() => getConfiguration(model, startJoints, forwardKinematics(model, startJoints)), [model, startJoints]);
  const singularHint = jointInputValid && startConfiguration.singular ? `${startConfiguration.reason || '当前起点接近奇异位置'}。请点击“示例起始姿态”或先调整关节。` : '';
  const code = success && !genericModel && !endpointPreview ? formatMoveL(success.selected.pose, success.selected.configuration, speed) : '';

  function invalidate(resetJog = true) {
    collisionRun.current?.abort(); setEndpointPreviewId(null);
    if (resetJog) {
      jogController.current?.dispose(); jogController.current = null;
      setJogStatus({ pending: false, error: null }); setJogDragging(false); setJogJoints(null);
      setJogResetRevision(revision => revision + 1);
    }
    currentRequest.current?.abort();
    currentRequest.current = null;
    exportRequest.current?.abort(); exportRequest.current = null;
    setExporting(false); setExportMessage(''); setValidatedRequest(null);
    setBusy(false); setPlaying(false); setProgress(0); setResult(null); setError(''); setCopyState('');
  }
  function updateArray(setter: (values: string[]) => void, values: string[], index: number, value: string) {
    invalidate(); setter(values.map((item, i) => i === index ? value : item));
  }
  function changeModel(id: RobotId) {
    const next = getRobotModel(id);
    invalidate(); setModelId(id); setJointDraft(strings(next.readyJoints));
    setOrientationDraft(strings(forwardKinematics(next, next.readyJoints).pose.orientation));
    setEulerDraft(strings(quaternionToEulerZYX(forwardKinematics(next, next.readyJoints).pose.orientation)));
    setPositionDraft(initialTarget(id)); setMode('auto'); setViewRevision(v => v + 1);
  }
  function selectCatalogModel(loaded: LoadedCatalogModel) {
    invalidate(); setJogEnabled(false); workspace.setActiveView('scene');
    if (loaded.kind === 'bundled') {
      setCatalogModel(null); setCatalogJointDraft([]); setCatalogJoints([]);
      changeModel(loaded.motionModelId);
    } else {
      setCatalogModel(loaded);
      setCatalogJoints([...loaded.model.description.homeJoints]);
      setCatalogJointDraft(strings(loaded.model.description.homeJoints));
      setViewRevision(value => value + 1);
    }
  }
  function editCatalogJoint(index: number, value: string) {
    if (!genericModel) return;
    const drafts = catalogJointDraft.map((current, i) => i === index ? value : current);
    setCatalogJointDraft(drafts);
    try {
      const next = finite(drafts, '模型关节');
      if (next.every((angle, i) => angle >= genericModel.description.actuatedJoints[i].minDeg && angle <= genericModel.description.actuatedJoints[i].maxDeg)) setCatalogJoints(next);
    } catch { /* Partial numeric drafts retain the last rendered valid pose. */ }
  }
  function example() {
    invalidate(); setJointDraft(strings(model.readyJoints)); setPositionDraft(initialTarget(modelId)); setMode('auto');
    setOrientationDraft(strings(forwardKinematics(model, model.readyJoints).pose.orientation));
    setEulerDraft(strings(quaternionToEulerZYX(forwardKinematics(model, model.readyJoints).pose.orientation)));

  }
  function changeOrientationFormat(next: 'quaternion' | 'euler') {
    invalidate();
    try {
      if (next === 'euler') setEulerDraft(strings(quaternionToEulerZYX(finite(orientationDraft, '四元数') as Quaternion)));
      else setOrientationDraft(strings(eulerZYXToQuaternion(finite(eulerDraft, '欧拉角') as Vec3)));
      setOrientationFormat(next);
    } catch (cause) { setError(cause instanceof Error ? cause.message : '请先填写有效的工具方向。'); }
  }
  function targetInput(): { position: Vec3; orientation: Quaternion | null } {
    const position = finite(positionDraft, '目标位置') as Vec3;
    if (mode === 'auto') return { position, orientation: null };
    const orientation = orientationFormat === 'quaternion'
      ? normalizeOrientation(finite(orientationDraft, '四元数') as Quaternion)
      : eulerZYXToQuaternion(finite(eulerDraft, '欧拉角') as Vec3);
    return { position, orientation };
  }
  async function solve(event: FormEvent) {
    event.preventDefault();
    if (genericModel) return;
    invalidate();
    let target: ReturnType<typeof targetInput>;
    let joints: JointAngles;
    try {
      target = targetInput(); joints = jogJoints ?? finite(jointDraft, '起始关节') as JointAngles;
      joints.forEach((v, i) => { const limit = model.limits[i]; if (v < limit.minDeg || v > limit.maxDeg) throw new Error(`J${i + 1} 超出关节范围 ${limit.minDeg}° 至 ${limit.maxDeg}°。`); });
    } catch (cause) { setError(cause instanceof Error ? cause.message : '输入格式不正确。'); return; }
    const controller = new AbortController();
    currentRequest.current = controller; setBusy(true);
    try {
      const request: PlanRequest = { modelId, startJoints: joints, target, speedMmPerSecond: speed };
      const response = await solveMotion(request, controller.signal);
      if (currentRequest.current !== controller) return;
      setResult(response); setValidatedRequest(response.status === 'success' ? request : null); setProgress(response.status === 'success' ? 1 : 0);
    } catch (cause) {
      if (currentRequest.current === controller && !controller.signal.aborted) setError(cause instanceof Error ? cause.message : '运动学计算未完成。');
    } finally {
      if (currentRequest.current === controller) { currentRequest.current = null; setBusy(false); }
    }
  }
  function beginJog() {
    if (genericModel || !jointInputValid || exporting || !jogEnabled) return;
    const joints = [...displayJoints] as JointAngles;
    const pose = forwardKinematics(model, joints).pose;
    jogController.current?.dispose();
    invalidate(false);
    setJointDraft(strings(joints)); setJogJoints(joints); setJogDragging(true);
    setJogStatus({ pending: false, error: null });
    jogController.current = createJogController({
      initial: { modelId, joints, pose, speedMmPerSecond: speed },
      solve: solveMotion,
      onAccept: accepted => {
        setJogJoints(accepted.selected.joints);
        setJointDraft(strings(accepted.selected.joints));
        // Jog changes the current pose. A previous target's RAPID is no longer valid.
        setResult(null); setValidatedRequest(null); setCopyState(''); setError('');
      },
      onStatus: setJogStatus,
    });
  }
  function stopJog() {
    const joints = [...displayJoints] as JointAngles;
    invalidate(); setJogJoints(joints); setJointDraft(strings(joints));
  }
  function changeView(next: View) {
    if (jogStatus.pending || jogDragging) stopJog();
    setView(next); setViewRevision(value => value + 1);
  }
  async function copy() {
    if (!code || busy) return;
    try {
      await copyRapid(code); setCopyState('已复制');
      if (copyTimer.current) clearTimeout(copyTimer.current);
      copyTimer.current = setTimeout(() => setCopyState(''), 2000);
    } catch { setCopyState('请选中代码手动复制'); }
  }
  async function exportModule() {
    if (genericModel || !validatedRequest || !success || endpointPreview || exporting) return;
    const controller = new AbortController();
    exportRequest.current = controller; setExporting(true); setExportMessage('');
    try {
      const path = await saveProgram({ programName: moduleName, controller: 'abb_rapid', motion: validatedRequest }, controller.signal);
      if (exportRequest.current === controller) setExportMessage(path ? `已保存：${path}` : '已取消保存');
    } catch (cause) {
      if (exportRequest.current === controller && !controller.signal.aborted) setExportMessage(cause instanceof Error ? cause.message : '模块导出失败');
    } finally { if (exportRequest.current === controller) { exportRequest.current = null; setExporting(false); } }
  }
  useEffect(() => {
    if (!isNativeUiSmoke()) return;
    const start = forwardKinematics(model, model.readyJoints).pose;
    const request: PlanRequest = { modelId, startJoints: model.readyJoints, target: { position: [start.position[0], 30, start.position[2]], orientation: start.orientation }, speedMmPerSecond: 100 };
    void runNativeUiSmoke(request, result => { setJointDraft(strings(model.readyJoints)); setResult(result); setValidatedRequest(request); setProgress(1); }, selectCatalogModel);
    // This runs only for the installed-app CI smoke flag, once per WebView.
  }, []);
  useEffect(() => () => { collisionRun.current?.abort(); jogController.current?.dispose(); exportRequest.current?.abort(); currentRequest.current?.abort(); if (copyTimer.current) clearTimeout(copyTimer.current); }, []);
  useEffect(() => {
    if (!playing || !success) return;
    const started = performance.now();
    const from = progress;
    const duration = Math.max(1.2, success.durationSeconds) * 1000;
    let frame = 0;
    const tick = (time: number) => {
      const next = Math.min(1, from + (time - started) / duration);
      setProgress(next);
      if (next >= 1) setPlaying(false); else frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
    // The animation captures the starting progress; scrubbing first pauses it.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [playing, success]);

  const previewTarget = useMemo<Pose | null>(() => {
    if (genericModel) return null;
    if (endpointPreview) return endpointPreview.pose;
    if (success) return success.selected.pose;
    if (mode !== 'auto') { try { const input = targetInput(); if (input.orientation) return { position: input.position, orientation: input.orientation }; } catch { /* Invalid drafts have no preview. */ } }
    return null;
  }, [genericModel, endpointPreview, success, mode, positionDraft, orientationDraft, orientationFormat, eulerDraft]);
  const problem = error || (result?.status === 'failure' ? result.message : '');

  return <div className="app-shell">
    <header className="topbar">
      <a className="brand" href="./" aria-label="Emulation 首页"><span className="brand-symbol"><Icon name="cube" size={25} /></span><span>emulation<span className="brand-caption">机器人运动仿真</span></span></a>
      <div className="header-divider"/><div className="project-label"><span className="project-context">运动工作台</span><span className="project-separator">/</span><button type="button" className="robot-model-button" aria-label="选择机器人型号" title={activeName} disabled={exporting} onClick={() => setCatalogOpen(true)}><span className="robot-model-name">{activeName}</span></button></div>
      <div className="workspace-view-switch" aria-label="工作区视图">{(['scene', 'program'] as const).map(item => <button key={item} type="button" aria-pressed={activeView === item} className={activeView === item ? 'selected' : ''} onClick={() => changeWorkspaceView(item)}>{item === 'scene' ? '场景' : '程序'}</button>)}</div>
      <div className="header-right"><span className="local-badge"><i/>{nativeDesktop ? 'Rust 原生引擎' : '浏览器参考引擎'}</span><span className="version">v0.2.0</span></div>
    </header>
    <main className="workspace" data-active-view={activeView} data-left-collapsed={leftCollapsed}>
      <aside className="object-panel" aria-label="对象与状态"><div className="object-panel-header"><h2>对象与状态</h2><button type="button" className="icon-button" aria-label={leftCollapsed ? '展开左侧面板' : '折叠左侧面板'} title={leftCollapsed ? '展开左侧面板' : '折叠左侧面板'} onClick={() => workspace.setLeftCollapsed(value => !value)}>{leftCollapsed ? '›' : '‹'}</button></div><div className="object-panel-content" aria-hidden={leftCollapsed} inert={leftCollapsed}><div className="workspace-object-list" aria-label="场景对象"><button type="button" className={selectedZoneId === null ? 'selected' : ''} onClick={() => workspace.setSelectedZoneId(null)}><Icon name="cube" size={14}/><span>{activeName}</span></button>{zones.map(zone => <button key={zone.id} type="button" data-zone-id={zone.id} aria-label={`选择危险区 ${zone.name.trim() || '未命名危险区'}`} className={selectedZoneId === zone.id ? 'selected' : ''} onClick={() => workspace.setSelectedZoneId(zone.id)}><span className={`zone-object-dot ${zone.enabled ? '' : 'disabled'}`}/><span>{zone.name.trim() || '未命名危险区'}</span>{zoneConfiguration.errors.some(error => error.id === zone.id) && <b title="区域参数无效">!</b>}</button>)}</div><details className="current-state-panel" open><summary>当前位置 position</summary><section className="tcp-readout" aria-label="当前 TCP"><span className="strip-title">当前 TCP <small>mm</small></span><div>{axes.map((axis, i) => <span key={axis} data-axis={axis}><i className={`axis-${axis.toLowerCase()}`}>{axis}</i><strong>{currentPose ? currentPose.position[i].toFixed(2) : '—'}</strong></span>)}</div></section></details><details className="current-state-panel" open><summary>当前姿态</summary><div className="joint-readout" aria-label="当前关节姿态"><span className="strip-title">关节姿态 <small>deg</small></span><div>{visibleJoints.map((value, i) => <span key={i}><small>J{i + 1}</small><strong>{value.toFixed(1)}°</strong></span>)}</div></div></details></div></aside>
      <section className="scene-panel" aria-label="机器人视口" aria-hidden={activeView !== 'scene'} inert={activeView !== 'scene'}>
        <div className="scene-heading"><div><span className="eyebrow">ROBOT WORKSPACE</span><h1>机器人工作区</h1><p>{genericModel ? 'URDF 模型 · 关节姿态 · 工具坐标' : '位置与姿态 · Jog · 干涉检测'}</p></div><span className="robot-tag"><span className={activeBrand === 'ABB' ? 'abb-mark' : 'robot-brand-mark'}>{activeBrand}</span>{activeName}<span className="tag-divider"/>{genericModel?.description.actuatedJoints.length ?? 6} 轴</span></div>
        <div className="viewport">
          <RobotScene model={model} joints={displayJoints} urdfModel={genericModel ?? undefined} urdfJoints={catalogJoints} targetPose={previewTarget} path={endpointPreview ? EMPTY_PATH : success?.path ?? EMPTY_PATH} endJoints={endpointPreview ? undefined : success?.selected.joints} showFrames={showFrames} view={view} viewRevision={viewRevision} jogEnabled={!genericModel && jogEnabled && !exporting && !busy && jointInputValid} jogMode={jogMode} jogSpace={jogSpace} jogPending={jogStatus.pending} jogResetRevision={jogResetRevision} dangerZones={sceneZones} selectedDangerZoneId={selectedZoneId ?? undefined} onJogStart={beginJog} onJogMove={pose => jogController.current?.update(pose)} onJogEnd={() => setJogDragging(false)}/>
          <div className="viewport-toolbar"><div className="view-options" aria-label="观察视角">{([['iso', '透视'], ['front', '正视'], ['top', '俯视']] as const).map(([id, label]) => <button key={id} className={view === id ? 'selected' : ''} onClick={() => changeView(id)}>{label}</button>)}</div><button className="icon-button" title="重置视角" aria-label="重置视角" onClick={() => changeView('iso')}><Icon name="reset" size={15}/></button></div>
          {!genericModel && <div className="jog-toolbar" style={{ position: 'absolute', top: 57, left: 18, right: 18, display: 'flex', gap: 6, flexWrap: 'wrap', pointerEvents: 'none' }}><div className="view-options" style={{ pointerEvents: 'auto' }}><button type="button" aria-pressed={jogEnabled} className={jogEnabled ? 'selected' : ''} disabled={exporting || busy || !jointInputValid} onClick={() => { stopJog(); setJogEnabled(value => !value); }}>拖动法兰</button>{jogEnabled && <><button type="button" aria-pressed={jogMode === 'translate'} className={jogMode === 'translate' ? 'selected' : ''} onClick={() => { stopJog(); setJogMode('translate'); }}>平移</button><button type="button" aria-pressed={jogMode === 'rotate'} className={jogMode === 'rotate' ? 'selected' : ''} onClick={() => { stopJog(); setJogMode('rotate'); }}>旋转</button><button type="button" aria-pressed={jogSpace === 'world'} className={jogSpace === 'world' ? 'selected' : ''} onClick={() => { stopJog(); setJogSpace('world'); }}>世界坐标</button><button type="button" aria-pressed={jogSpace === 'local'} className={jogSpace === 'local' ? 'selected' : ''} onClick={() => { stopJog(); setJogSpace('local'); }}>工具坐标</button></>}</div></div>}
          {!genericModel && jogEnabled && <div role="status" aria-label="拖动状态" style={{ position: 'absolute', left: 18, right: 18, bottom: 46, padding: '7px 10px', borderRadius: 5, background: '#ffffffeb', fontSize: 11, color: jogStatus.error || singularHint ? '#ad6259' : '#647182', pointerEvents: 'none' }}>{jogStatus.error ? `已保持上一个有效姿态：${jogStatus.error}` : singularHint || (jogStatus.pending ? '正在检查拖动路径…' : jogDragging ? '正在跟随法兰目标' : jogMode === 'translate' ? '拖动 X / Y / Z 箭头平移法兰；无效目标不会移动机器人。' : '拖动控制环旋转法兰；无效目标不会移动机器人。')}</div>}
          <div className="viewport-caption"><span className="axis-dot"/>基座位于世界原点 <code>[0, 0, 0]</code></div>
          <div className="scene-legend"><span><i className="legend-current"/>当前 TCP</span>{success && <span><i className="legend-target"/>目标姿态</span>}{success && !endpointPreview && <span><i className="legend-path"/>MoveL 路径</span>}</div>
        </div>
        <div className="playback"><button className="play-button" aria-label={playing ? '暂停播放' : '播放路径'} disabled={!success || Boolean(endpointPreview) || busy} onClick={() => { if (!playing && progress >= 1) setProgress(0); setPlaying(v => !v); }}><Icon name={playing ? 'pause' : 'play'} size={15}/></button><div className="playback-track"><input aria-label="路径进度" type="range" min="0" max="100" step="0.1" value={progress * 100} disabled={!success || Boolean(endpointPreview) || busy} onChange={e => { setPlaying(false); setProgress(Number(e.target.value) / 100); }}/></div><span className="progress-value">{Math.round(progress * 100)}<small>%</small></span><span className="playback-separator"/><label className="frame-toggle"><input type="checkbox" checked={showFrames} onChange={e => setShowFrames(e.target.checked)}/>显示关节坐标系</label></div>
        {endpointPreview && <div className="endpoint-preview-notice" role="status"><div><strong>端点姿态预览</strong><span>未验证从起点到此解的路径；当前不能复制、播放或导出该解。</span><code>robconf [{endpointPreview.configuration.join(', ')}]</code></div>{collisionBadge(endpointPreview.id)}<button type="button" className="copy-button" onClick={restoreValidatedPath}>{success ? '返回已验证路径' : '返回起始姿态'}</button></div>}

      </section>

      <aside className="control-panel"><form onSubmit={solve} noValidate><fieldset disabled={exporting} style={{ border: 0, padding: 0, margin: 0, minWidth: 0 }}>
        <div className="panel-title"><div className="section-number">01</div><h2>{genericModel ? '机器人与关节' : '机器人与起点'}</h2></div>
        {genericModel ? <><p className="generic-model-notice" role="status" aria-label="型号能力"><strong>{activeName}</strong>当前支持模型查看、关节调整与 URDF 正运动学；MoveL、拖动求解、干涉检测和程序导出尚未适配。</p><div className="field-heading"><label className="field-label">模型关节 <span>deg</span></label><button type="button" className="text-button" onClick={() => { setCatalogJoints([...genericModel.description.homeJoints]); setCatalogJointDraft(strings(genericModel.description.homeJoints)); }}>模型初始姿态</button></div><div className="joints-grid">{genericModel.description.actuatedJoints.map((joint, i) => <label className="joint-field" key={joint.name} title={joint.name}><span>J{i + 1}</span><input aria-label={`模型关节 ${joint.name}`} type="number" step="any" min={joint.minDeg} max={joint.maxDeg} value={catalogJointDraft[i] ?? ''} onChange={event => editCatalogJoint(i, event.target.value)}/></label>)}</div><p className="generic-joint-names">{genericModel.description.actuatedJoints.map((joint, i) => `J${i + 1}: ${joint.name}`).join(' · ')}</p>{catalogJointError && <p className="danger-zone-error" role="alert">{catalogJointError}</p>}<details className="details-panel"><summary>URDF 关节范围</summary><div className="joint-limits">{genericModel.description.actuatedJoints.map((joint, i) => <span key={joint.name} title={joint.name}>J{i + 1} <strong>{numberText(joint.minDeg)}° ~ {numberText(joint.maxDeg)}°</strong></span>)}</div></details><p className="field-hint">工具坐标：{genericModel.description.toolLink ?? '源模型未定义 tool0 / flange'}。这里只显示源 URDF 正运动学，不代表控制器标定。</p></> : <>
        <div className="spec-line"><span>额定负载 <strong>{model.payloadKg} kg</strong></span><span>工作半径 <strong>{model.reachMm} mm</strong></span></div>
        <div className="field-heading"><label className="field-label">起始关节 <span>deg</span></label><button type="button" className="text-button" onClick={() => { invalidate(); setJointDraft(strings(model.homeJoints)); }}>机械零位</button></div>
        <div className="joints-grid">{jointDraft.map((value, i) => <label className="joint-field" key={i}><span>J{i + 1}</span><input aria-label={`起始关节 J${i + 1}`} type="number" step="any" min={model.limits[i].minDeg} max={model.limits[i].maxDeg} value={value} onChange={e => updateArray(setJointDraft, jointDraft, i, e.target.value)}/></label>)}</div>
        <button type="button" className="example-button" onClick={example}><Icon name="reset" size={14}/>示例起始姿态<Icon name="arrow" size={14}/></button>
        <p className="field-hint" role={singularHint ? 'status' : undefined} aria-label="起点提示">{singularHint || '默认使用非奇异的示例起始姿态；机械零位可单独查看，基座始终位于原点。'}</p>
        </>}
        {!genericModel && <>
        <div className="section-rule"/>
        <div className="panel-title"><div className="section-number">02</div><h2>目标点</h2><span className="frame-badge">wobj0 · tool0</span></div>
        <div className="mode-switch" aria-label="目标输入方式">{([['auto', 'position'], ['orientation', 'target']] as const).map(([id, label]) => <button type="button" key={id} aria-pressed={mode === id} className={mode === id ? 'selected' : ''} onClick={() => { invalidate(); setMode(id); }}>{label}</button>)}</div>
        <div className="field-heading"><label className="field-label">位置 <span>mm</span></label><span className="muted-label">世界坐标</span></div><div className="position-grid">{axes.map((axis, i) => <label key={axis}><span className={`axis-${axis.toLowerCase()}`}>{axis}</span><input aria-label={`目标 ${axis}`} type="number" step="any" value={positionDraft[i]} onChange={e => updateArray(setPositionDraft, positionDraft, i, e.target.value)}/></label>)}</div>
          {mode === 'auto' ? <p className="field-hint auto-hint"><Icon name="target" size={15}/><span>只需输入 XYZ，系统自动选择可达姿态。</span></p> : <><div className="field-heading"><label className="field-label">工具方向</label><select className="compact-select" aria-label="姿态表示" value={orientationFormat} onChange={e => changeOrientationFormat(e.target.value as 'quaternion' | 'euler')}><option value="quaternion">四元数</option><option value="euler">欧拉角</option></select></div>{orientationFormat === 'quaternion' ? <><div className="quaternion-grid">{orientationDraft.map((value, i) => <label key={i}><span>q{i + 1}</span><input aria-label={`四元数 q${i + 1}`} type="number" step="any" value={value} onChange={e => updateArray(setOrientationDraft, orientationDraft, i, e.target.value)}/></label>)}</div><p className="field-hint">ABB 顺序 [w, x, y, z]，q1 为标量。</p></> : <><div className="position-grid">{['Rz', 'Ry', 'Rx'].map((axis, i) => <label key={axis}><span>{axis}</span><input aria-label={`欧拉角 ${axis}`} type="number" step="any" value={eulerDraft[i]} onChange={e => updateArray(setEulerDraft, eulerDraft, i, e.target.value)}/></label>)}</div><p className="field-hint">单位 °，旋转顺序 Rz · Ry · Rx。</p></>}</>}
        <div className="speed-row"><label className="field-label" htmlFor="motion-speed">运动速度</label><select id="motion-speed" value={speed} onChange={e => { invalidate(); setSpeed(Number(e.target.value)); }}>{[10, 50, 100, 200].map(v => <option key={v} value={v}>{v} mm/s</option>)}</select></div>
        <div className="solve-row"><button className="solve-button" type="submit" disabled={busy || jogStatus.pending || jogDragging}>{busy ? <span className="spinner"/> : <Icon name="arrow" size={18}/>} {busy ? '正在求解…' : '求解 MoveL'}</button>{busy && <button type="button" className="cancel-button" onClick={() => invalidate()}>取消</button>}</div>
        <p className="solve-note">检查关节限位、姿态误差及直线采样路径。</p>
        {problem && <div className="result-error" role="alert"><Icon name="info"/><div><strong>暂时无法生成 MoveL</strong><p>{problem}</p>{result?.status === 'failure' && result.failedAt !== undefined && <small>路径进度 {numberText(result.failedAt * 100, 1)}%</small>}</div></div>}
        {success && <div className="result-success" aria-live="polite"><div className="success-heading"><span><Icon name="check" size={16}/></span><div><h3>{endpointPreview ? '已保存的路径解' : '运动学可达'}</h3><p>采样路径通过 · {success.automaticOrientation ? '自动选择方向' : '保持指定方向'}</p></div></div><div className="configuration"><span>robconf</span><code>[{success.selected.configuration.join(', ')}]</code></div><dl className="metrics"><div><dt>位置误差</dt><dd>{numberText(success.selected.positionErrorMm, 4)}<small> mm</small></dd></div><div><dt>姿态误差</dt><dd>{numberText(success.selected.orientationErrorDeg, 4)}<small> °</small></dd></div><div><dt>路径长度</dt><dd>{numberText(success.lengthMm)}<small> mm</small></dd></div><div><dt>检查采样</dt><dd>{success.path.length}<small> 点</small></dd></div></dl><div className="result-orientation"><span>目标四元数 [w, x, y, z]</span><code>{success.selected.pose.orientation.map(v => v.toFixed(5)).join(' · ')}</code></div>{success.warnings.map((warning, i) => <p className="result-warning" key={i}>{warning}</p>)}</div>}
        {allSolutions.length > 0 && <details className="details-panel endpoint-details"><summary>端点姿态与干涉检测 <span>{allSolutions.length}</span></summary><p className="field-hint">“应用”查看整机端点姿态。! 表示确认干涉，? 表示无法确定；端点检测不代表 MoveL 路径已验证。</p><div className="endpoint-list">{allSolutions.map((item, index) => <div className={`endpoint-solution ${endpointPreview?.id === item.id ? 'endpoint-applied' : ''}`} key={item.id} role="group" aria-label={`端点解 ${index + 1}`} data-endpoint-joints={JSON.stringify(item.joints)} data-collision-state={collisionInfo(item.id).state} data-collision-zone-ids={JSON.stringify(collisionZoneIds(item.id))} data-collision-uncertain-zone-ids={JSON.stringify(uncertainZoneIds(item.id))}><div className="endpoint-solution-heading"><code>[{item.configuration.join(', ')}]</code>{success?.selected.id === item.id && <small>已验证路径解</small>}<button type="button" className="copy-button endpoint-apply" aria-label={`应用姿态 ${index + 1}`} aria-pressed={endpointPreview?.id === item.id} disabled={busy || exporting || jogStatus.pending || jogDragging} onClick={() => applyEndpoint(item.id)}>应用</button></div><span className="endpoint-joints">{item.joints.map(n => numberText(n, 1)).join(' / ')}°</span>{collisionBadge(item.id)}</div>)}</div><p className="field-hint">部分解仅轴圈数不同，本体外观相同。这里只检查端点模型与危险区，未检查整段路径或自碰撞。</p></details>}
        </>}
        <details className="details-panel danger-zone-settings"><summary>干涉区域设置 <span>{zones.length}</span></summary><div className="danger-zone-list-controls"><select aria-label="当前危险区" value={selectedZoneId ?? ''} onChange={event => workspace.setSelectedZoneId(event.target.value || null)}><option value="">选择区域</option>{zones.map(zone => <option key={zone.id} value={zone.id}>{zone.name.trim() || '未命名危险区'}</option>)}</select><button type="button" className="copy-button" disabled={!workspace.canAddZone} onClick={() => { collisionRun.current?.abort(); workspace.addZone(); }}>添加危险区</button></div>{selectedZone ? <div className="danger-zone-selection" data-selected-zone-id={selectedZone.id}><label className="field-label danger-zone-name">名称<input aria-label="危险区名称" maxLength={80} value={selectedZone.name} onChange={event => editSelectedZone({ name: event.target.value })}/></label><div className="danger-zone-actions"><label className="danger-zone-toggle"><input type="checkbox" aria-label="显示干涉区域" checked={selectedZone.enabled} onChange={event => editSelectedZone({ enabled: event.target.checked })}/>{genericModel ? '显示区域' : '显示并检测'}</label><button type="button" className="text-button" onClick={() => { collisionRun.current?.abort(); workspace.removeSelectedZone(); }}>删除危险区</button></div><label className="field-label">世界坐标中心 <span>mm</span></label><div className="position-grid">{axes.map((axis, i) => <label key={axis}><span className={`axis-${axis.toLowerCase()}`}>{axis}</span><input type="number" aria-label={`危险区中心 ${axis}`} min={-1e6} max={1e6} step="any" value={selectedZone.center[i]} onChange={event => editZoneVector('center', i, event.target.value)}/></label>)}</div><label className="field-label danger-zone-size-label">区域尺寸 <span>mm</span></label><div className="position-grid">{axes.map((axis, i) => <label key={axis}><span>{axis}</span><input type="number" aria-label={`危险区尺寸 ${axis}`} min={0.001} max={1e6} step="any" value={selectedZone.size[i]} onChange={event => editZoneVector('size', i, event.target.value)}/></label>)}</div><button type="button" className="text-button danger-zone-reset" onClick={() => { collisionRun.current?.abort(); workspace.resetSelectedZone(); }}>恢复默认区域</button></div> : <p className="field-hint danger-zone-empty">选择一个危险区进行编辑，或添加新区域。</p>}{zoneConfiguration.errors.map(error => <p key={error.id} className="danger-zone-error" role="alert">{error.message} {genericModel ? '无效区域未显示。' : '所有端点暂未完成干涉检测。'}</p>)}<p className="field-hint">{genericModel ? '最多 64 个区域。当前型号仅显示区域，不执行机器人干涉检查。' : '最多 64 个区域。按机器人实际模型检测相交、接触和包含；表面缺口影响内部判断时标记无法确定。未检查整段路径或自碰撞。'}</p></details>
        {!genericModel && <details className="details-panel dh-details"><summary>DH 参数与关节范围<Icon name="info" size={14}/></summary><p className="field-hint">标准 DH · 长度 mm · 角度 deg</p><div className="table-scroll"><table><thead><tr><th>轴</th><th>a</th><th>d</th><th>α</th><th>θ 偏置</th></tr></thead><tbody>{model.dh.map((row, i) => <tr key={i}><th>J{i + 1}</th><td>{row.aMm}</td><td>{row.dMm}</td><td>{row.alphaDeg}°</td><td>{row.thetaOffsetDeg}°</td></tr>)}</tbody></table></div><div className="joint-limits">{model.limits.map((limit, i) => <span key={i}>J{i + 1} <strong>{limit.minDeg}° ~ {limit.maxDeg}°</strong></span>)}</div></details>}

      </fieldset></form></aside>

      <section className="code-panel" aria-label={genericModel ? '程序输出' : 'RAPID 输出'} aria-hidden={activeView !== 'program'} inert={activeView !== 'program'}>{genericModel ? <><div className="code-heading"><div><Icon name="code" size={18}/><h2>程序输出</h2></div></div><div className="code-empty"><div className="generic-model-notice" role="status"><strong>{activeName} · 控制器程序尚未适配</strong>当前型号支持模型查看与 URDF 关节调整。适配对应的运动求解、配置规则和控制器后处理后，才能生成程序文件。</div></div><div className="code-footer"><span>当前型号没有可导出的运动程序</span><span>{activeBrand}</span></div></> : <><div className="code-heading"><div><Icon name="code" size={18}/><h2>RAPID 指令</h2><span className="code-language">MoveL</span></div><button className="copy-button" disabled={!code || busy} onClick={copy}><Icon name={copyState === '已复制' ? 'check' : 'copy'} size={14}/>{copyState || '复制 MoveL'}</button></div>{code ? <textarea className="rapid-code" aria-label="RAPID MoveL 指令" spellCheck={false} readOnly value={code}/> : <div className="code-empty"><span className="line-number">01</span><code><span>//</span> {endpointPreview ? '端点预览没有已验证的运动指令。请返回场景，恢复已验证路径。' : '设置起始姿态与目标，求解后在这里查看 MoveL 指令。'}</code></div>}<div className="module-export-controls" style={{ padding: '12px 18px', display: 'flex', gap: 12, alignItems: 'center', flexWrap: 'wrap', borderTop: '1px solid #eceef1' }}><label style={{ display: 'flex', alignItems: 'center', gap: 8 }}>模块名称<input aria-label="程序模块名称" value={moduleName} maxLength={32} disabled={exporting} onChange={event => { setModuleName(event.target.value); setExportMessage(''); }} style={{ width: 155, height: 30, border: '1px solid #e2e5e9', borderRadius: 5, padding: '0 8px' }}/></label><select aria-label="程序控制器" defaultValue="abb_rapid" style={{ width: 180, height: 30 }} disabled={exporting}><option value="abb_rapid">ABB · RAPID (.mod)</option><option disabled>FANUC · LS（待适配）</option><option disabled>KUKA · KRL（待适配）</option><option disabled>安川 · INFORM（待适配）</option></select><button className="copy-button" onClick={exportModule} disabled={!nativeDesktop || !success || Boolean(endpointPreview) || busy || exporting}>{exporting ? '正在导出…' : '导出 .mod 模块'}</button><span style={{ width: '100%', fontSize: 10, color: '#7c8592', overflowWrap: 'anywhere' }} role="status">{exportMessage || (nativeDesktop ? '模块会检查实际起始关节；不匹配时停止，不自动移动到起点。' : '浏览器版用于参考与回归比较；完整模块请使用桌面客户端导出。')}</span></div><div className="code-footer"><span><span className={`result-dot ${success && !endpointPreview ? 'ready' : ''}`}/>{endpointPreview ? '端点姿态预览中，未启用运动指令' : success ? '已生成当前目标的运动指令' : busy ? '正在检查可达性与路径连续性' : '等待目标求解'}</span><span>ABB RAPID · tool0 · wobj0</span></div></>}</section>
    </main>
    {catalogOpen && <RobotCatalogDialog currentModelId={activeModelId} onSelect={selectCatalogModel} onClose={() => setCatalogOpen(false)}/>}
    <footer className="app-footer"><span><Icon name="info" size={13}/>{genericModel ? '源 URDF 模型预览，运动规划、干涉检查与控制器程序尚未适配。' : '端点模型干涉检测，未做整段碰撞、自碰撞及控制器动态验证。'}</span><span>世界坐标 Z 向上 · 长度 mm · 角度 deg</span></footer>
  </div>;
}
