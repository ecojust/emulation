import { useEffect, useMemo, useRef, useState, type FormEvent, type ReactNode } from 'react';
import RobotScene from './scene/RobotScene';
import { copyRapid, nativeDesktop, saveProgram, solveMotion } from './robot/client';
import { isNativeUiSmoke, runNativeUiSmoke } from './robot/ui-smoke';
import { forwardKinematics } from './robot/kinematics';
import { getRobotModel, ROBOT_MODELS } from './robot/model';
import { eulerZYXToQuaternion, formatMoveL, normalizeOrientation, parseRobtarget } from './robot/rapid';
import type { JointAngles, PlanRequest, PlanResult, Pose, Quaternion, RobotId, Vec3 } from './robot/types';
import './styles.css';

type Mode = 'auto' | 'orientation' | 'robtarget';
type View = 'iso' | 'front' | 'top';
const axes = ['X', 'Y', 'Z'] as const;
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
  const [jointDraft, setJointDraft] = useState<string[]>(strings(model.homeJoints));
  const [positionDraft, setPositionDraft] = useState<string[]>(() => initialTarget(modelId));
  const [orientationDraft, setOrientationDraft] = useState<string[]>(['0.707107', '0', '0.707107', '0']);
  const [eulerDraft, setEulerDraft] = useState<string[]>(['0', '90', '0']);
  const [orientationFormat, setOrientationFormat] = useState<'quaternion' | 'euler'>('quaternion');
  const [robtargetDraft, setRobtargetDraft] = useState('');
  const [mode, setMode] = useState<Mode>('auto');
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
  const exportRequest = useRef<AbortController | null>(null);
  const currentRequest = useRef<AbortController | null>(null);
  const copyTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const success = result?.status === 'success' ? result : null;
  const startJoints = useMemo(() => {
    try {
      const values = finite(jointDraft, '起始关节');
      if (values.some((v, i) => v < model.limits[i].minDeg || v > model.limits[i].maxDeg)) return model.homeJoints;
      return values as JointAngles;
    } catch { return model.homeJoints; }
  }, [jointDraft, model]);
  const displayJoints = useMemo(() => {
    if (!success?.path.length) return startJoints;
    const samples = success.path;
    if (progress <= 0) return samples[0].joints;
    if (progress >= 1) return samples[samples.length - 1].joints;
    const next = samples.findIndex(sample => sample.t >= progress);
    const before = samples[Math.max(0, next - 1)], after = samples[next < 0 ? samples.length - 1 : next];
    const t = after.t === before.t ? 0 : (progress - before.t) / (after.t - before.t);
    return before.joints.map((v, i) => v + (after.joints[i] - v) * t) as JointAngles;
  }, [success, progress, startJoints]);
  const currentPose = useMemo(() => forwardKinematics(model, displayJoints).pose, [model, displayJoints]);
  const code = success ? formatMoveL(success.selected.pose, success.selected.configuration, speed) : '';

  function invalidate() {
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
    invalidate(); setModelId(id); setJointDraft(strings(getRobotModel(id).homeJoints));
    setPositionDraft(initialTarget(id)); setMode('auto'); setViewRevision(v => v + 1);
  }
  function example() {
    invalidate(); setJointDraft(strings(model.readyJoints)); setPositionDraft(initialTarget(modelId)); setMode('auto');
    setOrientationDraft(strings(forwardKinematics(model, model.readyJoints).pose.orientation));
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
    if (mode === 'robtarget') {
      const parsed = parseRobtarget(robtargetDraft);
      return parsed.pose;
    }
    const position = finite(positionDraft, '目标位置') as Vec3;
    if (mode === 'auto') return { position, orientation: null };
    const orientation = orientationFormat === 'quaternion'
      ? normalizeOrientation(finite(orientationDraft, '四元数') as Quaternion)
      : eulerZYXToQuaternion(finite(eulerDraft, '欧拉角') as Vec3);
    return { position, orientation };
  }
  async function solve(event: FormEvent) {
    event.preventDefault(); invalidate();
    let target: ReturnType<typeof targetInput>;
    let joints: JointAngles;
    try {
      target = targetInput(); joints = finite(jointDraft, '起始关节') as JointAngles;
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
  async function copy() {
    if (!code || busy) return;
    try {
      await copyRapid(code); setCopyState('已复制');
      if (copyTimer.current) clearTimeout(copyTimer.current);
      copyTimer.current = setTimeout(() => setCopyState(''), 2000);
    } catch { setCopyState('请选中代码手动复制'); }
  }
  async function exportModule() {
    if (!validatedRequest || !success || exporting) return;
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
    void runNativeUiSmoke(request, result => { setJointDraft(strings(model.readyJoints)); setResult(result); setValidatedRequest(request); setProgress(1); });
    // This runs only for the installed-app CI smoke flag, once per WebView.
  }, []);
  useEffect(() => () => { exportRequest.current?.abort(); currentRequest.current?.abort(); if (copyTimer.current) clearTimeout(copyTimer.current); }, []);
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

  let previewTarget: Pose | null = success?.selected.pose ?? null;
  if (!previewTarget && mode !== 'auto') {
    try { const input = targetInput(); if (input.orientation) previewTarget = { position: input.position, orientation: input.orientation }; } catch { /* Invalid draft has no preview frame. */ }
  }
  const problem = error || (result?.status === 'failure' ? result.message : '');

  return <div className="app-shell">
    <header className="topbar">
      <a className="brand" href="./" aria-label="Emulation 首页"><span className="brand-symbol"><Icon name="cube" size={25} /></span><span>emulation<span className="brand-caption">机器人运动仿真</span></span></a>
      <div className="header-divider"/><div className="project-label">运动工作台<span>/</span><strong>IRB 1100</strong></div>
      <div className="header-right"><span className="local-badge"><i/>{nativeDesktop ? 'Rust 原生引擎' : '浏览器参考引擎'}</span><span className="version">v0.2.0</span></div>
    </header>
    <main className="workspace">
      <section className="scene-panel" aria-label="机器人视口">
        <div className="scene-heading"><div><span className="eyebrow">ROBOT WORKSPACE</span><h1>从目标，到运动<span className="heading-dot">.</span></h1><p>在三维空间检查位置、姿态与直线运动。</p></div><span className="robot-tag"><span className="abb-mark">ABB</span>IRB 1100<span className="tag-divider"/>6 轴</span></div>
        <div className="viewport">
          <RobotScene model={model} joints={displayJoints} targetPose={previewTarget} path={success?.path ?? []} endJoints={success?.selected.joints} showFrames={showFrames} view={view} viewRevision={viewRevision}/>
          <div className="viewport-toolbar"><div className="view-options" aria-label="观察视角">{([['iso', '透视'], ['front', '正视'], ['top', '俯视']] as const).map(([id, label]) => <button key={id} className={view === id ? 'selected' : ''} onClick={() => { setView(id); setViewRevision(v => v + 1); }}>{label}</button>)}</div><button className="icon-button" title="重置视角" aria-label="重置视角" onClick={() => { setView('iso'); setViewRevision(v => v + 1); }}><Icon name="reset" size={15}/></button></div>
          <div className="viewport-caption"><span className="axis-dot"/>基座位于世界原点 <code>[0, 0, 0]</code></div>
          <div className="scene-legend"><span><i className="legend-current"/>当前 TCP</span>{success && <span><i className="legend-target"/>目标姿态</span>}{success && <span><i className="legend-path"/>MoveL 路径</span>}</div>
        </div>
        <div className="playback"><button className="play-button" aria-label={playing ? '暂停播放' : '播放路径'} disabled={!success || busy} onClick={() => { if (!playing && progress >= 1) setProgress(0); setPlaying(v => !v); }}><Icon name={playing ? 'pause' : 'play'} size={15}/></button><div className="playback-track"><input aria-label="路径进度" type="range" min="0" max="100" step="0.1" value={progress * 100} disabled={!success || busy} onChange={e => { setPlaying(false); setProgress(Number(e.target.value) / 100); }}/></div><span className="progress-value">{Math.round(progress * 100)}<small>%</small></span><span className="playback-separator"/><label className="frame-toggle"><input type="checkbox" checked={showFrames} onChange={e => setShowFrames(e.target.checked)}/>显示关节坐标系</label></div>
        <div className="pose-strip"><section className="tcp-readout" aria-label="当前 TCP"><span className="strip-title">当前 TCP <small>mm</small></span><div>{axes.map((axis, i) => <span key={axis} data-axis={axis}><i className={`axis-${axis.toLowerCase()}`}>{axis}</i><strong>{currentPose.position[i].toFixed(2)}</strong></span>)}</div></section><div className="joint-readout" aria-label="当前关节姿态"><span className="strip-title">关节姿态 <small>deg</small></span><div>{displayJoints.map((value, i) => <span key={i}><small>J{i + 1}</small><strong>{value.toFixed(1)}°</strong></span>)}</div></div></div>
      </section>

      <aside className="control-panel"><form onSubmit={solve} noValidate><fieldset disabled={exporting} style={{ border: 0, padding: 0, margin: 0, minWidth: 0 }}>
        <div className="panel-title"><div className="section-number">01</div><h2>机器人与起点</h2></div>
        <label className="field-label" htmlFor="robot-model">机器人型号</label>
        <select id="robot-model" aria-label="机器人型号" value={modelId} onChange={e => changeModel(e.target.value as RobotId)}>{Object.values(ROBOT_MODELS).map(item => <option key={item.id} value={item.id}>{item.name}</option>)}</select>
        <div className="spec-line"><span>额定负载 <strong>{model.payloadKg} kg</strong></span><span>工作半径 <strong>{model.reachMm} mm</strong></span></div>
        <div className="field-heading"><label className="field-label">起始关节 <span>deg</span></label><button type="button" className="text-button" onClick={() => { invalidate(); setJointDraft(strings(model.homeJoints)); }}>机械零位</button></div>
        <div className="joints-grid">{jointDraft.map((value, i) => <label className="joint-field" key={i}><span>J{i + 1}</span><input aria-label={`起始关节 J${i + 1}`} type="number" step="any" min={model.limits[i].minDeg} max={model.limits[i].maxDeg} value={value} onChange={e => updateArray(setJointDraft, jointDraft, i, e.target.value)}/></label>)}</div>
        <button type="button" className="example-button" onClick={example}><Icon name="reset" size={14}/>示例起始姿态<Icon name="arrow" size={14}/></button>
        <p className="field-hint">机械零位的腕部处于奇异位置。可使用示例姿态开始规划。</p>
        <div className="section-rule"/>
        <div className="panel-title"><div className="section-number">02</div><h2>目标点</h2><span className="frame-badge">wobj0 · tool0</span></div>
        <div className="mode-switch" aria-label="目标输入方式">{([['auto', '自动姿态'], ['orientation', '指定姿态'], ['robtarget', 'ABB robtarget']] as const).map(([id, label]) => <button type="button" key={id} aria-pressed={mode === id} className={mode === id ? 'selected' : ''} onClick={() => { invalidate(); setMode(id); }}>{label}</button>)}</div>
        {mode !== 'robtarget' ? <><div className="field-heading"><label className="field-label">位置 <span>mm</span></label><span className="muted-label">世界坐标</span></div><div className="position-grid">{axes.map((axis, i) => <label key={axis}><span className={`axis-${axis.toLowerCase()}`}>{axis}</span><input aria-label={`目标 ${axis}`} type="number" step="any" value={positionDraft[i]} onChange={e => updateArray(setPositionDraft, positionDraft, i, e.target.value)}/></label>)}</div>
          {mode === 'auto' ? <p className="field-hint auto-hint"><Icon name="target" size={15}/><span>按目标位置寻找可达方向，优先保持起始姿态。</span></p> : <><div className="field-heading"><label className="field-label">工具方向</label><select className="compact-select" aria-label="姿态表示" value={orientationFormat} onChange={e => changeOrientationFormat(e.target.value as 'quaternion' | 'euler')}><option value="quaternion">四元数</option><option value="euler">欧拉角</option></select></div>{orientationFormat === 'quaternion' ? <><div className="quaternion-grid">{orientationDraft.map((value, i) => <label key={i}><span>q{i + 1}</span><input aria-label={`四元数 q${i + 1}`} type="number" step="any" value={value} onChange={e => updateArray(setOrientationDraft, orientationDraft, i, e.target.value)}/></label>)}</div><p className="field-hint">ABB 顺序 [w, x, y, z]，q1 为标量。</p></> : <><div className="position-grid">{['Rz', 'Ry', 'Rx'].map((axis, i) => <label key={axis}><span>{axis}</span><input aria-label={`欧拉角 ${axis}`} type="number" step="any" value={eulerDraft[i]} onChange={e => updateArray(setEulerDraft, eulerDraft, i, e.target.value)}/></label>)}</div><p className="field-hint">单位 °，旋转顺序 Rz · Ry · Rx。</p></>}</>}
        </> : <><label className="field-label" htmlFor="robtarget-input">完整 robtarget</label><textarea id="robtarget-input" aria-label="ABB robtarget" className="robtarget-input" value={robtargetDraft} spellCheck={false} placeholder="[[x,y,z],[q1,q2,q3,q4],[cf1,cf4,cf6,cfx],[9E9,9E9,9E9,9E9,9E9,9E9]]" onChange={e => { invalidate(); setRobtargetDraft(e.target.value); }}/><p className="field-hint">读取位置与方向，重新选择可达 robconf。当前无外部轴。</p></>}
        <div className="speed-row"><label className="field-label" htmlFor="motion-speed">运动速度</label><select id="motion-speed" value={speed} onChange={e => { invalidate(); setSpeed(Number(e.target.value)); }}>{[10, 50, 100, 200].map(v => <option key={v} value={v}>{v} mm/s</option>)}</select></div>
        <div className="solve-row"><button className="solve-button" type="submit" disabled={busy}>{busy ? <span className="spinner"/> : <Icon name="arrow" size={18}/>} {busy ? '正在求解…' : '求解 MoveL'}</button>{busy && <button type="button" className="cancel-button" onClick={invalidate}>取消</button>}</div>
        <p className="solve-note">检查关节限位、姿态误差及直线采样路径。</p>
        {problem && <div className="result-error" role="alert"><Icon name="info"/><div><strong>暂时无法生成 MoveL</strong><p>{problem}</p>{result?.status === 'failure' && result.failedAt !== undefined && <small>路径进度 {numberText(result.failedAt * 100, 1)}%</small>}</div></div>}
        {success && <div className="result-success" aria-live="polite"><div className="success-heading"><span><Icon name="check" size={16}/></span><div><h3>运动学可达</h3><p>采样路径通过 · {success.automaticOrientation ? '自动选择方向' : '保持指定方向'}</p></div></div><div className="configuration"><span>robconf</span><code>[{success.selected.configuration.join(', ')}]</code></div><dl className="metrics"><div><dt>位置误差</dt><dd>{numberText(success.selected.positionErrorMm, 4)}<small> mm</small></dd></div><div><dt>姿态误差</dt><dd>{numberText(success.selected.orientationErrorDeg, 4)}<small> °</small></dd></div><div><dt>路径长度</dt><dd>{numberText(success.lengthMm)}<small> mm</small></dd></div><div><dt>检查采样</dt><dd>{success.path.length}<small> 点</small></dd></div></dl><div className="result-orientation"><span>目标四元数 [w, x, y, z]</span><code>{success.selected.pose.orientation.map(v => v.toFixed(5)).join(' · ')}</code></div>{success.warnings.map((warning, i) => <p className="result-warning" key={i}>{warning}</p>)}</div>}
        {success && success.solutions.length > 1 && <details className="details-panel"><summary>其他端点解 <span>{success.solutions.length - 1}</span></summary><p className="field-hint">仅为端点逆解参考，未作为本次直线路径验证。</p>{success.solutions.filter(item => item.id !== success.selected.id).slice(0, 12).map(item => <div className="endpoint-solution" key={item.id}><code>[{item.configuration.join(', ')}]</code><span>{item.joints.map(n => numberText(n, 1)).join(' / ')}°</span></div>)}</details>}
        <details className="details-panel dh-details"><summary>DH 参数与关节范围<Icon name="info" size={14}/></summary><p className="field-hint">标准 DH · 长度 mm · 角度 deg</p><div className="table-scroll"><table><thead><tr><th>轴</th><th>a</th><th>d</th><th>α</th><th>θ 偏置</th></tr></thead><tbody>{model.dh.map((row, i) => <tr key={i}><th>J{i + 1}</th><td>{row.aMm}</td><td>{row.dMm}</td><td>{row.alphaDeg}°</td><td>{row.thetaOffsetDeg}°</td></tr>)}</tbody></table></div><div className="joint-limits">{model.limits.map((limit, i) => <span key={i}>J{i + 1} <strong>{limit.minDeg}° ~ {limit.maxDeg}°</strong></span>)}</div></details>
      </fieldset></form></aside>

      <section className="code-panel" aria-label="RAPID 输出"><div className="code-heading"><div><Icon name="code" size={18}/><h2>RAPID 指令</h2><span className="code-language">MoveL</span></div><button className="copy-button" disabled={!code || busy} onClick={copy}><Icon name={copyState === '已复制' ? 'check' : 'copy'} size={14}/>{copyState || '复制 MoveL'}</button></div>{code ? <textarea className="rapid-code" aria-label="RAPID MoveL 指令" spellCheck={false} readOnly value={code}/> : <div className="code-empty"><span className="line-number">01</span><code><span>//</span> 设置起始姿态与目标，求解后在这里查看 MoveL 指令。</code></div>}<div className="module-export-controls" style={{ padding: '12px 18px', display: 'flex', gap: 12, alignItems: 'center', flexWrap: 'wrap', borderTop: '1px solid #eceef1' }}><label style={{ display: 'flex', alignItems: 'center', gap: 8 }}>模块名称<input aria-label="程序模块名称" value={moduleName} maxLength={32} disabled={exporting} onChange={event => { setModuleName(event.target.value); setExportMessage(''); }} style={{ width: 155, height: 30, border: '1px solid #e2e5e9', borderRadius: 5, padding: '0 8px' }}/></label><select aria-label="程序控制器" defaultValue="abb_rapid" style={{ width: 180, height: 30 }} disabled={exporting}><option value="abb_rapid">ABB · RAPID (.mod)</option><option disabled>FANUC · LS（待适配）</option><option disabled>KUKA · KRL（待适配）</option><option disabled>安川 · INFORM（待适配）</option></select><button className="copy-button" onClick={exportModule} disabled={!nativeDesktop || !success || busy || exporting}>{exporting ? '正在导出…' : '导出 .mod 模块'}</button><span style={{ width: '100%', fontSize: 10, color: '#7c8592', overflowWrap: 'anywhere' }} role="status">{exportMessage || (nativeDesktop ? '模块会检查实际起始关节；不匹配时停止，不自动移动到起点。' : '浏览器版用于参考与回归比较；完整模块请使用桌面客户端导出。')}</span></div><div className="code-footer"><span><span className={`result-dot ${success ? 'ready' : ''}`}/>{success ? '已生成当前目标的运动指令' : busy ? '正在检查可达性与路径连续性' : '等待目标求解'}</span><span>ABB RAPID · tool0 · wobj0</span></div></section>
    </main>
    <footer className="app-footer"><span><Icon name="info" size={13}/>标称参数仿真，未做碰撞与控制器动态验证。</span><span>世界坐标 Z 向上 · 长度 mm · 角度 deg</span></footer>
  </div>;
}
