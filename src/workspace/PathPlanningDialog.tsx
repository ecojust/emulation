import { useEffect, useRef, useState } from 'react';
import Icon from '../ui/Icon';
import { normalizeQuaternion, quaternionDistanceDeg } from '../robot/kinematics';
import type { Pose, Quaternion, Vec3 } from '../robot/types';
import type { GrindingInput, GrindingResult, GrindingSuccess } from '../robot/grinding-types';
import './path-planning.css';

export function parseGrindingTargets(text: string): Pose[] {
  const rows = text.split(/\r?\n/).map(line => line.trim()).filter(line => line && !line.startsWith('#'));
  if (rows.length < 2 || rows.length > 64) throw new Error('请输入 2 至 64 个目标点，每行包含位置和姿态。');
  return rows.map((line, index) => {
    const tokens = line.split(/[\s,，;；]+/);
    const values = tokens.map(token => token === '' ? NaN : Number(token));
    if (values.length !== 7 || !values.every(Number.isFinite) || values.slice(0, 3).some(value => Math.abs(value) > 1e6)) {
      throw new Error(`第 ${index + 1} 行需要 7 个有限数值：X Y Z qw qx qy qz；坐标须在 ±1000000 mm 内。`);
    }
    const raw = values.slice(3) as Quaternion;
    if (Math.hypot(...raw) < 1e-10) throw new Error(`第 ${index + 1} 行的四元数不能全为零。`);
    return { position: values.slice(0, 3) as Vec3, orientation: normalizeQuaternion(raw) };
  });
}
export function formatGrindingTargets(targets: readonly Pose[]): string {
  return targets.map(pose => [...pose.position, ...pose.orientation].map(value => Number(value.toFixed(9))).join(' ')).join('\n');
}
export interface PathPlanningDialogProps {
  pathName: string;
  modelName: string;
  initialTargets: Pose[];
  contextKey: string;
  onPlan: (input: GrindingInput, signal: AbortSignal) => Promise<GrindingResult>;
  onPreview: (result: GrindingSuccess | null) => void;
  onApply: (result: GrindingSuccess, input: GrindingInput) => void;
  onClose: () => void;
}
export default function PathPlanningDialog({ pathName, modelName, initialTargets, contextKey, onPlan, onPreview, onApply, onClose }: PathPlanningDialogProps) {
  const dialog = useRef<HTMLDialogElement>(null), active = useRef<AbortController | null>(null);
  const callbacks = useRef({ onPreview, onClose }); callbacks.current = { onPreview, onClose };
  const mounted = useRef(true), context = useRef(contextKey), previousContext = useRef(contextKey); context.current = contextKey;
  const [targets, setTargets] = useState(() => formatGrindingTargets(initialTargets));
  const [orientation, setOrientation] = useState(() => (initialTargets[0]?.orientation ?? [1, 0, 0, 0]).map(String));
  const [directionMode, setDirectionMode] = useState<'first' | 'custom'>('first');
  const [speed, setSpeed] = useState('50'), [clearance, setClearance] = useState('5');
  const [busy, setBusy] = useState(false), [message, setMessage] = useState('');
  const [result, setResult] = useState<{ value: GrindingSuccess; input: GrindingInput; context: string } | null>(null);
  const [failed, setFailed] = useState(false), [previewMode, setPreviewMode] = useState(false);
  function invalidate() {
    active.current?.abort(); active.current = null; setBusy(false); setResult(null); setFailed(false); setMessage('');
    callbacks.current.onPreview(null);
  }
  function close() { invalidate(); onClose(); }
  function changeTargets(text: string) {
    invalidate(); setTargets(text);
    if (directionMode === 'first') {
      try { setOrientation(parseGrindingTargets(text)[0].orientation.map(String)); }
      catch { /* Keep partially entered text; generation will report invalid rows. */ }
    }
  }
  useEffect(() => {
    mounted.current = true;
    return () => { mounted.current = false; active.current?.abort(); active.current = null; callbacks.current.onPreview(null); };
  }, []);
  useEffect(() => {
    const panel = dialog.current;
    if (!panel) return;
    // Native modal dialogs make the scene inert. Preview intentionally leaves
    // the top layer so the original canvas can receive orbit and wheel events.
    if (panel.open) panel.close();
    if (previewMode) panel.show(); else panel.showModal();
    const focusTarget = previewMode
      ? panel.querySelector<HTMLButtonElement>('[data-return-to-edit]')
      : panel.querySelector<HTMLTextAreaElement>('[aria-label="目标点列表"]');
    focusTarget?.focus({ preventScroll: true });
  }, [previewMode]);
  useEffect(() => {
    active.current?.abort(); active.current = null; setBusy(false); setResult(null); setFailed(false);
    callbacks.current.onPreview(null);
    if (previousContext.current !== contextKey) setMessage('场景或路径已改变，请重新生成路径。');
    previousContext.current = contextKey;
  }, [contextKey]);
  let directionDifference = 0;
  try {
    const poses = parseGrindingTargets(targets), raw = orientation.map(value => value.trim() ? Number(value) : NaN) as Quaternion;
    if (raw.every(Number.isFinite) && Math.hypot(...raw) > 1e-10) directionDifference = Math.max(...poses.map(pose => quaternionDistanceDeg(pose.orientation, normalizeQuaternion(raw))));
  } catch { /* Display actionable validation only when the user generates. */ }
  async function generate() {
    invalidate();
    let operation: AbortController | null = null;
    try {
      const poses = parseGrindingTargets(targets);
      const raw = directionMode === 'first' ? poses[0].orientation : orientation.map(value => value.trim() ? Number(value) : NaN) as Quaternion;
      if (!raw.every(Number.isFinite) || Math.hypot(...raw) < 1e-10) throw new Error('统一工具方向需要有效的非零四元数。');
      const speedMmPerSecond = speed.trim() ? Number(speed) : NaN, clearanceMm = clearance.trim() ? Number(clearance) : NaN;
      if (!Number.isFinite(speedMmPerSecond) || speedMmPerSecond < 1 || speedMmPerSecond > 2000) throw new Error('打磨速度须在 1 至 2000 mm/s 内。');
      if (!Number.isFinite(clearanceMm) || clearanceMm < 0 || clearanceMm > 1000) throw new Error('最小安全间距须在 0 至 1000 mm 内。');
      const input: GrindingInput = { targets: poses, orientation: normalizeQuaternion(raw), speedMmPerSecond, clearanceMm };
      const controller = new AbortController(), startedContext = context.current;
      operation = controller; active.current = controller; setBusy(true); setMessage('正在检查可达性、关节连续性和整条路径的禁入区间距…');
      const planned = await onPlan(input, controller.signal);
      if (!mounted.current || active.current !== controller || controller.signal.aborted || context.current !== startedContext) return;
      active.current = null; setBusy(false);
      if (planned.status !== 'success') { setFailed(true); setMessage(planned.message); return; }
      setResult({ value: planned, input, context: startedContext });
      setMessage(`已生成 ${planned.waypoints.length} 个路径点，长度 ${planned.lengthMm.toFixed(1)} mm。`);
      onPreview(planned);
    } catch (error) {
      if (!mounted.current || (operation && active.current !== operation) || (error instanceof DOMException && error.name === 'AbortError')) return;
      active.current = null; setBusy(false); setFailed(true); setMessage(error instanceof Error ? error.message : '路径规划失败。');
    }
  }
  const validResult = result?.context === contextKey ? result : null;
  const feedback = message && <section className={'planning-result' + (failed ? ' planning-error' : '')} role="status" aria-label="路径规划结果" data-grinding-status={busy ? 'planning' : failed ? 'failure' : validResult ? 'success' : 'idle'} data-grinding-points={validResult?.value.waypoints.length ?? 0}><p>{message}</p>{validResult && <><p>已通过禁入区间距检查 · 预计 {validResult.value.durationSeconds.toFixed(2)} s</p>{validResult.value.warnings.map((warning, index) => <p key={index}>{warning}</p>)}</>}</section>;
  return <dialog ref={dialog} className={'path-planning-dialog' + (previewMode ? ' is-preview' : '')}
    aria-label="打磨路径规划" aria-modal={!previewMode} data-planning-mode={previewMode ? 'preview' : 'edit'}
    onCancel={event => { event.preventDefault(); close(); }}
    onKeyDown={event => { if (previewMode && event.key === 'Escape') { event.preventDefault(); close(); } }}>

    <header><div><h2>打磨路径规划</h2><p>{pathName} · {modelName}</p></div><button type="button" aria-label="关闭路径规划" onClick={close}><Icon name="close"/></button></header>
    <div className="path-planning-body">
      {previewMode ? <>
        <p>拖动画布旋转视角，滚轮缩放。预览只显示计算结果，原机器人姿态和路径保持不变。</p>
        {feedback}
        {!validResult && <p>场景已改变，请返回编辑后重新生成。</p>}
      </> : <>
      <p>所有加工点使用同一个工具方向；保持关节解连续，避免打磨头突然翻转。当前位置到首点的接近运动会单独检查。</p>
      <label className="field-label">目标点列表 <small>世界坐标 mm · 四元数 [w, x, y, z]</small><textarea aria-label="目标点列表" spellCheck={false} value={targets} onChange={event => changeTargets(event.target.value)} placeholder={'X Y Z qw qx qy qz\n每行一个目标点，至少两行'}/></label>
      <p className="field-hint">每行：X Y Z qw qx qy qz，支持空格或逗号分隔。输入姿态用于选择和核对统一方向，规划保留每个目标点的位置。</p>
      <section><div className="field-heading"><h3>统一工具方向</h3><button type="button" className="text-button" onClick={() => { try { const parsed = parseGrindingTargets(targets); invalidate(); setDirectionMode('first'); setOrientation(parsed[0].orientation.map(String)); } catch (error) { setMessage((error as Error).message); setFailed(true); } }}>使用首点方向</button></div>
        <div className="planning-orientation">{['qw', 'qx', 'qy', 'qz'].map((name, index) => <label key={name}>{name}<input type="number" step="any" aria-label={'统一工具方向 ' + name} value={orientation[index]} onChange={event => { invalidate(); setDirectionMode('custom'); setOrientation(values => values.map((value, axis) => axis === index ? event.target.value : value)); }}/></label>)}</div>
        {directionDifference > 0.01 && <p className="field-hint" role="status">输入点方向与统一方向最大相差 {directionDifference.toFixed(2)}°。生成时加工段统一采用上方方向。</p>}
      </section>
      <div className="planning-options"><label>打磨速度 <small>mm/s</small><input type="number" min="1" max="2000" aria-label="打磨速度" value={speed} onChange={event => { invalidate(); setSpeed(event.target.value); }}/></label><label>最小安全间距 <small>mm</small><input type="number" min="0" max="1000" step="any" aria-label="最小安全间距" value={clearance} onChange={event => { invalidate(); setClearance(event.target.value); }}/></label></div>
      <p className="field-hint">检查机器人、工具包络与已启用禁入区的整段运动；包络重叠而无法证明间距时拒绝生成。路径中的绕行点为转移段。场景几何检查不代表控制器或实机验收。</p>
      {feedback}
      </>}
    </div>
    <footer><div>{previewMode ? <button type="button" data-return-to-edit onClick={() => setPreviewMode(false)}>返回编辑</button> : busy ? <button type="button" onClick={() => { invalidate(); setMessage('已取消计算，原路径保持不变。'); }}>取消计算</button> : <button type="button" onClick={() => void generate()}><Icon name="route" size={15}/>生成路径</button>}</div>{!previewMode && <button type="button" disabled={!validResult || busy} onClick={() => { if (validResult) { onPreview(validResult.value); setPreviewMode(true); } }}>预览路径</button>}<button type="button" disabled={!validResult || busy} onClick={() => { if (validResult) { try { onApply(validResult.value, validResult.input); close(); } catch (error) { invalidate(); setFailed(true); setMessage(error instanceof Error ? error.message : '应用路径失败。'); } } }}>应用到路径</button></footer>
  </dialog>;
}
