import { useEffect, useRef, useState } from 'react';
import Icon from '../ui/Icon';
import type { SurfaceUvSettings } from '../workspace/surface-path';
import './surface.css';

export interface SurfaceUvInput extends SurfaceUvSettings { maxNormalStepDeg: number }
export interface SurfaceUvSummary {
  pointCount: number;
  rowCount: number;
  lengthMm?: number;
  maxSpacingErrorMm?: number;
  maxRegularSpacingErrorMm?: number;
  maxOrientationStepDeg?: number;
  warnings?: readonly string[];
}
export interface SurfaceUvDialogProps<T> {
  pathName: string;
  contextKey: string;
  controlCount: number;
  initialSettings?: Partial<SurfaceUvInput>;
  onGenerate: (settings: SurfaceUvInput, signal: AbortSignal) => Promise<T>;
  summarize: (result: T) => SurfaceUvSummary;
  onPreview: (result: T | null) => void;
  onApply: (result: T, settings: SurfaceUvInput) => void | Promise<void>;
  onClose: () => void;
}

/** Generated data belongs to one scene/path revision and is committed only by explicit confirmation. */
export default function SurfaceUvDialog<T>({ pathName, contextKey, controlCount, initialSettings, onGenerate, summarize, onPreview, onApply, onClose }: SurfaceUvDialogProps<T>) {
  const dialog = useRef<HTMLDialogElement>(null), active = useRef<AbortController | null>(null), mounted = useRef(true);
  const context = useRef(contextKey), previousContext = useRef(contextKey); context.current = contextKey;
  const callbacks = useRef({ onPreview, onClose }); callbacks.current = { onPreview, onClose };
  const [u, setU] = useState(String(initialSettings?.uSpacingMm ?? 10)), [v, setV] = useState(String(initialSettings?.vSpacingMm ?? 10));
  const [reverse, setReverse] = useState(initialSettings?.reverseNormal ?? false), [maxAngle, setMaxAngle] = useState(String(initialSettings?.maxNormalStepDeg ?? 5));
  const [busy, setBusy] = useState(false), [applying, setApplying] = useState(false), [failed, setFailed] = useState(false), [message, setMessage] = useState('');
  const [previewMode, setPreviewMode] = useState(false);
  const [result, setResult] = useState<{ value: T; input: SurfaceUvInput; context: string; summary: SurfaceUvSummary } | null>(null);
  function invalidate() {
    active.current?.abort(); active.current = null; setBusy(false); setFailed(false); setResult(null); setMessage(''); callbacks.current.onPreview(null);
  }
  function close() { if (applying) return; invalidate(); onClose(); }
  useEffect(() => {
    mounted.current = true;
    return () => { mounted.current = false; active.current?.abort(); active.current = null; callbacks.current.onPreview(null); };
  }, []);
  useEffect(() => {
    const element = dialog.current; if (!element) return;
    if (element.open) element.close();
    if (previewMode) element.show(); else element.showModal();
    element.querySelector<HTMLInputElement | HTMLButtonElement>(previewMode ? '[data-return-to-edit]' : '[aria-label="U 方向间距"]')?.focus({ preventScroll: true });
  }, [previewMode]);
  useEffect(() => {
    active.current?.abort(); active.current = null; setBusy(false); setResult(null); setFailed(false); callbacks.current.onPreview(null);
    if (previousContext.current !== contextKey) setMessage('场景、区域或机器人状态已改变，请重新生成。');
    previousContext.current = contextKey;
  }, [contextKey]);
  async function generate() {
    invalidate(); let operation: AbortController | null = null;
    try {
      if (controlCount !== 4) throw new Error('请先按边界顺序标记 4 个角控制点。');
      const uSpacingMm = u.trim() ? Number(u) : NaN, vSpacingMm = v.trim() ? Number(v) : NaN;
      const maxNormalStepDeg = maxAngle.trim() ? Number(maxAngle) : NaN;
      if (![uSpacingMm, vSpacingMm].every(value => Number.isFinite(value) && value > 0 && value <= 1e6)) throw new Error('U、V 间距须大于 0，且不超过 1000000 mm。');
      if (!Number.isFinite(maxNormalStepDeg) || maxNormalStepDeg < .1 || maxNormalStepDeg > 30) throw new Error('相邻姿态角度上限须在 0.1° 至 30° 内。');
      const input = { uSpacingMm, vSpacingMm, reverseNormal: reverse, maxNormalStepDeg };
      const controller = new AbortController(), startedContext = context.current; operation = controller; active.current = controller;
      setBusy(true); setMessage('正在沿模型表面生成路径，并检查法向与机器人姿态连续性…');
      const value = await onGenerate(input, controller.signal);
      if (!mounted.current || active.current !== controller || controller.signal.aborted || context.current !== startedContext) return;
      const summary = summarize(value);
      if (!Number.isSafeInteger(summary.pointCount) || summary.pointCount < 1 || summary.pointCount > 256 || !Number.isSafeInteger(summary.rowCount) || summary.rowCount < 1) throw new Error('生成结果包含无效的点数或行数。');
      onPreview(value);
      active.current = null; setBusy(false); setResult({ value, input, context: startedContext, summary });
      setMessage(`已生成 ${summary.pointCount} 个表面点，共 ${summary.rowCount} 行。`);
    } catch (cause) {
      if (!mounted.current || operation && active.current !== operation || operation?.signal.aborted) return;
      active.current = null; setBusy(false); setResult(null); setFailed(true); setMessage(cause instanceof Error ? cause.message : 'UV 加工路径生成失败。');
    }
  }
  const validResult = result?.context === contextKey ? result : null;
  async function apply() {
    if (!validResult || applying || busy) return;
    setApplying(true);
    try {
      await onApply(validResult.value, validResult.input);
      if (!mounted.current) return;
      setApplying(false); invalidate(); onClose();
    } catch (cause) {
      if (!mounted.current) return;
      setApplying(false); invalidate(); setFailed(true); setMessage(cause instanceof Error ? cause.message : '创建路径失败，原路径保持不变。');
    }
  }
  const feedback = message && <section className={'surface-feedback' + (failed ? ' surface-error' : '')} role="status" aria-label="UV 路径结果" data-uv-status={busy ? 'planning' : failed ? 'failure' : validResult ? 'success' : 'idle'} data-uv-points={validResult?.summary.pointCount ?? 0}>
    <p>{message}</p>{validResult && <>{validResult.summary.lengthMm !== undefined && <p>路径长度 {validResult.summary.lengthMm.toFixed(2)} mm</p>}{validResult.summary.maxRegularSpacingErrorMm !== undefined && <p>常规段最大间距误差 {validResult.summary.maxRegularSpacingErrorMm.toFixed(3)} mm</p>}{validResult.summary.maxSpacingErrorMm !== undefined && <p>最大间距差（含边界余段）{validResult.summary.maxSpacingErrorMm.toFixed(3)} mm</p>}{validResult.summary.maxOrientationStepDeg !== undefined && <p>最大相邻姿态变化 {validResult.summary.maxOrientationStepDeg.toFixed(2)}°</p>}{validResult.summary.warnings?.map((warning, index) => <p key={index}>{warning}</p>)}</>}
  </section>;
  return <dialog ref={dialog} className={'surface-dialog surface-uv-dialog' + (previewMode ? ' is-preview' : '')} aria-label="生成 UV 加工路径" aria-modal={!previewMode} data-surface-uv-mode={previewMode ? 'preview' : 'edit'} onCancel={event => { event.preventDefault(); close(); }} onKeyDown={event => { if (previewMode && event.key === 'Escape') { event.preventDefault(); close(); } }}>
    <header><div><h2>生成 UV 加工路径</h2><p>{pathName} · {controlCount} / 4 个角控制点</p></div><button type="button" aria-label="关闭 UV 路径生成" disabled={applying} onClick={close}><Icon name="close"/></button></header>
    <div className="surface-dialog-body">
      {previewMode ? <p>拖动场景旋转视角，滚轮缩放。确认后创建新的 Path，原区域保持不变。</p> : <>
        <p>4 个角点按边界顺序确定加工区域。1 → 2 为 U 方向，1 → 4 为 V 方向；路径按行往返排列。</p>
        <div className="surface-uv-options"><label>U 方向间距 <small>mm</small><input type="number" min="0.001" max="1000000" step="any" aria-label="U 方向间距" value={u} disabled={applying} onChange={event => { invalidate(); setU(event.target.value); }}/></label><label>V 方向间距 <small>mm</small><input type="number" min="0.001" max="1000000" step="any" aria-label="V 方向间距" value={v} disabled={applying} onChange={event => { invalidate(); setV(event.target.value); }}/></label></div>
        <p className="field-hint">间距以模型表面弧长为目标；到达边界的最后一段可以短于设定间距。常规段误差超过较小间距的 1% 或 0.05 mm（取较大值）时停止生成，结果会列出实际误差。每条路径最多 256 个点。</p>
        <label className="surface-checkbox"><input type="checkbox" aria-label="反转工具法向" checked={reverse} disabled={applying} onChange={event => { invalidate(); setReverse(event.target.checked); }}/>工具 Z 轴沿反向法向</label>
        <p className="field-hint">{reverse ? '工具 +Z 指向表面法向的反方向。' : '工具 +Z 与表面法向同向。'}换行时保持工具滚转方向连续。</p>
        <details className="surface-continuity"><summary><span>姿态连续性</span><Icon name="chevron-down" size={13}/></summary><label className="field-label">相邻姿态角度上限 <small>deg</small><input type="number" min="0.1" max="30" step="0.1" aria-label="相邻姿态角度上限" value={maxAngle} disabled={applying} onChange={event => { invalidate(); setMaxAngle(event.target.value); }}/></label><p className="field-hint">遇到锐边、法向突变或无法连续求解的机器人姿态时停止生成。可缩小间距或重新选择区域。</p></details>
      </>}
      {feedback}
      {previewMode && !validResult && <p className="field-hint">原预览已失效，请返回编辑重新生成。</p>}
    </div>
    <footer><div>{previewMode ? <button type="button" data-return-to-edit disabled={applying} onClick={() => setPreviewMode(false)}>返回编辑</button> : busy ? <button type="button" onClick={() => { invalidate(); setMessage('已取消生成，原区域保持不变。'); }}>取消生成</button> : <button type="button" disabled={controlCount !== 4 || applying} onClick={() => void generate()}><Icon name="path" size={15}/>生成路径</button>}</div>{!previewMode && <button type="button" disabled={!validResult || busy || applying} onClick={() => { if (validResult) { onPreview(validResult.value); setPreviewMode(true); } }}>预览路径</button>}<button type="button" className="surface-primary" disabled={!validResult || busy || applying} onClick={() => void apply()}>{applying ? '正在创建…' : '创建新 Path'}</button></footer>
  </dialog>;
}
