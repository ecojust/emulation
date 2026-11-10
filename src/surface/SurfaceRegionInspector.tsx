import { useEffect, useRef, useState } from 'react';
import Icon from '../ui/Icon';
import { PoseReadout } from '../workspace/SceneFrameInspector';
import type { ProgramPath } from '../workspace/program';
import './surface.css';

export interface SurfaceRegionInspectorProps {
  path: ProgramPath;
  surfaceName?: string;
  selectedPointId?: string | null;
  picking: boolean;
  disabled?: boolean;
  staleReason?: string | null;
  onSelectPoint: (pointId: string) => void;
  onSetControl: (pointId: string, enabled: boolean) => void;
  onRemovePoint: (pointId: string) => void;
  onStartPicking: () => void;
  onFinishPicking: () => void;
  onGenerate: () => void;
  /** Clear this region and invalidate its derived solutions before starting fresh picks. */
  onReset?: () => void;
}

const vectorText = (values: readonly number[]) => values.map(value => Number(value.toFixed(6))).join(', ');
export default function SurfaceRegionInspector({ path, surfaceName, selectedPointId, picking, disabled, staleReason, onSelectPoint, onSetControl, onRemovePoint, onStartPicking, onFinishPicking, onGenerate, onReset }: SurfaceRegionInspectorProps) {
  const region = path.surface;
  const [error, setError] = useState('');
  const [resetPending, setResetPending] = useState(false);
  const resetTrigger = useRef<HTMLButtonElement>(null), resetCancel = useRef<HTMLButtonElement>(null), wasResetPending = useRef(false);
  useEffect(() => { setError(''); setResetPending(false); }, [path.id, region?.revision, region?.invalidated]);
  useEffect(() => {
    if (resetPending) resetCancel.current?.focus({ preventScroll: true });
    else if (wasResetPending.current) resetTrigger.current?.focus({ preventScroll: true });
    wasResetPending.current = resetPending;
  }, [resetPending]);
  function action(callback: () => void) {
    try { callback(); setError(''); return true; }
    catch (cause) { setError(cause instanceof Error ? cause.message : '表面区域操作失败。'); return false; }
  }
  function cancelReset() { setResetPending(false); }
  const selectedPoint = region?.points.find(point => point.id === selectedPointId);
  const controls = region?.controlPointIds ?? [];
  const canGenerate = region?.kind === 'region' && controls.length === 4 && !disabled && !staleReason && !picking;
  return <section className="surface-region-inspector" aria-label="表面区域" data-surface-path-id={path.id}>
    <div className="field-heading"><h3>{region?.kind === 'uv' ? 'UV 加工路径' : '表面区域'}</h3><small>{region?.points.length ?? 0} 点</small></div>
    {surfaceName && <p className="field-hint">工件：{surfaceName}</p>}
    {region?.kind === 'uv' ? <>
      <p className="field-hint">{region.resolvedRevision === region.revision ? '已保存表面目标及对应机器人姿态。运行前仍会检查整条运动路径。' : '表面目标尚未完成机器人姿态求解，暂不能运行或转换为程序。'}</p>
      {region.settings && <p className="surface-spacing-readout">U {region.settings.uSpacingMm} mm · V {region.settings.vSpacingMm} mm</p>}
    </> : <>
      <p className="field-hint">沿同一模型的表面依次点选区域边界，再将其中 4 个角点设为控制点。角点按边界顺序编号。</p>
      <div className="surface-region-actions"><button type="button" className="copy-button" disabled={disabled || Boolean(staleReason)} onClick={() => action(picking ? onFinishPicking : onStartPicking)}><Icon name={picking ? 'check' : 'target'} size={14}/>{picking ? '结束点选' : region?.points.length ? '继续点选区域' : '开始点选区域'}</button><button type="button" className="copy-button" disabled={!canGenerate} onClick={() => action(onGenerate)}><Icon name="path" size={14}/>生成 UV 加工路径</button></div>
      <p className="surface-control-summary">控制点 {controls.length} / 4 <span>1 → 2 为 U 方向，1 → 4 为 V 方向</span></p>
      {picking && <p className="surface-feedback" role="status">点击模型表面添加边界点；拖动鼠标只旋转视角。按 Esc 或“结束点选”完成。</p>}
    </>}
    {staleReason && <p className="surface-feedback surface-error" role="status">{staleReason}</p>}
    {region?.kind === 'region' && region.points.length > 0 && onReset && <div className="surface-region-reset">
      <button ref={resetTrigger} type="button" className="text-button surface-reset-trigger" disabled={disabled || resetPending} onClick={() => { setError(''); setResetPending(true); }}><Icon name="reset" size={13}/>重新点选区域</button>
      {resetPending && <div className="surface-reset-confirmation" role="group" aria-label="确认重新点选区域" onKeyDown={event => { if (event.key === 'Escape') { event.preventDefault(); event.stopPropagation(); cancelReset(); } }}>
        <strong>清空现有区域点并重新点选？</strong>
        <p>当前边界点和控制点会被清空，关联的 UV 路径需要重新生成。已生成的模块源码保留。</p>
        <div><button ref={resetCancel} type="button" className="copy-button" disabled={disabled} onClick={cancelReset}>保留现有区域</button><button type="button" className="copy-button surface-reset-confirm" disabled={disabled} onClick={() => { if (action(onReset)) setResetPending(false); }}>清空并重新点选</button></div>
      </div>}
    </div>}
    {error && <p className="surface-feedback surface-error" role="alert">{error}</p>}
    <ol className="surface-point-list" aria-label="表面点列表">
      {region?.points.map((point, index) => {
        const controlIndex = controls.indexOf(point.id);
        return <li key={point.id} className={point.id === selectedPointId ? 'is-selected' : ''} data-surface-point-id={point.id}>
          <button type="button" className="surface-point-select" disabled={disabled} aria-label={'选择表面点 ' + point.name} aria-pressed={point.id === selectedPointId} onClick={() => action(() => onSelectPoint(point.id))}><Icon name={controlIndex >= 0 ? 'frame' : 'target'} size={13}/><span>{point.name || `点 ${index + 1}`}</span>{controlIndex >= 0 && <small>角点 {controlIndex + 1}</small>}</button>
          {region.kind === 'region' && <><label className="surface-control-check" title={controlIndex < 0 && controls.length >= 4 ? '已有 4 个控制点，请先取消其中一个。' : '设为区域角点'}><input type="checkbox" aria-label={point.name + '设为控制点'} checked={controlIndex >= 0} disabled={disabled || Boolean(staleReason) || controlIndex < 0 && controls.length >= 4} onChange={event => action(() => onSetControl(point.id, event.target.checked))}/><span>控制点</span></label><button type="button" className="surface-point-remove" aria-label={'删除表面点 ' + point.name} disabled={disabled} onClick={() => action(() => onRemovePoint(point.id))}><Icon name="trash" size={13}/></button></>}
        </li>;
      })}
    </ol>
    {!region?.points.length && <p className="field-hint">尚未记录表面点。</p>}
    {selectedPoint && <div className="surface-selected-point"><h4>{selectedPoint.name}</h4><PoseReadout label="路径坐标位姿" pose={selectedPoint.pose}/><h4>拾取时世界法向</h4><code>{vectorText(selectedPoint.pick.normal)}</code><p className="field-hint">位置取自模型表面；工具 Z 轴沿表面{region?.kind === 'uv' && region.settings?.reverseNormal ? '反向' : ''}法向。</p></div>}
  </section>;
}
