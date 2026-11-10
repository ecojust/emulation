import { useEffect, useState } from 'react';
import Icon from '../ui/Icon';
import type { Pose } from '../robot/types';
import { PoseEditor } from '../workspace/SceneFrameInspector';
import type { SceneSurface } from './types';
import './surface.css';

export interface SurfaceInspectorProps {
  surface: SceneSurface;
  disabled?: boolean;
  onNameChange: (name: string) => void;
  onVisibleChange: (visible: boolean) => void;
  onPoseChange: (pose: Pose) => void;
  onFocus: () => void;
  onRemove: () => void;
}

export default function SurfaceInspector({ surface, disabled, onNameChange, onVisibleChange, onPoseChange, onFocus, onRemove }: SurfaceInspectorProps) {
  const [name, setName] = useState(surface.name), [error, setError] = useState('');
  useEffect(() => { setName(surface.name); setError(''); }, [surface.id, surface.name]);
  function rename() {
    if (disabled || name.trim() === surface.name) return;
    try {
      if (!name.trim()) throw new Error('模型名称不能为空。');
      onNameChange(name.trim()); setError('');
    } catch (cause) { setError(cause instanceof Error ? cause.message : '模型重命名失败。'); }
  }
  return <div className="surface-inspector" data-selected-surface-id={surface.id}>
    <div className="panel-title"><div className="section-number"><Icon name="cube" size={14}/></div><h2>工件模型</h2></div>
    <label className="field-label">名称<input aria-label="工件模型名称" value={name} maxLength={80} disabled={disabled} onChange={event => setName(event.target.value)} onBlur={rename} onKeyDown={event => { if (event.key === 'Enter') { event.preventDefault(); rename(); } }}/></label>
    {error && <p className="surface-feedback surface-error" role="alert">{error}</p>}
    <dl className="surface-properties"><div><dt>格式</dt><dd>{surface.source.format.toUpperCase()}</dd></div><div><dt>三角面</dt><dd>{(surface.mesh.positions.length / 9).toLocaleString()}</dd></div><div><dt>源文件</dt><dd title={surface.source.name}>{surface.source.name}</dd></div></dl>
    <label className="surface-checkbox"><input type="checkbox" aria-label="显示工件模型" checked={surface.visible} disabled={disabled} onChange={event => onVisibleChange(event.target.checked)}/>在场景中显示</label>
    <button type="button" className="copy-button surface-focus" disabled={disabled || !surface.visible} onClick={onFocus}><Icon name="expand" size={14}/>聚焦模型</button>
    <PoseEditor key={surface.id} label="工件模型" value={surface.pose} disabled={disabled} onChange={onPoseChange}/>
    <p className="field-hint">位置和姿态相对于世界坐标。移动模型后，与原表面位置关联的路径需要重新检查。</p>
    <button type="button" className="text-button" disabled={disabled} onClick={onRemove}><Icon name="trash" size={13}/>删除工件模型</button>
  </div>;
}
