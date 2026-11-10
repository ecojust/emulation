import { useEffect, useState } from 'react';
import Icon from '../ui/Icon';
import { eulerZYXToQuaternion } from '../robot/rapid';
import type { Pose, Vec3 } from '../robot/types';
import { poseEulerZYX, type FrameObjectRef, type FrameParent, type ReferenceFrame } from './frame-graph';
import './scene-objects.css';

const precision = (value: number) => String(Number(value.toFixed(8)));
const parentKey = (parent: FrameParent) => parent.kind === 'frame' ? 'frame:' + parent.id : parent.kind;
const parentValue = (value: string): FrameParent => value === 'world' ? { kind: 'world' } : value === 'flange' ? { kind: 'flange' } : { kind: 'frame', id: value.slice(6) };

export interface AttachmentSelectorProps {
  object: FrameObjectRef;
  parent: FrameParent;
  frames: readonly ReferenceFrame[];
  disabled?: boolean;
  label?: string;
  onReparent: (object: FrameObjectRef, parent: FrameParent) => void;
}
/** Keyboard equivalent of tree drag/drop; deliberately has no implicit snap-to-parent. */
export function AttachmentSelector({ object, parent, frames, disabled, label = '挂接到', onReparent }: AttachmentSelectorProps) {
  const [error, setError] = useState('');
  const objectId = 'id' in object ? object.id : '';
  useEffect(() => setError(''), [object.kind, objectId, parentKey(parent)]);
  const blocked = new Set<string>();
  if (object.kind === 'frame') {
    blocked.add(object.id);
    // Exclude this frame's descendants as well as itself from the chooser.
    for (let pass = 0; pass < frames.length; pass++) for (const frame of frames) {
      if (frame.parent.kind === 'frame' && blocked.has(frame.parent.id)) blocked.add(frame.id);
    }
  }
  return <div>
    <label className="scene-attachment-control"><span>{label}</span><select aria-label={label} value={parentKey(parent)} disabled={disabled} onChange={event => {
      try { onReparent(object, parentValue(event.target.value)); setError(''); }
      catch (cause) { setError(cause instanceof Error ? cause.message : '挂接失败，原位置保持不变。'); }
    }}><option value="world">世界坐标</option>{object.kind === 'tool' ? <option value="flange">末端法兰</option> : frames.filter(frame => !blocked.has(frame.id)).map(frame => <option key={frame.id} value={'frame:' + frame.id}>{frame.name}</option>)}</select></label>
    <p className="field-hint">换父级保持世界位置和姿态，局部坐标随之换算。</p>
    {error && <p className="danger-zone-error" role="alert">{error}</p>}
  </div>;
}

export interface PoseEditorProps { label: string; value: Pose; disabled?: boolean; onChange: (pose: Pose) => void }
/** Complete-pose apply avoids committing a half-edited rotation or invalid number. */
export function PoseEditor({ label, value, disabled, onChange }: PoseEditorProps) {
  const signature = JSON.stringify(value);
  const [position, setPosition] = useState(() => value.position.map(precision));
  const [orientation, setOrientation] = useState(() => poseEulerZYX(value).map(precision));
  const [error, setError] = useState('');
  useEffect(() => { setPosition(value.position.map(precision)); setOrientation(poseEulerZYX(value).map(precision)); setError(''); }, [signature, label]);
  function apply() {
    try {
      if ([...position, ...orientation].some(text => !text.trim() || !Number.isFinite(Number(text)))) throw new Error('请填写完整的有限位置和角度。');
      const xyz = position.map(Number) as Vec3, angles = orientation.map(Number) as Vec3;
      if (xyz.some(number => Math.abs(number) > 1e6)) throw new Error('位置须在 ±1000000 mm 以内。');
      if (angles.some(number => Math.abs(number) > 360000)) throw new Error('角度须在 ±360000° 以内。');
      onChange({ position: xyz, orientation: eulerZYXToQuaternion(angles) }); setError('');
    } catch (cause) { setError(cause instanceof Error ? cause.message : '位姿无效，原位置保持不变。'); }
  }
  return <fieldset className="scene-pose-editor" disabled={disabled} data-pose-label={label}>
    <legend>{label}</legend><label className="field-label">位置 <span>mm</span></label><div className="position-grid">{['X', 'Y', 'Z'].map((axis, index) => <label key={axis}><span>{axis}</span><input type="number" step="any" min={-1e6} max={1e6} aria-label={label + '位置 ' + axis} value={position[index]} onChange={event => setPosition(values => values.map((item, i) => i === index ? event.target.value : item))}/></label>)}</div>
    <label className="field-label">姿态 <span>Rz · Ry · Rx / deg</span></label><div className="position-grid">{['Rz', 'Ry', 'Rx'].map((axis, index) => <label key={axis}><span>{axis}</span><input type="number" step="any" min={-360000} max={360000} aria-label={label + '姿态 ' + axis} value={orientation[index]} onChange={event => setOrientation(values => values.map((item, i) => i === index ? event.target.value : item))}/></label>)}</div>
    <button type="button" className="copy-button pose-apply" aria-label={'应用' + label + '位姿'} onClick={apply}>应用位置与姿态</button>
    {error && <p className="danger-zone-error" role="alert">{error}</p>}
  </fieldset>;
}

export function PoseReadout({ label, pose }: { label: string; pose: Pose }) {
  return <section className="scene-frame-readout" aria-label={label} data-pose={JSON.stringify(pose)}><h4>{label} · 位置 / mm</h4><code>{pose.position.map(precision).join(', ')}</code><h4>四元数 [w, x, y, z]</h4><code>{pose.orientation.map(precision).join(', ')}</code></section>;
}
export interface SceneFrameInspectorProps {
  frame: ReferenceFrame;
  frames: readonly ReferenceFrame[];
  worldPose: Pose;
  disabled?: boolean;
  onNameChange: (name: string) => void;
  onPoseChange: (pose: Pose) => void;
  onReparent: (object: FrameObjectRef, parent: FrameParent) => void;
  onRemove?: () => void;
}
export default function SceneFrameInspector({ frame, frames, worldPose, disabled, onNameChange, onPoseChange, onReparent, onRemove }: SceneFrameInspectorProps) {
  return <div className="frame-inspector" data-selected-frame-id={frame.id} data-world-pose={JSON.stringify(worldPose)} data-local-pose={JSON.stringify(frame.localPose)}>
    <div className="panel-title"><div className="section-number"><Icon name="frame" size={14}/></div><h2>参考坐标系</h2></div>
    <label className="field-label">名称<input aria-label="参考坐标系名称" maxLength={80} value={frame.name} disabled={disabled} onChange={event => onNameChange(event.target.value)}/></label>
    <AttachmentSelector object={{ kind: 'frame', id: frame.id }} parent={frame.parent} frames={frames} disabled={disabled} onReparent={onReparent}/>
    <PoseEditor key={frame.id} label="参考坐标系" value={frame.localPose} disabled={disabled} onChange={onPoseChange}/>
    <p className="field-hint">上方位姿相对于父坐标系。修改坐标系会带动其机器人基座和路径等子对象。</p>
    <PoseReadout label="世界位姿" pose={worldPose}/>
    {onRemove && <button type="button" className="text-button" disabled={disabled} onClick={onRemove}>删除参考坐标系</button>}
  </div>;
}
