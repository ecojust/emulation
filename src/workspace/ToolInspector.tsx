import { useEffect, useState } from 'react';
import Icon from '../ui/Icon';
import type { Pose } from '../robot/types';
import type { FrameObjectRef, FrameParent, ReferenceFrame, ToolFrame } from './frame-graph';
import { AttachmentSelector, PoseEditor, PoseReadout } from './SceneFrameInspector';
import './scene-objects.css';

export interface ToolInspectorProps {
  tool: ToolFrame;
  frames: readonly ReferenceFrame[];
  worldPose: Pose;
  tcpWorldPose: Pose;
  disabled?: boolean;
  onNameChange: (name: string) => void;
  onPoseChange: (pose: Pose) => void;
  onTcpChange: (pose: Pose) => void;
  onRadiusChange: (radiusMm: number) => void;
  onReparent: (object: FrameObjectRef, parent: FrameParent) => void;
  onAlignFlange?: () => void;
  onRemove?: () => void;
}
export default function ToolInspector(props: ToolInspectorProps) {
  const { tool, worldPose, tcpWorldPose, disabled } = props;
  const [radius, setRadius] = useState(String(tool.radiusMm));
  const [error, setError] = useState('');
  useEffect(() => { setRadius(String(tool.radiusMm)); setError(''); }, [tool.id, tool.radiusMm]);
  return <div className="tool-inspector" data-selected-tool-id={tool.id} data-world-pose={JSON.stringify(worldPose)} data-local-pose={JSON.stringify(tool.localPose)} data-tcp-world-pose={JSON.stringify(tcpWorldPose)} data-tool-tcp-pose={JSON.stringify(tool.tcpPose)}>
    <div className="panel-title"><div className="section-number"><Icon name="tool" size={14}/></div><h2>工具</h2></div>
    <label className="field-label">名称<input aria-label="工具名称" maxLength={80} value={tool.name} disabled={disabled} onChange={event => props.onNameChange(event.target.value)}/></label>
    <AttachmentSelector object={{ kind: 'tool', id: tool.id }} parent={tool.parent} frames={props.frames} disabled={disabled} onReparent={props.onReparent}/>
    <PoseEditor key={'mount-' + tool.id} label="工具安装" value={tool.localPose} disabled={disabled} onChange={props.onPoseChange}/>
    <p className="field-hint">安装位姿相对于工具的父级，挂到法兰时以法兰为基准。</p>
    {tool.parent.kind === 'flange' && props.onAlignFlange && <button type="button" className="copy-button" disabled={disabled} onClick={props.onAlignFlange}>对齐法兰</button>}
    <PoseEditor key={'tcp-' + tool.id} label="工具 TCP 偏置" value={tool.tcpPose} disabled={disabled} onChange={props.onTcpChange}/>
    <p className="field-hint">TCP 是工具中心点，偏置相对于工具安装原点。</p>
    <label className="field-label">工具包络半径 <span>mm</span><input type="number" min={0.001} max={10000} step="any" aria-label="工具包络半径" value={radius} disabled={disabled} onChange={event => setRadius(event.target.value)}/></label>
    <button type="button" className="copy-button" disabled={disabled} onClick={() => {
      try {
        const value = Number(radius);
        if (!radius.trim() || !Number.isFinite(value) || value <= 0 || value > 10000) throw new Error('包络半径必须大于 0 且不超过 10000 mm。');
        props.onRadiusChange(value); setError('');
      } catch (cause) { setError(cause instanceof Error ? cause.message : '包络尺寸无效。'); }
    }}>应用工具包络</button>
    {error && <p className="danger-zone-error" role="alert">{error}</p>}
    <p className="field-hint">包络为从安装原点到 TCP 的胶囊体，应覆盖完整工具外形，用于路径与禁入区检查。</p>
    <PoseReadout label="工具世界位姿" pose={worldPose}/><PoseReadout label="TCP 世界位姿" pose={tcpWorldPose}/>
    {props.onRemove && <button type="button" className="text-button" disabled={disabled} onClick={props.onRemove}>删除工具</button>}
  </div>;
}
