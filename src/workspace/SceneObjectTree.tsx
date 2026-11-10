import { useState, type DragEvent, type KeyboardEvent, type MouseEvent, type ReactNode } from 'react';
import Icon from '../ui/Icon';
import type { FrameObjectRef, FrameParent, ReferenceFrame, ToolFrame } from './frame-graph';
import type { ProgramPath, ProgramPoint } from './program';
import type { DangerZoneDraft } from './useWorkspace';
import './scene-objects.css';

export type SceneTreeSelection = { kind: 'robot' } | { kind: 'base' } | { kind: 'flange' } | { kind: 'joint'; index: number }
  | { kind: 'frame' | 'tool' | 'path' | 'target' | 'zone'; id: string };
export interface SceneTreeMenu { kind: 'path' | 'target'; id: string; x: number; y: number; trigger: HTMLElement }
export interface SceneTreePath extends ProgramPath { parent?: FrameParent }
export interface SceneObjectTreeProps {
  robotName: string;
  joints: readonly number[];
  robotParent: FrameParent;
  frames: readonly ReferenceFrame[];
  tools: readonly ToolFrame[];
  paths: readonly SceneTreePath[];
  points: readonly ProgramPoint[];
  zones: readonly DangerZoneDraft[];
  invalidZoneIds?: readonly string[];
  selection: SceneTreeSelection;
  executingPointId?: string | null;
  disabled?: boolean;
  canAddZone?: boolean;
  onSelect: (selection: SceneTreeSelection) => void;
  onContextMenu: (menu: SceneTreeMenu) => void;
  onReparent: (object: FrameObjectRef, parent: FrameParent) => void;
  onAddFrame: () => void;
  onAddTool: () => void;
  onAddPath: () => void;
  onAddZone: () => void;
}
const WORLD: FrameParent = { kind: 'world' };
const FLANGE: FrameParent = { kind: 'flange' };
const MIME = 'application/x-emulation-scene-object+json';
const parentKey = (parent: FrameParent) => parent.kind === 'frame' ? 'frame:' + parent.id : parent.kind;
function readDrag(event: DragEvent): FrameObjectRef {
  let value: unknown;
  try { value = JSON.parse(event.dataTransfer.getData(MIME)); } catch { throw new Error('无法识别拖动的场景对象。'); }
  if (!value || typeof value !== 'object' || !('kind' in value)) throw new Error('无法识别拖动的场景对象。');
  if (value.kind === 'robot') return { kind: 'robot' };
  if ((value.kind === 'path' || value.kind === 'tool') && 'id' in value && typeof value.id === 'string' && value.id.length > 0 && value.id.length < 256) return { kind: value.kind, id: value.id };
  throw new Error('该对象不支持拖动挂接。');
}
function accepts(object: FrameObjectRef, parent: FrameParent) {
  return object.kind === 'tool' ? parent.kind === 'flange' || parent.kind === 'world'
    : parent.kind === 'frame' || parent.kind === 'world';
}

/** This view never changes coordinates itself. Every attachment uses the graph's keep-world operation. */
export default function SceneObjectTree(props: SceneObjectTreeProps) {
  const [dragged, setDragged] = useState<FrameObjectRef | null>(null);
  const [over, setOver] = useState<string | null>(null);
  const [error, setError] = useState('');
  const selected = (kind: SceneTreeSelection['kind'], id?: string | number) => props.selection.kind === kind
    && (kind === 'joint' ? 'index' in props.selection && props.selection.index === id : !('id' in props.selection) || props.selection.id === id);
  const parentAttributes = (parent: FrameParent) => ({ 'data-parent-kind': parent.kind, 'data-parent-id': parent.kind === 'frame' ? parent.id : '' });
  const draggable = (object: FrameObjectRef) => ({
    draggable: !props.disabled,
    onDragStart: (event: DragEvent<HTMLElement>) => {
      if (props.disabled) { event.preventDefault(); return; }
      event.stopPropagation(); event.dataTransfer.effectAllowed = 'move'; event.dataTransfer.setData(MIME, JSON.stringify(object));
      setDragged(object); setError('');
    },
    onDragEnd: () => { setDragged(null); setOver(null); },
  });
  const droppable = (parent: FrameParent) => ({
    'data-drop-parent': parentKey(parent),
    'data-drop-active': over === parentKey(parent),
    onDragOver: (event: DragEvent<HTMLElement>) => {
      if (props.disabled || !event.dataTransfer.types.includes(MIME)) return;
      event.stopPropagation();
      if (dragged && !accepts(dragged, parent)) { event.dataTransfer.dropEffect = 'none'; return; }
      event.preventDefault(); event.dataTransfer.dropEffect = 'move'; setOver(parentKey(parent));
    },
    onDragLeave: (event: DragEvent<HTMLElement>) => {
      if (!(event.relatedTarget instanceof Node) || !event.currentTarget.contains(event.relatedTarget)) setOver(null);
    },
    onDrop: (event: DragEvent<HTMLElement>) => {
      event.preventDefault(); event.stopPropagation(); setOver(null); setDragged(null);
      if (props.disabled) return;
      try {
        const object = readDrag(event);
        if (!accepts(object, parent)) throw new Error(object.kind === 'tool' ? '工具只能挂接到末端法兰或世界坐标。' : '机器人基座和路径只能挂接到参考坐标系或世界坐标。');
        props.onReparent(object, parent); setError('');
      } catch (cause) { setError(cause instanceof Error ? cause.message : '挂接失败，原位置保持不变。'); }
    },
  });
  const context = (kind: 'path' | 'target', id: string, event: MouseEvent<HTMLButtonElement> | KeyboardEvent<HTMLButtonElement>) => {
    event.preventDefault(); props.onSelect({ kind, id });
    const rect = event.currentTarget.getBoundingClientRect();
    props.onContextMenu({ kind, id, x: 'clientX' in event ? event.clientX : rect.left, y: 'clientY' in event ? event.clientY : rect.bottom, trigger: event.currentTarget });
  };
  const menuKeys = (kind: 'path' | 'target', id: string, event: KeyboardEvent<HTMLButtonElement>) => {
    if (event.key === 'ContextMenu' || event.key === 'F10' && event.shiftKey) context(kind, id, event);
  };
  const toolNode = (tool: ToolFrame) => <button type="button" key={tool.id} data-object-kind="tool" data-object-id={tool.id} {...parentAttributes(tool.parent)} {...draggable({ kind: 'tool', id: tool.id })}
    aria-label={'选择工具 ' + tool.name} className={selected('tool', tool.id) ? 'selected' : ''} onClick={() => props.onSelect({ kind: 'tool', id: tool.id })}>
    <Icon name="tool" size={13}/><span>{tool.name}</span></button>;
  const robotNode = () => <details className="scene-tree-branch scene-robot-branch" open key="robot"><summary>
    <Icon name="chevron-right" className="summary-icon" size={13}/><button type="button" aria-label="选择机器人" data-object-kind="robot" {...parentAttributes(props.robotParent)} {...draggable({ kind: 'robot' })}
      className={selected('robot') ? 'selected' : ''} onClick={event => { event.preventDefault(); props.onSelect({ kind: 'robot' }); }}><Icon name="robot" size={14}/><span>{props.robotName}</span></button>
    </summary><div className="scene-tree-children">
      <button type="button" aria-label="选择机器人基座" data-object-kind="base" {...parentAttributes(props.robotParent)} {...draggable({ kind: 'robot' })} className={selected('base') ? 'selected' : ''} onClick={() => props.onSelect({ kind: 'base' })}><Icon name="base" size={13}/><span>机器人基座</span></button>
      {props.joints.map((value, index) => <button type="button" key={index} aria-label={'选择关节轴 ' + (index + 1)} data-object-kind="joint" data-joint-index={index} className={selected('joint', index) ? 'selected' : ''} onClick={() => props.onSelect({ kind: 'joint', index })}><Icon name="joint" size={13}/><span>关节轴 {index + 1}</span><small>{Number.isFinite(value) ? value.toFixed(1) + '°' : '—'}</small></button>)}
      <details className="scene-tree-branch scene-flange-branch" open><summary><Icon name="chevron-right" className="summary-icon" size={13}/><button type="button" aria-label="选择末端法兰" data-object-kind="flange" {...droppable(FLANGE)} className={selected('flange') ? 'selected' : ''} onClick={event => { event.preventDefault(); props.onSelect({ kind: 'flange' }); }}><Icon name="flange" size={13}/><span>末端法兰</span></button></summary>
        <div className="scene-tree-children">{props.tools.filter(tool => tool.parent.kind === 'flange').map(toolNode)}{!props.tools.some(tool => tool.parent.kind === 'flange') && <p className="field-hint object-empty">可将工具拖到法兰。</p>}</div>
      </details>
    </div></details>;
  const pathNode = (path: SceneTreePath) => <details key={path.id} className="object-path scene-tree-branch" open><summary><Icon name="chevron-right" size={13} className="summary-icon"/>
    <button type="button" aria-label={'选择路径 ' + path.name} data-object-kind="path" data-object-id={path.id} {...parentAttributes(path.parent ?? WORLD)} {...draggable({ kind: 'path', id: path.id })} className={selected('path', path.id) ? 'selected' : ''}
      onContextMenu={event => context('path', path.id, event)} onKeyDown={event => menuKeys('path', path.id, event)} onClick={event => { event.preventDefault(); props.onSelect({ kind: 'path', id: path.id }); }}><Icon name="path" size={13}/><span>{path.name}</span><small>{path.pointIds.length}</small></button>
    </summary><div className="object-path-points">{path.pointIds.map(id => {
      const point = props.points.find(item => item.id === id);
      return point && <button type="button" key={id} aria-label={'选择目标点 ' + point.pointName} data-object-kind="target" data-object-id={id} data-point-name={point.pointName} data-path-id={path.id} data-executing={props.executingPointId === id} className={selected('target', id) ? 'selected' : ''}
        onContextMenu={event => context('target', id, event)} onKeyDown={event => menuKeys('target', id, event)} onClick={() => props.onSelect({ kind: 'target', id })}><Icon name="target" size={12}/><span>{point.pointName}</span></button>;
    })}{!path.pointIds.length && <p className="field-hint object-empty">右键路径，添加当前点或规划路径。</p>}</div></details>;
  const children = (parent: FrameParent, visited = new Set<string>()): ReactNode => <>
    {parentKey(props.robotParent) === parentKey(parent) && robotNode()}
    {props.frames.filter(frame => parentKey(frame.parent) === parentKey(parent) && !visited.has(frame.id)).map(frame => <details key={frame.id} className="scene-tree-branch scene-frame-branch" data-frame-id={frame.id} open><summary>
      <Icon name="chevron-right" className="summary-icon" size={13}/><button type="button" aria-label={'选择参考坐标系 ' + frame.name} data-object-kind="frame" data-object-id={frame.id} {...parentAttributes(frame.parent)} {...droppable({ kind: 'frame', id: frame.id })} className={selected('frame', frame.id) ? 'selected' : ''}
        onClick={event => { event.preventDefault(); props.onSelect({ kind: 'frame', id: frame.id }); }}><Icon name="frame" size={13}/><span>{frame.name}</span></button>
      </summary><div className="scene-tree-children">{children({ kind: 'frame', id: frame.id }, new Set([...visited, frame.id]))}</div></details>)}
    {props.paths.filter(path => parentKey(path.parent ?? WORLD) === parentKey(parent)).map(pathNode)}
  </>;
  return <div className="workspace-object-list scene-object-tree">
    <div className="scene-object-tools" aria-label="添加场景对象">
      <button type="button" aria-label="添加参考坐标系" title="添加参考坐标系" disabled={props.disabled} onClick={props.onAddFrame}><Icon name="frame" size={14}/><Icon name="plus" size={11}/></button>
      <button type="button" aria-label="添加工具" title="添加工具" disabled={props.disabled} onClick={props.onAddTool}><Icon name="tool" size={14}/><Icon name="plus" size={11}/></button>
      <button type="button" aria-label="添加 Path" title="添加路径" disabled={props.disabled} onClick={props.onAddPath}><Icon name="path" size={14}/><Icon name="plus" size={11}/></button>
      <button type="button" aria-label="添加危险区" title="添加禁入区" disabled={props.disabled || props.canAddZone === false} onClick={props.onAddZone}><Icon name="zone" size={14}/><Icon name="plus" size={11}/></button>
    </div>
    <div className="scene-world-target" data-object-kind="world" {...droppable(WORLD)}><Icon name="frame" size={13}/><span>世界坐标</span></div>
    {children(WORLD)}
    {props.tools.filter(tool => tool.parent.kind === 'world').map(toolNode)}
    {!props.paths.length && <p className="field-hint object-empty">添加路径后，可记录当前点或进行路径规划。</p>}
    <div className="object-section-label"><span>禁入区</span></div>
    {props.zones.map(zone => <button key={zone.id} type="button" data-object-kind="zone" data-object-id={zone.id} data-zone-id={zone.id} aria-label={'选择危险区 ' + (zone.name.trim() || '未命名禁入区')} className={selected('zone', zone.id) ? 'selected' : ''} onClick={() => props.onSelect({ kind: 'zone', id: zone.id })}><Icon name="zone" size={13} className={'zone-object-icon ' + (zone.enabled ? '' : 'disabled')}/><span>{zone.name.trim() || '未命名禁入区'}</span>{props.invalidZoneIds?.includes(zone.id) && <Icon name="warning" size={13} className="zone-invalid-icon"/>}</button>)}
    {dragged && <p className="field-hint tree-drag-hint" role="status">{dragged.kind === 'tool' ? '拖到末端法兰或世界坐标。' : '拖到参考坐标系或世界坐标。'}世界位置与姿态保持不变。</p>}
    {error && <p className="danger-zone-error" role="alert">{error}</p>}
  </div>;
}
