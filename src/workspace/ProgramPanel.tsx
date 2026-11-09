import { useEffect, useLayoutEffect, useMemo, useRef, useState, type KeyboardEvent, type MouseEvent } from 'react';
import type { ProgramPath, ProgramPoint } from './program';
import { formatProgramPoint, getProgramProfile, renderProgramTemplate } from './program-templates';
import './program-panel.css';

export interface ProgramPanelProps {
  /** Changing model resets row anchors and any open menu. */
  modelKey: string;
  active?: boolean;
  points: readonly ProgramPoint[];
  paths: readonly ProgramPath[];
  selectedPointIds: readonly string[];
  captureDisabledReason?: string | null;
  onInsertCurrent: (afterPointId: string | null) => void;
  onSelectionChange: (pointIds: string[]) => void;
  onSelectTarget: (pointId: string) => void;
  onConvertToPath: (pointIds: string[]) => void;
  onRemovePoints?: (pointIds: string[]) => void;
  running?: boolean;
  runDisabledReason?: string | null;
  runStatus?: string;
  onRun?: () => void;
  onStop?: () => void;
  loop?: boolean;
  onLoopChange?: (loop: boolean) => void;
  executingPointId?: string | null;
}

interface MenuPosition { x: number; y: number; afterPointId: string | null; sequence: number }
const numberText = (value: number, digits = 2) => Number(value.toFixed(digits)).toString();

/** The ordered program is owned by useProgram; the panel never rearranges rows. */
export default function ProgramPanel(props: ProgramPanelProps) {
  return <ProgramPanelBody key={props.modelKey} {...props}/>;
}

function ProgramPanelBody({ modelKey, active = true, points, paths, selectedPointIds, captureDisabledReason,
  onInsertCurrent, onSelectionChange, onSelectTarget, onConvertToPath, onRemovePoints,
  running = false, runDisabledReason, runStatus, onRun, onStop, loop = false, onLoopChange, executingPointId }: ProgramPanelProps) {
  const list = useRef<HTMLDivElement>(null);
  const menuElement = useRef<HTMLDivElement>(null);
  const rowElements = useRef(new Map<string, HTMLButtonElement>());
  const anchor = useRef<string | null>(null);
  const activePoint = useRef<string | null>(null);
  const returnFocus = useRef<HTMLElement | null>(null);
  const sequence = useRef(0);
  const [menu, setMenu] = useState<MenuPosition | null>(null);
  const profile = getProgramProfile(modelKey);
  const formattedPoints = useMemo(() => points.map(point => formatProgramPoint(modelKey, point)), [modelKey, points]);
  const template = useMemo(() => renderProgramTemplate(modelKey, points), [modelKey, points]);
  const insertDisabledReason = running ? '仿真运行中，请停止后编辑程序。' : captureDisabledReason;
  const pathNames = useMemo(() => new Map(paths.map(path => [path.id, path.name])), [paths]);
  const requestedSelection = new Set(selectedPointIds);
  // Membership from another model or deleted rows is never shown or submitted.
  const selection = points.filter(point => requestedSelection.has(point.id)).map(point => point.id);
  const selected = new Set(selection);
  const lastSelected = selection.at(-1) ?? null;
  const insertionPoint = activePoint.current && selected.has(activePoint.current) ? activePoint.current : lastSelected;
  const focusPointId = activePoint.current && points.some(point => point.id === activePoint.current)
    ? activePoint.current : lastSelected ?? points[0]?.id;

  function closeMenu(restoreFocus = true) {
    setMenu(null);
    if (restoreFocus) {
      const destination = returnFocus.current;
      if (destination?.isConnected) destination.focus({ preventScroll: true });
      else list.current?.focus({ preventScroll: true });
    }
  }

  useEffect(() => {
    const ids = new Set(points.map(point => point.id));
    if (anchor.current && !ids.has(anchor.current)) anchor.current = null;
    if (activePoint.current && !ids.has(activePoint.current)) activePoint.current = null;
    if (menu?.afterPointId && !ids.has(menu.afterPointId)) setMenu(null);
  }, [points, menu?.afterPointId]);

  useEffect(() => { if (!active) setMenu(null); }, [active]);

  useLayoutEffect(() => {
    if (!menu || !menuElement.current) return;
    const bounds = menuElement.current.getBoundingClientRect();
    const x = Math.max(8, Math.min(menu.x, window.innerWidth - bounds.width - 8));
    const y = Math.max(8, Math.min(menu.y, window.innerHeight - bounds.height - 8));
    if (x !== menu.x || y !== menu.y) setMenu(current => current ? { ...current, x, y } : null);
  }, [menu, insertDisabledReason, selection.length]);

  useLayoutEffect(() => {
    if (!menu) return;
    const first = menuElement.current?.querySelector<HTMLButtonElement>('[role="menuitem"]:not(:disabled)');
    (first ?? menuElement.current)?.focus({ preventScroll: true });
  }, [menu?.sequence]);

  useEffect(() => {
    if (!menu) return;
    const outside = (event: PointerEvent) => {
      if (!menuElement.current?.contains(event.target as Node)) closeMenu(false);
    };
    const escape = (event: globalThis.KeyboardEvent) => {
      if (event.key === 'Escape') { event.preventDefault(); closeMenu(); }
    };
    const moved = (event: Event) => {
      if (!(event.target instanceof Node) || !menuElement.current?.contains(event.target)) closeMenu(false);
    };
    document.addEventListener('pointerdown', outside, true);
    document.addEventListener('keydown', escape);
    document.addEventListener('scroll', moved, true);
    window.addEventListener('resize', moved);
    return () => {
      document.removeEventListener('pointerdown', outside, true);
      document.removeEventListener('keydown', escape);
      document.removeEventListener('scroll', moved, true);
      window.removeEventListener('resize', moved);
    };
  }, [menu?.sequence]);

  function selectPoint(id: string, modifiers: { shiftKey?: boolean; ctrlKey?: boolean; metaKey?: boolean }) {
    const additive = modifiers.ctrlKey || modifiers.metaKey;
    const end = points.findIndex(point => point.id === id);
    if (end < 0) return;
    const start = points.findIndex(point => point.id === anchor.current);
    let next: string[];
    if (modifiers.shiftKey && start >= 0) {
      const range = points.slice(Math.min(start, end), Math.max(start, end) + 1).map(point => point.id);
      const rangeIds = new Set(additive ? [...selection, ...range] : range);
      next = points.filter(point => rangeIds.has(point.id)).map(point => point.id);
    } else {
      const ids = new Set(additive ? selection : []);
      if (additive && ids.has(id)) ids.delete(id); else ids.add(id);
      next = points.filter(point => ids.has(point.id)).map(point => point.id);
      anchor.current = id;
    }
    activePoint.current = id;
    onSelectionChange(next);
    onSelectTarget(id);
  }

  function openMenu(x: number, y: number, afterPointId: string | null, trigger: HTMLElement) {
    if (!active) return;
    returnFocus.current = trigger;
    setMenu({ x, y, afterPointId, sequence: ++sequence.current });
  }

  function contextMenu(event: MouseEvent, pointId: string | null) {
    event.preventDefault(); event.stopPropagation();
    if (pointId) {
      if (!selected.has(pointId)) selectPoint(pointId, {});
      else { activePoint.current = pointId; onSelectTarget(pointId); }
    }
    openMenu(event.clientX, event.clientY, pointId, event.currentTarget as HTMLElement);
  }

  function listKeyDown(event: KeyboardEvent<HTMLDivElement>) {
    if (menu) return;
    const focusedId = (event.target as HTMLElement).closest<HTMLElement>('[data-program-point-id]')?.dataset.programPointId;
    if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'a') {
      event.preventDefault(); onSelectionChange(points.map(point => point.id)); return;
    }
    if (event.key === 'ContextMenu' || (event.shiftKey && event.key === 'F10')) {
      event.preventDefault();
      const id = focusedId ?? (activePoint.current && points.some(point => point.id === activePoint.current) ? activePoint.current : lastSelected);
      if (id && !selected.has(id)) selectPoint(id, {});
      const trigger = id ? rowElements.current.get(id) ?? list.current : list.current;
      if (trigger) {
        const bounds = trigger.getBoundingClientRect();
        openMenu(bounds.left + Math.min(180, bounds.width / 2), bounds.bottom, id, trigger);
      }
      return;
    }
    if (event.key === 'Delete' && onRemovePoints && selection.length && !running) {
      event.preventDefault(); onRemovePoints(selection); return;
    }
    if (!['ArrowUp', 'ArrowDown', 'Home', 'End'].includes(event.key) || !points.length) return;
    event.preventDefault();
    const current = points.findIndex(point => point.id === (focusedId ?? activePoint.current ?? lastSelected));
    const next = event.key === 'Home' ? 0 : event.key === 'End' ? points.length - 1
      : current < 0 ? (event.key === 'ArrowDown' ? 0 : points.length - 1)
      : Math.max(0, Math.min(points.length - 1, current + (event.key === 'ArrowDown' ? 1 : -1)));
    const id = points[next].id;
    selectPoint(id, event);
    rowElements.current.get(id)?.focus({ preventScroll: true });
    rowElements.current.get(id)?.scrollIntoView({ block: 'nearest' });
  }

  function menuKeyDown(event: KeyboardEvent<HTMLDivElement>) {
    if (event.key === 'Tab') { event.preventDefault(); closeMenu(); return; }
    if (!['ArrowUp', 'ArrowDown', 'Home', 'End'].includes(event.key)) return;
    event.preventDefault();
    const buttons = [...event.currentTarget.querySelectorAll<HTMLButtonElement>('[role="menuitem"]:not(:disabled)')];
    if (!buttons.length) return;
    const index = buttons.indexOf(document.activeElement as HTMLButtonElement);
    const next = event.key === 'Home' ? 0 : event.key === 'End' ? buttons.length - 1
      : (index + (event.key === 'ArrowDown' ? 1 : -1) + buttons.length) % buttons.length;
    buttons[next].focus({ preventScroll: true });
  }

  return <section className="program-editor" aria-label={profile.brand === 'ABB' ? 'MoveL 程序编辑区' : `${profile.brand} ${profile.language} 程序编辑区`}
    data-program-brand={profile.brand} data-program-language={profile.language}>
    <header className="program-editor-header"><div><h2>运动程序</h2><span>{profile.brand} · {profile.language} · {points.length} 条 {profile.linearInstruction}</span></div><div className="program-editor-actions">
      <button type="button" onClick={() => onInsertCurrent(insertionPoint)} disabled={Boolean(insertDisabledReason)} title={insertDisabledReason ?? undefined}>＋ 插入当前姿态</button>
      <button type="button" onClick={() => onConvertToPath(selection)} disabled={running || !selection.length}>转为 Path</button>
    </div></header>
    {onRun && <div className="program-run-controls">
      <button type="button" className="program-run-button" disabled={running || Boolean(runDisabledReason) || !points.length} title={runDisabledReason ?? undefined} onClick={onRun}>运行程序</button>
      <button type="button" className="program-stop-button" disabled={!running || !onStop} onClick={onStop}>停止运行</button>
      {onLoopChange && <label><input type="checkbox" checked={loop} disabled={running} onChange={event => onLoopChange(event.target.checked)}/>循环运行</label>}
      <span role="status" aria-label="程序运行状态">{runStatus || runDisabledReason || (running ? '正在运行仿真…' : '尚未运行')}</span>
    </div>}
    <p className={`program-editor-hint${insertDisabledReason ? ' program-capture-unavailable' : ''}`} role={insertDisabledReason ? 'status' : undefined}>
      {insertDisabledReason || '右键插入当前姿态；按 Ctrl / ⌘ 多选，按 Shift 连选，再转为 Path。'}
    </p>
    <div ref={list} className="program-row-list" role="listbox" aria-label={`${profile.linearInstruction} 程序指令`} aria-multiselectable="true" tabIndex={0}
      onKeyDown={listKeyDown} onContextMenu={event => contextMenu(event, null)}>
      {points.length ? points.map((point, index) => <button key={point.id} type="button" role="option" aria-selected={selected.has(point.id)} aria-current={executingPointId === point.id ? 'step' : undefined}
        aria-label={`${profile.linearInstruction} ${formattedPoints[index].reference}`} className={`program-row${selected.has(point.id) ? ' selected' : ''}${executingPointId === point.id ? ' executing' : ''}`} tabIndex={point.id === focusPointId ? 0 : -1}
        data-program-point-id={point.id} data-program-point-name={point.pointName} data-program-target-reference={formattedPoints[index].reference}
        data-program-path-id={point.pathId ?? ''} data-executing={executingPointId === point.id}
        ref={element => { if (element) rowElements.current.set(point.id, element); else rowElements.current.delete(point.id); }}
        onClick={event => { closeMenu(false); selectPoint(point.id, event); }} onContextMenu={event => contextMenu(event, point.id)}>
        <span className="program-line-number">{String(index + 1).padStart(2, '0')}</span>
        <span className="program-row-content"><span className="program-row-title"><b>{profile.linearInstruction}</b><strong>{formattedPoints[index].reference}</strong>{profile.brand !== 'ABB' && <small>{point.pointName}</small>}<span className="program-speed">{numberText(point.speedMmPerSecond)} mm/s</span>{executingPointId === point.id && <small className="program-executing-label">执行中</small>}</span>
          <span className="program-pose-position">{['X', 'Y', 'Z'].map((axis, i) => <span key={axis}><i>{axis}</i>{numberText(point.pose.position[i])}</span>)}<small>mm</small></span>
          <span className="program-pose-orientation">{formattedPoints[index].orientationLabel} [{formattedPoints[index].orientationValues.map(value => numberText(value, 5)).join(', ')}]
            {point.configuration && <span>robconf [{point.configuration.join(', ')}]</span>}
            {profile.brand === 'FANUC' && <span>J1–J6 [{point.joints.map(value => numberText(value)).join(', ')}]°</span>}
            {profile.brand === 'Yaskawa' && <span>RCONF / TOOL 待配置</span>}
          </span>
        </span>
        <span className={`program-path-tag${point.pathId ? '' : ' ungrouped'}`}>{point.pathId ? pathNames.get(point.pathId) ?? 'Path' : '未分组'}</span>
      </button>) : <div className="program-editor-empty"><span aria-hidden="true">＋</span><strong>从当前机器人姿态开始</strong><p>在这里右键，选择“插入当前姿态（{profile.linearInstruction}）”。</p></div>}
    </div>
    <details className="program-template" data-program-template-language={profile.language}>
      <summary>{profile.brand} {profile.language} 程序模板 <span>仿真预览</span></summary>
      <p className="program-template-notice">{template.notice}</p>
      <textarea className="program-template-code" aria-label={`${profile.brand} ${profile.language} 程序模板`} readOnly spellCheck={false} value={template.code}/>
    </details>
    <footer className="program-editor-footer"><span>{selection.length ? `已选择 ${selection.length} 条 ${profile.linearInstruction}` : '未选择指令'}</span><span>Path 分组保留程序顺序</span></footer>
    {menu && active && <div ref={menuElement} className="program-action-menu" role="menu" aria-label="程序右键菜单" tabIndex={-1}
      style={{ left: menu.x, top: menu.y }} onKeyDown={menuKeyDown} onContextMenu={event => event.preventDefault()}>
      <button type="button" role="menuitem" disabled={Boolean(insertDisabledReason)} title={insertDisabledReason ?? undefined}
        onClick={() => { const after = menu.afterPointId; closeMenu(); onInsertCurrent(after); }}>插入当前姿态（{profile.linearInstruction}）</button>
      {insertDisabledReason && <p className="program-menu-reason">{insertDisabledReason}</p>}
      <button type="button" role="menuitem" disabled={running || !selection.length} onClick={() => { closeMenu(); onConvertToPath(selection); }}>转为 Path{selection.length ? `（${selection.length} 条）` : ''}</button>
      {onRemovePoints && <><div className="program-menu-divider" role="separator"/><button type="button" role="menuitem" className="program-menu-delete" disabled={running || !selection.length}
        onClick={() => { closeMenu(); onRemovePoints(selection); }}>删除所选指令</button></>}
    </div>}
  </section>;
}
