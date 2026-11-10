import { useEffect, useMemo, useRef, useState } from 'react';
import { COLLISION_LIMITS, type CollisionZone } from '../robot/collision';
import type { Vec3 } from '../robot/types';

export type WorkspaceView = 'scene' | 'program';
export type WorkspaceSelection = { kind: 'robot' } | { kind: 'base' } | { kind: 'flange' } | { kind: 'joint'; index: number } | { kind: 'zone' | 'path' | 'target' | 'frame' | 'tool'; id: string };
export interface WorkspaceInitialState {
  activeView?: WorkspaceView;
  leftCollapsed?: boolean;
  zones?: readonly DangerZoneDraft[];
  selection?: WorkspaceSelection;
}
export interface WorkspaceOptions {
  initial?: WorkspaceInitialState;
  modelKey?: string;
  pathIds?: readonly string[];
  targetIds?: readonly string[];
  frameIds?: readonly string[];
  toolIds?: readonly string[];
}
const ROBOT_SELECTION: WorkspaceSelection = Object.freeze({ kind: 'robot' });
export function sanitizeWorkspaceSelection(selection: WorkspaceSelection, scope: {
  zoneIds: readonly string[]; pathIds?: readonly string[]; targetIds?: readonly string[]; frameIds?: readonly string[]; toolIds?: readonly string[];
}, previousModelKey?: string, modelKey?: string): WorkspaceSelection {
  if (previousModelKey !== modelKey && selection.kind !== 'zone') return ROBOT_SELECTION;
  if (selection.kind === 'robot' || selection.kind === 'base' || selection.kind === 'flange') return selection;
  if (selection.kind === 'joint') return Number.isInteger(selection.index) && selection.index >= 0 && selection.index < 6 ? selection : ROBOT_SELECTION;
  const ids = selection.kind === 'zone' ? scope.zoneIds : selection.kind === 'path' ? scope.pathIds : selection.kind === 'frame' ? scope.frameIds : selection.kind === 'tool' ? scope.toolIds : scope.targetIds;
  return ids?.includes(selection.id) ? selection : ROBOT_SELECTION;
}
export interface DangerZoneDraft {
  id: string;
  name: string;
  enabled: boolean;
  center: string[];
  size: string[];
}

const defaultGeometry = () => ({ enabled: true, center: ['220', '-160', '350'], size: ['160', '160', '200'] });
export const createDefaultDangerZone = (): DangerZoneDraft => ({ id: 'danger-zone-1', name: '危险区 1', ...defaultGeometry() });

function zoneVector(values: string[], label: string): Vec3 {
  if (values.length !== 3 || values.some(value => !value.trim() || !Number.isFinite(Number(value)))) throw new Error(`${label}需要填写完整的有限数值。`);
  return values.map(Number) as Vec3;
}

/** Shared workspace state remains mounted when scene/program views switch. */
export function useWorkspace(options: WorkspaceOptions = {}) {
  const [activeView, setActiveView] = useState<WorkspaceView>(options.initial?.activeView ?? 'scene');
  const [leftCollapsed, setLeftCollapsed] = useState(options.initial?.leftCollapsed ?? false);
  const [zones, setZones] = useState<DangerZoneDraft[]>(() => options.initial?.zones ? structuredClone([...options.initial.zones]) : [createDefaultDangerZone()]);
  const [selectionState, setSelectionState] = useState<{ modelKey?: string; selection: WorkspaceSelection }>(() => ({ modelKey: options.modelKey, selection: options.initial?.selection ?? ROBOT_SELECTION }));
  const nextZoneNumber = useRef((options.initial?.zones?.length ?? 1) + 1);
  const scope = { zoneIds: zones.map(zone => zone.id), pathIds: options.pathIds, targetIds: options.targetIds, frameIds: options.frameIds, toolIds: options.toolIds };
  const selection = sanitizeWorkspaceSelection(selectionState.selection, scope, selectionState.modelKey, options.modelKey);
  const selectedZoneId = selection.kind === 'zone' ? selection.id : null;
  const selectedZone = zones.find(zone => zone.id === selectedZoneId) ?? null;
  useEffect(() => {
    if (selectionState.modelKey !== options.modelKey || selectionState.selection !== selection) {
      setSelectionState({ modelKey: options.modelKey, selection });
    }
  }, [options.modelKey, selectionState.modelKey, selectionState.selection, selection]);
  function selectObject(next: WorkspaceSelection) {
    // Creation and selection may be batched in one event. Validate against the
    // next render's committed object lists, not the caller's previous lists.
    const value: WorkspaceSelection = next.kind === 'robot' ? ROBOT_SELECTION : Object.freeze({ ...next });
    setSelectionState({ modelKey: options.modelKey, selection: value });
  }
  /** Kept while callers migrate to discriminated object selection. */
  function setSelectedZoneId(id: string | null) { selectObject(id ? { kind: 'zone', id } : ROBOT_SELECTION); }
  const zoneConfiguration = useMemo(() => {
    const validZones: CollisionZone[] = [];
    const errors: { id: string; message: string }[] = [];
    for (const draft of zones) {
      try {
        const centerMm = zoneVector(draft.center, '中心');
        const sizeMm = zoneVector(draft.size, '尺寸');
        if (centerMm.some(value => Math.abs(value) > COLLISION_LIMITS.coordinateMm)) throw new Error('中心须在 ±1000000 mm 以内。');
        if (sizeMm.some(value => value <= 0 || value > COLLISION_LIMITS.sizeMm)) throw new Error('尺寸必须大于 0 且不超过 1000000 mm。');
        validZones.push({ id: draft.id, enabled: draft.enabled, centerMm, sizeMm });
      } catch (cause) {
        errors.push({ id: draft.id, message: `${draft.name.trim() || '未命名危险区'}：${cause instanceof Error ? cause.message : '配置无效'}` });
      }
    }
    return { validZones, errors };
  }, [zones]);

  function editZone(id: string, patch: Partial<Omit<DangerZoneDraft, 'id'>>) {
    setZones(current => current.map(zone => zone.id === id ? { ...zone, ...patch,
      center: patch.center ? [...patch.center] : zone.center, size: patch.size ? [...patch.size] : zone.size,
    } : zone));
  }
  function addZone() {
    if (zones.length >= COLLISION_LIMITS.maximumZones) return;
    const number = nextZoneNumber.current++;
    const zone: DangerZoneDraft = { id: crypto.randomUUID(), name: `危险区 ${number}`, ...defaultGeometry() };
    setZones(current => current.length >= COLLISION_LIMITS.maximumZones ? current : [...current, zone]);
    // Both updates commit together; the new ID is not in this render's scope yet.
    setSelectionState({ modelKey: options.modelKey, selection: { kind: 'zone', id: zone.id } });
    return zone.id;
  }
  function removeSelectedZone() {
    const remaining = zones.filter(zone => zone.id !== selectedZoneId);
    setZones(remaining);
    setSelectionState({ modelKey: options.modelKey, selection: remaining[0] ? { kind: 'zone', id: remaining[0].id } : ROBOT_SELECTION });
  }
  function resetSelectedZone() {
    if (selectedZoneId) editZone(selectedZoneId, defaultGeometry());
  }

  return {
    activeView, setActiveView, leftCollapsed, setLeftCollapsed,
    selection, selectObject,
    zones, setZones, selectedZoneId, selectedZone, setSelectedZoneId,
    zoneConfiguration, editZone, addZone, removeSelectedZone, resetSelectedZone,
    canAddZone: zones.length < COLLISION_LIMITS.maximumZones,
  };
}
