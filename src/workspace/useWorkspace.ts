import { useMemo, useRef, useState } from 'react';
import { COLLISION_LIMITS, type CollisionZone } from '../robot/collision';
import type { Vec3 } from '../robot/types';

export type WorkspaceView = 'scene' | 'program';
export interface DangerZoneDraft {
  id: string;
  name: string;
  enabled: boolean;
  center: string[];
  size: string[];
}

const defaultGeometry = () => ({ enabled: true, center: ['220', '-160', '350'], size: ['160', '160', '200'] });
const firstZone = (): DangerZoneDraft => ({ id: 'danger-zone-1', name: '危险区 1', ...defaultGeometry() });

function zoneVector(values: string[], label: string): Vec3 {
  if (values.length !== 3 || values.some(value => !value.trim() || !Number.isFinite(Number(value)))) throw new Error(`${label}需要填写完整的有限数值。`);
  return values.map(Number) as Vec3;
}

/** Shared workspace state remains mounted when scene/program views switch. */
export function useWorkspace() {
  const [activeView, setActiveView] = useState<WorkspaceView>('scene');
  const [leftCollapsed, setLeftCollapsed] = useState(false);
  const [zones, setZones] = useState<DangerZoneDraft[]>(() => [firstZone()]);
  const [selectedZoneId, setSelectedZoneId] = useState<string | null>('danger-zone-1');
  const nextZoneNumber = useRef(2);
  const selectedZone = zones.find(zone => zone.id === selectedZoneId) ?? null;
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
    setZones(current => current.map(zone => zone.id === id ? { ...zone, ...patch } : zone));
  }
  function addZone() {
    if (zones.length >= COLLISION_LIMITS.maximumZones) return;
    const number = nextZoneNumber.current++;
    const zone: DangerZoneDraft = { id: crypto.randomUUID(), name: `危险区 ${number}`, ...defaultGeometry() };
    setZones(current => [...current, zone]);
    setSelectedZoneId(zone.id);
  }
  function removeSelectedZone() {
    const remaining = zones.filter(zone => zone.id !== selectedZoneId);
    setZones(remaining);
    setSelectedZoneId(remaining[0]?.id ?? null);
  }
  function resetSelectedZone() {
    if (selectedZoneId) editZone(selectedZoneId, defaultGeometry());
  }

  return {
    activeView, setActiveView, leftCollapsed, setLeftCollapsed,
    zones, selectedZoneId, selectedZone, setSelectedZoneId,
    zoneConfiguration, editZone, addZone, removeSelectedZone, resetSelectedZone,
    canAddZone: zones.length < COLLISION_LIMITS.maximumZones,
  };
}
