import { useEffect, useLayoutEffect, useRef, useState } from 'react';

export interface SceneMenuAction {
  label: string;
  disabled?: boolean;
  title?: string;
  onSelect: () => void;
}

export default function SceneObjectMenu({ x, y, label, trigger, actions, onClose }: {
  x: number; y: number; label: string; trigger: HTMLElement;
  actions: SceneMenuAction[]; onClose: () => void;
}) {
  const host = useRef<HTMLDivElement>(null);
  const [position, setPosition] = useState({ x, y });
  const close = (restore = false) => {
    onClose();
    if (restore && trigger.isConnected) trigger.focus({ preventScroll: true });
  };
  useLayoutEffect(() => {
    const box = host.current?.getBoundingClientRect();
    if (box) setPosition({ x: Math.max(8, Math.min(x, innerWidth - box.width - 8)), y: Math.max(8, Math.min(y, innerHeight - box.height - 8)) });
    const first = host.current?.querySelector<HTMLButtonElement>('button:not(:disabled)');
    (first ?? host.current)?.focus({ preventScroll: true });
  }, [x, y, label, trigger]);
  useEffect(() => {
    const outside = (event: Event) => { if (!host.current?.contains(event.target as Node)) close(); };
    const resize = () => close();
    document.addEventListener('pointerdown', outside, true);
    document.addEventListener('wheel', outside, true);
    window.addEventListener('resize', resize);
    return () => {
      document.removeEventListener('pointerdown', outside, true);
      document.removeEventListener('wheel', outside, true);
      window.removeEventListener('resize', resize);
    };
  }, [onClose]);
  return <div ref={host} className="point-action-menu" role="menu" aria-label={label} tabIndex={-1}
    style={{ left: position.x, top: position.y }} onKeyDown={event => {
      if (event.key === 'Escape' || event.key === 'Tab') { event.preventDefault(); close(true); return; }
      const buttons = [...(host.current?.querySelectorAll<HTMLButtonElement>('button:not(:disabled)') ?? [])];
      if (!buttons.length || !['ArrowUp', 'ArrowDown', 'Home', 'End'].includes(event.key)) return;
      event.preventDefault();
      const index = buttons.indexOf(document.activeElement as HTMLButtonElement);
      const next = event.key === 'Home' ? 0 : event.key === 'End' ? buttons.length - 1 : (index + (event.key === 'ArrowUp' ? -1 : 1) + buttons.length) % buttons.length;
      buttons[next].focus({ preventScroll: true });
    }}>
    {actions.map(action => <button key={action.label} type="button" role="menuitem" disabled={action.disabled} title={action.title}
      onClick={() => { close(true); action.onSelect(); }}>{action.label}</button>)}
  </div>;
}
