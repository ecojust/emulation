import { useLayoutEffect, useRef } from 'react';

export default function ObjectRemovalDialog({ kind, name, count, onCancel, onConfirm }: {
  kind: 'path' | 'target'; name: string; count: number; onCancel: () => void; onConfirm: () => void;
}) {
  const host = useRef<HTMLElement>(null);
  useLayoutEffect(() => {
    const previous = document.activeElement as HTMLElement | null;
    host.current?.querySelector<HTMLButtonElement>('button')?.focus({ preventScroll: true });
    return () => { if (previous?.isConnected) previous.focus({ preventScroll: true }); };
  }, []);
  const label = kind === 'path' ? '删除 Path' : '删除目标点';
  return <div className="source-close-backdrop"><section ref={host} className="source-close-dialog" role="dialog" aria-modal="true" aria-label={label}
    onKeyDown={event => {
      if (event.key === 'Escape') { event.preventDefault(); onCancel(); }
      if (event.key === 'Tab') {
        const buttons = [...(host.current?.querySelectorAll<HTMLButtonElement>('button') ?? [])];
        if (event.shiftKey && document.activeElement === buttons[0]) { event.preventDefault(); buttons.at(-1)?.focus(); }
        else if (!event.shiftKey && document.activeElement === buttons.at(-1)) { event.preventDefault(); buttons[0]?.focus(); }
      }
    }}><h2>{label}</h2><p>将删除 {name}{kind === 'path' ? ` 及其中的 ${count} 个点` : ''}。已生成的模块文件保持不变。</p>
    <div className="source-close-actions"><button type="button" onClick={onCancel}>取消</button><button type="button" className="source-discard" onClick={onConfirm}>删除</button></div>
  </section></div>;
}
