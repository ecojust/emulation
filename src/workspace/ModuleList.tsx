import Icon from '../ui/Icon';
import { useLayoutEffect, useRef, type KeyboardEvent } from 'react';
import type { SourceDocument } from '../programming/documents';
import './module-list.css';

export interface ModuleListProps {
  files: readonly SourceDocument[];
  activeFileId: string | null;
  onSelectFile: (id: string) => void;
  disabled?: boolean;
}

/** File selection is shared with the editor, including cross-file diagnostics. */
export default function ModuleList({ files, activeFileId, onSelectFile, disabled = false }: ModuleListProps) {
  const list = useRef<HTMLDivElement>(null);
  const elements = useRef(new Map<string, HTMLButtonElement>());
  const selectedId = files.some(file => file.id === activeFileId) ? activeFileId : null;
  const focusId = selectedId ?? files[0]?.id;

  function reveal(id: string) {
    const element = elements.current.get(id);
    if (!element || !list.current) return;
    const bounds = element.getBoundingClientRect(), viewport = list.current.getBoundingClientRect();
    if (bounds.top < viewport.top) list.current.scrollTop -= viewport.top - bounds.top;
    else if (bounds.bottom > viewport.bottom) list.current.scrollTop += bounds.bottom - viewport.bottom;
  }

  // Diagnostic navigation can select a file without moving keyboard focus here.
  useLayoutEffect(() => { if (selectedId) reveal(selectedId); }, [selectedId]);

  function navigate(event: KeyboardEvent<HTMLDivElement>) {
    if (disabled || !files.length || !['ArrowUp', 'ArrowDown', 'Home', 'End'].includes(event.key)) return;
    event.preventDefault();
    const currentId = (event.target as HTMLElement).closest<HTMLElement>('[data-source-file-id]')?.dataset.sourceFileId ?? focusId;
    const currentIndex = Math.max(0, files.findIndex(file => file.id === currentId));
    const nextIndex = event.key === 'Home' ? 0 : event.key === 'End' ? files.length - 1
      : Math.max(0, Math.min(files.length - 1, currentIndex + (event.key === 'ArrowDown' ? 1 : -1)));
    const file = files[nextIndex];
    onSelectFile(file.id);
    const element = elements.current.get(file.id);
    element?.focus({ preventScroll: true });
    // Scroll only the local module list; navigation never moves the workspace.
    reveal(file.id);
  }

  return <div className="module-browser">
    <div ref={list} className="module-list" role="listbox" aria-label="程序模块列表" aria-disabled={disabled || undefined} tabIndex={files.length ? -1 : 0} onKeyDown={navigate}>
      {files.map(file => <button type="button" role="option" key={file.id}
        ref={element => { if (element) elements.current.set(file.id, element); else elements.current.delete(file.id); }}
        aria-label={file.name} aria-selected={selectedId === file.id} disabled={disabled}
        tabIndex={file.id === focusId ? 0 : -1} data-source-file-id={file.id} data-source-dirty={file.dirty}
        title={file.name + (file.dirty ? ' · 已修改' : '')} onClick={() => onSelectFile(file.id)}>
        <Icon name="file" size={15} className="module-file-icon"/><span className="module-file-name">{file.name}</span>
        {file.dirty && <Icon name="dot" size={9} className="module-file-dirty"/>}
      </button>)}
    </div>
    {!files.length && <p className="module-list-empty">还没有模块文件。可新建、导入模块，或在场景中将 Path 转换成程序。</p>}
  </div>;
}
