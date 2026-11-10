import { useEffect, useRef, useState } from 'react';
import Icon from '../ui/Icon';
import { importSurface } from './import';
import { SURFACE_LIMITS, type SceneSurface, type SurfaceUnit } from './types';
import './surface.css';

export interface SurfaceImportDialogProps {
  onImport: (surface: SceneSurface) => void | Promise<void>;
  onClose: () => void;
}

/** The chosen file is copied into project-owned geometry; no external file path is retained. */
export default function SurfaceImportDialog({ onImport, onClose }: SurfaceImportDialogProps) {
  const dialog = useRef<HTMLDialogElement>(null), active = useRef<AbortController | null>(null), mounted = useRef(true);
  const [file, setFile] = useState<File | null>(null), [name, setName] = useState('');
  const [unit, setUnit] = useState<SurfaceUnit>('mm');
  const [busy, setBusy] = useState(false), [error, setError] = useState('');
  const glb = file?.name.toLowerCase().endsWith('.glb') ?? false;
  useEffect(() => {
    mounted.current = true; dialog.current?.showModal();
    return () => { mounted.current = false; active.current?.abort(); active.current = null; };
  }, []);
  function close() { active.current?.abort(); active.current = null; onClose(); }
  function choose(value: File | undefined) {
    active.current?.abort(); active.current = null; setBusy(false); setError(''); setFile(null);
    if (!value) return;
    if (!/\.(stl|obj|glb)$/i.test(value.name)) { setError('请选择 STL、OBJ 或 GLB 模型文件。'); return; }
    if (value.size > SURFACE_LIMITS.fileBytes) { setError(`文件不能超过 ${SURFACE_LIMITS.fileBytes / 1024 / 1024} MiB。`); return; }
    setFile(value); setName(value.name.replace(/\.[^.]+$/, '').slice(0, 80));
    setUnit(value.name.toLowerCase().endsWith('.glb') ? 'm' : 'mm');
  }
  async function submit() {
    if (!file || busy) return;
    const operation = new AbortController(); active.current?.abort(); active.current = operation; setBusy(true); setError('');
    try {
      if (!name.trim()) throw new Error('请填写模型名称。');
      const surface = await importSurface(file, { name: name.trim(), unit: glb ? 'm' : unit, signal: operation.signal });
      if (!mounted.current || active.current !== operation || operation.signal.aborted) return;
      await onImport(surface);
      if (!mounted.current || active.current !== operation || operation.signal.aborted) return;
      active.current = null; setBusy(false); onClose();
    } catch (cause) {
      if (!mounted.current || active.current !== operation || operation.signal.aborted) return;
      active.current = null; setBusy(false); setError(cause instanceof Error ? cause.message : '模型导入失败。');
    }
  }
  return <dialog ref={dialog} className="surface-dialog surface-import-dialog" aria-label="导入模型" onCancel={event => { event.preventDefault(); close(); }}>
    <header><div><h2>导入模型</h2><p>添加用于表面点选与加工路径生成的工件</p></div><button type="button" aria-label="关闭模型导入" onClick={close}><Icon name="close"/></button></header>
    <form onSubmit={event => { event.preventDefault(); void submit(); }}>
      <div className="surface-dialog-body">
        <label className="field-label surface-file-field">模型文件<input type="file" accept=".stl,.obj,.glb" aria-label="模型文件" disabled={busy} onChange={event => choose(event.target.files?.[0])}/></label>
        <p className="field-hint">支持 STL、OBJ、GLB；单文件不超过 {SURFACE_LIMITS.fileBytes / 1024 / 1024} MiB。模型会保存在项目内，重新打开时无需原始文件。</p>
        {file && <p className="surface-file-summary"><Icon name="file" size={15}/><span>{file.name}</span><small>{(file.size / 1024 / 1024).toFixed(2)} MiB</small></p>}
        <label className="field-label">模型名称<input aria-label="模型名称" value={name} maxLength={80} disabled={busy} onChange={event => setName(event.target.value)}/></label>
        <label className="field-label">原文件单位<select aria-label="模型文件单位" value={glb ? 'm' : unit} disabled={busy || glb} onChange={event => setUnit(event.target.value as SurfaceUnit)}><option value="mm">毫米（mm）</option><option value="m">米（m）</option></select></label>
        <p className="field-hint">{glb ? 'GLB 按标准米制和 Y 向上坐标导入，自动转换为场景的毫米与 Z 向上坐标。' : 'STL／OBJ 不规定长度单位，请按原文件选择；导入后场景统一使用毫米。'}</p>
        {busy && <p className="surface-feedback" role="status">正在读取并检查模型…</p>}
        {error && <p className="surface-feedback surface-error" role="alert">{error}</p>}
      </div>
      <footer><button type="button" onClick={close}>取消</button><button type="submit" className="surface-primary" disabled={!file || !name.trim() || busy}><Icon name="plus" size={15}/>{busy ? '正在导入…' : '导入场景'}</button></footer>
    </form>
  </dialog>;
}
