import { useEffect, useMemo, useRef, useState, type ReactNode } from 'react';
import Icon from '../ui/Icon';
import ThemeToggle from '../theme/ThemeToggle';
import type { SourceProjectRepository } from '../programming/storage';
import { ProjectRepository } from './repository';
import { PROJECT_MODELS, projectName, projectSummary, validateThumbnail, type ProjectScene, type SimulationProject, type ProjectSummary } from './schema';
import './projects.css';

export interface ProjectWorkspaceProps {
  project: SimulationProject;
  sourceRepository: SourceProjectRepository;
  /** Leave only the preload barrier; never discard an opened editor with this callback. */
  onCancelOpen?: () => void;
  /** Save before native close without navigating or replacing the last real cover. */
  onSaveScene: (scene: ProjectScene) => Promise<void>;
  /** Caller captures the actual renderer after stopping motion/preview and flushing source drafts. */
  onReturnHome: (scene: ProjectScene, thumbnail: string) => Promise<void>;
}
export interface ProjectAppProps {
  renderWorkspace: (props: ProjectWorkspaceProps) => ReactNode;
  repository?: ProjectRepository;
}
/** Project storage belongs to this persistent shell; each editor mounts with a fresh project identity. */
export default function ProjectApp({ renderWorkspace, repository: supplied }: ProjectAppProps) {
  const [repository] = useState(() => supplied ?? new ProjectRepository());
  const [projects, setProjects] = useState<ProjectSummary[]>([]);
  const [active, setActive] = useState<SimulationProject | null>(null);
  const [loading, setLoading] = useState(true), [busy, setBusy] = useState(false), [error, setError] = useState('');
  const [dialog, setDialog] = useState<{ kind: 'create' } | { kind: 'rename' | 'delete'; project: ProjectSummary } | null>(null);
  const [name, setName] = useState(''), [model, setModel] = useState('irb1100-4-0.58');
  const operation = useRef(false), mounted = useRef(true);
  const sourceRepository = useMemo(() => active ? repository.sourcesFor(active.id) : null, [repository, active?.id]);
  async function refresh() { const next = await repository.list(); if (mounted.current) setProjects(next); }
  async function initialize() {
    setLoading(true); setError('');
    try { await refresh(); await repository.initialize(); await refresh(); }
    catch (cause) { if (mounted.current) setError(cause instanceof Error ? cause.message : '项目列表加载失败。'); }
    finally { if (mounted.current) setLoading(false); }
  }
  useEffect(() => { mounted.current = true; void initialize(); return () => { mounted.current = false; }; }, [repository]);
  async function action(work: () => Promise<void>) {
    if (operation.current) return;
    operation.current = true; setBusy(true); setError('');
    try { await work(); }
    catch (cause) { setError(cause instanceof Error ? cause.message : '项目操作未完成，请重试。'); }
    finally { operation.current = false; if (mounted.current) setBusy(false); }
  }
  function openCreate() { setName(''); setModel('irb1100-4-0.58'); setDialog({ kind: 'create' }); setError(''); }
  if (active && sourceRepository) {
    const project = active;
    return <div className="project-editor" key={project.id} data-project-id={project.id}>{renderWorkspace({ project, sourceRepository, onCancelOpen: () => { if (!operation.current) setActive(null); },
      onSaveScene: async scene => {
        if (operation.current) throw new Error('项目正在保存，请稍后重试。');
        operation.current = true;
        try {
          const saved = await repository.saveScene(project.id, scene);
          if (mounted.current) setProjects(current => current.map(item => item.id === project.id ? projectSummary(saved) : item));
        } finally { operation.current = false; }
      },
      onReturnHome: async (scene, thumbnail) => {
        // Do not catch here: save failure must retain the mounted editor and its data.
        validateThumbnail(thumbnail);
        if (operation.current) throw new Error('项目正在保存，请稍后重试。');
        operation.current = true;
        try {
          await repository.saveScene(project.id, scene, thumbnail);
          const next = await repository.list();
          if (mounted.current) { setProjects(next); setActive(null); setError(''); }
        } finally { operation.current = false; }
      },
    })}</div>;
  }
  return <main className="project-home" aria-label="项目管理">
    <header className="project-home-header" inert={Boolean(dialog)}><div className="project-home-brand"><Icon name="cube" size={28}/><div><strong>emulation</strong><span>机器人仿真项目</span></div></div>
      <div className="project-home-header-actions"><ThemeToggle/><button type="button" className="project-primary" onClick={openCreate} disabled={loading || busy}><Icon name="plus" size={16}/>新建项目</button></div></header>
    <section className="project-home-content" aria-label="项目列表" inert={Boolean(dialog)}>
      <div className="project-list-heading"><div><h1>我的项目</h1><p>从实际场景继续编程与仿真</p></div><span>{projects.length} 个项目</span></div>
      {error && !dialog && <div className="project-error" role="alert">{error}{!dialog && <button type="button" disabled={busy || loading} onClick={() => void initialize()}>重新加载</button>}</div>}
      {loading ? <p role="status">正在读取本地项目…</p> : projects.length ? <div className="project-grid">{projects.map(project => {
        const robot = PROJECT_MODELS.find(item => item.id === project.activeModelId)!;
        return <article className="project-card" key={project.id} data-project-card={project.id}>
          <button type="button" className="project-card-open" aria-label={'打开项目 ' + project.name} disabled={busy} onClick={() => void action(async () => { setActive(await repository.load(project.id)); })}>
            <div className="project-cover">{project.thumbnail ? <img loading="lazy" decoding="async" src={project.thumbnail} alt={project.name + '的三维场景封面'}/> : <div className="project-cover-pending"><Icon name="cube" size={36}/><span>首次返回主页后生成场景封面</span></div>}</div>
            <div className="project-card-info"><h2>{project.name}</h2><p>{robot.name}{project.robotPresent === false ? ' · 本体已删除' : ''}</p><time dateTime={project.updatedAt}>{new Date(project.updatedAt).toLocaleString()}</time></div>
          </button>
          <div className="project-card-actions"><button type="button" disabled={busy} onClick={() => { setName(project.name); setDialog({ kind: 'rename', project }); setError(''); }} aria-label={'重命名项目 ' + project.name}><Icon name="file" size={14}/>重命名</button><button type="button" disabled={busy} onClick={() => { setDialog({ kind: 'delete', project }); setError(''); }} aria-label={'删除项目 ' + project.name}><Icon name="trash" size={14}/>删除</button></div>
        </article>;
      })}</div> : !error && <div className="project-empty"><Icon name="cube" size={48}/><h2>创建第一个仿真项目</h2><p>选择机器人，保存你的场景、工具、路径与程序。</p><button type="button" className="project-primary" onClick={openCreate}><Icon name="plus" size={16}/>新建项目</button></div>}
    </section>
    {dialog && <div className="project-modal-backdrop" onKeyDown={event => { if (event.key === 'Escape' && !busy) setDialog(null); if (event.key === 'Tab') { const focusable = [...event.currentTarget.querySelectorAll<HTMLElement>('button:not(:disabled),input:not(:disabled),select:not(:disabled)')]; const first = focusable[0], last = focusable.at(-1); if (event.shiftKey && document.activeElement === first) { event.preventDefault(); last?.focus(); } else if (!event.shiftKey && document.activeElement === last) { event.preventDefault(); first?.focus(); } } }}><section role="dialog" aria-modal="true" aria-labelledby="project-dialog-title" className="project-modal">
      <header><h2 id="project-dialog-title">{dialog.kind === 'create' ? '新建项目' : dialog.kind === 'rename' ? '重命名项目' : '删除项目'}</h2><button type="button" disabled={busy} aria-label="关闭项目对话框" onClick={() => setDialog(null)}><Icon name="close" size={18}/></button></header>
      <form onSubmit={event => {
        event.preventDefault();
        void action(async () => {
          if (dialog.kind === 'create') { const created = await repository.create(projectName(name), model); setProjects(current => [projectSummary(created), ...current]); setDialog(null); setActive(created); }
          else if (dialog.kind === 'rename') { await repository.rename(dialog.project.id, projectName(name)); await refresh(); setDialog(null); }
          else { await repository.remove(dialog.project.id); await refresh(); setDialog(null); }
        });
      }}>
        {dialog.kind === 'delete' ? <p>删除“{dialog.project.name}”及其中的场景和模块草稿？已另存到磁盘的程序文件不受影响。</p> : <label>项目名称<input autoFocus aria-label="项目名称" maxLength={80} value={name} disabled={busy} onChange={event => setName(event.target.value)} placeholder="例如：工件打磨验证" required/></label>}
        {dialog.kind === 'create' && <label>机器人<select aria-label="项目机器人" value={model} disabled={busy} onChange={event => setModel(event.target.value)}>{PROJECT_MODELS.map(item => <option key={item.id} value={item.id}>{item.name}</option>)}</select><small>未缓存的模型在首次打开项目时下载。</small></label>}
        {error && <p className="project-error" role="alert">{error}</p>}
        <footer><button type="button" disabled={busy} onClick={() => setDialog(null)}>取消</button><button type="submit" className={dialog.kind === 'delete' ? 'project-danger' : 'project-primary'} disabled={busy || dialog.kind !== 'delete' && !name.trim()}>{busy ? '正在处理…' : dialog.kind === 'create' ? '创建并打开' : dialog.kind === 'rename' ? '保存名称' : '删除项目'}</button></footer>
      </form>
    </section></div>}
  </main>;
}
