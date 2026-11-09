import { useEffect, useRef, useState } from 'react';
import { listModelCache, loadCatalogModel, MODEL_CATALOG, type LoadedCatalogModel, type ModelProgress } from './client';
import './catalog.css';

interface Props {
  currentModelId: string;
  onSelect: (model: LoadedCatalogModel) => void;
  onClose: () => void;
}
const brands = ['ABB', 'FANUC', 'KUKA', 'Yaskawa'] as const;
const bytesText = (bytes: number) => bytes >= 1024 * 1024 ? `${(bytes / (1024 * 1024)).toFixed(1)} MB` : `${Math.ceil(bytes / 1024)} KB`;

export default function RobotCatalogDialog({ currentModelId, onSelect, onClose }: Props) {
  const dialog = useRef<HTMLDialogElement>(null);
  const request = useRef<AbortController | null>(null);
  const [query, setQuery] = useState('');
  const [cached, setCached] = useState<Record<string, boolean>>({});
  const [cacheChecking, setCacheChecking] = useState(true);
  const [cacheError, setCacheError] = useState(false);
  const [cacheRevision, setCacheRevision] = useState(0);
  const [loadingId, setLoadingId] = useState<string | null>(null);
  const [progress, setProgress] = useState<ModelProgress | null>(null);
  const [message, setMessage] = useState('');
  const [failedId, setFailedId] = useState<string | null>(null);

  useEffect(() => {
    dialog.current?.showModal();
    return () => { request.current?.abort(); request.current = null; };
  }, []);
  useEffect(() => {
    const controller = new AbortController();
    setCacheChecking(true); setCacheError(false);
    void listModelCache(controller.signal).then(result => {
      if (!controller.signal.aborted) { setCached(result); setCacheChecking(false); }
    }, () => {
      if (!controller.signal.aborted) { setCacheChecking(false); setCacheError(true); }
    });
    return () => controller.abort();
  }, [cacheRevision]);

  function close() {
    request.current?.abort(); request.current = null;
    dialog.current?.close();
    onClose();
  }
  function cancel() {
    request.current?.abort(); request.current = null;
    setLoadingId(null); setProgress(null); setFailedId(null);
    setMessage('已取消加载，当前机器人保持不变。');
  }
  async function load(id: string) {
    request.current?.abort();
    const controller = new AbortController();
    request.current = controller;
    setLoadingId(id); setProgress(null); setFailedId(null); setMessage('');
    try {
      const loaded = await loadCatalogModel(id, {
        signal: controller.signal,
        onProgress: value => { if (request.current === controller && !controller.signal.aborted) setProgress(value); },
      });
      if (request.current !== controller || controller.signal.aborted) {
        if (loaded.kind === 'urdf') loaded.dispose();
        return;
      }
      request.current = null;
      dialog.current?.close();
      onSelect(loaded);
      onClose();
    } catch (cause) {
      if (request.current !== controller || controller.signal.aborted) return;
      setMessage(cause instanceof Error ? cause.message : '模型加载失败，请重试。');
      setFailedId(id);
    } finally {
      if (request.current === controller) { request.current = null; setLoadingId(null); }
    }
  }

  const search = query.trim().toLocaleLowerCase();
  const progressLabel = progress?.phase === 'preparing' ? '正在解析模型与网格' : progress?.phase === 'downloading' ? '正在下载模型' : '正在检查模型缓存';
  const selectedName = MODEL_CATALOG.find(model => model.id === loadingId)?.name;
  return <dialog ref={dialog} className="robot-catalog-dialog" aria-labelledby="robot-catalog-title" onCancel={event => { event.preventDefault(); close(); }}>
    <header className="robot-catalog-heading"><div><span className="eyebrow">ROBOT LIBRARY</span><h2 id="robot-catalog-title">机器人模型目录</h2><p>选择型号后加载完整模型。首次使用需要下载，后续使用本机缓存。</p></div><button type="button" className="icon-button" aria-label="关闭模型目录" onClick={close}>×</button></header>
    <label className="robot-catalog-search">搜索型号<input type="search" aria-label="搜索机器人型号" placeholder="品牌或型号" value={query} onChange={event => setQuery(event.target.value)}/></label>
    <div className="robot-catalog-list">
      {brands.map(brand => {
        const entries = MODEL_CATALOG.filter(model => model.brand === brand && (!search || `${model.brand} ${model.name}`.toLocaleLowerCase().includes(search)));
        return <section key={brand} className="robot-catalog-brand" aria-label={`${brand} 机器人`}><h3>{brand === 'Yaskawa' ? 'Yaskawa · 安川' : brand}<span>{entries.length}</span></h3>{entries.map(entry => <article key={entry.id} className={`robot-catalog-entry ${currentModelId === entry.id ? 'catalog-current' : ''}`} data-catalog-model-id={entry.id}>
          <div className="robot-catalog-entry-main"><h4>{entry.name}</h4><p>{entry.bundledMotionModelId ? '模型 · DH / IK / MoveL · ABB RAPID' : '模型 · URDF 关节调整与正运动学'}</p><span className="robot-catalog-meta">{entry.bundledMotionModelId ? '已内置' : cached[entry.id] ? '已缓存' : cacheChecking ? '检查缓存中' : '首次使用下载'}{!entry.bundledMotionModelId && ` · ${bytesText(entry.downloadBytes)}`} · {entry.licenses.map(license => license.spdx).filter((value, index, all) => all.indexOf(value) === index).join(' / ')}</span></div>
          <button type="button" className="copy-button" aria-label={`加载 ${entry.name}`} disabled={Boolean(loadingId) || currentModelId === entry.id} onClick={() => void load(entry.id)}>{currentModelId === entry.id ? '当前型号' : loadingId === entry.id ? '加载中…' : cached[entry.id] || entry.bundledMotionModelId ? '加载模型' : '下载并加载'}</button>
        </article>)}{entries.length === 0 && <p className="robot-catalog-empty">{brand === 'KUKA' ? '当前模型来源中暂无 KUKA 可下载资源。' : '没有匹配的型号。'}</p>}</section>;
      })}
    </div>
    <footer className="robot-catalog-footer">
      {cacheError && <p className="robot-catalog-message" role="status">缓存状态暂不可用，仍可选择型号加载。<button type="button" className="copy-button" disabled={Boolean(loadingId)} onClick={() => setCacheRevision(value => value + 1)}>重试缓存检查</button></p>}
      {loadingId && <div className="robot-catalog-progress" role="status" aria-label="模型加载进度"><div><strong>{selectedName}</strong><span>{progressLabel}{progress && progress.totalBytes > 0 ? ` · ${bytesText(progress.downloadedBytes)} / ${bytesText(progress.totalBytes)}` : ''}</span></div><progress aria-label="模型下载进度" max={Math.max(1, progress?.totalBytes ?? 1)} value={progress?.phase === 'downloading' ? progress.downloadedBytes : undefined}/><button type="button" className="copy-button" onClick={cancel}>取消加载</button></div>}
      {message && <p className={failedId ? 'robot-catalog-error' : 'robot-catalog-message'} role={failedId ? 'alert' : 'status'}>{message}{failedId && <button type="button" className="copy-button" onClick={() => void load(failedId)}>重试加载</button>}</p>}
      <p className="robot-catalog-note">目录中的模型能力分别标明。模型可显示不代表已经适配轨迹求解或控制器程序。</p>
    </footer>
  </dialog>;
}
