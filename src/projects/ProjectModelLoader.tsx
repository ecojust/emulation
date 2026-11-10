import { useEffect, useRef, useState, type ReactNode } from 'react';
import { loadCatalogModel, MODEL_CATALOG, type LoadedCatalogModel, type ModelProgress } from '../catalog/client';
import { forwardUrdfKinematics } from '../catalog/kinematics';
import { forwardKinematics } from '../robot/kinematics';
import { getRobotModel } from '../robot/model';
import Icon from '../ui/Icon';
import { validateProjectModelWorld, type SimulationProject } from './schema';
import './projects.css';
import './project-model-loader.css';

interface Props {
  project?: SimulationProject;
  onCancel?: () => void;
  children: (loaded: LoadedCatalogModel) => ReactNode;
}
const bytesText = (bytes: number) => bytes >= 1024 * 1024 ? `${(bytes / (1024 * 1024)).toFixed(1)} MB` : `${Math.ceil(bytes / 1024)} KB`;

/** Keep the editor unmounted until its own verified model and saved pose are ready. */
export default function ProjectModelLoader({ project, onCancel, children }: Props) {
  const requested = project?.scene.activeModelId ?? 'irb1100-4-0.58';
  const entry = MODEL_CATALOG.find(model => model.id === requested);
  const key = `${project?.id ?? 'default'}:${requested}`;
  const [ready, setReady] = useState<{ key: string; model: LoadedCatalogModel } | null>(null);
  const [error, setError] = useState(''), [retry, setRetry] = useState(0);
  const [progress, setProgress] = useState<ModelProgress>({ phase: 'checking', downloadedBytes: 0, totalBytes: entry?.downloadBytes ?? 0 });
  const request = useRef<AbortController | null>(null);
  useEffect(() => {
    const controller = new AbortController(); let prepared: LoadedCatalogModel | null = null;
    request.current = controller;
    setError(''); setReady(null);
    setProgress({ phase: 'checking', downloadedBytes: 0, totalBytes: entry?.downloadBytes ?? 0 });
    void loadCatalogModel(requested, {
      signal: controller.signal,
      onProgress: value => { if (!controller.signal.aborted && request.current === controller) setProgress(value); },
    }).then(value => {
      prepared = value;
      const savedModel = project?.scene.models[requested];
      const saved = savedModel?.joints;
      const limits = value.kind === 'bundled' ? getRobotModel(value.motionModelId).limits : value.model.description.actuatedJoints;
      if (saved && saved.some((angle, index) => angle < limits[index].minDeg || angle > limits[index].maxDeg)) throw new Error('项目保存的关节角超出当前型号范围，未更改已保存的数据。');
      if (savedModel) {
        const flange = value.kind === 'bundled' ? forwardKinematics(getRobotModel(value.motionModelId), savedModel.joints).pose : forwardUrdfKinematics(value.model.description, savedModel.joints).toolPose;
        if (!flange) throw new Error('保存的型号缺少有效法兰位姿，未更改项目。');
        validateProjectModelWorld(savedModel, flange);
      }
      if (controller.signal.aborted || request.current !== controller) { if (value.kind === 'urdf') value.dispose(); return; }
      setReady({ key, model: value });
    }).catch(cause => {
      if (prepared?.kind === 'urdf') prepared.dispose();
      if (!controller.signal.aborted && request.current === controller) setError(cause instanceof Error ? cause.message : '机器人模型加载失败');
    });
    return () => { controller.abort(); if (request.current === controller) request.current = null; };
  }, [key, requested, project, retry]);

  if (ready?.key === key) return children(ready.model);
  function cancel() { request.current?.abort(); request.current = null; onCancel?.(); }
  const downloading = progress.phase === 'downloading';
  const total = Math.max(0, progress.totalBytes), downloaded = Math.max(0, Math.min(total, progress.downloadedBytes));
  const percent = total > 0 ? Math.round(downloaded / total * 100) : 0;
  const phaseLabel = downloading ? '正在下载机器人模型' : progress.phase === 'preparing' || progress.phase === 'ready' ? '正在准备机器人模型' : '正在检查本机模型';

  return <main className="project-home project-model-loading" aria-label="打开项目">
    <header className="project-home-header"><div className="project-home-brand"><Icon name="cube" size={28}/><div><strong>emulation</strong><span>机器人仿真项目</span></div></div><span className="project-model-opening-label">打开项目</span></header>
    <div className="project-model-content">
      <section className={'project-model-card' + (error ? ' is-error' : '')} aria-labelledby="project-model-title" data-model-load-state={error ? 'error' : progress.phase}>
        <div className="project-model-heading"><span className="project-model-symbol"><Icon name={error ? 'warning' : 'robot'} size={30}/></span><div><h1 id="project-model-title">{error ? '模型加载未完成' : '准备机器人模型'}</h1><p>模型就绪后自动进入编辑区</p></div></div>
        <dl className="project-model-details"><div><dt>项目</dt><dd>{project?.name ?? '运动工作台'}</dd></div><div><dt>机器人</dt><dd>{entry?.name ?? requested}</dd></div></dl>
        {error ? <p className="project-model-error" role="alert">{error}</p> : <div className="project-model-progress" role="status" aria-label="模型加载状态">
          <div className="project-model-progress-heading"><span><Icon name={downloading ? 'download' : 'reset'} size={16} className={downloading ? '' : 'project-model-spinner'}/>{phaseLabel}</span>{downloading && <strong>{percent}%</strong>}</div>
          <progress aria-label="模型下载进度" max={Math.max(1, total)} value={downloading ? downloaded : undefined}/>
          <p>{entry?.bundledMotionModelId ? '内置模型，无需下载' : total > 0 ? `${bytesText(downloaded)} / ${bytesText(total)}` : '正在读取模型信息'}</p>
        </div>}
        <p className="project-model-note">{entry?.bundledMotionModelId ? '使用应用内置的机器人模型。' : '首次使用时下载模型并保存到本机，之后可离线打开。'}</p>
        <footer className="project-model-actions">{onCancel && <button type="button" onClick={cancel}><Icon name="chevron-left" size={15}/>返回项目主页</button>}{error && <button type="button" className="project-primary" onClick={() => setRetry(value => value + 1)}><Icon name="reset" size={15}/>重新加载模型</button>}</footer>
      </section>
    </div>
  </main>;
}
