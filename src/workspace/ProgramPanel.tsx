import { Suspense, lazy, useEffect, useState } from 'react';
import { getProgramProfile } from './program-templates';
import type { RobotCodeEditorProps } from '../programming/RobotCodeEditor';
import './program-panel.css';

const RobotCodeEditor = lazy(() => import('../programming/RobotCodeEditor'));
export type ProgramCodeWorkspace = Omit<RobotCodeEditorProps, 'modelKey' | 'active'>;
export interface ProgramPanelProps {
  /** A model change starts an independent source editor and undo history. */
  modelKey: string;
  active?: boolean;
  codeWorkspace?: ProgramCodeWorkspace;
  codeLoadingError?: string;
  onRetryCodeLoading?: () => void;
}

export default function ProgramPanel(props: ProgramPanelProps) {
  return <ProgramPanelBody key={props.modelKey} {...props}/>;
}

function ProgramPanelBody({ modelKey, active = true, codeWorkspace, codeLoadingError, onRetryCodeLoading }: ProgramPanelProps) {
  const [opened, setOpened] = useState(active);
  const profile = getProgramProfile(modelKey);
  // Load on the first visit and keep the editor mounted across workspace changes.
  useEffect(() => { if (active) setOpened(true); }, [active]);

  return <section className="program-editor" aria-label={`${profile.brand} 程序编辑区`}>
    <header className="program-editor-header"><h2>程序模块</h2><span>{profile.brand} · {profile.language}</span></header>
    <div className="program-code-view">
      {opened && codeWorkspace ? <Suspense fallback={<div className="program-editor-empty">正在加载模块编辑器…</div>}>
        <RobotCodeEditor {...codeWorkspace} modelKey={modelKey} active={active}/>
      </Suspense> : <div className="program-editor-empty" role={codeLoadingError ? 'alert' : undefined}>
        <p>{codeLoadingError || '正在加载本地语言支持…'}</p>
        {codeLoadingError && onRetryCodeLoading && <button type="button" onClick={onRetryCodeLoading}>重新加载语言支持</button>}
      </div>}
    </div>
  </section>;
}
