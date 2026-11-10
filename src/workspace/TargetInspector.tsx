import type { ReactNode } from 'react';
import type { ProgramPoint } from './program';
import { formatProgramPoint, getProgramProfile } from './program-templates';

interface Props {
  modelKey: string;
  point: ProgramPoint;
  updateDisabledReason: string | null;
  previewing?: boolean;
  checking: boolean;
  onUpdate: () => void;
  onCheck: () => void;
  children: ReactNode;
}
const text = (values: readonly number[], digits = 6) => values.map(value => Number(value.toFixed(digits))).join(', ');
export default function TargetInspector({ modelKey, point, updateDisabledReason, previewing = false, checking, onUpdate, onCheck, children }: Props) {
  const profile = getProgramProfile(modelKey), formatted = formatProgramPoint(modelKey, point);
  const robtarget = point.configuration ? '[[' + text(point.pose.position, 3) + '],[' + text(point.pose.orientation, 9) + '],[' + point.configuration.join(', ') + '],[9E9,9E9,9E9,9E9,9E9,9E9]]' : '';
  return <div className="target-inspector" data-target-point-id={point.id}>
    <div className="panel-title"><div className="section-number">P</div><h2>{point.pointName} · 目标点</h2></div>
    <p className="field-hint">已记录的 {profile.brand} {profile.linearInstruction} 姿态快照。选中目标点不会移动当前机器人。</p>
    <section className="inspector-values"><h3>position <small>mm</small></h3><code aria-label="目标 position">{text(point.pose.position, 3)}</code></section>
    <section className="inspector-values"><h3>四元数 <small>[w, x, y, z]</small></h3><code aria-label="目标四元数">{text(point.pose.orientation)}</code></section>
    {point.configuration ? <section className="inspector-values"><h3>robconf</h3><code aria-label="目标 robconf">{'[' + point.configuration.join(', ') + ']'}</code></section> : <>
      <section className="inspector-values"><h3>{formatted.orientationLabel} <small>deg</small></h3><code aria-label="目标品牌姿态">{text(formatted.orientationValues)}</code></section>
      <section className="inspector-values"><h3>已记录关节 <small>deg</small></h3><code aria-label="目标关节">{text(point.joints)}</code></section>
    </>}
    <section className="inspector-values"><h3>速度</h3><code aria-label="目标速度">{point.speedMmPerSecond} mm/s</code></section>
    <label className="field-label" htmlFor="target-robtarget">{formatted.targetType}</label><textarea id="target-robtarget" className="target-robtarget" aria-label={point.configuration ? '目标 robtarget' : '目标 ' + profile.brand + ' 位置数据'} value={point.configuration ? robtarget : formatted.declaration} readOnly spellCheck={false}/>
    {formatted.configurationNote && <p className="field-hint">{formatted.configurationNote}</p>}
    <button type="button" className="example-button" disabled={previewing || Boolean(updateDisabledReason)} title={previewing ? '请先退出端点预览。' : updateDisabledReason ?? ''} onClick={onUpdate}>用当前姿态更新点</button>
    <button type="button" className="solve-button" disabled={checking || previewing || Boolean(updateDisabledReason)} title={previewing ? '请先退出端点预览。' : ''} onClick={onCheck}>{checking ? '正在检查 ' + profile.linearInstruction + '…' : '检查到此点的 ' + profile.linearInstruction}</button>
    {updateDisabledReason && <p className="field-hint" role="status">{updateDisabledReason}</p>}
    <p className="field-hint">只验证当前机器人到此点的一段运动，不替代程序运行前的逐段检查。</p>
    {profile.brand !== 'ABB' && <p className="field-hint">此型号尚未支持端点候选枚举与危险区干涉检查。品牌位置数据为仿真模板，不是已验收的控制器文件。</p>}
    {children}
  </div>;
}
