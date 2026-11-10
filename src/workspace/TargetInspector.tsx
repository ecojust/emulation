import type { ReactNode } from 'react';
import Icon from '../ui/Icon';
import type { Pose } from '../robot/types';
import { PoseReadout } from './SceneFrameInspector';
import type { ProgramPoint } from './program';
import { formatProgramPoint, getProgramProfile } from './program-templates';

interface Props {
  modelKey: string;
  point: ProgramPoint;
  worldPose?: Pose;
  referenceName?: string;
  updateDisabledReason: string | null;
  previewing?: boolean;
  onUpdate: () => void;
  children: ReactNode;
}
const text = (values: readonly number[], digits = 6) => values.map(value => Number(value.toFixed(digits))).join(', ');
export default function TargetInspector({ modelKey, point, worldPose, referenceName, updateDisabledReason, previewing = false, onUpdate, children }: Props) {
  const profile = getProgramProfile(modelKey), formatted = formatProgramPoint(modelKey, point);
  const robtarget = point.configuration ? '[[' + text(point.pose.position, 3) + '],[' + text(point.pose.orientation, 9) + '],[' + point.configuration.join(', ') + '],[9E9,9E9,9E9,9E9,9E9,9E9]]' : '';
  return <div className="target-inspector" data-target-point-id={point.id} data-local-pose={JSON.stringify(point.pose)} data-world-pose={JSON.stringify(worldPose ?? point.pose)}>
    <div className="panel-title"><div className="section-number"><Icon name="target" size={14}/></div><h2>{point.pointName} · 目标点</h2></div>
    <p className="field-hint">已记录的工具位姿，相对于{referenceName ?? '路径坐标'}。选中目标点不会移动当前机器人。</p>
    <section className="inspector-values"><h3>位置 <small>mm · 路径坐标</small></h3><code aria-label="目标 position">{text(point.pose.position, 3)}</code></section>
    <section className="inspector-values"><h3>四元数 <small>[w, x, y, z]</small></h3><code aria-label="目标四元数">{text(point.pose.orientation)}</code></section>
    {point.configuration ? <section className="inspector-values"><h3>已记录轴配置 <small>ABB robconf</small></h3><code aria-label="目标 robconf">{'[' + point.configuration.join(', ') + ']'}</code></section> : <>
      <section className="inspector-values"><h3>{formatted.orientationLabel} <small>deg</small></h3><code aria-label="目标品牌姿态">{text(formatted.orientationValues)}</code></section>
      <section className="inspector-values"><h3>已记录关节 <small>deg</small></h3><code aria-label="目标关节">{text(point.joints)}</code></section>
    </>}
    <section className="inspector-values"><h3>速度</h3><code aria-label="目标速度">{point.speedMmPerSecond} mm/s</code></section>
    <label className="field-label" htmlFor="target-robtarget">{formatted.targetType}</label><textarea id="target-robtarget" className="target-robtarget" aria-label={point.configuration ? '目标 robtarget' : '目标 ' + profile.brand + ' 位置数据'} value={point.configuration ? robtarget : formatted.declaration} readOnly spellCheck={false}/>
    {worldPose && <PoseReadout label="目标世界位姿" pose={worldPose}/>}
    {point.motionRole && <p className="field-hint">用途：{point.motionRole === 'transfer' ? '转移段' : '加工段'}</p>}
    {formatted.configurationNote && <p className="field-hint">{formatted.configurationNote}</p>}
    <button type="button" className="example-button" disabled={Boolean(updateDisabledReason)} title={updateDisabledReason ?? (previewing ? '将预览的关节姿态与轴配置保存到此点，保留目标位置、方向和速度。' : '')} onClick={onUpdate}>用当前姿态更新点</button>
    {updateDisabledReason && <p className="field-hint" role="status">{updateDisabledReason}</p>}
    {profile.brand !== 'ABB' && <p className="field-hint">候选解使用此型号的运动链求解；品牌位置数据为仿真模板，控制器配置与现场标定仍需核对。</p>}
    {children}
  </div>;
}
