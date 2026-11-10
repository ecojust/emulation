import Icon from '../ui/Icon';
import type { Pose, RobConf, RobotModel, Vec3 } from '../robot/types';

export interface RobotInspectorProps {
  name: string;
  model: RobotModel | null;
  pose: Pose | null;
  joints: readonly number[];
  drafts: readonly string[];
  limits: readonly { name: string; minDeg: number; maxDeg: number }[];
  euler: Vec3 | null;
  configuration: RobConf | null;
  poseError: string;
  captureDisabledReason: string | null;
  speed: number;
  onSpeed: (speed: number) => void;
  onJoint: (index: number, value: string) => void;
  onInitial: () => void;
  onZero: () => void;
}
const valueText = (value: number, digits = 3) => Number(value.toFixed(digits)).toString();

export default function RobotInspector(props: RobotInspectorProps) {
  return <div className="robot-inspector">
    <div className="panel-title"><div className="section-number"><Icon name="robot" size={14}/></div><h2>机器人属性</h2></div>
    <strong className="inspector-object-name">{props.name}</strong>
    {!props.model && <p className="generic-model-notice" role="status" aria-label="型号能力">支持源 URDF 关节调整、记录点及直线运动仿真；端点候选枚举、干涉检测与控制器文件导出尚未适配。</p>}
    <section className="inspector-values" aria-label="当前 TCP">
      <h3>当前位置 position <small>mm</small></h3>
      <div className="inspector-vector">{(['X', 'Y', 'Z'] as const).map((axis, i) => <span key={axis} data-axis={axis}><small>{axis}</small><strong>{props.pose ? props.pose.position[i].toFixed(2) : '—'}</strong></span>)}</div>
    </section>
    <section className="inspector-values"><h3>当前四元数 <small>[w, x, y, z]</small></h3><code aria-label="当前四元数">{props.pose?.orientation.map(value => valueText(value, 6)).join(' · ') ?? '源模型未定义工具坐标'}</code></section>
    <section className="inspector-values"><h3>当前欧拉角 <small>Rz · Ry · Rx / deg</small></h3><code aria-label="当前欧拉角">{props.euler?.map(value => valueText(value, 3)).join(' · ') ?? '—'}</code></section>
    {props.model && <section className="inspector-values"><h3>当前 robconf</h3><code aria-label="当前 robconf">{props.configuration ? '[' + props.configuration.join(', ') + ']' : '当前姿态没有有效配置'}</code></section>}
    <div className="section-rule"/>
    <div className="field-heading"><label className="field-label">关节调姿 <span>deg</span></label><button type="button" className="text-button" onClick={props.onZero}>{props.model ? '机械零位' : '模型初始姿态'}</button></div>
    <div className="joints-grid">{props.drafts.map((value, i) => <label className="joint-field" key={i} title={props.limits[i]?.name}><span>J{i + 1}</span><input type="number" step="any" aria-label={'机器人关节 J' + (i + 1)} min={props.limits[i]?.minDeg} max={props.limits[i]?.maxDeg} value={value} onChange={event => props.onJoint(i, event.target.value)}/></label>)}</div>
    <div className="joint-readout inspector-joint-readout" aria-label="当前关节姿态"><div>{props.joints.map((joint, i) => <span key={i}><small>J{i + 1}</small><strong>{joint.toFixed(1)}°</strong></span>)}</div></div>
    {props.model && <button type="button" className="example-button" onClick={props.onInitial}>示例起始姿态</button>}
    {props.poseError && <p className="danger-zone-error" role="alert">{props.poseError}</p>}
    {<><div className="speed-row"><label className="field-label" htmlFor="capture-speed">记录速度</label><select id="capture-speed" value={props.speed} onChange={event => props.onSpeed(Number(event.target.value))}>{[10, 50, 100, 200].map(speed => <option key={speed} value={speed}>{speed} mm/s</option>)}</select></div><p className="field-hint" role="status" aria-label="记录姿态提示">{props.captureDisabledReason || '摆好机器人后，右键 Path 添加当前点。运行 Path 时会逐段检查运动。'}</p></>}
    <details className="details-panel dh-details"><summary>{props.model ? 'DH 参数与关节范围' : 'URDF 关节范围'}<Icon name="chevron-right" className="summary-icon" size={14}/></summary>
      {props.model && <><p className="field-hint">标准 DH · 长度 mm · 角度 deg</p><div className="table-scroll"><table><thead><tr><th>轴</th><th>a</th><th>d</th><th>α</th><th>θ 偏置</th></tr></thead><tbody>{props.model.dh.map((row, i) => <tr key={i}><th>J{i + 1}</th><td>{row.aMm}</td><td>{row.dMm}</td><td>{row.alphaDeg}°</td><td>{row.thetaOffsetDeg}°</td></tr>)}</tbody></table></div></>}
      <div className="joint-limits">{props.limits.map((limit, i) => <span key={i} title={limit.name}>J{i + 1} <strong>{valueText(limit.minDeg)}° ~ {valueText(limit.maxDeg)}°</strong></span>)}</div>
    </details>
  </div>;
}
