import catalog from '../catalog/catalog.json';
import { normalizeOrientation } from '../robot/rapid';
import type { JointAngles, Pose, Quaternion, RobConf } from '../robot/types';

export type ProgramBrand = 'ABB' | 'FANUC' | 'Yaskawa';
export interface ProgramProfile {
  readonly modelId: string;
  readonly brand: ProgramBrand;
  readonly language: 'RAPID' | 'LS' | 'INFORM';
  readonly linearInstruction: 'MoveL' | 'L' | 'MOVL';
  readonly targetType: 'robtarget' | 'P 关节位置' | 'C 笛卡尔位置';
  readonly controllerExport: 'verified-abb-segment' | 'unsupported';
}
/** A captured simulation pose, not a controller-calibrated target. */
export interface ProgramTemplatePoint {
  pointName: string;
  pose: Pose;
  joints: JointAngles;
  configuration: RobConf | null;
  speedMmPerSecond: number;
}
export interface FormattedProgramPoint {
  reference: string;
  instruction: string;
  targetType: ProgramProfile['targetType'];
  declaration: string;
  orientationLabel: string;
  orientationValues: number[];
  configurationNote: string | null;
}
export interface ProgramTemplate {
  profile: ProgramProfile;
  code: string;
  notice: string;
  unresolvedFields: readonly string[];
  /** Full programs are previews; only the separate native ABB segment exporter is implemented. */
  exportable: false;
}

const profiles = new Map<string, ProgramProfile>();
for (const model of catalog.models) {
  const brand = model.brand;
  if (brand !== 'ABB' && brand !== 'FANUC' && brand !== 'Yaskawa') continue;
  profiles.set(model.id, Object.freeze({
    modelId: model.id, brand,
    language: brand === 'ABB' ? 'RAPID' : brand === 'FANUC' ? 'LS' : 'INFORM',
    linearInstruction: brand === 'ABB' ? 'MoveL' : brand === 'FANUC' ? 'L' : 'MOVL',
    targetType: brand === 'ABB' ? 'robtarget' : brand === 'FANUC' ? 'P 关节位置' : 'C 笛卡尔位置',
    controllerExport: model.bundledMotionModelId === 'irb1100-4-0.475' || model.bundledMotionModelId === 'irb1100-4-0.58' ? 'verified-abb-segment' : 'unsupported',
  }));
}
export const PROGRAM_MODEL_IDS: readonly string[] = Object.freeze([...profiles.keys()]);
export function getProgramProfile(modelId: string): ProgramProfile {
  const profile = profiles.get(modelId);
  if (!profile) throw new Error('当前型号不在已适配的仿真程序目录中。');
  return profile;
}

function numeric(value: number, digits = 3): string {
  if (!Number.isFinite(value)) throw new Error('程序点包含非有限数值。');
  return String(Number(value.toFixed(digits)));
}
const tuple = (values: readonly number[], digits = 3) => values.map(value => numeric(value, digits)).join(',');
function finiteTuple(value: unknown, size: number): value is number[] {
  return Array.isArray(value) && value.length === size && Array.from(value).every(item => typeof item === 'number' && Number.isFinite(item));
}
function validatePoint(profile: ProgramProfile, point: ProgramTemplatePoint): number {
  const ordinal = /^P([0-9]+)$/.exec(point.pointName)?.[1];
  const number = ordinal ? Number(ordinal) : 0;
  if (!Number.isSafeInteger(number) || number < 1 || number > 99999) throw new Error('程序点名称必须是 P001 至 P99999 范围内的稳定编号。');
  if (!finiteTuple(point.pose?.position, 3) || !finiteTuple(point.pose?.orientation, 4) || !finiteTuple(point.joints, 6)
    || !Number.isFinite(point.speedMmPerSecond) || point.speedMmPerSecond < 1 || point.speedMmPerSecond > 2000) throw new Error('程序点需要有效的六轴姿态和速度。');
  normalizeOrientation(point.pose.orientation);
  if (profile.brand === 'ABB') {
    if (!finiteTuple(point.configuration, 4) || !point.configuration.every(Number.isSafeInteger) || point.configuration[3] < 0 || point.configuration[3] > 7) throw new Error('ABB 程序点需要有效的 robconf。');
  } else if (point.configuration !== null) throw new Error('非 ABB 程序点不能携带 ABB robconf。');
  return number;
}

/**
 * Degrees [Rx,Ry,Rz], R = Rz(Rz) Ry(Ry) Rx(Rx). FANUC labels these W/P/R;
 * Yaskawa Cartesian records use Rx/Ry/Rz. A gimbal-lock branch chooses Rx=0
 * and an equivalent Rz, preserving the rotation rather than fabricating axes.
 */
export function quaternionToControllerEuler(value: Quaternion): [number, number, number] {
  const [w, x, y, z] = normalizeOrientation(value);
  const r00 = 1 - 2 * (y * y + z * z), r10 = 2 * (x * y + w * z);
  const r20 = 2 * (x * z - w * y), r21 = 2 * (y * z + w * x), r22 = 1 - 2 * (x * x + y * y);
  const pitch = Math.atan2(-r20, Math.hypot(r00, r10));
  const locked = Math.hypot(r00, r10) < 1e-10;
  const roll = locked ? 0 : Math.atan2(r21, r22);
  const yaw = locked ? Math.atan2(-2 * (x * y - w * z), 1 - 2 * (x * x + z * z)) : Math.atan2(r10, r00);
  return [roll, pitch, yaw].map(angle => angle * 180 / Math.PI) as [number, number, number];
}

/* Syntax references, not bundled/copied postprocessor implementations:
 * https://github.com/ros-industrial/robodk_postprocessors/blob/d7e6c1c07758d67d2906cfd638049bdff88cca72/Fanuc_RJ3.py#L345-L362
 * https://github.com/ros-industrial/robodk_postprocessors/blob/d7e6c1c07758d67d2906cfd638049bdff88cca72/Fanuc_RJ3.py#L570-L599
 * https://github.com/ros-industrial/robodk_postprocessors/blob/d7e6c1c07758d67d2906cfd638049bdff88cca72/Motoman.py#L540-L631
 * FANUC L permits a joint-defined P target. Yaskawa pulse targets require a
 * calibrated pulse/degree ratio; this preview deliberately uses Cartesian
 * records with unresolved controller RCONF/tool metadata instead.
 */
export function formatProgramPoint(modelId: string, point: ProgramTemplatePoint): FormattedProgramPoint {
  const profile = getProgramProfile(modelId), ordinal = validatePoint(profile, point);
  const speed = numeric(point.speedMmPerSecond);
  if (profile.brand === 'ABB') {
    const target = `[[${tuple(point.pose.position)}],[${tuple(normalizeOrientation(point.pose.orientation), 9)}],[${tuple(point.configuration!)}],[9E9,9E9,9E9,9E9,9E9,9E9]]`;
    return {
      reference: point.pointName, targetType: profile.targetType,
      instruction: `MoveL ${point.pointName}, [${speed},500,5000,1000], fine, tool0\\WObj:=wobj0;`,
      declaration: `CONST robtarget ${point.pointName} := ${target};`,
      orientationLabel: '四元数 [w,x,y,z]', orientationValues: [...normalizeOrientation(point.pose.orientation)], configurationNote: null,
    };
  }
  const angles = quaternionToControllerEuler(point.pose.orientation);
  if (profile.brand === 'FANUC') return {
    reference: `P[${ordinal}]`, targetType: profile.targetType,
    instruction: `L P[${ordinal}] ${speed}mm/sec FINE ;`,
    declaration: `P[${ordinal}]{\n   GP1:\n    UF : {{UF}}, UT : {{UT}},\n    ${point.joints.slice(0, 3).map((angle, index) => `J${index + 1}=${numeric(angle)} deg`).join(', ')},\n    ${point.joints.slice(3).map((angle, index) => `J${index + 4}=${numeric(angle)} deg`).join(', ')}\n};`,
    orientationLabel: 'W/P/R [deg]', orientationValues: angles,
    configurationNote: '使用关节位置表示，无 ABB robconf；UF、UT 及控制器关节零位／耦合映射待配置。',
  };
  const reference = `C${String(ordinal - 1).padStart(5, '0')}`;
  return {
    reference, targetType: profile.targetType, instruction: `MOVL ${reference} V=${speed}`,
    declaration: `///RCONF {{RCONF_${point.pointName}}}\n${reference}=${tuple([...point.pose.position, ...angles], 4)}`,
    orientationLabel: 'Rx/Ry/Rz [deg]', orientationValues: angles,
    configurationNote: 'BASE 笛卡尔仿真点；控制器 RCONF、TOOL 与相对作业支持待配置，未生成脉冲值。',
  };
}

export function renderProgramTemplate(modelId: string, points: readonly ProgramTemplatePoint[], name = 'EMULATE'): ProgramTemplate {
  const profile = getProgramProfile(modelId);
  if (!/^[A-Za-z][A-Za-z0-9_]{0,7}$/.test(name)) throw new Error('仿真程序名称须以字母开头，由最多 8 个字母、数字或下划线组成。');
  if (new Set(points.map(point => point.pointName)).size !== points.length) throw new Error('程序点名称重复。');
  const formatted = points.map(point => formatProgramPoint(modelId, point));
  const declarations = formatted.map(point => point.declaration).join('\n');
  const motions = formatted.map(point => point.instruction);
  let code: string;
  let unresolvedFields: string[];
  if (profile.brand === 'ABB') {
    code = `MODULE ${name}\n  ! Simulation template; validate motion before controller export.\n${declarations.split('\n').filter(Boolean).map(line => `  ${line}`).join('\n')}\n  PROC main()\n    ConfL \\On;\n${motions.map(line => `    ${line}`).join('\n')}\n  ENDPROC\nENDMODULE`;
    unresolvedFields = [];
  } else if (profile.brand === 'FANUC') {
    code = `/PROG ${name}\n/ATTR\nOWNER = MNEDITOR;\nCOMMENT = "SIMULATION TEMPLATE";\n/MN\n${motions.map((line, index) => `   ${index + 1}:${line}`).join('\n')}\n/POS\n${declarations}\n/END`;
    unresolvedFields = ['UF', 'UT', '控制器关节零位与耦合映射'];
  } else {
    code = `/JOB\n//NAME ${name}\n//POS\n///NPOS ${points.length},0,0,0,0,0\n///TOOL {{TOOL}}\n///POSTYPE BASE\n///RECTAN\n${declarations}\n//INST\n///COMM SIMULATION TEMPLATE\n///ATTR SC,RW,RJ\n///GROUP1 RB1\nNOP\n${motions.join('\n')}\nEND`;
    unresolvedFields = ['TOOL', ...points.map(point => `RCONF_${point.pointName}`), '控制器相对作业支持与坐标标定'];
  }
  return {
    profile, code, exportable: false, unresolvedFields: Object.freeze(unresolvedFields),
    notice: profile.brand === 'ABB' ? 'RAPID 仿真模板；运行前逐段验证，控制器导出仍仅支持单独验证的 ABB 运动段。'
      : `${profile.language} 仿真模板；待配置：${unresolvedFields.join('、')}。当前不支持此品牌的控制器文件导出。`,
  };
}
