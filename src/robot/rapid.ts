import type { Pose, Quaternion, RobConf, Vec3 } from "./types";

const EXT_UNDEFINED = 9e9;
const numberPattern = /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?/;

function finiteTuple(value: unknown, length: number, label: string): number[] {
  if (!Array.isArray(value) || value.length !== length || !value.every(v => typeof v === "number" && Number.isFinite(v))) {
    throw new Error(`${label}必须包含 ${length} 个有限数值`);
  }
  return value;
}

/** ABB quaternion order [q1,q2,q3,q4] = [w,x,y,z], never Three.js's [x,y,z,w]. */
export function normalizeOrientation(q: Quaternion): Quaternion {
  const values = finiteTuple(q, 4, "姿态四元数");
  const sum = values.reduce((s, n) => s + n * n, 0);
  // ABB NOrient permits correction only when abs(sum(q_i²)-1) <= 0.1.
  if (!Number.isFinite(sum) || Math.abs(sum - 1) > 0.1 + 1e-12) throw new Error("四元数不是有效的单位姿态，请检查 [q1,q2,q3,q4]，其中 q1 为实部");
  const length = Math.sqrt(sum);
  return values.map(v => v / length) as Quaternion;
}

/** Degrees [Rz,Ry,Rx]; matrix R = Rz(Rz) Ry(Ry) Rx(Rx), base-fixed XYZ roll/pitch/yaw. */
export function eulerZYXToQuaternion(anglesDeg: Vec3): Quaternion {
  finiteTuple(anglesDeg, 3, "欧拉角");
  const [z, y, x] = anglesDeg.map(a => a * Math.PI / 360);
  const [cz, sz, cy, sy, cx, sx] = [Math.cos(z), Math.sin(z), Math.cos(y), Math.sin(y), Math.cos(x), Math.sin(x)];
  return normalizeOrientation([cz * cy * cx + sz * sy * sx, cz * cy * sx - sz * sy * cx, cz * sy * cx + sz * cy * sx, sz * cy * cx - cz * sy * sx]);
}

function checkedConfiguration(value: unknown): RobConf {
  const values = finiteTuple(value, 4, "robconf");
  if (!values.every(Number.isSafeInteger) || values[3] < 0 || values[3] > 7) throw new Error("robconf 的四项必须为整数，cfx 必须在 0–7 之间");
  return values as RobConf;
}

/** Strict numeric-array reader. No eval, code execution, arbitrary fields, or executable suffixes. */
function readArray(source: string): unknown[] {
  let offset = 0;
  const skip = () => { while (/\s/.test(source[offset] ?? "") && offset < source.length) offset++; };
  const parse = (depth: number): unknown[] => {
    if (depth > 2 || source[offset++] !== "[") throw new Error("robtarget 格式错误：需要四组数值数组");
    const result: unknown[] = [];
    skip();
    if (source[offset] === "]") { offset++; return result; }
    while (offset < source.length) {
      if (result.length >= 6) throw new Error("robtarget 数组项数错误");
      skip();
      if (source[offset] === "[") result.push(parse(depth + 1));
      else {
        const token = source.slice(offset).match(numberPattern)?.[0];
        if (!token) throw new Error("robtarget 只接受数值，不接受变量、表达式或其他指令");
        const value = Number(token);
        if (!Number.isFinite(value)) throw new Error("robtarget 中存在非有限数值");
        result.push(value);
        offset += token.length;
      }
      skip();
      if (source[offset] === "]") { offset++; return result; }
      if (source[offset++] !== ",") throw new Error("robtarget 格式错误：数值之间需要逗号");
      skip();
      if (source[offset] === "]") throw new Error("robtarget 末尾不能多出逗号");
    }
    throw new Error("robtarget 数组未闭合");
  };
  skip();
  const result = parse(0);
  skip();
  if (offset !== source.length) throw new Error("robtarget 后不能包含其他内容或指令");
  return result;
}

export function parseRobtarget(text: string): { pose: Pose; configuration: RobConf; externalAxes: number[] } {
  if (typeof text !== "string" || !text.trim() || text.length > 16000) throw new Error("请输入完整 robtarget（最多 16000 个字符）");
  let source = text.trim();
  const declaration = source.match(/^(?:(?:LOCAL|TASK)\s+)?(?:CONST|VAR|PERS)\s+robtarget\s+[A-Za-z_][A-Za-z0-9_]*\s*:=\s*/i);
  if (declaration) source = source.slice(declaration[0].length);
  if (source.endsWith(";")) source = source.slice(0, -1).trimEnd();
  const fields = readArray(source);
  if (fields.length !== 4) throw new Error("robtarget 需要位置、四元数、robconf、外轴四组数据");
  const position = finiteTuple(fields[0], 3, "目标位置") as Vec3;
  const orientation = normalizeOrientation(finiteTuple(fields[1], 4, "姿态四元数") as Quaternion);
  const configuration = checkedConfiguration(fields[2]);
  const externalAxes = finiteTuple(fields[3], 6, "外轴");
  if (externalAxes.some(value => value !== EXT_UNDEFINED)) throw new Error("当前场景没有外轴，六个外轴值都必须为 9E9");
  return { pose: { position, orientation }, configuration, externalAxes };
}

function decimal(value: number, precision: number): string {
  if (!Number.isFinite(value)) throw new Error("MoveL 参数必须是有限数值");
  const rounded = Number(value.toFixed(precision));
  return Object.is(rounded, -0) ? "0" : String(rounded);
}

/** Format a previously verified pose/configuration. This function is not a reachability check. */
export function formatMoveL(pose: Pose, configuration: RobConf, speedMmPerSecond: number): string {
  const position = finiteTuple(pose.position, 3, "目标位置");
  const orientation = normalizeOrientation(pose.orientation);
  const conf = checkedConfiguration(configuration);
  if (!Number.isFinite(speedMmPerSecond) || speedMmPerSecond < 1 || speedMmPerSecond > 2000) throw new Error("MoveL 速度必须在 1–2000 mm/s 之间");
  const target = `[[${position.map(v => decimal(v, 3)).join(",")}],[${orientation.map(v => decimal(v, 9)).join(",")}],[${conf.join(",")}],[9E9,9E9,9E9,9E9,9E9,9E9]]`;
  // An explicit speeddata value works for every allowed speed; v123 is not an ABB predefined variable.
  const speed = `[${decimal(speedMmPerSecond, 3)},500,5000,1000]`;
  return `ConfL \\On;\nMoveL ${target}, ${speed}, fine, tool0\\WObj:=wobj0;`;
}
