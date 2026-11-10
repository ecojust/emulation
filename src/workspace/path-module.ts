import { getModelProgrammingProfile, getProgramDialect, type ProgramDialectId } from '../programming/profiles';
import { resolveProgramPathPoints, type ProgramPath, type ProgramPoint } from './program';
import { renderProgramTemplate } from './program-templates';

export interface PathModule {
  readonly name: string;
  readonly text: string;
}

function moduleBaseName(pathName: string): string {
  const defaultNumber = /^Path\s+([1-9]\d*)$/i.exec(pathName.trim())?.[1];
  if (defaultNumber) return `PATH${defaultNumber}`.slice(0, 8);
  // Prefix custom names so titles such as MODULE/END cannot become keywords.
  const safeTitle = pathName.normalize('NFKD').toUpperCase().replace(/[\u0300-\u036f]/g, '')
    .replace(/[^A-Z0-9_]+/g, '_').replace(/^_+|_+$/g, '');
  return `P_${safeTitle || 'PATH'}`.slice(0, 8);
}

function uniqueModuleName(base: string, existingNames: readonly string[]): string {
  // Module identifiers must also be unique across extensions (RW6/RW7, etc.).
  const existing = new Set(existingNames.map(name => {
    const filename = name.split(/[\\/]/).at(-1) ?? name;
    return filename.replace(/\.[^.]*$/, '').toUpperCase();
  }));
  for (let attempt = 1; attempt <= existing.size + 1; attempt += 1) {
    const suffix = attempt === 1 ? '' : `_${attempt}`;
    if (suffix.length >= 8) throw new Error('无法生成可用的模块名称，请减少同名模块。');
    const candidate = `${base.slice(0, 8 - suffix.length)}${suffix}`;
    if (!existing.has(candidate)) return candidate;
  }
  throw new Error('无法生成可用的模块名称。');
}

/** A standalone source snapshot: later Path edits never rewrite this module. */
export function createPathModule(
  modelId: string,
  path: ProgramPath,
  points: readonly ProgramPoint[],
  dialectId: ProgramDialectId,
  existingNames: readonly string[],
): PathModule {
  const profile = getModelProgrammingProfile(modelId);
  const dialect = getProgramDialect(dialectId);
  if (!profile.dialectIds.includes(dialect.id)) throw new Error('所选语言版本不适用于当前机器人型号。');
  const pathPoints = resolveProgramPathPoints(path, points);
  if (!pathPoints.length) throw new Error('Path 为空，请先添加当前点再转换为程序。');
  const moduleName = uniqueModuleName(moduleBaseName(path.name), existingNames);
  const template = renderProgramTemplate(modelId, pathPoints, moduleName);
  return Object.freeze({ name: `${moduleName}.${dialect.extensions[0]}`, text: template.code });
}
