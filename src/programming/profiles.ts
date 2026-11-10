import type { RobotLanguageId } from './types';

export type ProgramDialectId = 'rapid-rw7' | 'rapid-rw6' | 'fanuc-r30ib' | 'inform-yrc1000' | 'krl-kss8';
export interface ProgramDialect {
  readonly id: ProgramDialectId;
  readonly languageId: RobotLanguageId;
  readonly name: string;
  readonly referenceVersion: string;
  readonly extensions: readonly string[];
}
export const PROGRAM_DIALECTS: readonly ProgramDialect[] = Object.freeze([
  { id: 'rapid-rw7', languageId: 'rapid', name: 'ABB RAPID · RobotWare 7', referenceVersion: '7.21', extensions: Object.freeze(['modx', 'sysx', 'mod', 'sys']) },
  { id: 'rapid-rw6', languageId: 'rapid', name: 'ABB RAPID · RobotWare 6', referenceVersion: '6.16', extensions: Object.freeze(['mod', 'sys']) },
  { id: 'fanuc-r30ib', languageId: 'fanuc-ls', name: 'FANUC LS · R-30iB', referenceVersion: 'R-30iB', extensions: Object.freeze(['ls']) },
  { id: 'inform-yrc1000', languageId: 'inform', name: 'Yaskawa INFORM · YRC1000', referenceVersion: 'YRC1000', extensions: Object.freeze(['jbi']) },
  { id: 'krl-kss8', languageId: 'krl', name: 'KUKA KRL · KSS 8', referenceVersion: 'KSS 8', extensions: Object.freeze(['src', 'dat']) },
].map(profile => Object.freeze(profile)) as ProgramDialect[]);

export interface ModelProgrammingProfile {
  readonly modelId: string;
  readonly languageId: RobotLanguageId;
  readonly defaultDialectId: ProgramDialectId;
  readonly dialectIds: readonly ProgramDialectId[];
}
const modelLanguages = Object.freeze({
  'irb1100-4-0.475': 'rapid', 'irb1100-4-0.58': 'rapid',
  'fanuc-crx10ia-l': 'fanuc-ls', 'fanuc-r2000ib-165f': 'fanuc-ls',
  'yaskawa-gp7': 'inform', 'yaskawa-gp110': 'inform',
} satisfies Record<string, RobotLanguageId>);
export const PROGRAMMING_MODEL_IDS: readonly string[] = Object.freeze(Object.keys(modelLanguages));
export function getProgramDialect(id: string): ProgramDialect {
  const result = PROGRAM_DIALECTS.find(profile => profile.id === id);
  if (!result) throw new Error('未知的机器人语言版本。');
  return result;
}
export function getModelProgrammingProfile(modelId: string): ModelProgrammingProfile {
  if (!Object.hasOwn(modelLanguages, modelId)) throw new Error('当前型号尚未接入源码工程。');
  const languageId = modelLanguages[modelId as keyof typeof modelLanguages];
  const dialectIds = PROGRAM_DIALECTS.filter(profile => profile.languageId === languageId).map(profile => profile.id);
  return Object.freeze({ modelId, languageId, defaultDialectId: dialectIds[0], dialectIds: Object.freeze(dialectIds) });
}
