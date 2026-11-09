import type { RobotLanguageDefinition } from './types';
import { getProgramDialect, type ProgramDialectId } from './profiles';
const loaded = new Map<ProgramDialectId, Promise<RobotLanguageDefinition>>();
/** The 3D workspace does not load editor grammars until a code workspace opens. */
export function loadLanguageDefinition(id: ProgramDialectId): Promise<RobotLanguageDefinition> {
  getProgramDialect(id);
  const existing = loaded.get(id); if (existing) return existing;
  const pending: Promise<RobotLanguageDefinition> = id === 'rapid-rw7' || id === 'rapid-rw6'
    ? import('./languages/rapid').then(module => id === 'rapid-rw6' ? module.createRapidLanguage('6.16') : module.rapidLanguage)
    : id === 'fanuc-r30ib' ? import('./languages/fanuc').then(module => module.fanucLanguage)
    : id === 'inform-yrc1000' ? import('./languages/inform').then(module => module.informLanguage)
    : import('./languages/krl').then(module => module.krlLanguage);
  loaded.set(id, pending);
  void pending.catch(() => { if (loaded.get(id) === pending) loaded.delete(id); });
  return pending;
}
