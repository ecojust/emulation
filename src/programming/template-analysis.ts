import type { LanguageAnalysis, RobotLanguageDefinition, RobotLanguageId } from './types';

/** Only a dedicated header marker opts into template editing. Never rewrite source or offsets. */
export function isGeneratedProgramTemplate(languageId: RobotLanguageId, source: string): boolean {
  if (languageId !== 'fanuc-ls' && languageId !== 'inform') return false;
  const lines = source.replace(/^\uFEFF/, '').split(/\r\n|\r|\n/);
  let started = false, section = '';
  for (const raw of lines) {
    const line = raw.trim();
    if (!line) continue;
    if (!started) {
      if (languageId === 'fanuc-ls' ? !/^\/PROG\s+\S+/i.test(line) : !/^\/JOB$/i.test(line)) return false;
      started = true;
      continue;
    }
    if (languageId === 'fanuc-ls') {
      if (/^\/(?:MN|POS|END)\b/i.test(line)) return false;
      if (/^\/[A-Z]+\b/i.test(line)) section = line.toUpperCase();
      // The long marker is retained for v0.6.0 drafts; its length warning stays visible.
      if (section === '/ATTR' && /^COMMENT\s*=\s*"(?:SIM TEMPLATE|SIMULATION TEMPLATE)"\s*;$/i.test(line)) return true;
    } else {
      if (line.startsWith('//') && !line.startsWith('///')) { section = line.toUpperCase(); continue; }
      if (section !== '//INST') continue;
      if (/^\/\/\/COMM\s+SIMULATION TEMPLATE$/i.test(line)) return true;
      if (!line.startsWith('///')) return false;
    }
  }
  return false;
}

export function analyzeProgramSource(language: RobotLanguageDefinition, source: string, fileName?: string): LanguageAnalysis {
  return language.analyzeTemplate && isGeneratedProgramTemplate(language.id, source)
    ? language.analyzeTemplate(source, fileName)
    : language.analyze(source, fileName);
}
