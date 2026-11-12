import { describe, expect, it } from 'vitest';
import { getModelProgrammingProfile, getProgramDialect } from '../src/programming/profiles';
import { loadLanguageDefinition } from '../src/programming/registry';
import { insertCurrentSourcePose } from '../src/programming/source-insertion';
import { analyzeProgramSource, isGeneratedProgramTemplate } from '../src/programming/template-analysis';
import type { LanguageAnalysis } from '../src/programming/types';
import {
  getProgramProfile, PROGRAM_MODEL_IDS, renderProgramTemplate,
  type ProgramTemplateContext, type ProgramTemplatePoint,
} from '../src/workspace/program-templates';

const context: ProgramTemplateContext = {
  referencePose: { position: [1234.567, -2345.678, 3456.789], orientation: [1, 2, 3, 4].map(n => n / Math.sqrt(30)) as [number, number, number, number] },
  toolPose: { position: [-123.456, 234.567, 345.678], orientation: [0.5, -0.5, 0.5, 0.5] },
};
function point(modelId: string, pointName = 'P001'): ProgramTemplatePoint {
  return {
    pointName, pose: { position: [350.125, -15.75, 620.625], orientation: [Math.SQRT1_2, 0, Math.SQRT1_2, 0] },
    joints: [10, 20, -30, 40, -50, 60],
    configuration: getProgramProfile(modelId).brand === 'ABB' ? [0, 0, 0, 1] : null,
    speedMmPerSecond: 125,
  };
}
function insert(modelId: string, source: string): string {
  const { pointName: _pointName, ...snapshot } = point(modelId);
  const edit = insertCurrentSourcePose(modelId, source, source.length, source.length, snapshot);
  return [...edit.changes].reverse().reduce((text, change) => text.slice(0, change.from) + change.insert + text.slice(change.to), source);
}
const errors = (analysis: LanguageAnalysis) => analysis.diagnostics.filter(diagnostic => diagnostic.severity === 'error');
const pending = (analysis: LanguageAnalysis) => analysis.diagnostics.filter(diagnostic => diagnostic.code === 'template-parameter');
function configuredSyntaxFixture(source: string): string {
  // Grammar-only fixture values, never configuration for any real controller.
  return source.replaceAll('{{UF}}', '1').replaceAll('{{UT}}', '2').replaceAll('{{TOOL}}', '3')
    .replace(/\{\{RCONF_P\d+\}\}/g, Array(24).fill(0).join(','));
}
const scenarios = [
  { name: 'empty', pointNames: [], frames: undefined },
  { name: 'multiple sparse points', pointNames: ['P001', 'P004', 'P003'], frames: undefined },
  { name: 'non-identity reference and tool', pointNames: ['P001', 'P004', 'P003'], frames: context },
] as const;

describe('generated Path programs through the editor analysis entry point', () => {
  for (const modelId of PROGRAM_MODEL_IDS) {
    for (const scenario of scenarios) {
      it(`${modelId}: ${scenario.name}, then insert current pose`, async () => {
        const profile = getModelProgrammingProfile(modelId);
        const language = await loadLanguageDefinition(profile.defaultDialectId);
        const fileName = 'EMULATE.' + getProgramDialect(profile.defaultDialectId).extensions[0];
        expect(language.id).toBe(profile.languageId);
        const generated = renderProgramTemplate(modelId, scenario.pointNames.map(name => point(modelId, name)), 'EMULATE', scenario.frames);
        expect(generated.exportable).toBe(false);
        for (const source of [generated.code, insert(modelId, generated.code)]) {
          const analysis = analyzeProgramSource(language, source, fileName);
          expect(errors(analysis)).toEqual([]);
          if (profile.languageId !== 'rapid' && source.includes('{{')) {
            // An empty FANUC /POS has no parameters yet; insertion introduces UF/UT.
            expect(pending(analysis).length).toBeGreaterThan(0);
            expect(pending(analysis).every(diagnostic => diagnostic.severity === 'warning')).toBe(true);
            expect(errors(language.analyze(source, fileName)).length).toBeGreaterThan(0);
          } else {
            expect(pending(analysis)).toEqual([]);
            expect(errors(language.analyze(source, fileName))).toEqual([]);
          }
          const configured = configuredSyntaxFixture(source);
          expect(configured).not.toContain('{{');
          expect(errors(language.analyze(configured, fileName))).toEqual([]);
          expect(pending(analyzeProgramSource(language, configured, fileName))).toEqual([]);
        }
      });
    }
  }

  it.each(PROGRAM_MODEL_IDS)('%s keeps actual program errors visible in a generated template', async modelId => {
    const profile = getModelProgrammingProfile(modelId);
    const language = await loadLanguageDefinition(profile.defaultDialectId);
    const source = renderProgramTemplate(modelId, [point(modelId)]).code;
    const invalid = profile.languageId === 'rapid' ? source.replace('MoveL P001,', 'MoveL ,')
      : profile.languageId === 'fanuc-ls' ? source.replace('125mm/sec', '0mm/sec')
        : source.replace('///NPOS 1,0,0,0,0,0', '///NPOS 2,0,0,0,0,0');
    expect(invalid).not.toBe(source);
    const analysis = analyzeProgramSource(language, invalid);
    expect(errors(analysis).length).toBeGreaterThan(0);
    if (profile.languageId !== 'rapid') expect(pending(analysis).length).toBeGreaterThan(0);
  });

  it.each(PROGRAM_MODEL_IDS.filter(modelId => getProgramProfile(modelId).brand !== 'ABB'))('%s restores strict analysis when its dedicated template marker is removed', async modelId => {
    const profile = getModelProgrammingProfile(modelId);
    const language = await loadLanguageDefinition(profile.defaultDialectId);
    const source = renderProgramTemplate(modelId, [point(modelId)]).code;
    expect(isGeneratedProgramTemplate(language.id, source)).toBe(true);
    const unmarked = source.replace(/COMMENT = "(?:SIM TEMPLATE|SIMULATION TEMPLATE)";/, 'COMMENT = "EDITED JOB";')
      .replace('///COMM SIMULATION TEMPLATE', '///COMM EDITED JOB');
    expect(unmarked).not.toBe(source);
    expect(isGeneratedProgramTemplate(language.id, unmarked)).toBe(false);
    const analysis = analyzeProgramSource(language, unmarked);
    expect(analysis).toEqual(language.analyze(unmarked));
    expect(pending(analysis)).toEqual([]);
    expect(errors(analysis).length).toBeGreaterThan(0);
  });
});
