import { describe, expect, it } from 'vitest';
import { isGeneratedProgramTemplate } from '../src/programming/template-analysis';

describe('explicit program template header markers', () => {
  it.each(['SIM TEMPLATE', 'SIMULATION TEMPLATE'])('recognizes current and legacy FANUC marker %s with original line endings', marker => {
    expect(isGeneratedProgramTemplate('fanuc-ls', `\uFEFF/PROG TEST\r\n/ATTR\r\nCOMMENT = "${marker}";\r\n/MN\r\n/POS\r\n/END`)).toBe(true);
  });
  it('recognizes INFORM only in the instruction header', () => {
    expect(isGeneratedProgramTemplate('inform', '\uFEFF/JOB\r//NAME TEST\r//POS\r///NPOS 0,0,0,0,0,0\r//INST\r///COMM SIMULATION TEMPLATE\rNOP\rEND')).toBe(true);
  });
  it.each([
    '/PROG TEST\n/ATTR\nOWNER = MNEDITOR;\n/MN\n: !COMMENT = "SIM TEMPLATE";\n/POS\n/END',
    '/PROG TEST\n/ATTR\nCOMMENT = "OTHER";\n/MN\nCOMMENT = "SIM TEMPLATE";\n/POS\n/END',
    '/PROG TEST\n/POS\n/ATTR\nCOMMENT = "SIM TEMPLATE";\n/END',
    '/PROG TEST\n/ATTR\nCOMMENT = "SIM TEMPLATE extra";\n/MN\n/POS\n/END',
    '/PROG TEST\n/ATTR\nCOMMENT = "SIM TEMPLATE"; trailing\n/MN\n/POS\n/END',
    '/PROG TEST\n/ATTR\n! COMMENT = "SIM TEMPLATE";\n/MN\n/POS\n/END',
    '/PROG TEST\n/APPL\nCOMMENT = "SIM TEMPLATE";\n/MN\n/POS\n/END',
    'COMMENT = "SIM TEMPLATE";\n/PROG TEST\n/ATTR\n/MN\n/POS\n/END',
  ])('does not opt in FANUC by arbitrary text, comments or a wrong section: %s', source => {
    expect(isGeneratedProgramTemplate('fanuc-ls', source)).toBe(false);
  });
  it.each([
    '/JOB\n//NAME TEST\n//POS\n///COMM SIMULATION TEMPLATE\n//INST\nNOP\nEND',
    '/JOB\n//INST\nNOP\n///COMM SIMULATION TEMPLATE\nEND',
    '/JOB\n//INST\n\'///COMM SIMULATION TEMPLATE\nNOP\nEND',
    '/JOB\n//INST\n///COMM SIMULATION TEMPLATE extra\nNOP\nEND',
    '/JOB\n//INST\n///COMM "SIMULATION TEMPLATE"\nNOP\nEND',
    '/JOB\n//INST\nMSG "///COMM SIMULATION TEMPLATE"\nEND',
    '///COMM SIMULATION TEMPLATE\n/JOB\n//INST\nNOP\nEND',
  ])('does not opt in INFORM by arbitrary text, comments or a wrong section: %s', source => {
    expect(isGeneratedProgramTemplate('inform', source)).toBe(false);
  });
  it('does not enable a template mode for another language', () => {
    for (const language of ['rapid', 'krl'] as const) expect(isGeneratedProgramTemplate(language, '/PROG TEST\n/ATTR\nCOMMENT = "SIM TEMPLATE";')).toBe(false);
  });
});
