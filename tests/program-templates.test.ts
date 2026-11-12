import { describe, expect, it } from 'vitest';
import { Euler, Quaternion } from 'three';
import catalog from '../src/catalog/catalog.json';
import { fanucLanguage } from '../src/programming/languages/fanuc';
import { informLanguage } from '../src/programming/languages/inform';
import {
  formatProgramPoint, getProgramProfile, PROGRAM_MODEL_IDS, quaternionToControllerEuler, renderProgramTemplate,
  type ProgramTemplateContext, type ProgramTemplatePoint,
} from '../src/workspace/program-templates';

const point = (abb = false): ProgramTemplatePoint => ({
  pointName: 'P001', pose: { position: [123.4567, -234.5678, 345.6789], orientation: [1, 0, 0, 0] },
  joints: [-10, 20, -30, 40, -50, 60], configuration: abb ? [-1, 0, 0, 3] : null, speedMmPerSecond: 123.5,
});

describe('manufacturer-specific simulation templates', () => {
  it('binds the six catalog models to their own manufacturer without inventing KUKA support', () => {
    expect(PROGRAM_MODEL_IDS).toHaveLength(6);
    for (const model of catalog.models) expect(getProgramProfile(model.id).brand).toBe(model.brand);
    expect(getProgramProfile('irb1100-4-0.58').controllerExport).toBe('verified-abb-segment');
    for (const model of catalog.models.filter(model => model.brand !== 'ABB')) expect(getProgramProfile(model.id).controllerExport).toBe('unsupported');
    expect(() => getProgramProfile('kuka-kr6')).toThrow('目录');
    expect(() => getProgramProfile('__proto__')).toThrow('目录');
  });

  it('preserves ABB quaternion/configuration data and uses valid explicit speeddata', () => {
    const input = point(true);
    const formatted = formatProgramPoint('irb1100-4-0.58', input);
    expect(formatted.declaration).toBe('CONST robtarget P001 := [[123.457,-234.568,345.679],[1,0,0,0],[-1,0,0,3],[9E9,9E9,9E9,9E9,9E9,9E9]];');
    expect(formatted.instruction).toBe('MoveL P001, [123.5,500,5000,1000], fine, tool0\\WObj:=wobj0;');
    const template = renderProgramTemplate('irb1100-4-0.58', [input]);
    expect(template.code).toContain('MODULE EMULATE');
    expect(template.code).toContain('ConfL \\On;');
    expect(template.exportable).toBe(false);
    expect(template.notice).toContain('运行前逐段验证');
    expect(template.unresolvedFields).toEqual([]);
  });

  it('uses FANUC joint-defined position records without inventing a Cartesian CONFIG', () => {
    const input = point(), target = formatProgramPoint('fanuc-crx10ia-l', input);
    expect(target.instruction).toBe('L P[1] 123.5mm/sec FINE ;');
    expect(target.declaration).toContain('GP1:');
    for (const [index, angle] of input.joints.entries()) expect(target.declaration).toContain(`J${index + 1}=${angle} deg`);
    expect(target.declaration).not.toContain('CONFIG');
    expect(target.declaration).not.toContain('robconf');
    const template = renderProgramTemplate('fanuc-crx10ia-l', [input]);
    expect(template.code.indexOf('/MN')).toBeLessThan(template.code.indexOf('5:L P[1]'));
    expect(template.code.indexOf('/POS')).toBeLessThan(template.code.indexOf('P[1]{'));
    expect(template.code).toContain('UF : {{UF}}, UT : {{UT}}');
    expect(template.unresolvedFields).toContain('控制器关节零位与耦合映射');
    expect(template.notice).toContain('当前不支持此品牌的控制器文件导出');
    expect(template.exportable).toBe(false);
  });

  it('uses Yaskawa Cartesian records with explicit missing tool/RCONF and no fabricated pulses', () => {
    const input = point(), template = renderProgramTemplate('yaskawa-gp7', [input]);
    expect(template.code).toContain('///POSTYPE BASE\n///RECTAN');
    expect(template.code).toContain('///TOOL {{TOOL}}');
    expect(template.code).toContain('///RCONF {{RCONF_P001}}\nC00000=123.4567,-234.5678,345.6789,0,0,0');
    expect(template.code).toContain('MOVL C00000 V=123.5');
    expect(template.code).toContain('///ATTR SC,RW,RJ');
    expect(template.code).not.toContain('PULSE');
    expect(template.code).not.toContain('robconf');
    expect(template.unresolvedFields).toContain('RCONF_P001');
    expect(template.exportable).toBe(false);
  });

  it('keeps instruction order and point references consistent after insertion, deletion or Path filtering', () => {
    const inputs = ['P001', 'P004', 'P003'].map(pointName => ({ ...point(), pointName }));
    const fanuc = renderProgramTemplate('fanuc-r2000ib-165f', inputs).code;
    expect([...fanuc.matchAll(/\d+:L P\[(\d+)\]/g)].map(match => Number(match[1]))).toEqual([1, 4, 3]);
    expect([...fanuc.matchAll(/^P\[(\d+)\]\{/gm)].map(match => Number(match[1]))).toEqual([1, 4, 3]);
    const yaskawa = renderProgramTemplate('yaskawa-gp110', inputs).code;
    expect([...yaskawa.matchAll(/^MOVL (C\d+) /gm)].map(match => match[1])).toEqual(['C00000', 'C00003', 'C00002']);
    expect([...yaskawa.matchAll(/^(C\d+)=/gm)].map(match => match[1])).toEqual(['C00000', 'C00003', 'C00002']);
    expect(formatProgramPoint('yaskawa-gp110', inputs[1]).reference).toBe('C00003');
    expect(renderProgramTemplate('yaskawa-gp110', [inputs[1]]).code).toContain('MOVL C00003');
  });

  it('rejects foreign controller metadata, missing ABB configuration and unsafe names', () => {
    expect(() => formatProgramPoint('fanuc-crx10ia-l', point(true))).toThrow('非 ABB');
    expect(() => formatProgramPoint('yaskawa-gp7', point(true))).toThrow('非 ABB');
    expect(() => formatProgramPoint('irb1100-4-0.58', point())).toThrow('robconf');
    expect(() => renderProgramTemplate('yaskawa-gp7', [point(), point()])).toThrow('重复');
    expect(() => renderProgramTemplate('fanuc-crx10ia-l', [point()], 'X\n/MN')).toThrow('名称');
    expect(() => formatProgramPoint('yaskawa-gp7', { ...point(), pointName: 'P001\nMOVJ' })).toThrow('名称');
    expect(() => formatProgramPoint('yaskawa-gp7', { ...point(), speedMmPerSecond: NaN })).toThrow('速度');
  });

  it.each(['yaskawa-gp7', 'yaskawa-gp110'])('enforces the %s Cartesian record limit without restricting ABB or FANUC to it', model => {
    const last = { ...point(), pointName: 'P9998' };
    expect(formatProgramPoint(model, last).reference).toBe('C09997');
    expect(renderProgramTemplate(model, [last]).code).toContain('///RCONF {{RCONF_P9998}}\nC09997=');
    for (const pointName of ['P9999', 'P10000', 'P99999']) {
      expect(() => formatProgramPoint(model, { ...point(), pointName })).toThrow('P9998');
      expect(() => renderProgramTemplate(model, [{ ...point(), pointName }])).toThrow('P9998');
    }
    expect(formatProgramPoint('fanuc-crx10ia-l', { ...point(), pointName: 'P99999' }).reference).toBe('P[99999]');
    expect(formatProgramPoint('irb1100-4-0.58', { ...point(true), pointName: 'P99999' }).reference).toBe('P99999');
  });

  it.each(['fanuc-crx10ia-l', 'fanuc-r2000ib-165f', 'yaskawa-gp7', 'yaskawa-gp110'])('keeps %s comment bodies within 32 ASCII characters and preserves all frame values', model => {
    const context: ProgramTemplateContext = {
      referencePose: { position: [98765.12345, -87654.23456, 76543.34567], orientation: [0.9238795325112867, 0, 0.3826834323650898, 0] },
      toolPose: { position: [12345.123456, -23456.234567, 34567.345678], orientation: [0.5, 0.5, 0.5, 0.5] },
    };
    for (const frames of [undefined, context]) {
      const result = renderProgramTemplate(model, [point()], 'FRAME', frames);
      const fanuc = model.startsWith('fanuc');
      const comments = [...result.code.matchAll(fanuc ? /^\s*\d+:!([^;\r\n]*);$/gm : /^'([^\r\n]*)$/gm)].map(match => match[1]);
      expect(comments.length).toBeGreaterThanOrEqual(4);
      expect(comments.every(text => text.length <= 32 && /^[\x20-\x7e]+$/.test(text))).toBe(true);
      expect(comments).toContain('SIMULATION TEMPLATE');
      expect(comments).toContain('Controller use not validated.');
      expect(comments).toContain(fanuc ? 'Set UF/UT for the controller.' : 'Set TOOL/RCONF for controller.');
      if (frames) {
        const joined = comments.join('').replaceAll(' ', '');
        expect(joined).toContain('REFERENCE_BASE_XYZ98765.123,-87654.235,76543.346');
        expect(joined).toContain('REFERENCE_BASE_Q0.923879533,0,0.382683432,0');
        expect(joined).toContain('TOOL_FLANGE_XYZ12345.123,-23456.235,34567.346');
        expect(joined).toContain('TOOL_FLANGE_Q0.5,0.5,0.5,0.5');
      }
      if (fanuc) {
        expect(result.code).toContain('COMMENT = "SIM TEMPLATE";');
        expect(fanucLanguage.analyze(result.code).diagnostics.some(item => item.code === 'fanuc.version')).toBe(false);
      }
    }
  });

  it('keeps ordinary language analysis strict for unresolved generated controller parameters', () => {
    expect(fanucLanguage.analyze(renderProgramTemplate('fanuc-crx10ia-l', [point()]).code).diagnostics.some(item => item.severity === 'error' && item.code === 'fanuc.frame-index')).toBe(true);
    expect(informLanguage.analyze(renderProgramTemplate('yaskawa-gp7', [point()]).code).diagnostics.some(item => item.severity === 'error')).toBe(true);
  });
});

describe('controller orientation conversion', () => {
  it.each([[21, -32, 73], [-144, 69, 178], [31, 90, -82], [-41, -90, 107], [0, 0, 90]])('preserves the rotation at Rx/Ry/Rz=%j including gimbal lock', (rx, ry, rz) => {
    const radians = Math.PI / 180;
    const original = new Quaternion().setFromEuler(new Euler(rx * radians, ry * radians, rz * radians, 'ZYX'));
    const xyzw = [original.w, original.x, original.y, original.z] as [number, number, number, number];
    for (const sign of [1, -1]) {
      const angles = quaternionToControllerEuler(xyzw.map(value => value * sign) as typeof xyzw);
      const reconstructed = new Quaternion().setFromEuler(new Euler(angles[0] * radians, angles[1] * radians, angles[2] * radians, 'ZYX'));
      expect(reconstructed.angleTo(original)).toBeLessThan(1e-7);
      expect(angles.every(Number.isFinite)).toBe(true);
    }
  });
});
