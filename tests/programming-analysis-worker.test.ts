import { afterAll, beforeAll, beforeEach, describe, expect, it, vi } from 'vitest';
import type { AnalysisMessage, AnalysisReply } from '../src/programming/analysis-protocol';

const port = { onmessage: null as null | ((event: MessageEvent) => void), replies: [] as AnalysisReply[], postMessage(reply: AnalysisReply) { this.replies.push(structuredClone(reply)); } };
beforeAll(async () => {
  vi.stubGlobal('self', port);
  await import('../src/programming/analysis.worker');
});
beforeEach(() => { port.replies.length = 0; });
afterAll(() => { vi.unstubAllGlobals(); });
const request = (): AnalysisMessage => ({ requestId: 'request-one', modelId: 'irb1100-4-0.58', sourceRevision: 8, dialectId: 'rapid-rw7', files: [{ id: 'source-one', name: 'main.mod', text: 'MODULE Demo\nPROC main()\nStop;\nENDPROC\nENDMODULE' }] });
function send(data: unknown): AnalysisReply { port.onmessage!({ data } as MessageEvent); expect(port.replies).toHaveLength(1); return port.replies[0]; }

describe('source analysis worker protocol', () => {
  it('echoes context and analyzes the actual grammar rather than returning a synthetic clean result', () => {
    const input = request(); input.files = [{ ...input.files[0], text: 'MODULE Demo\nPROC main()\nvalue:=2E52786;\nENDPROC\nENDMODULE' }];
    const reply = send(input);
    expect(reply).toMatchObject({ requestId: input.requestId, modelId: input.modelId, sourceRevision: 8, dialectId: 'rapid-rw7' });
    expect(reply.error).toBeUndefined();
    expect(reply.result!.byFile['source-one'].diagnostics.some(d => d.severity === 'error' && d.message.includes('IEEE'))).toBe(true);
    expect(reply.result!.byFile['source-one'].symbols.map(s => s.name)).toEqual(['Demo', 'main']);
  });

  it('applies the requested RW6 profile and keeps independent multi-file results', () => {
    const input = request(); input.dialectId = 'rapid-rw6';
    input.files = [{ id: 'six', name: 'one.mod', text: 'MODULE A\nVAR num Break:=1;\nENDMODULE' }, { id: 'bad', name: 'two.mod', text: 'MODULE B\nCONST num missing;\nENDMODULE' }];
    const reply = send(input);
    expect(reply.result!.byFile.six.diagnostics).toEqual([]);
    expect(reply.result!.byFile.bad.diagnostics.some(d => d.severity === 'error')).toBe(true);
    expect(Object.keys(reply.result!.byFile)).toEqual(['six', 'bad']);
  });

  it('accepts an empty project but rejects missing files and invalid dialect/context explicitly', () => {
    expect(send({ ...request(), files: [] }).result).toEqual({ byFile: {} }); port.replies.length = 0;
    for (const patch of [{ files: null }, { dialectId: 'toString' }, { modelId: '' }, { sourceRevision: Infinity }, { requestId: '' }]) {
      const reply = send({ ...request(), ...patch }); expect(reply.error).toBeTruthy(); expect(reply.result).toBeUndefined(); port.replies.length = 0;
    }
  });

  it('preflights the entire request and returns no partial diagnostics when a later file is malformed', () => {
    const input = request(); input.files = [...input.files, { id: 'too-large', name: 'other.mod', text: '中'.repeat(350_000) }];
    const reply = send(input); expect(reply.error).toMatch(/大小限制/); expect(reply.result).toBeUndefined();
  });

  it('rejects duplicate IDs and oversized file counts before parsing', () => {
    const input = request(); input.files = [input.files[0], input.files[0]];
    expect(send(input).error).toMatch(/文件无效/); port.replies.length = 0;
    input.files = Array.from({ length: 33 }, (_, i) => ({ id: String(i), name: `source${i}.mod`, text: '' }));
    expect(send(input).error).toMatch(/请求无效/);
  });
});
