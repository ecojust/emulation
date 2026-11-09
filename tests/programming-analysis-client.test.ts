import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import type { AnalysisMessage, AnalysisReply, SourceAnalysisRequest } from '../src/programming/analysis-protocol';

const harness = vi.hoisted(() => ({ workers: [] as MockWorker[], constructorError: false, postError: false, onCreate: null as null | (() => void) }));
class MockWorker {
  onmessage: ((event: MessageEvent<unknown>) => void) | null = null;
  onerror: (() => void) | null = null;
  onmessageerror: (() => void) | null = null;
  sent: AnalysisMessage[] = [];
  terminate = vi.fn();
  constructor() {
    if (harness.constructorError) throw new Error('No worker');
    harness.workers.push(this); harness.onCreate?.();
  }
  postMessage(message: AnalysisMessage) {
    if (harness.postError) throw new DOMException('Cannot clone', 'DataCloneError');
    this.sent.push(structuredClone(message));
  }
  emit(data: unknown) { this.onmessage?.({ data } as MessageEvent<unknown>); }
}
vi.mock('../src/programming/analysis.worker?worker&inline', () => ({ default: MockWorker }));

let client: typeof import('../src/programming/analysis-client');
const request = (): SourceAnalysisRequest => ({ modelId: 'irb1100-4-0.58', sourceRevision: 3, dialectId: 'rapid-rw7', files: [{ id: 'file-one', name: 'main.mod', text: '! 😀\nMODULE Demo\nENDMODULE' }] });
const emptyAnalysis = () => ({ diagnostics: [], symbols: [], folds: [] });
function reply(worker: MockWorker, patch: Record<string, unknown> = {}): AnalysisReply {
  const sent = worker.sent.at(-1)!;
  return { requestId: sent.requestId, modelId: sent.modelId, sourceRevision: sent.sourceRevision, dialectId: sent.dialectId,
    result: { byFile: Object.fromEntries(sent.files.map(file => [file.id, emptyAnalysis()])) }, ...patch } as AnalysisReply;
}
beforeEach(async () => {
  vi.resetModules(); vi.useFakeTimers(); harness.workers.length = 0; harness.constructorError = false; harness.postError = false; harness.onCreate = null;
  client = await import('../src/programming/analysis-client');
});
afterEach(() => { vi.useRealTimers(); });

describe('isolated source analysis requests', () => {
  it('posts source identity and accepts UTF-16 source ranges without aliasing request/reply objects', async () => {
    const input = request(); const promise = client.analyzeSourceProject(input); const worker = harness.workers[0];
    expect(worker.sent[0]).toMatchObject({ modelId: input.modelId, sourceRevision: 3, dialectId: 'rapid-rw7' });
    const response = reply(worker);
    response.result!.byFile['file-one'].diagnostics.push({ from: 2, to: 4, severity: 'warning', message: 'emoji span' });
    worker.emit(response); const result = await promise;
    expect(input.files[0].text.slice(result.byFile['file-one'].diagnostics[0].from, result.byFile['file-one'].diagnostics[0].to)).toBe('😀');
    response.result!.byFile['file-one'].diagnostics[0].message = 'mutated payload';
    expect(result.byFile['file-one'].diagnostics[0].message).toBe('emoji span');
    expect(worker.terminate).not.toHaveBeenCalled(); expect(vi.getTimerCount()).toBe(0);
  });

  it('validates against the posted immutable snapshot even if a caller changes its input later', async () => {
    const input = request(); const originalLength = input.files[0].text.length;
    const promise = client.analyzeSourceProject(input); const worker = harness.workers[0];
    input.files[0].text = 'x'; input.sourceRevision = 4; input.modelId = 'yaskawa-gp7';
    const response = reply(worker); response.result!.byFile['file-one'].diagnostics.push({ from: originalLength, to: originalLength, severity: 'error', message: 'EOF' });
    worker.emit(response); expect((await promise).byFile['file-one'].diagnostics[0].from).toBe(originalLength);
    expect(worker.sent[0].sourceRevision).toBe(3);
  });

  it('ignores late IDs on a reused worker without resolving or clearing the newer request', async () => {
    const first = client.analyzeSourceProject(request()); const worker = harness.workers[0]; const stale = reply(worker);
    worker.emit(stale); await first;
    const secondInput = { ...request(), sourceRevision: 4 };
    const second = client.analyzeSourceProject(secondInput); const finished = vi.fn(); void second.then(finished);
    expect(harness.workers).toHaveLength(1); expect(worker.sent).toHaveLength(2);
    worker.emit(stale); await Promise.resolve(); expect(finished).not.toHaveBeenCalled();
    const current = reply(worker); current.result!.byFile['file-one'].diagnostics.push({ from: 0, to: 1, severity: 'error', message: 'current error' });
    worker.emit(current); expect((await second).byFile['file-one'].diagnostics[0].message).toBe('current error');
  });

  it.each([{ modelId: 'yaskawa-gp7' }, { sourceRevision: 2 }, { dialectId: 'rapid-rw6' }])('rejects a matching ID with wrong source context %j', async patch => {
    const promise = client.analyzeSourceProject(request()); const worker = harness.workers[0]; const rejected = expect(promise).rejects.toThrow(/型号或源码版本/);
    worker.emit(reply(worker, patch)); await rejected; expect(worker.terminate).toHaveBeenCalledOnce();
  });

  it('handles a pre-aborted signal without creating a worker', async () => {
    const controller = new AbortController(); controller.abort();
    await expect(client.analyzeSourceProject(request(), controller.signal)).rejects.toMatchObject({ name: 'AbortError' });
    expect(harness.workers).toHaveLength(0);
  });

  it('detaches the completed request abort listener before reusing its worker', async () => {
    const controller = new AbortController();
    const first = client.analyzeSourceProject(request(), controller.signal); const worker = harness.workers[0];
    worker.emit(reply(worker)); await first;
    const second = client.analyzeSourceProject({ ...request(), sourceRevision: 4 });
    controller.abort(); expect(worker.terminate).not.toHaveBeenCalled();
    worker.emit(reply(worker)); await second; expect(vi.getTimerCount()).toBe(0);
  });

  it('terminates aborted work and ignores an already queued completion', async () => {
    const controller = new AbortController(); const old = client.analyzeSourceProject(request(), controller.signal); const worker = harness.workers[0];
    const queued = worker.onmessage!; const oldReply = reply(worker);
    const rejected = expect(old).rejects.toMatchObject({ name: 'AbortError' }); controller.abort(); await rejected;
    expect(worker.terminate).toHaveBeenCalledOnce(); expect(worker.onmessage).toBeNull();
    const next = client.analyzeSourceProject({ ...request(), sourceRevision: 4 }); const nextWorker = harness.workers[1];
    queued({ data: oldReply } as MessageEvent); nextWorker.emit(reply(nextWorker)); await next;
    expect(worker.terminate).toHaveBeenCalledOnce(); expect(vi.getTimerCount()).toBe(0);
  });

  it('handles re-entrant abort during worker construction without posting work', async () => {
    const controller = new AbortController(); harness.onCreate = () => controller.abort();
    await expect(client.analyzeSourceProject(request(), controller.signal)).rejects.toMatchObject({ name: 'AbortError' });
    expect(harness.workers[0].sent).toHaveLength(0); expect(harness.workers[0].terminate).toHaveBeenCalledOnce();
    expect(vi.getTimerCount()).toBe(0);
  });

  it('kills a stuck worker at the five-second deadline and never reuses it', async () => {
    const promise = client.analyzeSourceProject(request()); const worker = harness.workers[0];
    const rejected = expect(promise).rejects.toThrow(/超时/);
    await vi.advanceTimersByTimeAsync(4999); expect(worker.terminate).not.toHaveBeenCalled();
    await vi.advanceTimersByTimeAsync(1); await rejected; expect(worker.terminate).toHaveBeenCalledOnce();
    const next = client.analyzeSourceProject(request()); const fresh = harness.workers[1]; fresh.emit(reply(fresh)); await next;
  });

  it.each(['onerror', 'onmessageerror'] as const)('turns %s into an explicit failed analysis', async event => {
    const promise = client.analyzeSourceProject(request()); const worker = harness.workers[0]; const rejected = expect(promise).rejects.toThrow();
    worker[event]!(); await rejected; expect(worker.terminate).toHaveBeenCalledOnce();
    expect(worker.onmessage).toBeNull(); expect(worker.onerror).toBeNull(); expect(worker.onmessageerror).toBeNull(); expect(vi.getTimerCount()).toBe(0);
  });

  it('cleans up construction and postMessage failures', async () => {
    harness.constructorError = true; await expect(client.analyzeSourceProject(request())).rejects.toThrow(/启动/);
    expect(vi.getTimerCount()).toBe(0); harness.constructorError = false; harness.postError = true;
    await expect(client.analyzeSourceProject(request())).rejects.toMatchObject({ name: 'DataCloneError' });
    expect(harness.workers[0].terminate).toHaveBeenCalledOnce(); expect(vi.getTimerCount()).toBe(0);
  });

  it('returns worker errors without converting them to a clean diagnostics list', async () => {
    const promise = client.analyzeSourceProject(request()); const worker = harness.workers[0]; const rejected = expect(promise).rejects.toThrow('parser failed');
    worker.emit(reply(worker, { result: undefined, error: 'parser failed' })); await rejected;
    expect(worker.terminate).toHaveBeenCalledOnce();
  });

  it('rejects ambiguous error-plus-success replies', async () => {
    const promise = client.analyzeSourceProject(request()); const worker = harness.workers[0]; const rejected = expect(promise).rejects.toThrow(/无效错误/);
    worker.emit(reply(worker, { error: 'error and result' })); await rejected;
  });

  it('preflights invalid source revisions, dialects, duplicate IDs and UTF-8 bounds before worker creation', async () => {
    await expect(client.analyzeSourceProject({ ...request(), sourceRevision: -1 })).rejects.toThrow();
    await expect(client.analyzeSourceProject({ ...request(), dialectId: 'bogus' } as unknown as SourceAnalysisRequest)).rejects.toThrow();
    const input = request(); await expect(client.analyzeSourceProject({ ...input, files: [input.files[0], input.files[0]] })).rejects.toThrow();
    await expect(client.analyzeSourceProject({ ...input, files: [{ ...input.files[0], text: '中'.repeat(350_000) }] })).rejects.toThrow(/大小限制/);
    expect(harness.workers).toHaveLength(0);
  });

  it('does not terminate another concurrently active worker when caching an idle one', async () => {
    const first = client.analyzeSourceProject(request()); const a = harness.workers[0];
    const second = client.analyzeSourceProject({ ...request(), sourceRevision: 4 }); const b = harness.workers[1];
    a.emit(reply(a)); await first;
    const third = client.analyzeSourceProject({ ...request(), sourceRevision: 5 }); expect(harness.workers).toHaveLength(2);
    b.emit(reply(b)); await second; expect(a.terminate).not.toHaveBeenCalled();
    a.emit(reply(a)); await third; expect(b.terminate).toHaveBeenCalledOnce(); expect(a.terminate).not.toHaveBeenCalled();
  });
});

describe('analysis reply source positions and structure', () => {
  it.each([
    { diagnostics: [{ from: -1, to: 1, severity: 'error', message: 'bad' }] },
    { diagnostics: [{ from: 1, to: 0, severity: 'error', message: 'bad' }] },
    { diagnostics: [{ from: 0, to: 100_000, severity: 'error', message: 'bad' }] },
    { diagnostics: [{ from: 0.5, to: 1, severity: 'error', message: 'bad' }] },
    { diagnostics: [{ from: 0, to: 1, severity: ['error'], message: 'bad' }] },
    { diagnostics: [{ from: 0, to: 1, severity: 'error', message: 'x'.repeat(16_385) }] },
    { symbols: [{ from: 0, to: 1, name: 'x', kind: 'made-up' }] },
    { symbols: [{ from: 0, to: 1, name: '', kind: 'variable' }] },
    { symbols: [{ from: 0, to: 1, name: 'x', kind: 'variable', description: [] }] },
    { folds: [{ from: 2, to: 1 }] },
    { folds: [{ from: 2, to: 2 }] },
  ])('fails closed for malformed payload %j', async patch => {
    const promise = client.analyzeSourceProject(request()); const worker = harness.workers[0];
    const rejected = expect(promise).rejects.toThrow(/源码位置/);
    worker.emit(reply(worker, { result: { byFile: { 'file-one': { ...emptyAnalysis(), ...patch } } } })); await rejected;
    expect(worker.terminate).toHaveBeenCalledOnce();
  });

  it('requires one own-property result per requested file, without extra files', () => {
    expect(() => client.validateProjectAnalysis({ byFile: {} }, request())).toThrow();
    expect(() => client.validateProjectAnalysis({ byFile: { 'file-one': emptyAnalysis(), extra: emptyAnalysis() } }, request())).toThrow();
    const inherited = Object.create({ 'file-one': emptyAnalysis() }); inherited.extra = emptyAnalysis();
    expect(() => client.validateProjectAnalysis({ byFile: inherited }, request())).toThrow();
    const input = { ...request(), files: [{ ...request().files[0], id: '__proto__' }] };
    const own = Object.create(null); own.__proto__ = emptyAnalysis();
    const result = client.validateProjectAnalysis({ byFile: own }, input);
    expect(Object.getPrototypeOf(result.byFile)).toBeNull(); expect(Object.hasOwn(result.byFile, '__proto__')).toBe(true);
  });
});
