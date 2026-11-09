import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { createHash } from 'node:crypto';
const mock = vi.hoisted(() => ({ invoke: vi.fn(), fetch: vi.fn(), prepare: vi.fn(), dispose: vi.fn(), channels: [] as { onmessage: (message: unknown) => void }[] }));
vi.mock('@tauri-apps/api/core', () => ({
  isTauri: () => true,
  invoke: mock.invoke,
  convertFileSrc: (path: string) => `http://asset.localhost/${path}`,
  Channel: class { onmessage = (_message: unknown) => {}; constructor() { mock.channels.push(this); } },
}));
vi.mock('../src/catalog/urdf', () => ({ prepareUrdfModel: mock.prepare, disposePreparedUrdfModel: mock.dispose }));
import { CATALOG_REVISION, MODEL_CATALOG, listModelCache, loadCatalogModel, verifyModelFile } from '../src/catalog/client';
const downloadable = () => MODEL_CATALOG.find(entry => !entry.bundledMotionModelId)!;
const nativeResult = () => ({ modelId: downloadable().id, revision: CATALOG_REVISION, files: downloadable().files.map(file => ({ logicalPath: file.path, localPath: `/cache/${file.path}`, bytes: file.bytes, sha256: file.sha256 })) });
beforeEach(() => { vi.clearAllMocks(); mock.channels.length = 0; vi.stubGlobal('fetch', mock.fetch); });
afterEach(() => { vi.unstubAllGlobals(); });

describe('catalog download integrity and native boundary', () => {
  it('validates both exact byte count and SHA-256', async () => {
    const data = new TextEncoder().encode('robot').buffer;
    const file = { path: 'mesh.stl', bytes: data.byteLength, sha256: createHash('sha256').update(new Uint8Array(data)).digest('hex') };
    await expect(verifyModelFile(file, data)).resolves.toBeUndefined();
    await expect(verifyModelFile({ ...file, bytes: 4 }, data)).rejects.toThrow('大小不符');
    await expect(verifyModelFile({ ...file, sha256: '0'.repeat(64) }, data)).rejects.toThrow('校验失败');
  });
  it('rejects unknown IDs and already-cancelled requests without invoking downloads', async () => {
    await expect(loadCatalogModel('../../other')).rejects.toThrow('型号不在模型目录');
    const controller = new AbortController(); controller.abort();
    await expect(loadCatalogModel(downloadable().id, { signal: controller.signal })).rejects.toMatchObject({ name: 'AbortError' });
    expect(mock.invoke).not.toHaveBeenCalled();
  });
  it('never falls back to browser fetch after a native download error', async () => {
    mock.invoke.mockRejectedValue('连接失败');
    await expect(loadCatalogModel(downloadable().id)).rejects.toThrow('连接失败');
    expect(mock.fetch).not.toHaveBeenCalled();
    expect(mock.prepare).not.toHaveBeenCalled();
  });
  it('rejects a stale catalog revision before loading file URLs', async () => {
    mock.invoke.mockResolvedValue({ ...nativeResult(), revision: 'stale' });
    await expect(loadCatalogModel(downloadable().id)).rejects.toThrow('目录不匹配');
    expect(mock.fetch).not.toHaveBeenCalled();
  });
  it('rejects malformed IPC metadata without leaving the request pending', async () => {
    mock.invoke.mockResolvedValue({ modelId: downloadable().id, revision: CATALOG_REVISION });
    await expect(loadCatalogModel(downloadable().id)).rejects.toThrow('目录不匹配');
    expect(mock.fetch).not.toHaveBeenCalled();
  });
  it('cancels native work and ignores its late response', async () => {
    let finish: (value: unknown) => void = () => {};
    mock.invoke.mockImplementation((command: string) => command === 'ensure_model' ? new Promise(resolve => { finish = resolve; }) : Promise.resolve());
    const controller = new AbortController();
    const loading = loadCatalogModel(downloadable().id, { signal: controller.signal });
    const assertion = expect(loading).rejects.toMatchObject({ name: 'AbortError' });
    controller.abort();
    await assertion;
    finish(nativeResult());
    await Promise.resolve();
    expect(mock.invoke).toHaveBeenCalledWith('cancel_model_download', { requestId: expect.any(String) });
    expect(mock.fetch).not.toHaveBeenCalled();
  });
  it('returns prepared assets only once and disposes successful selections idempotently', async () => {
    const prepared = { id: downloadable().id };
    mock.invoke.mockResolvedValue(nativeResult());
    mock.fetch.mockResolvedValue({ ok: true, text: async () => '<robot />' });
    mock.prepare.mockResolvedValue(prepared);
    const loaded = await loadCatalogModel(downloadable().id);
    expect(loaded.kind).toBe('urdf');
    if (loaded.kind !== 'urdf') throw new Error('expected downloaded model');
    expect(mock.prepare).toHaveBeenCalledWith(expect.objectContaining({ id: downloadable().id, assetUrls: expect.objectContaining({ [downloadable().urdfPath]: expect.stringContaining('asset.localhost') }) }));
    loaded.dispose(); loaded.dispose();
    expect(mock.dispose).toHaveBeenCalledTimes(1);
    expect(mock.dispose).toHaveBeenCalledWith(prepared);
  });
  it('ignores cache claims from other revisions and always exposes bundled models offline', async () => {
    mock.invoke.mockResolvedValue([{ modelId: downloadable().id, revision: 'old', cached: true }]);
    const status = await listModelCache();
    expect(status[downloadable().id]).toBe(false);
    for (const entry of MODEL_CATALOG.filter(model => model.bundledMotionModelId)) expect(status[entry.id]).toBe(true);
  });
});
