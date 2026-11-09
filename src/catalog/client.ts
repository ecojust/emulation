import { Channel, convertFileSrc, invoke, isTauri } from '@tauri-apps/api/core';
import catalogSource from './catalog.json';
import type { RobotCatalog, RobotCatalogFile, RobotCatalogModel } from './types';
import { disposePreparedUrdfModel, prepareUrdfModel, type PreparedUrdfModel } from './urdf';
import type { RobotId } from '../robot/types';

const catalog = catalogSource as RobotCatalog;
export const MODEL_CATALOG: readonly RobotCatalogModel[] = catalog.models;
export const CATALOG_REVISION = catalog.source.revision;
export interface ModelProgress {
  phase: 'checking' | 'downloading' | 'preparing' | 'ready';
  downloadedBytes: number;
  totalBytes: number;
  file?: string;
}
export type LoadedCatalogModel =
  | { kind: 'bundled'; entry: RobotCatalogModel; motionModelId: RobotId }
  | { kind: 'urdf'; entry: RobotCatalogModel; model: PreparedUrdfModel; dispose(): void };
export interface ModelLoadOptions { signal?: AbortSignal; onProgress?: (progress: ModelProgress) => void }
interface StoredModel { key: string; id: string; revision: string; files: { path: string; bytes: ArrayBuffer }[] }
interface NativeCachedModel { modelId: string; revision: string; files: { logicalPath: string; localPath: string; bytes: number; sha256: string }[] }
interface NativeProgress extends ModelProgress { requestId: string; modelId: string }
const DB_NAME = 'emulation-model-cache';
const STORE = 'models';
const abortError = () => new DOMException('模型下载已取消', 'AbortError');
function checkAbort(signal?: AbortSignal) { if (signal?.aborted) throw abortError(); }
function motionModelId(entry: RobotCatalogModel): RobotId | null {
  const id = entry.bundledMotionModelId;
  if (id === 'irb1100-4-0.58' || id === 'irb1100-4-0.475') return id;
  if (id) throw new Error('型号目录中的运动适配器尚未实现');
  return null;
}
function getEntry(id: string): RobotCatalogModel {
  const entry = MODEL_CATALOG.find(model => model.id === id);
  if (!entry) throw new Error('机器人型号不在模型目录中');
  return entry;
}
function cacheKey(id: string) { return `${id}@${CATALOG_REVISION}`; }

export async function verifyModelFile(file: Pick<RobotCatalogFile, 'bytes' | 'sha256' | 'path'>, data: ArrayBuffer): Promise<void> {
  if (!(data instanceof ArrayBuffer) || data.byteLength !== file.bytes) throw new Error(`模型文件大小不符：${file.path}`);
  const digest = await crypto.subtle.digest('SHA-256', data);
  const actual = Array.from(new Uint8Array(digest), byte => byte.toString(16).padStart(2, '0')).join('');
  if (actual !== file.sha256) throw new Error(`模型文件校验失败：${file.path}`);
}

function openCache(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    if (typeof indexedDB === 'undefined') { reject(new Error('浏览器无法保存模型缓存，请使用桌面客户端')); return; }
    const request = indexedDB.open(DB_NAME, 1);
    request.onupgradeneeded = () => request.result.createObjectStore(STORE, { keyPath: 'key' });
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(new Error('无法打开模型缓存，请检查存储权限'));
    request.onblocked = () => reject(new Error('模型缓存正在升级，请关闭其他应用页面后重试'));
  });
}
async function readCache(id: string): Promise<StoredModel | undefined> {
  const db = await openCache();
  try {
    return await new Promise<StoredModel | undefined>((resolve, reject) => {
      const request = db.transaction(STORE, 'readonly').objectStore(STORE).get(cacheKey(id));
      request.onsuccess = () => resolve(request.result);
      request.onerror = () => reject(new Error('无法读取模型缓存'));
    });
  } finally { db.close(); }
}
async function writeCache(record: StoredModel, signal?: AbortSignal): Promise<void> {
  checkAbort(signal);
  const db = await openCache();
  try {
    await new Promise<void>((resolve, reject) => {
      const transaction = db.transaction(STORE, 'readwrite');
      const abort = () => { try { transaction.abort(); } catch { /* Already finished. */ } };
      signal?.addEventListener('abort', abort, { once: true });
      const cleanup = () => signal?.removeEventListener('abort', abort);
      transaction.objectStore(STORE).put(record);
      transaction.oncomplete = () => { cleanup(); resolve(); };
      transaction.onerror = () => { cleanup(); reject(new Error('无法保存模型缓存，请检查可用存储空间')); };
      transaction.onabort = () => { cleanup(); reject(signal?.aborted ? abortError() : new Error('模型缓存写入失败')); };
      if (signal?.aborted) abort();
    });
  } finally { db.close(); }
}
async function verifiedCache(entry: RobotCatalogModel, signal?: AbortSignal): Promise<Map<string, ArrayBuffer> | null> {
  const record = await readCache(entry.id);
  checkAbort(signal);
  if (!record || record.id !== entry.id || record.revision !== CATALOG_REVISION || !Array.isArray(record.files) || record.files.length !== entry.files.length) return null;
  const files = new Map<string, ArrayBuffer>();
  for (const item of record.files) {
    if (!item || typeof item.path !== 'string' || files.has(item.path)) return null;
    files.set(item.path, item.bytes);
  }
  for (const file of entry.files) {
    checkAbort(signal);
    const bytes = files.get(file.path);
    if (!bytes) return null;
    try { await verifyModelFile(file, bytes); } catch { return null; }
  }
  return files;
}

/** Cache status is verified, never inferred merely from a previous download flag. */
export async function listModelCache(signal?: AbortSignal): Promise<Record<string, boolean>> {
  checkAbort(signal);
  const status: Record<string, boolean> = {};
  for (const entry of MODEL_CATALOG) status[entry.id] = !!motionModelId(entry);
  if (isTauri()) {
    const results = await invoke<{ modelId: string; revision: string; cached: boolean }[]>('list_model_cache');
    checkAbort(signal);
    for (const result of results) if (result.revision === CATALOG_REVISION && Object.hasOwn(status, result.modelId)) status[result.modelId] ||= result.cached;
  } else {
    for (const entry of MODEL_CATALOG) {
      checkAbort(signal);
      if (!status[entry.id]) status[entry.id] = !!(await verifiedCache(entry, signal));
    }
  }
  return status;
}

async function downloadBrowserFile(file: RobotCatalogFile, signal: AbortSignal, onBytes: (bytes: number) => void): Promise<ArrayBuffer> {
  const url = new URL(file.url);
  if (url.protocol !== 'https:' || url.hostname !== 'raw.githubusercontent.com' || !/^\/(?:work-r-labs\/robots|spdx\/license-list-data)\/[a-f0-9]{40}\//.test(url.pathname)) throw new Error('模型下载来源不在已验证目录中');
  const response = await fetch(url, { signal, redirect: 'error', credentials: 'omit' });
  if (!response.ok) throw new Error(`模型下载失败（HTTP ${response.status}）：${file.path}`);
  if (!response.body) throw new Error(`模型下载未返回文件：${file.path}`);
  const reader = response.body.getReader();
  const data = new Uint8Array(file.bytes);
  let received = 0;
  try {
    for (;;) {
      checkAbort(signal);
      const { value, done } = await reader.read();
      if (done) break;
      if (value.length > data.length - received) throw new Error(`模型文件超出目录中的大小：${file.path}`);
      data.set(value, received);
      received += value.length;
      onBytes(value.length);
    }
    if (received !== data.length) throw new Error(`模型文件下载不完整：${file.path}`);
    await verifyModelFile(file, data.buffer);
    return data.buffer;
  } finally { await reader.cancel().catch(() => {}); reader.releaseLock(); }
}

async function browserAssets(entry: RobotCatalogModel, signal: AbortSignal, onProgress: (value: ModelProgress) => void): Promise<{ urls: Record<string, string>; dispose(): void }> {
  let files = await verifiedCache(entry, signal);
  if (!files) {
    files = new Map();
    let downloadedBytes = 0;
    for (const file of entry.files) {
      const data = await downloadBrowserFile(file, signal, bytes => {
        downloadedBytes += bytes;
        onProgress({ phase: 'downloading', downloadedBytes, totalBytes: entry.downloadBytes, file: file.path });
      });
      files.set(file.path, data);
    }
    checkAbort(signal);
    await writeCache({ key: cacheKey(entry.id), id: entry.id, revision: CATALOG_REVISION, files: [...files].map(([path, bytes]) => ({ path, bytes })) }, signal);
  }
  checkAbort(signal);
  const urls: Record<string, string> = {};
  try {
    for (const [path, bytes] of files) urls[path] = URL.createObjectURL(new Blob([bytes], { type: path.endsWith('.urdf') ? 'application/xml' : 'application/octet-stream' }));
  } catch (error) {
    for (const url of Object.values(urls)) URL.revokeObjectURL(url);
    throw error;
  }
  return { urls, dispose() { for (const url of Object.values(urls)) URL.revokeObjectURL(url); } };
}

function nativeAssets(entry: RobotCatalogModel, signal: AbortSignal, onProgress: (value: ModelProgress) => void): Promise<{ urls: Record<string, string>; dispose(): void }> {
  return new Promise((resolve, reject) => {
    checkAbort(signal);
    const requestId = crypto.randomUUID();
    const channel = new Channel<NativeProgress>();
    let settled = false;
    const abort = () => {
      if (settled) return;
      settled = true;
      signal.removeEventListener('abort', abort);
      void invoke('cancel_model_download', { requestId }).catch(() => {});
      reject(abortError());
    };
    signal.addEventListener('abort', abort, { once: true });
    channel.onmessage = progress => {
      if (!settled && progress.requestId === requestId && progress.modelId === entry.id) onProgress(progress);
    };
    void invoke<NativeCachedModel>('ensure_model', { requestId, modelId: entry.id, onProgress: channel }).then(result => {
      if (settled) return;
      settled = true;
      signal.removeEventListener('abort', abort);
      try {
      if (!result || result.modelId !== entry.id || result.revision !== CATALOG_REVISION || !Array.isArray(result.files) || result.files.length !== entry.files.length) { reject(new Error('原生模型缓存返回的目录不匹配')); return; }
      const urls: Record<string, string> = {};
      for (const file of result.files) {
        const expected = entry.files.find(item => item.path === file.logicalPath);
        if (!expected || urls[file.logicalPath] || file.bytes !== expected.bytes || file.sha256 !== expected.sha256 || !file.localPath) { reject(new Error('原生模型缓存返回的文件不匹配')); return; }
        urls[file.logicalPath] = convertFileSrc(file.localPath);
      }
      resolve({ urls, dispose() { /* Native verified files remain on disk. */ } });
      } catch { reject(new Error('原生模型缓存返回了无效文件信息')); }
    }, cause => {
      if (settled) return;
      settled = true;
      signal.removeEventListener('abort', abort);
      reject(new Error(typeof cause === 'string' ? cause : '原生模型下载失败，请重试'));
    });
    if (signal.aborted) abort();
  });
}

let activeLoad: AbortController | null = null;
/** Selection is committed by the caller only after every mesh has been prepared successfully. */
export async function loadCatalogModel(id: string, options: ModelLoadOptions = {}): Promise<LoadedCatalogModel> {
  const entry = getEntry(id);
  checkAbort(options.signal);
  activeLoad?.abort();
  const controller = new AbortController();
  activeLoad = controller;
  const abort = () => controller.abort();
  options.signal?.addEventListener('abort', abort, { once: true });
  let timedOut = false;
  const timer = setTimeout(() => { timedOut = true; controller.abort(); }, 300_000);
  const report = (progress: ModelProgress) => { if (!controller.signal.aborted) options.onProgress?.(progress); };
  let assets: { urls: Record<string, string>; dispose(): void } | null = null;
  let model: PreparedUrdfModel | null = null;
  try {
    const supported = motionModelId(entry);
    if (supported) return { kind: 'bundled', entry, motionModelId: supported };
    report({ phase: 'checking', downloadedBytes: 0, totalBytes: entry.downloadBytes });
    assets = isTauri() ? await nativeAssets(entry, controller.signal, report) : await browserAssets(entry, controller.signal, report);
    checkAbort(controller.signal);
    report({ phase: 'preparing', downloadedBytes: entry.downloadBytes, totalBytes: entry.downloadBytes });
    const response = await fetch(assets.urls[entry.urdfPath], { signal: controller.signal });
    if (!response.ok) throw new Error('无法读取已下载的机器人描述文件');
    const urdfText = await response.text();
    model = await prepareUrdfModel({ id: entry.id, name: entry.name, urdfText, assetUrls: assets.urls, signal: controller.signal });
    checkAbort(controller.signal);
    report({ phase: 'ready', downloadedBytes: entry.downloadBytes, totalBytes: entry.downloadBytes });
    const prepared = model;
    const preparedAssets = assets;
    let disposed = false;
    return { kind: 'urdf', entry, model: prepared, dispose() {
      if (disposed) return;
      disposed = true;
      disposePreparedUrdfModel(prepared);
      preparedAssets.dispose();
    } };
  } catch (error) {
    if (model) disposePreparedUrdfModel(model);
    assets?.dispose();
    if (timedOut) throw new Error('模型下载超时，请检查网络后重试');
    if (controller.signal.aborted) throw abortError();
    throw error;
  } finally {
    clearTimeout(timer);
    options.signal?.removeEventListener('abort', abort);
    if (activeLoad === controller) activeLoad = null;
  }
}
