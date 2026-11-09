import { createHash } from 'node:crypto';
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { basename, dirname, join } from 'node:path';
import type { RobotCatalog, RobotCatalogFile } from '../../src/catalog/types';

export async function readCatalog(): Promise<RobotCatalog> {
  return JSON.parse(await readFile(new URL('../../src/catalog/catalog.json', import.meta.url), 'utf8')) as RobotCatalog;
}

function verified(bytes: Buffer, file: RobotCatalogFile) {
  return bytes.length === file.bytes && createHash('sha256').update(bytes).digest('hex') === file.sha256;
}

/** Real pinned upstream bytes; only transport timing/failure is controlled in UI tests. */
export async function prepareCatalogFixtures(catalog: RobotCatalog): Promise<Map<string, Buffer>> {
  const result = new Map<string, Buffer>();
  const cache = join(tmpdir(), 'emulation-catalog-e2e');
  for (const model of catalog.models.filter(entry => !entry.bundledMotionModelId)) {
    for (const file of model.files) {
      if (result.has(file.url)) continue;
      const cached = join(cache, `${file.sha256}.asset`);
      const sourceRelative = decodeURIComponent(new URL(file.url).pathname).split('/').filter(Boolean).slice(3);
      const candidates = [cached, join(tmpdir(), 'emulation-catalog-research', ...sourceRelative), join(tmpdir(), 'emulation-catalog-research', basename(new URL(file.url).pathname))];
      if (process.env.EMULATION_CATALOG_FIXTURES) candidates.unshift(join(process.env.EMULATION_CATALOG_FIXTURES, model.id, file.path));
      let bytes: Buffer | undefined;
      for (const path of candidates) {
        const existing = await readFile(path).catch(() => undefined);
        if (existing && verified(existing, file)) { bytes = existing; break; }
      }
      if (!bytes) {
        const response = await fetch(file.url, { signal: AbortSignal.timeout(45_000) });
        if (!response.ok) throw new Error(`Pinned catalog fixture HTTP ${response.status}: ${file.url}`);
        bytes = Buffer.from(await response.arrayBuffer());
        if (!verified(bytes, file)) throw new Error(`Pinned catalog fixture size/SHA-256 mismatch: ${file.url}`);
        await mkdir(dirname(cached), { recursive: true });
        await writeFile(cached, bytes);
      }
      result.set(file.url, bytes);
    }
  }
  return result;
}
