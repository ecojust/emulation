#!/usr/bin/env node
/** Regenerate structural RAPID facts; ABB manual prose/PDFs are never vendored. */
import { createHash } from 'node:crypto';
import { readFile, writeFile, mkdir, mkdtemp, rm } from 'node:fs/promises';
import { existsSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawnSync } from 'node:child_process';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const index = process.argv.indexOf('--cache-dir');
const cache = index >= 0 ? path.resolve(process.argv[index + 1]) : path.join(tmpdir(), 'emulation-rapid-research');
const check = process.argv.includes('--check');
const manuals = [
  { profile: '6.16', version: 'RobotWare 6.16.04', id: '3HAC050917-001', revision: 'AC', sha256: '0b027a9d7c569407edd3c74d0789e0fc97b688672b9e80b261878d775eda06b1', url: 'https://library.e.abb.com/public/c74d669abbda47debfa11262540152d6/3HAC050917%20TRM%20RAPID%20RW%206-en.pdf', count: 691 },
  { profile: '7.21', version: 'RobotWare 7.21', id: '3HAC065038-001', revision: 'W', sha256: 'e62e169817b9ab867403fdac557c69cc2f891003cba900041c87317cb283911f', url: 'https://library.e.abb.com/public/3399634bb95042bb8fe7632cbb8f2255/3HAC065038%20TRM%20RAPID%20RW%207-en.pdf', count: 665 },
];
await mkdir(cache, { recursive: true });
const output = { provenance: { generator: 'scripts/generate-rapid-catalog.mjs', extractor: 'ichbinmeraj/abb-rws-vscode@c213a07746432b359aa3b8c6ecfbf1b4e8b9c545', extractorLicense: 'Apache-2.0', factsOnly: true }, profiles: {} };
for (const manual of manuals) {
  const pdf = path.join(cache, `${manual.id}.pdf`);
  if (!existsSync(pdf)) {
    const response = await fetch(manual.url);
    if (!response.ok) throw new Error(`Cannot download ${manual.id}: ${response.status}`);
    await writeFile(pdf, new Uint8Array(await response.arrayBuffer()));
  }
  if (createHash('sha256').update(await readFile(pdf)).digest('hex') !== manual.sha256) throw new Error(`${manual.id}: PDF hash mismatch; do not silently update the language profile`);
  const textPath = path.join(cache, `${manual.id}.txt`);
  const conversion = spawnSync('pdftotext', ['-layout', pdf, textPath], { encoding: 'utf8' });
  if (conversion.status !== 0) throw new Error(`pdftotext failed: ${conversion.stderr}`);
  const text = await readFile(textPath, 'utf8');
  const headings = new Map();
  for (const match of text.matchAll(/^\s*([123])\.(\d+)\s+(\w+)\s+[-–—]/gm)) headings.set(match[3].toLowerCase(), { name: match[3], section: `${match[1]}.${match[2]}`, kind: { 1: 'instruction', 2: 'function', 3: 'datatype' }[match[1]] });
  if (headings.size !== manual.count) throw new Error(`${manual.id}: expected ${manual.count} unique reference names, got ${headings.size}`);
  const scratch = await mkdtemp(path.join(tmpdir(), 'rapid-extract-'));
  try {
    const child = spawnSync(process.execPath, [path.join(root, 'vendor/robot-languages/rapid/parse-rapid-manual.mjs'), textPath], { cwd: scratch, encoding: 'utf8' });
    if (child.status !== 0) throw new Error(child.stderr || child.stdout);
    const parsed = JSON.parse(await readFile(path.join(scratch, 'resources/rapid-language-data.json'), 'utf8'));
    // RW6 SCWrite has an Arguments section, but no formal Syntax section.
    if (parsed.scwrite) {
      parsed.scwrite.signature = 'SCWrite [\\ToNode], Variable;';
      parsed.scwrite.parameters = [{ name: '\\ToNode', type: 'datatype', optional: true }, { name: 'Variable', type: 'anytype', optional: false }];
    }
    const entries = [...headings.entries()].map(([key, heading]) => {
      const entry = parsed[key];
      if (!entry) throw new Error(`${manual.id}: missing official reference entry ${heading.name}`);
      if (entry.kind !== heading.kind) throw new Error(`${manual.id}: wrong category for ${heading.name}`);
      if (entry.kind !== 'datatype' && !entry.signature) throw new Error(`${manual.id}: missing signature for ${heading.name}`);
      // Parameter facts and synthetic call shapes only: no manual prose, examples or brief headings.
      return { name: heading.name, kind: heading.kind, section: heading.section, signature: entry.signature, ...(entry.returns ? { returns: entry.returns } : {}), parameters: entry.parameters };
    }).sort((a, b) => a.name.toLowerCase().localeCompare(b.name.toLowerCase(), 'en'));
    output.profiles[manual.profile] = { ...manual, entries };
    console.log(`${manual.version}: ${entries.length}/${headings.size} official reference names; ${entries.filter(e => e.kind === 'instruction').length} instructions, ${entries.filter(e => e.kind === 'function').length} functions, ${entries.filter(e => e.kind === 'datatype').length} data types`);
  } finally { await rm(scratch, { recursive: true, force: true }); }
}
const file = path.join(root, 'src/programming/languages/rapid-catalog.json');
const encoded = JSON.stringify(output, null, 2) + '\n';
if (check) {
  if (await readFile(file, 'utf8') !== encoded) throw new Error('Generated RAPID catalog differs from the checked-in file');
} else await writeFile(file, encoded);
