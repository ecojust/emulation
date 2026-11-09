import { createHash } from "node:crypto";
import { readFile, writeFile } from "node:fs/promises";
import { fileURLToPath } from "node:url";

// No mesh simplification or guessed robot dimensions: inspect every pinned STL vertex.
const root = new URL("../", import.meta.url);
const manifestPath = "vendor/work-r-labs/SOURCE.json";
const destination = new URL("src/robot/link-bounds.json", root);
const manifestBytes = await readFile(new URL(manifestPath, root));
const manifest = JSON.parse(manifestBytes.toString("utf8"));
const sha256 = (bytes) => createHash("sha256").update(bytes).digest("hex");
const grouped = new Map();

if (process.argv.slice(2).some((argument) => argument !== "--check")) {
  throw new Error("Usage: node scripts/generate-collision-bounds.mjs [--check]");
}

for (const entry of manifest.files) {
  const name = entry.localPath.split("/").at(-1)?.replace(/\.stl$/, "");
  const index = name === "base_link" ? 0 : Number(name?.match(/^link_([1-6])$/)?.[1]);
  if (!Number.isInteger(index) || !entry.localPath.startsWith(`public/models/${entry.modelId}/`)) {
    throw new Error(`Unexpected mesh mapping: ${entry.localPath}`);
  }
  const bytes = await readFile(new URL(entry.localPath, root));
  if (bytes.length < 84 || bytes.length !== entry.bytes || sha256(bytes) !== entry.sha256) {
    throw new Error(`Pinned mesh verification failed: ${entry.localPath}`);
  }
  const triangles = bytes.readUInt32LE(80);
  if (!triangles || bytes.length !== 84 + triangles * 50 || triangles !== entry.triangles) {
    throw new Error(`Invalid binary STL: ${entry.localPath}`);
  }
  const minMm = [Infinity, Infinity, Infinity];
  const maxMm = [-Infinity, -Infinity, -Infinity];
  for (let triangle = 0; triangle < triangles; triangle++) {
    for (let vertex = 0; vertex < 3; vertex++) {
      for (let axis = 0; axis < 3; axis++) {
        const value = bytes.readFloatLE(84 + triangle * 50 + 12 + vertex * 12 + axis * 4);
        if (!Number.isFinite(value)) throw new Error(`Non-finite vertex: ${entry.localPath}`);
        minMm[axis] = Math.min(minMm[axis], value);
        maxMm[axis] = Math.max(maxMm[axis], value);
      }
    }
  }
  if (JSON.stringify([minMm, maxMm]) !== JSON.stringify(entry.bounds_mm)) {
    throw new Error(`Bounds differ from pinned provenance: ${entry.localPath}`);
  }
  const parts = grouped.get(entry.modelId) ?? [];
  if (parts.some((part) => part.index === index)) throw new Error(`Duplicate link: ${entry.localPath}`);
  parts.push({ index, name, minMm, maxMm, sha256: entry.sha256, sourcePath: entry.localPath });
  grouped.set(entry.modelId, parts);
}

const models = {};
for (const modelId of [...grouped.keys()].sort()) {
  const parts = grouped.get(modelId).sort((a, b) => a.index - b.index);
  if (parts.length !== 7 || parts.some((part, index) => part.index !== index)) {
    throw new Error(`Expected seven split CAD parts for ${modelId}`);
  }
  models[modelId] = { parts };
}

const output = {
  schemaVersion: 1,
  units: "mm",
  approximation: "conservative_per_link_obb",
  coordinates: "mechanical_zero_common_base_frame",
  source: {
    repository: manifest.repository,
    commit: manifest.commit,
    license: manifest.license,
    manifestPath,
    manifestSha256: sha256(manifestBytes),
  },
  models,
};
const text = JSON.stringify(output, null, 2) + "\n";
if (process.argv.includes("--check")) {
  if (await readFile(destination, "utf8") !== text) {
    throw new Error("Collision bounds are stale; rerun node scripts/generate-collision-bounds.mjs");
  }
  console.log(`Verified ${grouped.size * 7} pinned mesh bounds.`);
} else {
  await writeFile(destination, text, "utf8");
  console.log(`Generated ${fileURLToPath(destination)} from ${grouped.size * 7} pinned meshes.`);
}
