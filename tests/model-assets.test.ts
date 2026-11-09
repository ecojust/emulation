import { createHash } from "node:crypto";
import { readFileSync } from "node:fs";
import { basename } from "node:path";
import { describe, expect, it } from "vitest";

interface Asset { localPath: string; modelId: string; bytes: number; sha256: string; triangles: number; bounds_mm: number[][] }
const manifest = JSON.parse(readFileSync("vendor/work-r-labs/SOURCE.json", "utf8")) as {
  commit: string; licenseSha256: string; licenseText: string; files: Asset[];
};
const hash = (bytes: Uint8Array) => createHash("sha256").update(bytes).digest("hex");

describe("existing IRB 1100 assets and provenance", () => {
  it("retains the upstream MIT license and pinned source", () => {
    const bytes = readFileSync("vendor/work-r-labs/LICENSE");
    expect(hash(bytes)).toBe(manifest.licenseSha256);
    expect(bytes.toString("utf8")).toBe(manifest.licenseText);
    expect(manifest.commit).toBe("ab6aa489855256de07ba46c7b45887fdbc42b607");
  });

  it.each([
    ["irb1100-4-0.58", 364, 617],
    ["irb1100-4-0.475", 314, 562],
  ] as const)("%s contains all seven original, correctly scaled zero-pose parts", (modelId, flangeX, flangeZ) => {
    const files = manifest.files.filter((file) => file.modelId === modelId);
    expect(files.map((file) => basename(file.localPath)).sort()).toEqual([
      "base_link.stl", ...Array.from({ length: 6 }, (_, i) => `link_${i + 1}.stl`),
    ]);
    for (const file of files) {
      const bytes = readFileSync(file.localPath);
      expect(bytes.byteLength).toBe(file.bytes);
      expect(hash(bytes)).toBe(file.sha256);
      expect(bytes.readUInt32LE(80)).toBe(file.triangles);
      expect(bytes.byteLength).toBe(84 + 50 * file.triangles);
    }
    const base = files.find((file) => basename(file.localPath) === "base_link.stl")!;
    expect(base.bounds_mm[0][2]).toBe(0);
    const flange = files.find((file) => basename(file.localPath) === "link_6.stl")!;
    expect(flange.bounds_mm[1][0]).toBeCloseTo(flangeX, 3);
    expect(Math.abs((flange.bounds_mm[0][2] + flange.bounds_mm[1][2]) / 2 - flangeZ)).toBeLessThan(0.1);
  });
});
