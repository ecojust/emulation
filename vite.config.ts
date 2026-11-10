import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { readFileSync, readdirSync } from "node:fs";

function notices() {
  return {
    name: "distributed-licenses",
    generateBundle(this: { emitFile: (asset: { type: "asset"; fileName: string; source: string }) => unknown }) {
      const read = (path: string) => readFileSync(new URL(path, import.meta.url), "utf8");
      for (const [fileName, sourcePath] of [
        ["LICENSE.txt", "LICENSE"],
        ["third-party/work-r-labs/LICENSE.txt", "vendor/work-r-labs/LICENSE"],
        ["third-party/work-r-labs/SOURCE.json", "vendor/work-r-labs/SOURCE.json"],
      ]) this.emitFile({ type: "asset", fileName, source: read(sourcePath) });
      for (const name of readdirSync(new URL("vendor/work-r-labs/model-notices/", import.meta.url))) {
        this.emitFile({ type: "asset", fileName: `third-party/work-r-labs/model-notices/${name}`, source: read(`vendor/work-r-labs/model-notices/${name}`) });
      }
      const packages = ["react", "react-dom", "scheduler", "three", "three-mesh-bvh"];
      const source = packages.map((name) => `${name}\n${"=".repeat(60)}\n${read(`node_modules/${name}/LICENSE`)}`).join("\n\n")
        + `\n\n@tauri-apps/api (MIT option)\n${"=".repeat(60)}\n${read("node_modules/@tauri-apps/api/LICENSE-MIT")}`;
      this.emitFile({ type: "asset", fileName: "THIRD_PARTY_NOTICES.txt", source });
    },
  };
}

export default defineConfig({
  plugins: [react(), notices()],
  base: "./",
  server: { port: 5173, strictPort: true },
  build: { rollupOptions: { output: { manualChunks: { three: ["three"] } } } },
});
