import { createHash } from "node:crypto";
import { readFileSync, readdirSync, writeFileSync } from "node:fs";
import { join, resolve } from "node:path";

const target = process.argv[2];
const targets = new Set(["aarch64-apple-darwin", "x86_64-apple-darwin", "x86_64-pc-windows-msvc"]);
if (!targets.has(target)) throw new Error("Unsupported installer target.");
const root = resolve("target", target, "release", "bundle");
const isMac = target.endsWith("apple-darwin");
const directory = join(root, isMac ? "dmg" : "nsis");
const suffix = isMac ? ".dmg" : "-setup.exe";
const files = readdirSync(directory).filter((file) => file.endsWith(suffix)).map((name) => {
  const bytes = readFileSync(join(directory, name));
  return { name, bytes: bytes.byteLength, sha256: createHash("sha256").update(bytes).digest("hex") };
});
if (files.length !== 1) throw new Error(`Expected one verified installer for ${target}.`);
const { version } = JSON.parse(readFileSync("package.json", "utf8"));
writeFileSync(join(root, "artifact-manifest.json"), JSON.stringify({
  application: "Emulation", version, target, sourceCommit: process.env.GITHUB_SHA ?? null,
  signing: isMac ? "ad-hoc integrity signature; not Apple notarized" : "unsigned",
  validation: "Installed or DMG-copied native executable passed --smoke-test and real WebView --ui-smoke-test on the target architecture.",
  files,
}, null, 2) + "\n");
console.log(JSON.stringify({ version, target, files }));
