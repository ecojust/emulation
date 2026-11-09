import { execFileSync } from "node:child_process";
import { existsSync, mkdtempSync, readdirSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, resolve } from "node:path";

const target = process.argv[2];
const architectures = {
  "aarch64-apple-darwin": "arm64",
  "x86_64-apple-darwin": "x86_64",
};
if (process.platform !== "darwin" || !architectures[target]) {
  throw new Error("Run this check on the matching macOS runner with a supported target.");
}

const directory = resolve("target", target, "release", "bundle", "dmg");
const installers = readdirSync(directory).filter((name) => name.endsWith(".dmg"));
if (installers.length !== 1) throw new Error(`Expected one DMG, found ${installers.length}.`);
const scratch = mkdtempSync(join(tmpdir(), "emulation-dmg-check-"));
const mount = join(scratch, "mounted");
const app = join(scratch, "Emulation.app");
let attached = false;
try {
  execFileSync("hdiutil", ["attach", "-readonly", "-nobrowse", "-mountpoint", mount, join(directory, installers[0])], { stdio: "inherit", timeout: 60_000 });
  attached = true;
  execFileSync("ditto", [join(mount, "Emulation.app"), app], { stdio: "inherit", timeout: 60_000 });
  execFileSync("codesign", ["--verify", "--deep", "--strict", "--verbose=2", app], { stdio: "inherit", timeout: 30_000 });
  const executable = join(app, "Contents", "MacOS", "emulation");
  if (!existsSync(executable)) throw new Error("The copied app does not contain the native executable.");
  const architecture = execFileSync("lipo", ["-archs", executable], { encoding: "utf8" }).trim();
  if (architecture !== architectures[target]) throw new Error(`Unexpected executable architecture: ${architecture}`);
  execFileSync(executable, ["--smoke-test"], { stdio: "inherit", timeout: 30_000 });
  execFileSync(executable, ["--ui-smoke-test"], { stdio: "inherit", timeout: 60_000 });
  console.log(`Verified copied DMG app: ${target}, valid integrity signature, native engine and WebView IPC smoke tests passed.`);
} finally {
  if (attached) execFileSync("hdiutil", ["detach", mount], { stdio: "inherit", timeout: 30_000 });
  rmSync(scratch, { recursive: true, force: true });
}
