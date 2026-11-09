import { execFileSync } from "node:child_process";
import { existsSync, mkdirSync, mkdtempSync, readdirSync, rmSync, writeFileSync } from "node:fs";
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
const diagnostics = resolve("target", target, "release", "bundle", "diagnostics");
mkdirSync(diagnostics, { recursive: true });
const installers = readdirSync(directory).filter((name) => name.endsWith(".dmg"));
if (installers.length !== 1) throw new Error(`Expected one DMG, found ${installers.length}.`);
const scratch = mkdtempSync(join(tmpdir(), "emulation-dmg-check-"));
const mount = join(scratch, "mounted");
const app = join(scratch, "Emulation.app");
let attached = false;

function reportGraphics() {
  try {
    const description = execFileSync("/usr/sbin/system_profiler", ["SPDisplaysDataType", "-detailLevel", "mini"], { encoding: "utf8", stdio: ["ignore", "pipe", "pipe"], timeout: 15_000, maxBuffer: 1024 * 1024 });
    // Keep capability facts only, never monitor/computer serial numbers or user-named displays.
    const fields = /^\s*(Chipset Model|Type|Bus|VRAM(?: \(Dynamic, Max\))?|Vendor|Metal(?: Family)? Support|Display Type|Resolution|Main Display|Mirror|Online|Connection Type):/;
    const lines = description.split(/\r?\n/).filter((line) => fields.test(line)).map((line) => line.trim());
    const summary = lines.length ? lines.join("\n") : "No display capability fields were reported by system_profiler.";
    writeFileSync(join(diagnostics, "graphics.txt"), summary + "\n");
    console.log(`macOS display capabilities:\n${summary}`);
  } catch {
    const message = "Display capability collection failed; native UI smoke is still required.";
    writeFileSync(join(diagnostics, "graphics.txt"), message + "\n");
    console.log(message);
  }
}

function smoke(executable, mode, timeout) {
  const destination = join(diagnostics, mode === "--smoke-test" ? "native-smoke.log" : "ui-smoke.log");
  try {
    const output = execFileSync(executable, [mode], { encoding: "utf8", timeout, maxBuffer: 2 * 1024 * 1024 });
    writeFileSync(destination, output);
    if (output) console.log(output.trimEnd());
  } catch (error) {
    const output = `${error.stdout ?? ""}${error.stderr ?? ""}`;
    writeFileSync(destination, `Mode: ${mode}\nStatus: ${error.status ?? "unknown"}\nSignal: ${error.signal ?? "none"}\n${output}`);
    if (output) console.error(output.trimEnd());
    throw error;
  }
}

try {
  execFileSync("hdiutil", ["attach", "-readonly", "-nobrowse", "-mountpoint", mount, join(directory, installers[0])], { stdio: "inherit", timeout: 60_000 });
  attached = true;
  execFileSync("ditto", [join(mount, "Emulation.app"), app], { stdio: "inherit", timeout: 60_000 });
  execFileSync("codesign", ["--verify", "--deep", "--strict", "--verbose=2", app], { stdio: "inherit", timeout: 30_000 });
  const executable = join(app, "Contents", "MacOS", "emulation");
  if (!existsSync(executable)) throw new Error("The copied app does not contain the native executable.");
  const architecture = execFileSync("lipo", ["-archs", executable], { encoding: "utf8" }).trim();
  if (architecture !== architectures[target]) throw new Error(`Unexpected executable architecture: ${architecture}`);
  smoke(executable, "--smoke-test", 30_000);
  reportGraphics();
  smoke(executable, "--ui-smoke-test", 60_000);
  console.log(`Verified copied DMG app: ${target}, valid integrity signature, native engine and WebView IPC smoke tests passed.`);
} finally {
  if (attached) execFileSync("hdiutil", ["detach", mount], { stdio: "inherit", timeout: 30_000 });
  rmSync(scratch, { recursive: true, force: true });
}
