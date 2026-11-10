import { defineConfig } from "@playwright/test";

const preview = process.env.EMULATION_UI_PREVIEW === "1";
const baseURL = process.env.EMULATION_BASE_URL || (preview ? "http://127.0.0.1:4173" : "http://127.0.0.1:5173");

export default defineConfig({
  testDir: "./tests/ui",
  fullyParallel: false,
  // SwiftShader renders on shared CPU resources. Run scenes serially so a
  // second browser cannot starve the real animation frames sampled by tests.
  workers: 1,
  use: {
    baseURL,
    viewport: { width: 1440, height: 1000 },
    launchOptions: {
      executablePath: process.env.EMULATION_CHROMIUM || undefined,
      args: ["--enable-webgl", "--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
    },
    trace: "retain-on-failure",
  },
  webServer: {
    command: preview ? "npm run preview -- --host 127.0.0.1 --port 4173 --strictPort" : "npm run dev -- --host 127.0.0.1",
    url: baseURL,
    reuseExistingServer: !process.env.CI,
  },
});
