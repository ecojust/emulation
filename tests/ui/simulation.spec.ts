import { expect, test, type Page } from "@playwright/test";

test.setTimeout(60_000);

const longArm = "irb1100-4-0.58";
const shortArm = "irb1100-4-0.475";
const readyTarget = [444.89711, 30, 620.056465] as const;
const readyQuaternion = [0.5, 0, 0.866025404, 0] as const;

function observeBrowser(page: Page) {
  const errors: string[] = [];
  const meshes = new Set<string>();
  page.on("pageerror", (error) => errors.push(error.message));
  page.on("console", (message) => {
    if (message.type() === "error") errors.push(`${message.text()} (${message.location().url})`);
  });
  page.on("response", (response) => {
    const path = new URL(response.url()).pathname;
    if (path.endsWith(".stl")) {
      if (response.ok()) meshes.add(path);
      else errors.push(`Mesh HTTP ${response.status()}: ${path}`);
    }
  });
  return { errors, meshes };
}

async function loaded(page: Page, meshes: Set<string>, model = longArm) {
  await expect(page.getByText("机器人模型已加载 · 7/7", { exact: true })).toBeVisible();
  await expect.poll(() => [...meshes].filter((name) => name.includes(`/models/${model}/`)).length).toBe(7);
  await expect(page.getByLabel("ABB IRB 1100 三维仿真场景")).toBeVisible();
}

async function setTarget(page: Page, position: readonly number[] = readyTarget) {
  for (const [index, axis] of ["X", "Y", "Z"].entries()) {
    await page.getByRole("spinbutton", { name: `目标 ${axis}`, exact: true }).fill(String(position[index]));
  }
}

async function solveReady(page: Page) {
  await page.getByRole("button", { name: "示例起始姿态", exact: true }).click();
  await setTarget(page);
  await page.getByRole("button", { name: "求解 MoveL", exact: true }).click();
  await expect(page.getByRole("heading", { name: "运动学可达", exact: true })).toBeVisible();
  await expect(page.getByRole("button", { name: "复制 MoveL", exact: true })).toBeEnabled();
}

test("the official zero pose loads all seven existing mesh parts and refuses a singular MoveL start", async ({ page }) => {
  const browser = observeBrowser(page);
  await page.goto("/");
  await loaded(page, browser.meshes);
  await expect(page.getByLabel("机器人型号")).toHaveValue(longArm);
  for (let joint = 1; joint <= 6; joint++) await expect(page.getByRole("spinbutton", { name: `起始关节 J${joint}`, exact: true })).toHaveValue("0");
  for (const [axis, coordinate] of [["X", 364], ["Y", 0], ["Z", 617]] as const) {
    await expect.poll(async () => Number(await page.getByLabel("当前 TCP", { exact: true }).locator(`[data-axis="${axis}"] strong`).textContent())).toBe(coordinate);
  }
  await expect(page.getByRole("button", { name: "复制 MoveL", exact: true })).toBeDisabled();
  await page.getByRole("button", { name: "求解 MoveL", exact: true }).click();
  await expect(page.getByRole("alert")).toContainText(/起点处于奇异区域/);
  await expect(page.getByRole("heading", { name: "运动学可达", exact: true })).toHaveCount(0);
  await expect(page.getByRole("button", { name: "复制 MoveL", exact: true })).toBeDisabled();
  await page.screenshot({ path: "/tmp/emulation-zero-pose.png", fullPage: true });
  expect(browser.errors).toEqual([]);
});

test("a real worker validates a reachable path and emits copyable RAPID before animation", async ({ page, context }) => {
  const browser = observeBrowser(page);
  await context.grantPermissions(["clipboard-read", "clipboard-write"]);
  await page.goto("/");
  await loaded(page, browser.meshes);
  await solveReady(page);
  await page.getByRole("button", { name: "复制 MoveL", exact: true }).click();
  const rapid = await page.evaluate(() => navigator.clipboard.readText());
  expect(rapid).toContain("ConfL \\On;");
  expect(rapid).toContain("MoveL");
  expect(rapid).toContain("tool0\\WObj:=wobj0");
  expect(rapid).toMatch(/444\.897/);
  expect(rapid).toContain("[0,-1,0,0]");
  expect(rapid.match(/9E9/gi)).toHaveLength(6);
  await page.screenshot({ path: "/tmp/emulation-reachable-path.png", fullPage: true });
  const progress = page.getByRole("slider", { name: "路径进度", exact: true });
  await progress.focus();
  await progress.press("Home");
  await expect(progress).toHaveValue("0");
  await expect(page.getByLabel("当前 TCP", { exact: true }).locator('[data-axis="Y"] strong')).toHaveText("0.00");
  await page.screenshot({ path: "/tmp/emulation-path-start.png", fullPage: true });
  await page.getByRole("button", { name: "播放路径", exact: true }).click();
  await expect(progress).toHaveValue("100", { timeout: 15_000 });
  for (const [index, axis] of ["X", "Y", "Z"].entries()) {
    const coordinate = Number(await page.getByLabel("当前 TCP", { exact: true }).locator(`[data-axis="${axis}"] strong`).textContent());
    expect(coordinate).toBeCloseTo(readyTarget[index], 1);
  }
  await page.screenshot({ path: "/tmp/emulation-path-end.png", fullPage: true });
  expect(browser.errors).toEqual([]);
});

test("editing invalidates the previous instruction and an unreachable or incomplete target cannot be copied", async ({ page }) => {
  const browser = observeBrowser(page);
  await page.goto("/");
  await solveReady(page);
  await setTarget(page, [3000, 0, 3000]);
  await expect(page.getByRole("button", { name: "复制 MoveL", exact: true })).toBeDisabled();
  await expect(page.getByRole("heading", { name: "运动学可达", exact: true })).toHaveCount(0);
  await page.getByRole("button", { name: "求解 MoveL", exact: true }).click();
  await expect(page.getByRole("alert")).toContainText(/不可达|无法到达|工作范围|没有满足/);
  await expect(page.getByRole("button", { name: "复制 MoveL", exact: true })).toBeDisabled();
  await page.getByRole("spinbutton", { name: "目标 X", exact: true }).fill("");
  await page.getByRole("button", { name: "求解 MoveL", exact: true }).click();
  await expect(page.getByRole("alert")).toContainText("目标位置需要填写完整的有限数值");
  await expect(page.getByRole("button", { name: "复制 MoveL", exact: true })).toBeDisabled();
  expect(browser.errors).toEqual([]);
});

test("fixed scalar-first orientation and a complete ABB robtarget use the same real planner", async ({ page }) => {
  const browser = observeBrowser(page);
  await page.goto("/");
  await page.getByRole("button", { name: "示例起始姿态", exact: true }).click();
  await page.getByRole("button", { name: "指定姿态", exact: true }).click();
  await setTarget(page);
  for (const [index, value] of readyQuaternion.entries()) {
    await page.getByRole("spinbutton", { name: `四元数 q${index + 1}`, exact: true }).fill(String(value));
  }
  await page.getByRole("button", { name: "求解 MoveL", exact: true }).click();
  await expect(page.getByRole("heading", { name: "运动学可达", exact: true })).toBeVisible();
  await page.getByLabel("姿态表示").selectOption("euler");
  await expect(page.getByRole("button", { name: "复制 MoveL", exact: true })).toBeDisabled();
  await expect(page.getByRole("spinbutton", { name: "欧拉角 Ry", exact: true })).toBeVisible();
  await page.getByRole("button", { name: "求解 MoveL", exact: true }).click();
  await expect(page.getByRole("heading", { name: "运动学可达", exact: true })).toBeVisible();
  await expect(page.getByRole("textbox", { name: "RAPID MoveL 指令", exact: true })).toHaveValue(/\[0,-1,0,0\]/);
  await page.getByRole("button", { name: "ABB robtarget", exact: true }).click();
  await expect(page.getByRole("button", { name: "复制 MoveL", exact: true })).toBeDisabled();
  await expect(page.getByText("读取位置与方向，重新选择可达 robconf。当前无外部轴。", { exact: true })).toBeVisible();
  const target = `[${JSON.stringify(readyTarget)},${JSON.stringify(readyQuaternion)},[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]`;
  await page.getByRole("textbox", { name: "ABB robtarget", exact: true }).fill(target);
  await page.getByRole("button", { name: "求解 MoveL", exact: true }).click();
  await expect(page.getByRole("heading", { name: "运动学可达", exact: true })).toBeVisible();
  await expect(page.getByRole("textbox", { name: "RAPID MoveL 指令", exact: true })).toHaveValue(/\[0,-1,0,0\]/);
  await page.getByRole("textbox", { name: "ABB robtarget", exact: true }).fill(target.replace("[0.5,0,0.866025404,0]", "[0,0,0,0]"));
  await page.getByRole("button", { name: "求解 MoveL", exact: true }).click();
  await expect(page.getByRole("alert")).toContainText(/四元数.*单位/);
  await expect(page.getByRole("button", { name: "复制 MoveL", exact: true })).toBeDisabled();
  expect(browser.errors).toEqual([]);
});

test("changing reach loads the other real model and a 1000px workspace remains usable", async ({ page }) => {
  const browser = observeBrowser(page);
  await page.setViewportSize({ width: 1000, height: 1000 });
  await page.goto("/");
  await loaded(page, browser.meshes);
  await page.getByLabel("机器人型号").selectOption(shortArm);
  await loaded(page, browser.meshes, shortArm);
  await expect(page.getByRole("button", { name: "复制 MoveL", exact: true })).toBeDisabled();
  await page.getByRole("button", { name: "示例起始姿态", exact: true }).click();
  await page.getByRole("button", { name: "机械零位", exact: true }).click();
  for (const [axis, coordinate] of [["X", 314], ["Y", 0], ["Z", 562]] as const) {
    await expect.poll(async () => Number(await page.getByLabel("当前 TCP", { exact: true }).locator(`[data-axis="${axis}"] strong`).textContent())).toBe(coordinate);
  }
  await expect(page.getByRole("button", { name: "求解 MoveL", exact: true })).toBeVisible();
  const overflow = await page.evaluate(() => document.documentElement.scrollWidth - window.innerWidth);
  expect(overflow).toBeLessThanOrEqual(1);
  await page.screenshot({ path: "/tmp/emulation-short-arm-1000.png", fullPage: true });
  const canvas = page.getByLabel("ABB IRB 1100 三维仿真场景");
  const before = await canvas.screenshot();
  const box = (await canvas.boundingBox())!;
  await page.mouse.move(box.x + box.width * 0.48, box.y + box.height * 0.55);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.64, box.y + box.height * 0.45, { steps: 10 });
  await page.mouse.up();
  expect((await canvas.screenshot()).equals(before)).toBe(false);
  expect(browser.errors).toEqual([]);
});
