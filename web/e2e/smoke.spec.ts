import { expect, test } from "@playwright/test";
import fs from "node:fs";
import path from "node:path";

test("simulation console can run demos, export, replay, and show acceptance", async ({ page, request }) => {
  await page.goto("/");
  await expect(page.getByRole("heading", { name: "UAV Simulation Console" })).toBeVisible();
  await expect(page.getByRole("button", { name: /^Operator$/ })).toHaveAttribute("aria-pressed", "true");
  await expect(page.getByRole("heading", { name: "预置演示" })).toBeHidden();
  await expect(page.getByRole("heading", { name: "Acceptance" })).toBeHidden();
  await expect(page.getByText("Not started")).toBeVisible();
  await expect(page.getByText("Waiting for mission start")).toBeVisible();
  await expect(page.getByText(/^FAIL$/)).toBeHidden();

  await page.getByRole("button", { name: /^Developer$/ }).click();
  await expect(page.getByRole("button", { name: /^Developer$/ })).toHaveAttribute("aria-pressed", "true");
  await expect(page.getByRole("heading", { name: "预置演示" })).toBeVisible();
  await expect(page.getByRole("heading", { name: "Acceptance" })).toBeVisible();

  await page.getByRole("button", { name: /Multi-UAV Search/i }).click();
  await expect(page.getByLabel("Algorithm")).toBeVisible();
  await page.getByLabel("Algorithm").selectOption("adaptive_component_sweep_v1");
  await page.getByRole("button", { name: /Reset Custom/i }).click();
  await expect(page.getByText(/adaptive_component_sweep_v1/).first()).toBeVisible();
  await expect(page.getByRole("heading", { name: "Mission Map" })).toBeVisible();
  const canvas = page.locator("canvas.map-canvas");
  await expect(canvas).toBeVisible();

  await page.getByRole("button", { name: /^Start$/i }).first().click();
  await page.waitForTimeout(400);
  await page.getByRole("button", { name: /^Pause$/i }).first().click();
  const before = await page.locator("dd").filter({ hasText: /^0\.0$/ }).count();
  const firstCanvas = await canvas.screenshot();
  await page.getByRole("button", { name: /Step 1/i }).click();
  await expect(page.getByRole("cell", { name: "FOLLOW_PATH" }).first()).toBeVisible();
  const steppedCanvas = await canvas.screenshot();
  expect(Buffer.compare(firstCanvas, steppedCanvas)).not.toBe(0);
  expect(before).toBeGreaterThanOrEqual(0);

  await page.getByRole("button", { name: /Export Run/i }).click();
  await expect(page.getByText(/summary\.json/)).toBeVisible({ timeout: 10000 });
  const exportResponse = await request.post("http://127.0.0.1:8000/api/sim/export");
  expect(exportResponse.ok()).toBe(true);
  const exportPayload = await exportResponse.json();
  expect(exportPayload.files).toContain("snapshots.json");
  const adaptiveSummary = JSON.parse(fs.readFileSync(path.resolve("..", exportPayload.export_dir, "summary.json"), "utf-8"));
  expect(adaptiveSummary.algorithm_version).toBe("adaptive_component_sweep_v1");

  await page.getByLabel("Algorithm").selectOption("baseline_sparse_boustrophedon");
  await page.getByRole("button", { name: /Reset Custom/i }).click();
  await expect(page.getByText(/baseline_sparse_boustrophedon/).first()).toBeVisible();
  await page.getByRole("button", { name: /Step 1/i }).click();
  const baselineExport = await request.post("http://127.0.0.1:8000/api/sim/export");
  expect(baselineExport.ok()).toBe(true);
  const baselinePayload = await baselineExport.json();
  const baselineSummary = JSON.parse(fs.readFileSync(path.resolve("..", baselinePayload.export_dir, "summary.json"), "utf-8"));
  expect(baselineSummary.algorithm_version).toBe("baseline_sparse_boustrophedon");
  expect(baselineSummary.algorithm_version).not.toBe(adaptiveSummary.algorithm_version);

  await page.locator('input[type="file"]').setInputFiles(path.resolve("..", exportPayload.export_dir, "snapshots.json"));
  await expect(page.getByText(/Replay mode/i)).toBeVisible({ timeout: 10000 });
  await page.locator('input[type="range"]').last().fill("1");
  await page.getByRole("button", { name: /Exit replay/i }).click();
  await expect(page.getByText(/Replay mode/i)).toBeHidden({ timeout: 10000 });

  const box = await canvas.boundingBox();
  expect(box).not.toBeNull();
  if (!box) return;

  await page.getByRole("button", { name: /Target Confirm/i }).click();
  await page.getByRole("button", { name: /Reset Custom/i }).click();
  await page.getByRole("button", { name: "Inject Target" }).click();
  const mapSide = Math.min(box.width, box.height);
  await canvas.click({
    position: {
      x: (box.width - mapSide) / 2 + mapSide * 0.03,
      y: (box.height - mapSide) / 2 + mapSide * 0.03,
    },
  });
  await expect(page.getByText(/TARGET_FOUND|CONFIRM_TARGET|server_target_found/).first()).toBeVisible({ timeout: 10000 });
  await page.locator(".inline-control").getByRole("spinbutton").fill("80");
  await page.getByRole("button", { name: /Step N/i }).click();
  await expect(page.getByRole("row", { name: /CONFIRM_TARGET.*completed/ }).first()).toBeVisible({ timeout: 15000 });

  await page.getByRole("button", { name: /Dynamic Obstacle/i }).click();
  await page.getByRole("button", { name: /Reset Custom/i }).click();
  await page.getByRole("button", { name: "Add Obstacle" }).click();
  const obstacleBox = await canvas.boundingBox();
  expect(obstacleBox).not.toBeNull();
  if (!obstacleBox) return;
  const obstacleSide = Math.min(obstacleBox.width, obstacleBox.height);
  const obstacleFrameX = obstacleBox.x + (obstacleBox.width - obstacleSide) / 2;
  const obstacleFrameY = obstacleBox.y + (obstacleBox.height - obstacleSide) / 2;
  await canvas.dragTo(canvas, {
    sourcePosition: {
      x: obstacleFrameX - obstacleBox.x + obstacleSide * 0.55,
      y: obstacleFrameY - obstacleBox.y + obstacleSide * 0.55,
    },
    targetPosition: {
      x: obstacleFrameX - obstacleBox.x + obstacleSide * 0.63,
      y: obstacleFrameY - obstacleBox.y + obstacleSide * 0.63,
    },
  });
  await expect(page.getByText(/MAP_UPDATE|server_map_update/).first()).toBeVisible({ timeout: 10000 });

  await page.getByRole("button", { name: /UAV Offline \/ Recover/i }).click();
  await page.getByRole("button", { name: /Reset Custom/i }).click();
  await page.getByRole("button", { name: /^Offline$/i }).first().click();
  await expect(page.getByText(/UAV_OFFLINE|OFFLINE/).first()).toBeVisible({ timeout: 10000 });
  await expect(page.getByText(/uav_offline|failed|cancelled/i).first()).toBeVisible({ timeout: 10000 });
  await page.getByRole("button", { name: /^Recover$/i }).first().click();
  await expect(page.getByText(/UAV_RECOVERED|IDLE|SEARCHING/).first()).toBeVisible({ timeout: 10000 });
});

test("mission draft can add a UAV, reset custom mission, and run it", async ({ page, request }) => {
  await request.post("http://127.0.0.1:8000/api/sim/reset", {
    data: {
      config_path: "config/default.yaml",
      scenario_path: "config/scenarios/area_search_1uav.yaml",
      algorithm_version: "adaptive_component_sweep_v1",
    },
  });
  await page.goto("/");
  await expect(page.getByRole("heading", { name: "Mission Draft" })).toBeVisible();
  const canvas = page.locator("canvas.map-canvas");
  await expect(canvas).toBeVisible();
  const box = await canvas.boundingBox();
  expect(box).not.toBeNull();
  if (!box) return;

  await page.getByRole("button", { name: /^Add UAV$/i }).first().click();
  await expect(page.getByText("Click map to place UAV")).toBeVisible();
  await expect(page.getByText("Add UAV").last()).toBeVisible();
  await canvas.click({ position: { x: box.width * 0.72, y: box.height * 0.42 } });
  await expect(page.getByText("uav_02").first()).toBeVisible();

  await page.getByRole("button", { name: /Reset Custom/i }).click();
  await expect(page.getByText(/mission_draft/i)).toBeVisible({ timeout: 10000 });
  const customState = await (await request.get("http://127.0.0.1:8000/api/sim/state?include_map=true&state_level=full")).json();
  expect(customState.algorithm_version).toBe("adaptive_component_sweep_v1");
  expect(customState.uavs.map((uav: { id: string }) => uav.id)).toContain("uav_02");
  const added = customState.uavs.find((uav: { id: string }) => uav.id === "uav_02");
  expect(added.position.x).toBeGreaterThan(0);
  expect(added.position.y).toBeGreaterThan(0);

  await page.getByRole("button", { name: /^Start$/i }).first().click();
  await expect
    .poll(
      async () => {
        const state = await (await request.get("http://127.0.0.1:8000/api/sim/state?include_map=false&state_level=lite")).json();
        return state.uavs.find((uav: { id: string }) => uav.id === "uav_02")?.total_distance_m || 0;
      },
      { timeout: 10000 },
    )
    .toBeGreaterThan(0);
  await page.getByRole("button", { name: /^Pause$/i }).first().click();
  const runningState = await (await request.get("http://127.0.0.1:8000/api/sim/state?include_map=false&state_level=lite")).json();
  const customUav = runningState.uavs.find((uav: { id: string }) => uav.id === "uav_02");
  expect(customUav.total_distance_m).toBeGreaterThan(0);
});

test("operator can request building modeling from a dragged footprint", async ({ page, request }) => {
  await request.post("http://127.0.0.1:8000/api/sim/reset", {
    data: {
      config_path: "config/default.yaml",
      scenario_path: "config/scenarios/area_search_3uav.yaml",
      algorithm_version: "adaptive_component_sweep_v1",
    },
  });
  await page.goto("/");
  await expect(page.getByRole("button", { name: /^Operator$/ })).toHaveAttribute("aria-pressed", "true");
  const canvas = page.locator("canvas.map-canvas");
  await expect(canvas).toBeVisible();
  const box = await canvas.boundingBox();
  expect(box).not.toBeNull();
  if (!box) return;

  await page.getByRole("button", { name: "Model Building" }).click();
  await expect(page.getByText("Drag rectangle for building footprint")).toBeVisible();
  await page.getByLabel("UAVs").fill("2");
  await page.getByLabel("Standoff").fill("3");
  await page.getByLabel("Laps").fill("1");

  await canvas.dragTo(canvas, {
    sourcePosition: { x: box.width * 0.6, y: box.height * 0.2 },
    targetPosition: { x: box.width * 0.74, y: box.height * 0.34 },
  });

  await expect
    .poll(
      async () => {
        const state = await (await request.get("http://127.0.0.1:8000/api/sim/state?include_map=false&state_level=lite")).json();
        return state.diagnostics.scheduler.modeling_jobs_total;
      },
      { timeout: 10000 },
    )
    .toBeGreaterThan(0);
  await expect(page.locator(".mission-status-row", { hasText: "Building modeling" })).toBeVisible({ timeout: 10000 });
  const state = await (await request.get("http://127.0.0.1:8000/api/sim/state?include_map=false&state_level=lite")).json();
  expect(state.algorithm_version).toBe("adaptive_component_sweep_v1");
  expect(state.diagnostics.scheduler.modeling_jobs_total).toBeGreaterThan(0);
  expect(state.tasks.modeling_tasks.length).toBeGreaterThan(0);
  expect(state.commands.some((command: { command: string }) => command.command === "MODEL_STRUCTURE")).toBeTruthy();
});
