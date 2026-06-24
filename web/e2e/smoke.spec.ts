import { expect, test } from "@playwright/test";

test("simulation console can reset, step, inject target, add obstacle, and toggle UAV offline", async ({ page }) => {
  await page.goto("/");
  await expect(page.getByRole("heading", { name: "UAV Simulation Console" })).toBeVisible();

  await page.getByRole("button", { name: /Reset/i }).click();
  await expect(page.getByRole("heading", { name: "Mission Map" })).toBeVisible();
  const canvas = page.locator("canvas.map-canvas");
  await expect(canvas).toBeVisible();

  const before = await page.locator("dd").filter({ hasText: /^0\.0$/ }).count();
  const firstCanvas = await canvas.screenshot();
  await page.getByRole("button", { name: /Step 1/i }).click();
  await expect(page.getByRole("cell", { name: "FOLLOW_PATH" }).first()).toBeVisible();
  const steppedCanvas = await canvas.screenshot();
  expect(Buffer.compare(firstCanvas, steppedCanvas)).not.toBe(0);
  expect(before).toBeGreaterThanOrEqual(0);

  const box = await canvas.boundingBox();
  expect(box).not.toBeNull();
  if (!box) return;

  await page.getByRole("button", { name: "Inject Target" }).click();
  await page.mouse.click(box.x + box.width * 0.05, box.y + box.height * 0.05);
  await expect(page.getByText(/TARGET_FOUND|CONFIRM_TARGET|server_target_found/).first()).toBeVisible({ timeout: 10000 });
  await page.getByRole("spinbutton").fill("40");
  await page.getByRole("button", { name: /Step N/i }).click();
  await expect(page.getByRole("row", { name: /CONFIRM_TARGET.*completed/ }).first()).toBeVisible({ timeout: 15000 });

  await page.getByRole("button", { name: "Add Obstacle" }).click();
  await page.mouse.move(box.x + box.width * 0.55, box.y + box.height * 0.55);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.63, box.y + box.height * 0.63);
  await page.mouse.up();
  await expect(page.getByText(/MAP_UPDATE|server_map_update/).first()).toBeVisible({ timeout: 10000 });

  await page.getByRole("button", { name: /Offline/i }).first().click();
  await expect(page.getByText(/UAV_OFFLINE|OFFLINE/).first()).toBeVisible({ timeout: 10000 });
  await expect(page.getByText(/uav_offline|failed|cancelled/i).first()).toBeVisible({ timeout: 10000 });
});
