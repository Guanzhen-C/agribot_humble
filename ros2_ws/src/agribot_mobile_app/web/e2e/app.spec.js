import { expect, test } from "@playwright/test";


function mapGrid() {
  const width = 150;
  const height = 90;
  const values = new Int8Array(width * height);
  values.fill(-1);
  for (let y = 8; y < height - 8; y += 1) {
    for (let x = 10; x < width - 10; x += 1) {
      const wall = x === 10 || x === width - 11 || y === 8 || y === height - 9;
      const divider = x > 62 && x < 66 && y > 30;
      values[y * width + x] = wall || divider ? 100 : 0;
    }
  }
  const bytes = Buffer.from(values.buffer);
  return {
    layer: "map",
    revision: 3,
    width,
    height,
    resolution: 0.1,
    origin: { x: -7.5, y: -4.5, yaw: 0 },
    encoding: "int8-base64",
    data: bytes.toString("base64"),
  };
}

const mockState = {
  revision: 8,
  server_time: Date.now() / 1000,
  ros: { node: "mobile_gateway", domain_id: "0" },
  pose: { frame: "map", x: -2.4, y: 0.3, z: 0.02, yaw: 0.18, linear_speed: 0.12, angular_speed: 0.01 },
  paths: {
    global: [[-2.4, 0.3], [-1, 0.5], [1.2, 1.3], [3.5, 1.5], [5.1, 2.2]],
    local: [[-2.4, 0.3], [-1.7, 0.34], [-1.0, 0.5]],
  },
  localization: {
    ready: true,
    lidar_ready: true,
    fusion_ready: true,
    fixed_active: true,
    rtk_seed_ready: true,
    status: "accepted: NDT/GICP RMSE 0.08 m",
    rtk_initializer_status: "ready",
    fix_quality: 4,
    heading_solution: "SOL_COMPUTED,L1_INT",
  },
  chassis: { linear_velocity: 0.12, angular_velocity: 0.01, control_mode: 1, base_state: 0, fault_code: 0, battery_voltage: 25.4 },
  command: { linear: 0.2, angular: 0.02 },
  navigation: { kind: "route", status: "executing", feedback: { distance_remaining: 7.24, number_of_poses_remaining: 3 }, goal: null, route: [] },
  active_runtime: { profile_id: "ackermann_outdoor_0811", map_id: "map_lio_sam_0811", motion: true },
  active_collection: null,
  active_processing: null,
  rates: {
    "/lidar/points": { hz: 9.8, age_sec: 0.04 },
    "/imu/data": { hz: 100.1, age_sec: 0.01 },
    "/camera/rgb/image_raw": { hz: 30.0, age_sec: 0.02 },
    "/rtk/fix": { hz: 10.0, age_sec: 0.05 },
    "/fastlivo_rtk/odometry": { hz: 10.0, age_sec: 0.03 },
    "/scout_status": { hz: 10.0, age_sec: 0.03 },
  },
  grids: { map: { revision: 3, width: 150, height: 90, resolution: 0.1 } },
  processes: {
    runtime: { state: "running", running: true, tail: [] },
    collection: { state: "idle", running: false, tail: [] },
    processing: { state: "idle", running: false, tail: [] },
  },
  storage: {
    bags: { root: "/mnt/agribot_data/bags", free_bytes: 196000000000, total_bytes: 512000000000 },
    maps: { root: "/home/sunrise/agribot_maps/test_site", free_bytes: 52000000000, total_bytes: 64000000000 },
  },
};

async function mockApi(page) {
  await page.route("**/api/v1/**", async (route) => {
    const request = route.request();
    const url = new URL(request.url());
    if (request.method() === "POST") {
      await route.fulfill({ contentType: "application/json", body: JSON.stringify({ ok: true }) });
      return;
    }
    if (url.pathname === "/api/v1/events") {
      await route.fulfill({
        status: 200,
        contentType: "text/event-stream",
        headers: { "Cache-Control": "no-store" },
        body: `event: state\ndata: ${JSON.stringify(mockState)}\n\n`,
      });
      return;
    }
    if (url.pathname === "/api/v1/state") {
      await route.fulfill({ contentType: "application/json", body: JSON.stringify(mockState) });
      return;
    }
    if (url.pathname === "/api/v1/maps") {
      await route.fulfill({ contentType: "application/json", body: JSON.stringify({ maps: [{ id: "map_lio_sam_0811", resolution: 0.1, modified_at: "2026-08-15T10:00:00", has_3d: true, has_georeference: true, has_manifest: true }] }) });
      return;
    }
    if (url.pathname === "/api/v1/bags") {
      await route.fulfill({ contentType: "application/json", body: JSON.stringify({ bags: [{ id: "map_lio_sam_0811_20260815_100000", modified_at: "2026-08-15T11:00:00" }] }) });
      return;
    }
    if (url.pathname === "/api/v1/profiles") {
      await route.fulfill({ contentType: "application/json", body: JSON.stringify({ profiles: [{ id: "ackermann_indoor", label: "阿克曼室内地图" }, { id: "ackermann_outdoor_0811", label: "阿克曼室外0811" }], processing_enabled: true }) });
      return;
    }
    if (url.pathname === "/api/v1/grid" || url.pathname.endsWith("/grid")) {
      await route.fulfill({ contentType: "application/json", body: JSON.stringify(mapGrid()) });
      return;
    }
    await route.continue();
  });
}

test("desktop operations surface renders without overflow or console errors", async ({ page }) => {
  const errors = [];
  page.on("console", (message) => message.type() === "error" && errors.push(message.text()));
  page.on("pageerror", (error) => errors.push(error.message));
  await mockApi(page);
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto("/");
  await expect(page.getByRole("heading", { name: "农车控制台" })).toBeVisible();
  await expect(page.getByText("定位就绪")).toBeVisible();
  await expect(page.locator("canvas")).toBeVisible();
  await page.getByRole("button", { name: "经停点" }).click();
  const canvas = page.locator("canvas");
  const box = await canvas.boundingBox();
  await page.mouse.move(box.x + box.width * 0.45, box.y + box.height * 0.55);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.52, box.y + box.height * 0.5);
  await page.mouse.up();
  await expect(page.locator(".route-item")).toHaveCount(1);
  await page.screenshot({ path: "/tmp/agribot-mobile-desktop.png", fullPage: true });
  expect(errors).toEqual([]);
});

test("mobile layout keeps map and controls usable", async ({ page }) => {
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await mockApi(page);
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto("/");
  await expect(page.locator("canvas")).toBeVisible();
  await page.getByRole("button", { name: "状态" }).click();
  await expect(page.getByRole("heading", { name: "数据通道" })).toBeVisible();
  const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
  expect(bodyWidth).toBeLessThanOrEqual(390);
  await page.screenshot({ path: "/tmp/agribot-mobile-phone.png", fullPage: true });
  expect(errors).toEqual([]);
});
