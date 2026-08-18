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
  ros: { node: "mobile_gateway", domain_id: "0", localhost_only: true },
  pose: { frame: "map", x: -2.4, y: 0.3, z: 0.02, yaw: 0.18, linear_speed: 0.12, angular_speed: 0.01 },
  paths: {
    history: [[-5.5, -0.8], [-4.1, -0.2], [-3.0, 0.15], [-2.4, 0.3]],
    global: [[-2.4, 0.3], [-1, 0.5], [1.2, 1.3], [3.5, 1.5], [5.1, 2.2]],
    local: [[-2.4, 0.3], [-1.7, 0.34], [-1.0, 0.5]],
  },
  footprint: null,
  vehicle: {
    footprint: [[0.754818, 0.485974], [0.754818, -0.485974], [-0.2275, -0.485974], [-0.2275, 0.485974]],
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
  semantic: {
    available: true,
    map_id: "map_lio_sam_0811",
    status: "ready",
    instruction: "去白色建筑入口",
    route: [
      { x: -2.4, y: 0.3, yaw: 0.1, place_id: "place_000" },
      { x: 0.0, y: 0.8, yaw: 0.2, place_id: "place_001" },
      { x: 3.5, y: 1.5, yaw: 0.3, place_id: "place_002" },
    ],
    destinations: [{ place_id: "place_002", name: "道路地点002", semantic_summary: ["白色建筑入口"] }],
    avoid_node_ids: [],
    execution_allowed: true,
    statistics: { route_navigation_places: 3, drivable_route_length_m: 8.4 },
    provider: "ollama_local",
    model: "qwen3.8:27b",
    graph_sha256: "e".repeat(64),
    error: "",
  },
  active_runtime: { profile_id: "ackermann_outdoor_0811", map_id: "map_lio_sam_0811", motion: true },
  active_collection: null,
  active_processing: null,
  topics: {
    "/lidar/points": { available: true },
    "/imu/data": { available: true },
    "/camera/rgb/image_raw": { available: true },
    "/rtk/fix": { available: true },
    "/fastlivo_rtk/odometry": { available: true },
    "/scout_status": { available: true },
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

async function mockApi(page, { state = mockState, onPost = () => {} } = {}) {
  await page.route("**/api/v1/**", async (route) => {
    const request = route.request();
    const url = new URL(request.url());
    if (request.method() === "POST") {
      const body = request.postDataJSON();
      onPost({ path: url.hostname === "172.18.80.26" ? `server:${url.pathname}` : url.pathname, body });
      if (url.hostname === "172.18.80.26" && url.pathname === "/api/v1/semantic/plan") {
        await route.fulfill({
          contentType: "application/json",
          headers: { "Access-Control-Allow-Origin": "*" },
          body: JSON.stringify({ request_id: "phone_test", semantic: { ...mockState.semantic, instruction: body.instruction } }),
        });
      } else {
        await route.fulfill({ contentType: "application/json", body: JSON.stringify({ ok: true }) });
      }
      return;
    }
    if (url.pathname === "/api/v1/events") {
      await route.fulfill({
        status: 200,
        contentType: "text/event-stream",
        headers: { "Cache-Control": "no-store" },
        body: `event: state\ndata: ${JSON.stringify(state)}\n\n`,
      });
      return;
    }
    if (url.pathname === "/api/v1/state") {
      await route.fulfill({ contentType: "application/json", body: JSON.stringify(state) });
      return;
    }
    if (url.pathname === "/api/v1/maps") {
      await route.fulfill({ contentType: "application/json", body: JSON.stringify({ maps: [{ id: "map_lio_sam_0811", resolution: 0.1, modified_at: "2026-08-15T10:00:00", has_3d: true, has_georeference: true, has_manifest: true, has_semantic: true }] }) });
      return;
    }
    if (url.pathname === "/api/v1/semantic/config") {
      await route.fulfill({ contentType: "application/json", body: JSON.stringify({ service_url: "http://172.18.80.26:8090", map_ids: ["map_lio_sam_0811"], provider: "ollama_local" }) });
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
  await expect(page.getByRole("heading", { name: "农机控制台" })).toBeVisible();
  await expect(page.getByText("定位就绪")).toBeVisible();
  await expect(page.locator("canvas")).toBeVisible();
  await expect(page.getByLabel("车辆当前位置")).toContainText("X -2.40");
  const canvasSignals = await page.locator("canvas").evaluate((canvas) => {
    const pixels = canvas.getContext("2d").getImageData(0, 0, canvas.width, canvas.height).data;
    const colors = {
      history: [104, 119, 130],
      global: [36, 99, 165],
      local: [217, 119, 6],
      vehicle: [23, 107, 91],
    };
    const counts = Object.fromEntries(Object.keys(colors).map((name) => [name, 0]));
    for (let index = 0; index < pixels.length; index += 4) {
      for (const [name, color] of Object.entries(colors)) {
        if (pixels[index] === color[0] && pixels[index + 1] === color[1] && pixels[index + 2] === color[2]) {
          counts[name] += 1;
        }
      }
    }
    return counts;
  });
  expect(canvasSignals.history).toBeGreaterThan(3);
  expect(canvasSignals.global).toBeGreaterThan(3);
  expect(canvasSignals.local).toBeGreaterThan(3);
  expect(canvasSignals.vehicle).toBeGreaterThan(3);
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

test("collection always records the camera and exposes an explicit save action", async ({ page }) => {
  const posts = [];
  await mockApi(page, { onPost: (request) => posts.push(request) });
  await page.goto("/");
  await page.getByRole("button", { name: "采集" }).click();
  await expect(page.getByRole("heading", { name: "原始数据采集" })).toBeVisible();
  await expect(page.getByText("离线地图处理")).toHaveCount(0);
  await expect(page.getByText("存储", { exact: true })).toHaveCount(0);
  await expect(page.getByText("控制口令")).toHaveCount(0);
  await expect(page.getByRole("checkbox", { name: "NTRIP" })).toHaveCount(0);
  await expect(page.getByRole("checkbox", { name: "右目相机" })).toHaveCount(0);
  await page.getByRole("button", { name: "开始采集" }).click();
  await expect.poll(() => posts.length).toBe(1);
  expect(posts[0]).toEqual({
    path: "/api/v1/collection/start",
    body: { map_name: "map_lio_sam_" },
  });

  const recordingState = {
    ...mockState,
    active_collection: {
      bag_id: "map_lio_sam_test_20260816_120000",
      bag_path: "/mnt/agribot_data/bags/map_lio_sam_test_20260816_120000",
      map_name: "map_lio_sam_test",
    },
    processes: {
      ...mockState.processes,
      collection: {
        state: "running",
        running: true,
        started_at: Date.now() / 1000 - 10,
        tail: [],
      },
    },
  };
  const recordingPage = await page.context().newPage();
  await mockApi(recordingPage, { state: recordingState });
  await recordingPage.goto("/");
  await recordingPage.getByRole("button", { name: "采集" }).click();
  await expect(recordingPage.getByRole("button", { name: "停止采集并保存" })).toBeVisible();
});


test("semantic navigation coexists with the unchanged manual planner", async ({ page }) => {
  const posts = [];
  const idleState = {
    ...mockState,
    navigation: { kind: null, status: "idle", feedback: {}, goal: null, route: [] },
  };
  await mockApi(page, { state: idleState, onPost: (request) => posts.push(request) });
  await page.goto("/");
  await expect(page.getByRole("button", { name: "手动规划" })).toBeVisible();
  await expect(page.getByRole("button", { name: "经停点" })).toBeVisible();
  await page.getByRole("button", { name: "语义导航" }).click();
  await expect(page.getByRole("textbox", { name: "任务描述" })).toHaveValue("去白色建筑入口");
  await expect(page.getByText("白色建筑入口", { exact: true })).toBeVisible();
  await page.screenshot({ path: "/tmp/agribot-semantic-desktop.png", fullPage: true });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.screenshot({ path: "/tmp/agribot-semantic-phone.png", fullPage: true });
  await page.getByRole("button", { name: "执行语义路线" }).click();
  await expect.poll(() => posts.length).toBe(1);
  expect(posts[0]).toEqual({ path: "/api/v1/semantic/execute", body: {} });
  await page.getByRole("button", { name: "手动规划" }).click();
  await expect(page.getByRole("button", { name: "目标", exact: true })).toBeVisible();
  await expect(page.getByRole("button", { name: "经停点" })).toBeVisible();
});

test("phone requests the 172 model service then submits the validated route to RDK", async ({ page }) => {
  const posts = [];
  const idleState = {
    ...mockState,
    navigation: { kind: null, status: "idle", feedback: {}, goal: null, route: [] },
    semantic: { ...mockState.semantic, status: "idle", instruction: "", route: [], destinations: [] },
  };
  await mockApi(page, { state: idleState, onPost: (request) => posts.push(request) });
  await page.goto("/");
  await page.getByRole("button", { name: "语义导航" }).click();
  await page.getByRole("textbox", { name: "任务描述" }).fill("先去白色建筑，再到北门");
  await page.getByRole("button", { name: "生成语义路线" }).click();
  await expect.poll(() => posts.length).toBe(2);
  expect(posts[0]).toEqual({
    path: "server:/api/v1/semantic/plan",
    body: {
      map_id: "map_lio_sam_0811",
      instruction: "先去白色建筑，再到北门",
      start_position: { x: -2.4, y: 0.3 },
    },
  });
  expect(posts[1].path).toBe("/api/v1/semantic/route");
  expect(posts[1].body.request_id).toBe("phone_test");
  expect(posts[1].body.semantic.provider).toBe("ollama_local");
  await expect(page.getByText("172服务器已生成路线，请检查后执行")).toBeVisible();
});


test("map supports two-finger zoom without committing a route point", async ({ page }) => {
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await mockApi(page);
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto("/");
  await page.getByRole("button", { name: "经停点" }).click();

  const map = page.locator(".map-shell");
  const canvas = page.locator("canvas");
  await expect(canvas).toBeVisible();
  const box = await canvas.boundingBox();
  const initialScale = Number(await map.getAttribute("data-view-scale"));
  const centerX = box.x + box.width / 2;
  const centerY = box.y + box.height / 2;
  const client = await page.context().newCDPSession(page);

  await client.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [
      { id: 0, x: centerX - 35, y: centerY, radiusX: 5, radiusY: 5, force: 1 },
      { id: 1, x: centerX + 35, y: centerY, radiusX: 5, radiusY: 5, force: 1 },
    ],
  });
  await client.send("Input.dispatchTouchEvent", {
    type: "touchMove",
    touchPoints: [
      { id: 0, x: centerX - 90, y: centerY, radiusX: 5, radiusY: 5, force: 1 },
      { id: 1, x: centerX + 90, y: centerY, radiusX: 5, radiusY: 5, force: 1 },
    ],
  });
  await client.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
  await client.detach();

  await expect.poll(async () => Number(await map.getAttribute("data-view-scale")))
    .toBeGreaterThan(initialScale * 2);
  await expect(page.locator(".route-item")).toHaveCount(0);
  expect(errors).toEqual([]);
});


test("installed web interface opens after the network is disconnected", async ({ page, context }) => {
  await page.goto("/");
  await page.evaluate(() => navigator.serviceWorker.ready);
  await page.reload();
  await expect.poll(() => page.evaluate(() => Boolean(navigator.serviceWorker.controller)))
    .toBe(true);

  await context.setOffline(true);
  await page.reload({ waitUntil: "domcontentloaded" });
  await expect(page.getByRole("heading", { name: "农机控制台" })).toBeVisible();
  await context.setOffline(false);
});
