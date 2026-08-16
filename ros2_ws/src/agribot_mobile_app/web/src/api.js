const BUNDLED_OFFLINE_UI = window.location.protocol === "file:";

async function responseJson(response) {
  const document = await response.json().catch(() => ({ error: `HTTP ${response.status}` }));
  if (!response.ok) {
    throw new Error(document.error || `HTTP ${response.status}`);
  }
  return document;
}

export async function getJson(path) {
  if (BUNDLED_OFFLINE_UI) throw new Error("当前未连接RDK");
  return responseJson(await fetch(path, { cache: "no-store" }));
}

export async function postJson(path, body = {}) {
  if (BUNDLED_OFFLINE_UI) throw new Error("当前未连接RDK，无法执行命令");
  const headers = { "Content-Type": "application/json" };
  return responseJson(
    await fetch(path, {
      method: "POST",
      headers,
      body: JSON.stringify(body),
    }),
  );
}

export function subscribeState(onState, onConnection) {
  if (BUNDLED_OFFLINE_UI) {
    onConnection(false);
    return () => {};
  }
  const events = new EventSource("/api/v1/events");
  events.addEventListener("open", () => onConnection(true));
  events.addEventListener("state", (event) => {
    onConnection(true);
    onState(JSON.parse(event.data));
  });
  events.addEventListener("error", () => onConnection(false));
  return () => events.close();
}

export function formatDuration(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) return "--";
  const minutes = Math.floor(seconds / 60);
  const remaining = Math.floor(seconds % 60);
  return `${String(minutes).padStart(2, "0")}:${String(remaining).padStart(2, "0")}`;
}
