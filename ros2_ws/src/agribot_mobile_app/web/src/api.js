const TOKEN_KEY = "agribot-mobile-token";


export function getToken() {
  return window.localStorage.getItem(TOKEN_KEY) || "";
}

export function setToken(token) {
  window.localStorage.setItem(TOKEN_KEY, token.trim());
}

async function responseJson(response) {
  const document = await response.json().catch(() => ({ error: `HTTP ${response.status}` }));
  if (!response.ok) {
    throw new Error(document.error || `HTTP ${response.status}`);
  }
  return document;
}

export async function getJson(path) {
  return responseJson(await fetch(path, { cache: "no-store" }));
}

export async function postJson(path, body = {}) {
  const headers = { "Content-Type": "application/json" };
  const token = getToken();
  if (token) {
    headers["X-Agribot-Token"] = token;
  }
  return responseJson(
    await fetch(path, {
      method: "POST",
      headers,
      body: JSON.stringify(body),
    }),
  );
}

export function subscribeState(onState, onConnection) {
  const events = new EventSource("/api/v1/events");
  events.addEventListener("open", () => onConnection(true));
  events.addEventListener("state", (event) => {
    onConnection(true);
    onState(JSON.parse(event.data));
  });
  events.addEventListener("error", () => onConnection(false));
  return () => events.close();
}

export function formatBytes(value) {
  if (!Number.isFinite(value)) return "--";
  const units = ["B", "KiB", "MiB", "GiB", "TiB"];
  let amount = value;
  let unit = 0;
  while (amount >= 1024 && unit < units.length - 1) {
    amount /= 1024;
    unit += 1;
  }
  return `${amount.toFixed(unit < 2 ? 0 : 1)} ${units[unit]}`;
}

export function formatDuration(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) return "--";
  const minutes = Math.floor(seconds / 60);
  const remaining = Math.floor(seconds % 60);
  return `${String(minutes).padStart(2, "0")}:${String(remaining).padStart(2, "0")}`;
}
