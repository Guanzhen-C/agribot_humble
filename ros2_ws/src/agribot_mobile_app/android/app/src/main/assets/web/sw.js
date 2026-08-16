const CACHE = "agribot-mobile-v2";
const CORE = ["./manifest.webmanifest", "./icons/agribot.svg"];

async function cacheApplicationShell() {
  const cache = await caches.open(CACHE);
  const response = await fetch("./", { cache: "reload" });
  if (!response.ok) throw new Error(`无法缓存控制台: HTTP ${response.status}`);
  const markup = await response.clone().text();
  await cache.put("./", response.clone());
  await cache.put("./index.html", response);
  const linked = [...markup.matchAll(/(?:src|href)="(\.\/[^\"]+)"/g)]
    .map((match) => match[1]);
  await cache.addAll([...new Set([...CORE, ...linked])]);
}

self.addEventListener("install", (event) => {
  event.waitUntil(cacheApplicationShell());
  self.skipWaiting();
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(keys.filter((key) => key !== CACHE).map((key) => caches.delete(key))),
    ),
  );
  self.clients.claim();
});

self.addEventListener("fetch", (event) => {
  const requestUrl = new URL(event.request.url);
  if (requestUrl.pathname.startsWith("/api/")) {
    return;
  }
  event.respondWith(
    fetch(event.request)
      .then((response) => {
        const copy = response.clone();
        caches.open(CACHE).then((cache) => cache.put(event.request, copy));
        return response;
      })
      .catch(() => caches.match(event.request).then((response) => response || caches.match("./"))),
  );
});
