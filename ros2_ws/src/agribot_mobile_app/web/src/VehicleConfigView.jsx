import { useEffect, useRef, useState } from "react";
import { AlertTriangle, Maximize2, RefreshCw } from "lucide-react";
import { getJson, isBundledOfflineUi } from "./api";


const NATIVE_EVENT = "agribot-vehicle-assets";
const APP_ASSET_ORIGIN = "https://appassets.androidplatform.net";
const NATIVE_POLL_MS = 750;
const UNITY_QUIT_TIMEOUT_MS = 5000;
let unityShutdown = Promise.resolve();


function formatBytes(bytes) {
  if (!Number.isFinite(Number(bytes))) return "--";
  return `${(Number(bytes) / (1024 * 1024)).toFixed(1)} MiB`;
}

function parseDocument(value) {
  if (typeof value === "string") return JSON.parse(value);
  return value && typeof value === "object" ? value : null;
}

function nativeBridge() {
  const bridge = window.AgribotAndroid || window.AgribotNative;
  return bridge
    && typeof bridge.getVehicleAssetState === "function"
    && typeof bridge.ensureVehicleAssets === "function"
    ? bridge
    : null;
}

function nativeState(bridge) {
  try {
    return parseDocument(bridge?.getVehicleAssetState());
  } catch (_error) {
    return null;
  }
}

function safeRelativePath(value, label) {
  if (typeof value !== "string") throw new Error(`Unity资源清单缺少${label}`);
  const path = value.trim();
  const parts = path.split("/");
  if (
    !path
    || path.startsWith("/")
    || path.includes("\\")
    || parts.some((part) => !part || part === "." || part === "..")
  ) {
    throw new Error(`Unity ${label}路径无效`);
  }
  return path;
}

function validateDescriptor(state) {
  const manifest = state?.manifest || state;
  if (!manifest?.available) throw new Error(manifest?.message || "三维配置资源尚未部署");
  const version = String(manifest.version || "").trim();
  if (!version) throw new Error("Unity资源清单缺少版本号");

  const unity = {
    loader: safeRelativePath(manifest?.unity?.loader, "loader"),
    data: safeRelativePath(manifest?.unity?.data, "data"),
    framework: safeRelativePath(manifest?.unity?.framework, "framework"),
    code: safeRelativePath(manifest?.unity?.code, "code"),
  };
  const listedFiles = Array.isArray(manifest.files)
    ? new Set(manifest.files.map((entry) => entry?.path).filter(Boolean))
    : null;
  if (listedFiles && Object.values(unity).some((path) => !listedFiles.has(path))) {
    throw new Error("Unity核心资源与清单文件列表不一致");
  }

  const assetBase = state?.asset_base || manifest.asset_base;
  if (typeof assetBase !== "string" || !assetBase.trim()) {
    throw new Error("Unity资源地址无效");
  }
  const base = new URL(
    assetBase.endsWith("/") ? assetBase : `${assetBase}/`,
    window.location.href,
  );
  const page = new URL(window.location.href);
  const sameOrigin = page.protocol === "file:"
    ? base.protocol === "file:"
    : base.origin === page.origin
      || (nativeBridge() && base.origin === APP_ASSET_ORIGIN);
  if (!sameOrigin) {
    throw new Error("三维资源与控制台不在同一安全域，请检查资源部署或更新App");
  }

  return {
    manifest: { ...manifest, version, unity },
    assetBase: base.href,
    source: state?.source || (isBundledOfflineUi() ? "android_cache" : "gateway"),
  };
}

function assetUrl(descriptor, relativePath) {
  const url = new URL(safeRelativePath(relativePath, "文件"), descriptor.assetBase);
  url.searchParams.set("v", descriptor.manifest.version);
  return url.href;
}

function normalizedProgress(value) {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) return 0;
  return Math.max(0, Math.min(1, numeric > 1 ? numeric / 100 : numeric));
}

function releaseCanvas(canvas) {
  if (!canvas) return;
  canvas.width = 1;
  canvas.height = 1;
}

function queueUnityQuit(instance, canvas) {
  unityShutdown = unityShutdown
    .catch(() => {})
    .then(async () => {
      try {
        if (typeof instance?.Quit === "function") {
          await Promise.race([
            Promise.resolve(instance.Quit()).catch(() => {}),
            new Promise((resolve) => window.setTimeout(resolve, UNITY_QUIT_TIMEOUT_MS)),
          ]);
        }
      } catch (_error) {
        // A broken Unity instance must not block a later clean load.
      } finally {
        releaseCanvas(canvas);
      }
    });
  return unityShutdown;
}

export default function VehicleConfigView({ active, vehicleType, onStatusChange, onReload, reloadToken }) {
  const canvasRef = useRef(null);
  const instanceRef = useRef(null);
  const [descriptor, setDescriptor] = useState(null);
  const [phase, setPhase] = useState("idle");
  const [progress, setProgress] = useState(0);
  const [message, setMessage] = useState("");

  useEffect(() => {
    const bridge = nativeBridge();
    if (!bridge || typeof bridge.setVehicleConfigActive !== "function") return undefined;
    const configurationActive = active && vehicleType === "ackermann";
    bridge.setVehicleConfigActive(configurationActive);
    return () => bridge.setVehicleConfigActive(false);
  }, [active, vehicleType]);

  useEffect(() => {
    if (!active || vehicleType !== "ackermann") return undefined;
    const controller = new AbortController();
    const packagedUi = isBundledOfflineUi();
    const bridge = nativeBridge();
    let cancelled = false;
    let pollTimer = null;

    setDescriptor(null);
    setPhase("preparing");
    setProgress(0);
    setMessage(packagedUi ? "正在检查App本地三维资源" : "正在读取三维配置资源");

    const accept = (candidate) => {
      if (cancelled) return false;
      try {
        const resolved = validateDescriptor(candidate);
        setDescriptor(resolved);
        setPhase("preparing");
        setProgress(0);
        setMessage("");
        return true;
      } catch (error) {
        setPhase("error");
        setMessage(error.message);
        return true;
      }
    };

    const consumeNativeState = (rawState) => {
      if (cancelled) return true;
      let state;
      try {
        state = parseDocument(rawState);
      } catch (_error) {
        setPhase("error");
        setMessage("App返回的三维资源状态无效");
        return true;
      }
      if (!state) return false;
      if (state.status === "ready" || state.available || state.manifest?.available) {
        return accept(state);
      }
      if (state.status === "error") {
        setPhase("error");
        setMessage(state.message || "三维配置资源准备失败");
        return true;
      }
      setPhase("downloading");
      setProgress(normalizedProgress(state.progress));
      setMessage(state.message || "正在准备App本地三维资源");
      return false;
    };

    const schedulePoll = () => {
      pollTimer = window.setTimeout(() => {
        const complete = consumeNativeState(nativeState(bridge));
        if (!complete && !cancelled) schedulePoll();
      }, NATIVE_POLL_MS);
    };

    if (bridge) {
      const handleNativeEvent = (event) => consumeNativeState(event.detail);
      window.addEventListener(NATIVE_EVENT, handleNativeEvent);
      const alreadyReady = consumeNativeState(nativeState(bridge));
      if (!alreadyReady) {
        try {
          const result = bridge.ensureVehicleAssets();
          if (!consumeNativeState(result)) schedulePoll();
        } catch (error) {
          setPhase("error");
          setMessage(String(error?.message || "无法准备App本地三维资源"));
        }
      }
      return () => {
        cancelled = true;
        controller.abort();
        if (pollTimer) window.clearTimeout(pollTimer);
        window.removeEventListener(NATIVE_EVENT, handleNativeEvent);
      };
    }

    if (packagedUi) {
        setPhase("error");
        setMessage("当前App未提供三维资源缓存接口，请更新App");
    } else {
      getJson("/api/v1/vehicle-config/manifest", { signal: controller.signal })
        .then(accept)
        .catch((error) => {
          if (!cancelled && error.name !== "AbortError") {
            setPhase("error");
            setMessage(error.message);
          }
        });

    }

    return () => {
      cancelled = true;
      controller.abort();
      if (pollTimer) window.clearTimeout(pollTimer);
    };
  }, [active, reloadToken, vehicleType]);

  useEffect(() => {
    if (!active || !descriptor || vehicleType !== "ackermann") return undefined;
    let cancelled = false;
    let script = null;
    let localInstance = null;
    let loaderFactory = null;
    const canvas = canvasRef.current;
    const manifest = descriptor.manifest;

    const removeLoader = () => {
      if (script) {
        script.onload = null;
        script.onerror = null;
        script.remove();
      }
      if (loaderFactory && window.createUnityInstance === loaderFactory) {
        try {
          delete window.createUnityInstance;
        } catch (_error) {
          window.createUnityInstance = undefined;
        }
      }
    };

    const start = async () => {
      await unityShutdown.catch(() => {});
      if (cancelled) return;
      setPhase("loading");
      setProgress(0);
      setMessage("");

      script = document.createElement("script");
      script.src = assetUrl(descriptor, manifest.unity.loader);
      script.async = true;
      script.onload = async () => {
        if (cancelled) return;
        loaderFactory = window.createUnityInstance;
        if (typeof loaderFactory !== "function") {
          setPhase("error");
          setMessage("Unity加载器未正确初始化");
          removeLoader();
          return;
        }
        try {
          const configuration = {
            arguments: [],
            dataUrl: assetUrl(descriptor, manifest.unity.data),
            frameworkUrl: assetUrl(descriptor, manifest.unity.framework),
            codeUrl: assetUrl(descriptor, manifest.unity.code),
            streamingAssetsUrl: assetUrl(descriptor, "StreamingAssets"),
            companyName: manifest.product?.company || "",
            productName: manifest.product?.name || "AgriculturalMachinery",
            productVersion: manifest.product?.version || manifest.version,
            showBanner: (text, type) => {
              if (!cancelled && type === "error") {
                setPhase("error");
                setMessage(text);
              }
            },
          };
          if (/Android|iPhone|iPad|iPod/i.test(navigator.userAgent)) {
            configuration.devicePixelRatio = 1;
          }
          const unityInstance = await loaderFactory(
            canvas,
            configuration,
            (value) => !cancelled && setProgress(normalizedProgress(value)),
          );
          localInstance = unityInstance;
          if (cancelled) {
            await queueUnityQuit(unityInstance, canvas);
            removeLoader();
            return;
          }
          instanceRef.current = unityInstance;
          setPhase("ready");
          setProgress(1);
        } catch (error) {
          if (!cancelled) {
            setPhase("error");
            setMessage(String(error?.message || error));
          }
          removeLoader();
        }
      };
      script.onerror = () => {
        if (!cancelled) {
          setPhase("error");
          setMessage("无法加载Unity运行程序，请检查资源版本和同源部署");
        }
        removeLoader();
      };
      document.head.appendChild(script);
    };
    start();

    return () => {
      cancelled = true;
      const instance = instanceRef.current || localInstance;
      instanceRef.current = null;
      if (instance) {
        queueUnityQuit(instance, canvas).finally(removeLoader);
      } else {
        releaseCanvas(canvas);
        removeLoader();
      }
    };
  }, [active, descriptor, vehicleType]);

  useEffect(() => {
    onStatusChange?.({
      phase,
      progress,
      message,
      version: descriptor?.manifest?.version || null,
      productVersion: descriptor?.manifest?.product?.version || null,
      totalBytes: descriptor?.manifest?.total_bytes || null,
      source: descriptor?.source || null,
    });
  }, [descriptor, message, onStatusChange, phase, progress]);

  if (vehicleType !== "ackermann") {
    return <div className="unity-unavailable">当前三维配置适用于阿克曼车</div>;
  }

  return (
    <div className="unity-shell" data-phase={phase} aria-busy={phase !== "ready"}>
      <canvas ref={canvasRef} className="unity-canvas" width="960" height="600" tabIndex="-1" />
      {phase !== "ready" && (
        <div className="unity-loading" role="status" aria-live="polite">
          {phase === "error" ? <AlertTriangle size={28} /> : <span className="unity-spinner" />}
          <strong>{phase === "error" ? "三维配置加载失败" : "正在加载三维配置"}</strong>
          <span>{message || `${Math.round(progress * 100)}%`}</span>
          {phase !== "error" && <progress max="1" value={progress} />}
        </div>
      )}
      <div className="unity-tools">
        <button
          type="button"
          title="全屏"
          aria-label="全屏"
          disabled={phase !== "ready"}
          onClick={() => instanceRef.current?.SetFullscreen?.(1)}
        >
          <Maximize2 size={18} />
        </button>
        <button type="button" title="重新加载" aria-label="重新加载" onClick={onReload}>
          <RefreshCw size={18} />
        </button>
      </div>
      <span className="unity-resource-size">{formatBytes(descriptor?.manifest?.total_bytes)}</span>
    </div>
  );
}
