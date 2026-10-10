import { useSyncExternalStore } from "react";
import LiquidGlass from "simple-liquid-glass";

// FieldGate VCU's navigation material; never used on the live map surface.
const key = "agribot.glass-effects.v1";
const eventName = "agribot:glass-effects";
const queries = ["(prefers-reduced-transparency: reduce)", "(prefers-contrast: more)", "(forced-colors: active)"];
let sessionChoice;

function readEnabled() {
  if (sessionChoice !== undefined) return sessionChoice;
  try { return window.localStorage.getItem(key) !== "off"; } catch { return true; }
}

function subscribeEnabled(changed) {
  const storageChanged = (event) => {
    if (event.key === key || event.key === null) { sessionChoice = undefined; changed(); }
  };
  window.addEventListener(eventName, changed);
  window.addEventListener("storage", storageChanged);
  return () => {
    window.removeEventListener(eventName, changed);
    window.removeEventListener("storage", storageChanged);
  };
}

function readSupported() {
  return /Chrome\//.test(navigator.userAgent) && window.CSS?.supports("backdrop-filter", "blur(1px)")
    && !queries.some(query => window.matchMedia(query).matches);
}

function subscribePreferences(changed) {
  const media = queries.map(query => window.matchMedia(query));
  media.forEach(query => query.addEventListener("change", changed));
  return () => media.forEach(query => query.removeEventListener("change", changed));
}

export function GlassSwitch() {
  const enabled = useSyncExternalStore(subscribeEnabled, readEnabled, () => false);
  return <label className="glass-switch" title="玻璃特效">
    <input type="checkbox" role="switch" aria-label="玻璃特效" checked={enabled} onChange={(event) => {
      sessionChoice = event.target.checked;
      try { window.localStorage.setItem(key, sessionChoice ? "on" : "off"); } catch { /* Keep the current session usable without storage. */ }
      window.dispatchEvent(new Event(eventName));
    }} />
    <span className="switch-track" aria-hidden="true" /><span>玻璃特效</span>
  </label>;
}

export function GlassNavigation({ dock = false }) {
  const enabled = useSyncExternalStore(subscribeEnabled, readEnabled, () => false);
  const supported = useSyncExternalStore(subscribePreferences, readSupported, () => false);
  const active = enabled && supported;
  return <div className="glass-optics" aria-hidden="true" data-glass-active={active || undefined}>
    {active && <LiquidGlass style={{ position: "absolute", inset: 0, width: "100%", height: "100%" }}
      radius={dock ? 28 : 24} material="clear" renderer="svg" refraction="lens" quality="high"
      blur={6} saturation={115} glassColor="rgba(234,245,255,0.24)" liquid={false}>{null}</LiquidGlass>}
  </div>;
}
