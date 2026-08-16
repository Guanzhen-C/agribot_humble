import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Crosshair, Layers3, LocateFixed, Minus, Plus } from "lucide-react";
import { getJson } from "./api";


function decodeGrid(document) {
  const binary = window.atob(document.data);
  const data = new Int8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) {
    const value = binary.charCodeAt(index);
    data[index] = value > 127 ? value - 256 : value;
  }
  return { ...document, data };
}

function gridCanvas(grid, costmap = false) {
  if (!grid) return null;
  const canvas = document.createElement("canvas");
  canvas.width = grid.width;
  canvas.height = grid.height;
  const context = canvas.getContext("2d");
  const image = context.createImageData(grid.width, grid.height);
  for (let row = 0; row < grid.height; row += 1) {
    for (let column = 0; column < grid.width; column += 1) {
      const source = row * grid.width + column;
      const target = ((grid.height - 1 - row) * grid.width + column) * 4;
      const value = grid.data[source];
      if (costmap) {
        if (value <= 0) {
          image.data[target + 3] = 0;
        } else if (value >= 99) {
          image.data[target] = 190;
          image.data[target + 1] = 45;
          image.data[target + 2] = 35;
          image.data[target + 3] = 190;
        } else {
          image.data[target] = 235;
          image.data[target + 1] = Math.max(80, 190 - value);
          image.data[target + 2] = 30;
          image.data[target + 3] = Math.min(170, 35 + value);
        }
      } else {
        const shade = value < 0 ? 190 : Math.round(250 - (value / 100) * 230);
        image.data[target] = shade;
        image.data[target + 1] = shade;
        image.data[target + 2] = shade;
        image.data[target + 3] = 255;
      }
    }
  }
  context.putImageData(image, 0, 0);
  return canvas;
}

function arrow(context, x, y, yaw, color, size = 18) {
  context.save();
  context.translate(x, y);
  context.rotate(-yaw);
  context.fillStyle = color;
  context.strokeStyle = "#ffffff";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(size, 0);
  context.lineTo(-size * 0.62, size * 0.58);
  context.lineTo(-size * 0.32, 0);
  context.lineTo(-size * 0.62, -size * 0.58);
  context.closePath();
  context.fill();
  context.stroke();
  context.restore();
}

function useGrid(url, revision) {
  const [grid, setGrid] = useState(null);
  useEffect(() => {
    let active = true;
    if (!url) {
      setGrid(null);
      return () => {};
    }
    getJson(url)
      .then((document) => active && setGrid(decodeGrid(document)))
      .catch(() => active && setGrid(null));
    return () => {
      active = false;
    };
  }, [url, revision]);
  return grid;
}

export default function MapView({
  state,
  selectedMap,
  interactionMode,
  route,
  target,
  onPose,
  onRoutePoint,
}) {
  const canvasRef = useRef(null);
  const containerRef = useRef(null);
  const pointerRef = useRef(null);
  const fittedRef = useRef("");
  const [size, setSize] = useState({ width: 800, height: 600 });
  const [view, setView] = useState({ x: 0, y: 0, scale: 45 });
  const [layers, setLayers] = useState({ global: false, local: true });
  const [draft, setDraft] = useState(null);

  const liveMap = state?.grids?.map;
  const baseUrl = liveMap
    ? "/api/v1/grid?layer=map"
    : selectedMap
      ? `/api/v1/maps/${encodeURIComponent(selectedMap)}/grid`
      : null;
  const baseGrid = useGrid(baseUrl, liveMap?.revision || selectedMap || "");
  const globalGrid = useGrid(
    layers.global && state?.grids?.global_costmap ? "/api/v1/grid?layer=global_costmap" : null,
    state?.grids?.global_costmap?.revision,
  );
  const localGrid = useGrid(
    layers.local && state?.grids?.local_costmap ? "/api/v1/grid?layer=local_costmap" : null,
    state?.grids?.local_costmap?.revision,
  );
  const baseBitmap = useMemo(() => gridCanvas(baseGrid, false), [baseGrid]);
  const globalBitmap = useMemo(() => gridCanvas(globalGrid, true), [globalGrid]);
  const localBitmap = useMemo(() => gridCanvas(localGrid, true), [localGrid]);

  useEffect(() => {
    const observer = new ResizeObserver(([entry]) => {
      setSize({
        width: Math.max(1, Math.floor(entry.contentRect.width)),
        height: Math.max(1, Math.floor(entry.contentRect.height)),
      });
    });
    if (containerRef.current) observer.observe(containerRef.current);
    return () => observer.disconnect();
  }, []);

  const fitMap = useCallback(() => {
    if (!baseGrid) return;
    const yaw = baseGrid.origin.yaw || 0;
    const widthMeters = baseGrid.width * baseGrid.resolution;
    const heightMeters = baseGrid.height * baseGrid.resolution;
    const localCenterX = widthMeters / 2;
    const localCenterY = heightMeters / 2;
    const centerX = baseGrid.origin.x + Math.cos(yaw) * localCenterX - Math.sin(yaw) * localCenterY;
    const centerY = baseGrid.origin.y + Math.sin(yaw) * localCenterX + Math.cos(yaw) * localCenterY;
    setView({
      x: centerX,
      y: centerY,
      scale: Math.max(4, Math.min((size.width - 40) / widthMeters, (size.height - 40) / heightMeters)),
    });
  }, [baseGrid, size]);

  useEffect(() => {
    const key = baseUrl || "";
    if (baseGrid && fittedRef.current !== key) {
      fittedRef.current = key;
      fitMap();
    }
  }, [baseGrid, baseUrl, fitMap]);

  const worldToScreen = useCallback(
    (x, y) => ({
      x: size.width / 2 + (x - view.x) * view.scale,
      y: size.height / 2 - (y - view.y) * view.scale,
    }),
    [size, view],
  );
  const screenToWorld = useCallback(
    (x, y) => ({
      x: view.x + (x - size.width / 2) / view.scale,
      y: view.y - (y - size.height / 2) / view.scale,
    }),
    [size, view],
  );

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ratio = window.devicePixelRatio || 1;
    canvas.width = size.width * ratio;
    canvas.height = size.height * ratio;
    const context = canvas.getContext("2d");
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.fillStyle = "#e9eef0";
    context.fillRect(0, 0, size.width, size.height);

    const drawGrid = (grid, bitmap) => {
      if (!grid || !bitmap) return;
      const origin = worldToScreen(grid.origin.x, grid.origin.y);
      context.save();
      context.translate(origin.x, origin.y);
      context.rotate(-(grid.origin.yaw || 0));
      context.imageSmoothingEnabled = false;
      context.drawImage(
        bitmap,
        0,
        -grid.height * grid.resolution * view.scale,
        grid.width * grid.resolution * view.scale,
        grid.height * grid.resolution * view.scale,
      );
      context.restore();
    };
    drawGrid(baseGrid, baseBitmap);
    drawGrid(globalGrid, globalBitmap);
    drawGrid(localGrid, localBitmap);

    const drawPath = (points, color, width) => {
      if (!points || points.length < 2) return;
      context.strokeStyle = color;
      context.lineWidth = width;
      context.lineJoin = "round";
      context.lineCap = "round";
      context.beginPath();
      points.forEach(([x, y], index) => {
        const point = worldToScreen(x, y);
        if (index === 0) context.moveTo(point.x, point.y);
        else context.lineTo(point.x, point.y);
      });
      context.stroke();
    };
    drawPath(state?.paths?.global, "#2463a5", 4);
    drawPath(state?.paths?.local, "#d97706", 3);
    drawPath(route.map((pose) => [pose.x, pose.y]), "#176b5b", 2);

    route.forEach((pose, index) => {
      const point = worldToScreen(pose.x, pose.y);
      context.fillStyle = "#176b5b";
      context.beginPath();
      context.arc(point.x, point.y, 11, 0, Math.PI * 2);
      context.fill();
      context.fillStyle = "#fff";
      context.font = "600 11px system-ui";
      context.textAlign = "center";
      context.textBaseline = "middle";
      context.fillText(String(index + 1), point.x, point.y);
    });
    if (target) {
      const point = worldToScreen(target.x, target.y);
      arrow(context, point.x, point.y, target.yaw, "#b42318", 16);
    }
    if (draft) {
      const point = worldToScreen(draft.x, draft.y);
      arrow(context, point.x, point.y, draft.yaw, "#b56a00", 15);
    }
    if (state?.pose) {
      const point = worldToScreen(state.pose.x, state.pose.y);
      arrow(context, point.x, point.y, state.pose.yaw, "#176b5b", 20);
    }
  }, [
    baseBitmap,
    baseGrid,
    draft,
    globalBitmap,
    globalGrid,
    localBitmap,
    localGrid,
    route,
    size,
    state,
    target,
    view,
    worldToScreen,
  ]);

  const pointerPosition = (event) => {
    const rect = canvasRef.current.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  };

  const pointerDown = (event) => {
    event.currentTarget.setPointerCapture(event.pointerId);
    const screen = pointerPosition(event);
    pointerRef.current = {
      screen,
      world: screenToWorld(screen.x, screen.y),
      view,
    };
    if (interactionMode !== "browse") {
      setDraft({ ...pointerRef.current.world, yaw: state?.pose?.yaw || 0 });
    }
  };

  const pointerMove = (event) => {
    if (!pointerRef.current) return;
    const screen = pointerPosition(event);
    if (interactionMode === "browse") {
      const dx = screen.x - pointerRef.current.screen.x;
      const dy = screen.y - pointerRef.current.screen.y;
      setView({
        ...pointerRef.current.view,
        x: pointerRef.current.view.x - dx / pointerRef.current.view.scale,
        y: pointerRef.current.view.y + dy / pointerRef.current.view.scale,
      });
      return;
    }
    const current = screenToWorld(screen.x, screen.y);
    const start = pointerRef.current.world;
    const distance = Math.hypot(current.x - start.x, current.y - start.y);
    const yaw = distance > 0.08 ? Math.atan2(current.y - start.y, current.x - start.x) : state?.pose?.yaw || 0;
    setDraft({ ...start, yaw });
  };

  const pointerUp = () => {
    const pose = draft;
    pointerRef.current = null;
    setDraft(null);
    if (!pose) return;
    if (interactionMode === "route") onRoutePoint(pose);
    else if (interactionMode === "initial" || interactionMode === "goal") onPose(interactionMode, pose);
  };

  const wheel = (event) => {
    event.preventDefault();
    const screen = pointerPosition(event);
    const before = screenToWorld(screen.x, screen.y);
    const scale = Math.max(3, Math.min(400, view.scale * (event.deltaY < 0 ? 1.15 : 0.87)));
    const x = before.x - (screen.x - size.width / 2) / scale;
    const y = before.y + (screen.y - size.height / 2) / scale;
    setView({ x, y, scale });
  };

  const centerRobot = () => {
    if (state?.pose) setView((current) => ({ ...current, x: state.pose.x, y: state.pose.y }));
  };

  return (
    <div className="map-shell" ref={containerRef}>
      <canvas
        ref={canvasRef}
        className={`map-canvas mode-${interactionMode}`}
        onPointerDown={pointerDown}
        onPointerMove={pointerMove}
        onPointerUp={pointerUp}
        onPointerCancel={pointerUp}
        onWheel={wheel}
      />
      {!baseGrid && <div className="map-empty">地图未加载</div>}
      <div className="map-tools" role="toolbar" aria-label="地图工具">
        <button type="button" className="icon-button" title="放大" aria-label="放大" onClick={() => setView((current) => ({ ...current, scale: Math.min(400, current.scale * 1.25) }))}>
          <Plus size={19} />
        </button>
        <button type="button" className="icon-button" title="缩小" aria-label="缩小" onClick={() => setView((current) => ({ ...current, scale: Math.max(3, current.scale / 1.25) }))}>
          <Minus size={19} />
        </button>
        <button type="button" className="icon-button" title="适应地图" aria-label="适应地图" onClick={fitMap}>
          <Crosshair size={19} />
        </button>
        <button type="button" className="icon-button" title="跟随车辆" aria-label="跟随车辆" onClick={centerRobot} disabled={!state?.pose}>
          <LocateFixed size={19} />
        </button>
        <div className="tool-divider" />
        <button
          type="button"
          className={`icon-button ${layers.global ? "active" : ""}`}
          title="全局代价地图"
          aria-label="全局代价地图"
          onClick={() => setLayers((current) => ({ ...current, global: !current.global }))}
        >
          <Layers3 size={19} />
        </button>
        <button
          type="button"
          className={`icon-button local-layer ${layers.local ? "active" : ""}`}
          title="局部代价地图"
          aria-label="局部代价地图"
          onClick={() => setLayers((current) => ({ ...current, local: !current.local }))}
        >
          <Layers3 size={19} />
        </button>
      </div>
      <div className="map-scale">{(100 / view.scale).toFixed(1)} m</div>
    </div>
  );
}
