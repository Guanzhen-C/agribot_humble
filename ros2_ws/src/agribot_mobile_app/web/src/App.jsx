import { useEffect, useMemo, useState } from "react";
import {
  Activity,
  AlertTriangle,
  Check,
  BrainCircuit,
  CircleStop,
  Crosshair,
  Database,
  LocateFixed,
  Map,
  MapPin,
  Navigation,
  Play,
  Radio,
  RefreshCw,
  Route,
  Satellite,
  Send,
  Square,
  Trash2,
  Wifi,
  WifiOff,
} from "lucide-react";
import MapView from "./MapView";
import {
  formatDuration,
  getJson,
  planSemanticTask,
  postJson,
  subscribeState,
} from "./api";


const TABS = [
  { id: "navigate", label: "导航", icon: Navigation },
  { id: "collect", label: "采集", icon: Database },
  { id: "maps", label: "地图", icon: Map },
  { id: "status", label: "状态", icon: Activity },
];

const MODES = [
  { id: "browse", label: "浏览", icon: Map },
  { id: "initial", label: "初始位姿", icon: LocateFixed },
  { id: "goal", label: "目标", icon: MapPin },
  { id: "route", label: "经停点", icon: Route },
];

const NAVIGATION_KINDS = [
  { id: "manual", label: "手动规划", icon: MapPin },
  { id: "semantic", label: "语义导航", icon: BrainCircuit },
];

const SENSOR_ROWS = [
  ["/lidar/points", "C16"],
  ["/imu/data", "IMU"],
  ["/camera/rgb/image_raw", "右目相机"],
  ["/rtk/fix", "RTK"],
  ["/fastlivo_rtk/odometry", "融合定位"],
  ["/scout_status", "底盘反馈"],
];


function classNames(...values) {
  return values.filter(Boolean).join(" ");
}

function StatusDot({ ok, warning = false }) {
  return <span className={classNames("status-dot", ok ? "ok" : warning ? "warning" : "off")} />;
}

function Pill({ children, tone = "neutral" }) {
  return <span className={`pill ${tone}`}>{children}</span>;
}

function CommandButton({ icon: Icon, children, tone = "primary", ...props }) {
  return (
    <button type="button" className={`command-button ${tone}`} {...props}>
      <Icon size={18} />
      <span>{children}</span>
    </button>
  );
}

function Empty({ children }) {
  return <div className="empty-row">{children}</div>;
}

function ProcessTail({ process }) {
  if (!process?.tail?.length) return null;
  return (
    <pre className="process-tail">
      {process.tail.slice(-5).join("\n")}
    </pre>
  );
}

function NavigationPanel({
  state,
  navigationKind,
  setNavigationKind,
  interactionMode,
  setInteractionMode,
  target,
  route,
  setRoute,
  semanticInstruction,
  setSemanticInstruction,
  semanticPlanning,
  planSemantic,
  execute,
}) {
  const navigation = state?.navigation || {};
  const localization = state?.localization || {};
  const manualRequired = localization.manual_required === true;
  const running = ["sending", "accepted", "executing", "canceling"].includes(navigation.status);
  const sendGoal = () => target && execute("/api/v1/navigation/goal", { pose: target });
  const sendRoute = () => route.length >= 2 && execute("/api/v1/navigation/route", { poses: route });
  const semantic = state?.semantic || {};
  const semanticReady = semantic.status === "ready" && semantic.route?.length >= 2;
  const semanticStatus = semanticPlanning ? "planning" : (semantic.status || "idle");

  if (navigationKind === "semantic") {
    return (
      <div className="panel-content">
        <div className="segmented navigation-kind-selector">
          {NAVIGATION_KINDS.map(({ id, label, icon: Icon }) => (
            <button type="button" key={id} className={navigationKind === id ? "selected" : ""} onClick={() => setNavigationKind(id)}>
              <Icon size={16} /><span>{label}</span>
            </button>
          ))}
        </div>
        <section className="section-band">
          <div className="section-heading">
            <h2>自然语言任务</h2>
            <Pill tone={semanticReady ? "green" : semanticPlanning ? "blue" : semantic.status === "failed" ? "red" : "neutral"}>
              {semanticStatus}
            </Pill>
          </div>
          {semantic.available ? (
            <>
              <label className="field-label">
                <span>任务描述</span>
                <textarea
                  value={semanticInstruction}
                  onChange={(event) => setSemanticInstruction(event.target.value)}
                  maxLength={1000}
                  disabled={semanticPlanning || running}
                  placeholder="例如：先巡检北门，再去白色建筑附近"
                />
              </label>
              <CommandButton
                icon={BrainCircuit}
                disabled={!semanticInstruction.trim() || semanticPlanning || running}
                onClick={() => planSemantic(semanticInstruction)}
              >
                生成语义路线
              </CommandButton>
            </>
          ) : (
            <Empty>当前运行地图没有语义图谱</Empty>
          )}
          {semantic.error && <div className="status-message error-message">{semantic.error}</div>}
        </section>
        {semanticReady && (
          <section className="section-band">
            <div className="section-heading"><h2>Dijkstra 经停点</h2><Pill>{semantic.route.length}</Pill></div>
            <div className="metric-grid semantic-metrics">
              <div><span>拓扑点</span><strong>{semantic.statistics?.route_navigation_places ?? semantic.route.length}</strong></div>
              <div><span>路线长度</span><strong>{Number.isFinite(semantic.statistics?.drivable_route_length_m) ? `${semantic.statistics.drivable_route_length_m.toFixed(1)} m` : "--"}</strong></div>
            </div>
            <div className="semantic-destinations">
              {(semantic.destinations || []).map((destination, index) => (
                <div className="semantic-destination" key={`${destination.place_id}-${index}`}>
                  <span className="route-index">{index + 1}</span>
                  <span><strong>{destination.name || destination.place_id}</strong><small>{(destination.semantic_summary || []).slice(0, 2).join(" · ")}</small></span>
                </div>
              ))}
            </div>
            {semantic.avoid_node_ids?.length > 0 && (
              <div className="status-message error-message">该任务包含语义避让区，尚未配置 Keepout Filter，禁止真车执行。</div>
            )}
            <div className="button-stack semantic-actions">
              <CommandButton icon={Navigation} disabled={!semantic.execution_allowed || running} onClick={() => execute("/api/v1/semantic/execute", {})}>
                执行语义路线
              </CommandButton>
              <CommandButton icon={Trash2} tone="secondary" disabled={running} onClick={() => execute("/api/v1/semantic/clear", {})}>
                清除语义路线
              </CommandButton>
            </div>
          </section>
        )}
        <section className="section-band command-strip">
          <CommandButton icon={CircleStop} tone="danger" onClick={() => execute("/api/v1/navigation/cancel", {})} disabled={!running}>
            停止导航
          </CommandButton>
          <CommandButton icon={RefreshCw} tone="secondary" onClick={() => execute("/api/v1/navigation/clear-costmaps", {})}>
            清除代价图
          </CommandButton>
        </section>
      </div>
    );
  }

  return (
    <div className="panel-content">
      <div className="segmented navigation-kind-selector">
        {NAVIGATION_KINDS.map(({ id, label, icon: Icon }) => (
          <button type="button" key={id} className={navigationKind === id ? "selected" : ""} onClick={() => setNavigationKind(id)}>
            <Icon size={16} /><span>{label}</span>
          </button>
        ))}
      </div>
      <div className="segmented mode-selector">
        {MODES.map(({ id, label, icon: Icon }) => (
          <button
            type="button"
            key={id}
            className={interactionMode === id ? "selected" : ""}
            title={label}
            aria-label={label}
            disabled={id === "initial" && localization.initialization_stage != null && !manualRequired}
            onClick={() => setInteractionMode(id)}
          >
            <Icon size={16} />
            <span>{label}</span>
          </button>
        ))}
      </div>

      <section className="section-band">
        <div className="section-heading">
          <h2>导航任务</h2>
          <Pill tone={running ? "blue" : navigation.status === "succeeded" ? "green" : "neutral"}>
            {navigation.status || "idle"}
          </Pill>
        </div>
        {interactionMode === "goal" && (
          <>
            {target ? (
              <div className="coordinate-row">
                <span>X {target.x.toFixed(2)}</span>
                <span>Y {target.y.toFixed(2)}</span>
                <span>{(target.yaw * 180 / Math.PI).toFixed(1)}°</span>
              </div>
            ) : (
              <Empty>未选择目标</Empty>
            )}
            <CommandButton icon={Send} disabled={!target || running} onClick={sendGoal}>
              开始导航
            </CommandButton>
          </>
        )}
        {interactionMode === "initial" && (
          <Empty>{manualRequired ? "请在地图上标记当前位置和车头方向" : "RTK或视觉自动初始化正在进行"}</Empty>
        )}
        {interactionMode === "browse" && (
          <div className="metric-grid">
            <div><span>剩余距离</span><strong>{Number.isFinite(navigation.feedback?.distance_remaining) ? `${navigation.feedback.distance_remaining.toFixed(2)} m` : "--"}</strong></div>
            <div><span>剩余点位</span><strong>{navigation.feedback?.number_of_poses_remaining ?? "--"}</strong></div>
          </div>
        )}
        {interactionMode === "route" && (
          <>
            <div className="route-list">
              {route.length === 0 && <Empty>经停点为空</Empty>}
              {route.map((pose, index) => (
                <div className="route-item" key={`${pose.x}-${pose.y}-${index}`}>
                  <span className="route-index">{index + 1}</span>
                  <span>{pose.x.toFixed(2)}, {pose.y.toFixed(2)}</span>
                  <span>{(pose.yaw * 180 / Math.PI).toFixed(0)}°</span>
                  <button
                    type="button"
                    className="icon-button compact"
                    title="删除经停点"
                    aria-label="删除经停点"
                    onClick={() => setRoute((current) => current.filter((_, itemIndex) => itemIndex !== index))}
                  >
                    <Trash2 size={16} />
                  </button>
                </div>
              ))}
            </div>
            <div className="button-row">
              <CommandButton icon={Route} disabled={route.length < 2 || running} onClick={sendRoute}>
                执行路线
              </CommandButton>
              <button type="button" className="icon-button bordered" title="清空经停点" aria-label="清空经停点" onClick={() => setRoute([])} disabled={!route.length}>
                <Trash2 size={18} />
              </button>
            </div>
          </>
        )}
      </section>

      <section className="section-band command-strip">
        <CommandButton icon={CircleStop} tone="danger" onClick={() => execute("/api/v1/navigation/cancel", {})} disabled={!running}>
          停止导航
        </CommandButton>
        <CommandButton icon={RefreshCw} tone="secondary" onClick={() => execute("/api/v1/navigation/clear-costmaps", {})}>
          清除代价图
        </CommandButton>
      </section>
    </div>
  );
}

function CollectionPanel({ state, refreshCatalogs, execute }) {
  const [mapName, setMapName] = useState("map_lio_sam_");
  const collection = state?.processes?.collection;
  const activeCollection = state?.active_collection;
  const elapsed = collection?.started_at && collection.running ? Date.now() / 1000 - collection.started_at : NaN;

  const stopCollection = async () => {
    await execute("/api/v1/collection/stop", {});
    refreshCatalogs();
  };

  return (
    <div className="panel-content">
      <section className="section-band">
        <div className="section-heading">
          <h2>原始数据采集</h2>
          <Pill tone={collection?.running ? "red" : collection?.state === "completed" ? "green" : "neutral"}>
            {collection?.running ? "录制中" : collection?.state || "idle"}
          </Pill>
        </div>
        <label className="field-label">
          <span>数据集名称</span>
          <input value={mapName} onChange={(event) => setMapName(event.target.value)} disabled={collection?.running} />
        </label>
        {activeCollection && (
          <div className="recording-line">
            <Radio size={17} />
            <strong>{formatDuration(elapsed)}</strong>
            <span>{activeCollection.bag_id}</span>
          </div>
        )}
        {!collection?.running ? (
          <CommandButton icon={Play} onClick={() => execute("/api/v1/collection/start", { map_name: mapName })}>
            开始采集
          </CommandButton>
        ) : (
          <CommandButton icon={Square} tone="danger" onClick={stopCollection}>
            停止采集并保存
          </CommandButton>
        )}
        <ProcessTail process={collection} />
      </section>
    </div>
  );
}

function MapsPanel({ maps, profiles, selectedMap, setSelectedMap, state, execute, onMotionRequest }) {
  const [profileId, setProfileId] = useState("");
  useEffect(() => {
    if (!profileId && profiles.length) setProfileId(profiles[0].id);
  }, [profileId, profiles]);
  const runtime = state?.processes?.runtime;

  return (
    <div className="panel-content">
      <section className="section-band map-list-section">
        <div className="section-heading"><h2>导航地图</h2><Pill>{maps.length}</Pill></div>
        <div className="map-list">
          {!maps.length && <Empty>无可用地图</Empty>}
          {maps.map((map) => (
            <button
              type="button"
              key={map.id}
              className={classNames("map-row", selectedMap === map.id && "selected")}
              onClick={() => setSelectedMap(map.id)}
            >
              <Map size={18} />
              <span className="map-row-main"><strong>{map.id}</strong><small>{map.resolution.toFixed(3)} m/px</small></span>
              <span className="artifact-flags">
                {map.has_3d && <Pill tone="blue">3D</Pill>}
                {map.has_georeference && <Pill tone="green">RTK</Pill>}
                {map.has_visual && <Pill tone="blue">视觉</Pill>}
                {map.has_semantic && <Pill tone="green">语义</Pill>}
              </span>
              {selectedMap === map.id && <Check size={17} />}
            </button>
          ))}
        </div>
      </section>

      <section className="section-band">
        <div className="section-heading"><h2>运行配置</h2><Pill tone={runtime?.running ? "blue" : "neutral"}>{runtime?.state || "idle"}</Pill></div>
        <label className="field-label">
          <span>车辆流程</span>
          <select value={profileId} onChange={(event) => setProfileId(event.target.value)} disabled={runtime?.running}>
            {profiles.map((profile) => <option key={profile.id} value={profile.id}>{profile.label}</option>)}
          </select>
        </label>
        {!runtime?.running ? (
          <div className="button-stack">
            <CommandButton icon={Play} disabled={!selectedMap || !profileId} onClick={() => execute("/api/v1/runtime/start", { profile_id: profileId, map_id: selectedMap, motion: false })}>
              启动观察阶段
            </CommandButton>
            <CommandButton icon={Navigation} tone="warning" disabled={!selectedMap || !profileId} onClick={() => onMotionRequest({ profileId, mapId: selectedMap })}>
              启动真车阶段
            </CommandButton>
          </div>
        ) : (
          <CommandButton icon={Square} tone="danger" onClick={() => execute("/api/v1/runtime/stop", {})}>停止运行栈</CommandButton>
        )}
        {state?.active_runtime && (
          <div className="active-runtime">
            <strong>{state.active_runtime.map_id}</strong>
            <span>{state.active_runtime.profile_id}</span>
            <Pill tone={state.active_runtime.motion ? "red" : "blue"}>{state.active_runtime.motion ? "底盘启用" : "只观察"}</Pill>
          </div>
        )}
        <ProcessTail process={runtime} />
      </section>
    </div>
  );
}

function StatusPanel({ state }) {
  const localization = state?.localization || {};
  const stageLabels = {
    wait_rtk: "等待RTK",
    rtk_refining: "RTK精配准",
    wait_visual: "视觉识别",
    visual_refining: "视觉精配准",
    manual_required: "等待手动位姿",
    manual_refining: "手动精配准",
    ready: "已就绪",
  };
  const sourceLabels = { none: "尚未确定", rtk: "RTK", visual: "视觉", manual: "手动" };
  return (
    <div className="panel-content">
      <section className="section-band">
        <div className="section-heading">
          <h2>数据通道</h2>
          <Pill tone="blue">
            ROS {state?.ros?.domain_id ?? "0"} · {state?.ros?.localhost_only ? "本机" : "局域网"}
          </Pill>
        </div>
        <div className="status-table">
          {SENSOR_ROWS.map(([topic, label]) => {
            const available = state?.topics?.[topic]?.available === true;
            return (
              <div className="status-row" key={topic}>
                <StatusDot ok={available} />
                <span>{label}</span>
                <strong>{available ? "有数据" : "无数据"}</strong>
              </div>
            );
          })}
        </div>
      </section>
      <section className="section-band">
        <div className="section-heading"><h2>定位与底盘</h2></div>
        <div className="status-table">
          <div className="status-row"><StatusDot ok={localization.initialization_stage === "ready"} warning={localization.initialization_stage != null} /><span>初始化阶段</span><strong>{stageLabels[localization.initialization_stage] || "未收到"}</strong></div>
          <div className="status-row"><StatusDot ok={["rtk", "visual", "manual"].includes(localization.initialization_source)} /><span>初始化来源</span><strong>{sourceLabels[localization.initialization_source] || "--"}</strong></div>
          <div className="status-row"><StatusDot ok={localization.visual_available === true} warning={localization.visual_available === false} /><span>视觉位置数据库</span><strong>{localization.visual_available == null ? "未收到" : localization.visual_available ? "可用" : "不可用"}</strong></div>
          <div className="status-row"><StatusDot ok={localization.lidar_ready === true} /><span>NDT/GICP</span><strong>{localization.lidar_ready === true ? "就绪" : "未就绪"}</strong></div>
          <div className="status-row"><StatusDot ok={localization.fusion_ready === true} /><span>融合定位</span><strong>{localization.fusion_ready === true ? "就绪" : "未就绪"}</strong></div>
          <div className="status-row"><StatusDot ok={localization.fix_quality === 4} warning={localization.fix_quality != null} /><span>RTK位置质量</span><strong>{localization.fix_quality ?? "--"}</strong></div>
          <div className="status-row"><StatusDot ok={state?.chassis?.fault_code === 0} warning={Boolean(state?.chassis)} /><span>底盘故障码</span><strong>{state?.chassis?.fault_code ?? "--"}</strong></div>
          <div className="status-row"><StatusDot ok={Boolean(state?.chassis)} /><span>电池电压</span><strong>{Number.isFinite(state?.chassis?.battery_voltage) ? `${state.chassis.battery_voltage.toFixed(1)} V` : "--"}</strong></div>
        </div>
        <div className="status-message">{localization.initialization_status !== "未收到" ? localization.initialization_status : localization.status}</div>
      </section>
    </div>
  );
}

function MotionDialog({ request, onClose, execute }) {
  if (!request) return null;
  const start = async () => {
    const result = await execute("/api/v1/runtime/start", {
      profile_id: request.profileId,
      map_id: request.mapId,
      motion: true,
      motion_confirmed: true,
    });
    if (result) onClose();
  };
  return (
    <div className="modal-backdrop" role="presentation">
      <div className="modal" role="dialog" aria-modal="true" aria-labelledby="motion-title">
        <AlertTriangle size={26} />
        <h2 id="motion-title">启用真车运动</h2>
        <div className="confirmation-values"><strong>{request.mapId}</strong><span>{request.profileId}</span></div>
        <div className="modal-actions">
          <CommandButton icon={Navigation} tone="danger" onClick={start}>确认启动</CommandButton>
          <CommandButton icon={Square} tone="secondary" onClick={onClose}>取消</CommandButton>
        </div>
      </div>
    </div>
  );
}

export default function App() {
  const [state, setState] = useState(null);
  const [connected, setConnected] = useState(false);
  const [activeTab, setActiveTab] = useState("navigate");
  const [interactionMode, setInteractionMode] = useState("browse");
  const [navigationKind, setNavigationKind] = useState("manual");
  const [maps, setMaps] = useState([]);
  const [profiles, setProfiles] = useState([]);
  const [selectedMap, setSelectedMap] = useState("");
  const [target, setTarget] = useState(null);
  const [route, setRoute] = useState([]);
  const [semanticInstruction, setSemanticInstruction] = useState("");
  const [semanticPlanning, setSemanticPlanning] = useState(false);
  const [toast, setToast] = useState(null);
  const [motionRequest, setMotionRequest] = useState(null);

  const refreshCatalogs = async () => {
    const [mapDocument, profileDocument] = await Promise.all([
      getJson("/api/v1/maps"),
      getJson("/api/v1/profiles"),
    ]);
    setMaps(mapDocument.maps || []);
    setProfiles(profileDocument.profiles || []);
    setSelectedMap((current) => {
      const available = mapDocument.maps || [];
      return available.some((map) => map.id === current)
        ? current
        : available[0]?.id || "";
    });
  };

  useEffect(() => {
    getJson("/api/v1/state").then(setState).catch(() => setConnected(false));
    refreshCatalogs().catch(() => {});
    const unsubscribe = subscribeState(setState, setConnected);
    const timer = window.setInterval(() => refreshCatalogs().catch(() => {}), 15000);
    return () => {
      unsubscribe();
      window.clearInterval(timer);
    };
  }, []);

  useEffect(() => {
    if (!toast) return undefined;
    const timer = window.setTimeout(() => setToast(null), 3500);
    return () => window.clearTimeout(timer);
  }, [toast]);

  useEffect(() => {
    if (state?.localization?.manual_required === true) {
      setActiveTab("navigate");
      setInteractionMode("initial");
    }
  }, [state?.localization?.manual_required]);

  useEffect(() => {
    const instruction = state?.semantic?.instruction;
    if (typeof instruction === "string") {
      setSemanticInstruction(instruction);
    }
  }, [state?.semantic?.instruction]);

  const execute = async (path, body) => {
    try {
      const result = await postJson(path, body);
      const replaced = Array.isArray(result.stopped) && result.stopped.length > 0;
      setToast({ tone: "success", text: replaced ? "旧任务已退出，新任务已启动" : "命令已执行" });
      return result;
    } catch (error) {
      setToast({ tone: "error", text: error.message });
      return null;
    }
  };

  const planSemantic = async (instruction) => {
    const activeRuntime = state?.active_runtime;
    const pose = state?.pose;
    if (!activeRuntime?.map_id || !Number.isFinite(pose?.x) || !Number.isFinite(pose?.y)) {
      setToast({ tone: "error", text: "请先启动地图并等待定位就绪" });
      return null;
    }
    setSemanticPlanning(true);
    try {
      const planned = await planSemanticTask({
        map_id: activeRuntime.map_id,
        instruction,
        start_position: { x: pose.x, y: pose.y },
      });
      const accepted = await postJson("/api/v1/semantic/route", planned);
      setToast({ tone: "success", text: "172服务器已生成路线，请检查后执行" });
      return accepted;
    } catch (error) {
      setToast({ tone: "error", text: error.message });
      return null;
    } finally {
      setSemanticPlanning(false);
    }
  };

  const poseCommit = async (mode, pose) => {
    if (mode === "initial") {
      const result = await execute("/api/v1/localization/initial-pose", { pose });
      if (result) setInteractionMode("browse");
    } else {
      setTarget(pose);
    }
  };

  const tabPanel = useMemo(() => {
    if (activeTab === "navigate") {
      return (
        <NavigationPanel
          state={state}
          navigationKind={navigationKind}
          setNavigationKind={setNavigationKind}
          interactionMode={interactionMode}
          setInteractionMode={setInteractionMode}
          target={target}
          route={route}
          setRoute={setRoute}
          semanticInstruction={semanticInstruction}
          setSemanticInstruction={setSemanticInstruction}
          semanticPlanning={semanticPlanning}
          planSemantic={planSemantic}
          execute={execute}
        />
      );
    }
    if (activeTab === "collect") {
      return <CollectionPanel state={state} refreshCatalogs={refreshCatalogs} execute={execute} />;
    }
    if (activeTab === "maps") {
      return <MapsPanel maps={maps} profiles={profiles} selectedMap={selectedMap} setSelectedMap={setSelectedMap} state={state} execute={execute} onMotionRequest={setMotionRequest} />;
    }
    return <StatusPanel state={state} />;
  }, [activeTab, interactionMode, maps, navigationKind, profiles, route, selectedMap, semanticInstruction, semanticPlanning, state, target]);

  const localizationReady = state?.localization?.fusion_ready === true;
  const navActive = ["sending", "accepted", "executing", "canceling"].includes(state?.navigation?.status);

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand-block">
          <div className="brand-mark"><img src="./icons/agribot.svg" alt="" /></div>
          <div><h1>农机控制台</h1><span>{state?.active_runtime?.map_id || selectedMap || "未选择地图"}</span></div>
        </div>
        <div className="top-status">
          <Pill tone={connected ? "green" : "red"}>{connected ? <Wifi size={14} /> : <WifiOff size={14} />}{connected ? "在线" : "离线"}</Pill>
          <Pill tone={localizationReady ? "green" : "amber"}><Crosshair size={14} />{localizationReady ? "定位就绪" : "定位未就绪"}</Pill>
          <Pill tone={state?.localization?.fix_quality === 4 ? "green" : "neutral"}><Satellite size={14} />RTK {state?.localization?.fix_quality ?? "--"}</Pill>
          {navActive && (
            <button type="button" className="top-stop" title="停止导航" aria-label="停止导航" onClick={() => execute("/api/v1/navigation/cancel", {})}>
              <CircleStop size={20} />
            </button>
          )}
        </div>
      </header>

      <main className="workspace">
        <div className="map-region">
          <MapView
            state={state}
            selectedMap={selectedMap}
            interactionMode={interactionMode}
            route={route}
            target={target}
            onPose={poseCommit}
            onRoutePoint={(pose) => setRoute((current) => [...current, pose].slice(0, 100))}
          />
        </div>
        <aside className="control-panel">
          <nav className="tabbar" aria-label="功能导航">
            {TABS.map(({ id, label, icon: Icon }) => (
              <button type="button" key={id} className={activeTab === id ? "selected" : ""} onClick={() => setActiveTab(id)}>
                <Icon size={19} /><span>{label}</span>
              </button>
            ))}
          </nav>
          {tabPanel}
        </aside>
      </main>

      {toast && <div className={`toast ${toast.tone}`} role="status">{toast.tone === "success" ? <Check size={18} /> : <AlertTriangle size={18} />}{toast.text}</div>}
      <MotionDialog request={motionRequest} onClose={() => setMotionRequest(null)} execute={execute} />
    </div>
  );
}
