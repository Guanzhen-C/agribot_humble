import { useEffect, useMemo, useState } from "react";
import {
  Activity,
  AlertTriangle,
  Camera,
  Check,
  CircleStop,
  Crosshair,
  Database,
  Gauge,
  HardDrive,
  KeyRound,
  ListRestart,
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
  Settings2,
  Square,
  Trash2,
  Wifi,
  WifiOff,
} from "lucide-react";
import MapView from "./MapView";
import {
  formatBytes,
  formatDuration,
  getJson,
  getToken,
  postJson,
  setToken,
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

const SENSOR_ROWS = [
  ["/lidar/points", "C16", 8],
  ["/imu/data", "IMU", 50],
  ["/camera/rgb/image_raw", "右目相机", 8],
  ["/rtk/fix", "RTK", 5],
  ["/fastlivo_rtk/odometry", "融合定位", 5],
  ["/scout_status", "底盘反馈", 5],
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
  interactionMode,
  setInteractionMode,
  target,
  route,
  setRoute,
  execute,
}) {
  const navigation = state?.navigation || {};
  const running = ["sending", "accepted", "executing", "canceling"].includes(navigation.status);
  const sendGoal = () => target && execute("/api/v1/navigation/goal", { pose: target });
  const sendRoute = () => route.length >= 2 && execute("/api/v1/navigation/route", { poses: route });

  return (
    <div className="panel-content">
      <div className="segmented mode-selector">
        {MODES.map(({ id, label, icon: Icon }) => (
          <button
            type="button"
            key={id}
            className={interactionMode === id ? "selected" : ""}
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
        {interactionMode === "initial" && <Empty>等待初始位姿</Empty>}
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

function CollectionPanel({ state, bags, refreshCatalogs, execute, processingEnabled }) {
  const [mapName, setMapName] = useState("map_lio_sam_");
  const [startCamera, setStartCamera] = useState(true);
  const [enableNtrip, setEnableNtrip] = useState(false);
  const [selectedBag, setSelectedBag] = useState("");
  const [outputMap, setOutputMap] = useState("map_lio_sam_");
  const [withoutRtk, setWithoutRtk] = useState(false);
  const collection = state?.processes?.collection;
  const processing = state?.processes?.processing;
  const activeCollection = state?.active_collection;
  const elapsed = collection?.started_at && collection.running ? Date.now() / 1000 - collection.started_at : NaN;

  useEffect(() => {
    if (!selectedBag && bags.length) setSelectedBag(bags[0].id);
  }, [bags, selectedBag]);

  const stopCollection = async () => {
    await execute("/api/v1/collection/stop", {});
    refreshCatalogs();
  };
  const stopProcessing = async () => {
    await execute("/api/v1/processing/stop", {});
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
        <div className="toggle-row">
          <label><input type="checkbox" checked={startCamera} onChange={(event) => setStartCamera(event.target.checked)} disabled={collection?.running} />右目相机</label>
          <label><input type="checkbox" checked={enableNtrip} onChange={(event) => setEnableNtrip(event.target.checked)} disabled={collection?.running} />NTRIP</label>
        </div>
        {activeCollection && (
          <div className="recording-line">
            <Radio size={17} />
            <strong>{formatDuration(elapsed)}</strong>
            <span>{activeCollection.bag_id}</span>
          </div>
        )}
        {!collection?.running ? (
          <CommandButton icon={Play} onClick={() => execute("/api/v1/collection/start", { map_name: mapName, start_camera: startCamera, enable_ntrip: enableNtrip })}>
            开始采集
          </CommandButton>
        ) : (
          <CommandButton icon={Square} tone="danger" onClick={stopCollection}>
            结束并写盘
          </CommandButton>
        )}
        <ProcessTail process={collection} />
      </section>

      <section className="section-band">
        <div className="section-heading">
          <h2>离线地图处理</h2>
          <Pill tone={processingEnabled ? "green" : "amber"}>{processingEnabled ? "Jetson可用" : "未启用"}</Pill>
        </div>
        <label className="field-label">
          <span>数据包</span>
          <select value={selectedBag} onChange={(event) => setSelectedBag(event.target.value)} disabled={processing?.running}>
            {!bags.length && <option value="">无数据包</option>}
            {bags.map((bag) => <option key={bag.id} value={bag.id}>{bag.id}</option>)}
          </select>
        </label>
        <label className="field-label">
          <span>输出地图</span>
          <input value={outputMap} onChange={(event) => setOutputMap(event.target.value)} disabled={processing?.running} />
        </label>
        <div className="toggle-row">
          <label><input type="checkbox" checked={withoutRtk} onChange={(event) => setWithoutRtk(event.target.checked)} disabled={processing?.running} />室内无RTK</label>
        </div>
        {!processing?.running ? (
          <CommandButton
            icon={ListRestart}
            disabled={!processingEnabled || !selectedBag}
            onClick={() => execute("/api/v1/processing/start", { bag_id: selectedBag, map_name: outputMap, without_rtk: withoutRtk })}
          >
            提交处理
          </CommandButton>
        ) : (
          <CommandButton icon={Square} tone="danger" onClick={stopProcessing}>停止处理</CommandButton>
        )}
        <ProcessTail process={processing} />
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
  const [tokenValue, setTokenValue] = useState(getToken());
  const localization = state?.localization || {};
  return (
    <div className="panel-content">
      <section className="section-band">
        <div className="section-heading"><h2>数据通道</h2><Pill tone="blue">ROS {state?.ros?.domain_id ?? "0"}</Pill></div>
        <div className="status-table">
          {SENSOR_ROWS.map(([topic, label, minimum]) => {
            const value = state?.rates?.[topic];
            const ok = value && value.age_sec < 2 && value.hz >= minimum;
            return (
              <div className="status-row" key={topic}>
                <StatusDot ok={ok} warning={Boolean(value)} />
                <span>{label}</span>
                <strong>{value ? `${value.hz.toFixed(1)} Hz` : "--"}</strong>
              </div>
            );
          })}
        </div>
      </section>
      <section className="section-band">
        <div className="section-heading"><h2>定位与底盘</h2></div>
        <div className="status-table">
          <div className="status-row"><StatusDot ok={localization.lidar_ready === true} /><span>NDT/GICP</span><strong>{localization.lidar_ready === true ? "就绪" : "未就绪"}</strong></div>
          <div className="status-row"><StatusDot ok={localization.fusion_ready === true} /><span>融合定位</span><strong>{localization.fusion_ready === true ? "就绪" : "未就绪"}</strong></div>
          <div className="status-row"><StatusDot ok={localization.fix_quality === 4} warning={localization.fix_quality != null} /><span>RTK位置质量</span><strong>{localization.fix_quality ?? "--"}</strong></div>
          <div className="status-row"><StatusDot ok={state?.chassis?.fault_code === 0} warning={Boolean(state?.chassis)} /><span>底盘故障码</span><strong>{state?.chassis?.fault_code ?? "--"}</strong></div>
          <div className="status-row"><StatusDot ok={Boolean(state?.chassis)} /><span>电池电压</span><strong>{Number.isFinite(state?.chassis?.battery_voltage) ? `${state.chassis.battery_voltage.toFixed(1)} V` : "--"}</strong></div>
        </div>
        <div className="status-message">{localization.status}</div>
      </section>
      <section className="section-band">
        <div className="section-heading"><h2>存储</h2><HardDrive size={18} /></div>
        <div className="metric-grid">
          <div><span>数据盘可用</span><strong>{formatBytes(state?.storage?.bags?.free_bytes)}</strong></div>
          <div><span>地图盘可用</span><strong>{formatBytes(state?.storage?.maps?.free_bytes)}</strong></div>
        </div>
      </section>
      <section className="section-band">
        <div className="section-heading"><h2>控制口令</h2><KeyRound size={18} /></div>
        <div className="inline-field">
          <input type="password" value={tokenValue} onChange={(event) => setTokenValue(event.target.value)} />
          <button type="button" className="icon-button bordered" title="保存控制口令" aria-label="保存控制口令" onClick={() => setToken(tokenValue)}><Check size={18} /></button>
        </div>
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
  const [maps, setMaps] = useState([]);
  const [bags, setBags] = useState([]);
  const [profiles, setProfiles] = useState([]);
  const [processingEnabled, setProcessingEnabled] = useState(false);
  const [selectedMap, setSelectedMap] = useState("");
  const [target, setTarget] = useState(null);
  const [route, setRoute] = useState([]);
  const [toast, setToast] = useState(null);
  const [motionRequest, setMotionRequest] = useState(null);

  const refreshCatalogs = async () => {
    const [mapDocument, bagDocument, profileDocument] = await Promise.all([
      getJson("/api/v1/maps"),
      getJson("/api/v1/bags"),
      getJson("/api/v1/profiles"),
    ]);
    setMaps(mapDocument.maps || []);
    setBags(bagDocument.bags || []);
    setProfiles(profileDocument.profiles || []);
    setProcessingEnabled(Boolean(profileDocument.processing_enabled));
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

  const execute = async (path, body) => {
    try {
      const result = await postJson(path, body);
      setToast({ tone: "success", text: "命令已执行" });
      return result;
    } catch (error) {
      setToast({ tone: "error", text: error.message });
      return null;
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
          interactionMode={interactionMode}
          setInteractionMode={setInteractionMode}
          target={target}
          route={route}
          setRoute={setRoute}
          execute={execute}
        />
      );
    }
    if (activeTab === "collect") {
      return <CollectionPanel state={state} bags={bags} refreshCatalogs={refreshCatalogs} execute={execute} processingEnabled={processingEnabled} />;
    }
    if (activeTab === "maps") {
      return <MapsPanel maps={maps} profiles={profiles} selectedMap={selectedMap} setSelectedMap={setSelectedMap} state={state} execute={execute} onMotionRequest={setMotionRequest} />;
    }
    return <StatusPanel state={state} />;
  }, [activeTab, bags, interactionMode, maps, processingEnabled, profiles, route, selectedMap, state, target]);

  const localizationReady = state?.localization?.fusion_ready === true;
  const navActive = ["sending", "accepted", "executing", "canceling"].includes(state?.navigation?.status);

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand-block">
          <div className="brand-mark"><Navigation size={20} /></div>
          <div><h1>农车控制台</h1><span>{state?.active_runtime?.map_id || selectedMap || "未选择地图"}</span></div>
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
