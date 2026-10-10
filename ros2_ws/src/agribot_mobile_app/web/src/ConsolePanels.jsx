import { Activity, Battery, Cable, Camera, CircleCheck, Cpu, Gauge, Navigation, Network, Radio, Satellite, Settings, ShieldCheck, Wrench } from "lucide-react";

const value = (number, unit = "", digits = 1) => Number.isFinite(number) ? `${number.toFixed(digits)}${unit}` : "未提供";
const hex = number => Number.isInteger(number) ? `0x${number.toString(16).toUpperCase()}` : "未提供";
const CHANNELS = [
  ["/lidar/points", "C16 激光雷达", Radio], ["/imu/data", "惯性测量单元", Activity],
  ["/camera/rgb/image_raw", "单目相机", Camera], ["/rtk/fix", "RTK 卫星定位", Satellite],
];

function Rows({ rows }) {
  return <dl className="console-rows">{rows.map(([label, text]) => <div key={label}><dt>{label}</dt><dd>{text ?? "未提供"}</dd></div>)}</dl>;
}

function Band({ icon: Icon, title, className = "", children, status }) {
  return <section className={`section-band ${className}`}>
    <div className="section-heading"><h2><Icon size={18} strokeWidth={1.65} />{title}</h2>{status}</div>
    <div className="band-body">{children}</div>
  </section>;
}

function LinkSequence({ state, connected }) {
  const vcu = state?.vcu;
  const links = [
    ["手机 / 浏览器 ↔ RDK", "控制台链路", connected, connected ? "已连接" : "离线", Network],
    ["RDK ↔ HPM6750", "信号转发", connected && vcu?.online, connected && vcu?.online ? "身份已验证" : "未验证", Cpu],
    ["HPM6750 ↔ 执行控制器", "底盘反馈", connected && vcu?.online && vcu?.telemetry?.feedback_fresh,
      connected && vcu?.online && vcu?.telemetry?.feedback_fresh ? "反馈有效" : "未验证", Cable],
  ];
  return <div className="link-sequence">{links.map(([name, detail, ok, status, Icon]) => <div key={name}>
    <span className={`link-node ${ok ? "active" : ""}`}><Icon size={20} /></span>
    <span className="link-detail"><strong>{name}</strong><small>{detail}</small></span>
    <span className={`inline-status ${ok ? "good" : "muted"}`}><i />{status}</span>
  </div>)}</div>;
}

function Overview({ state, connected, vehicleType }) {
  const fresh = connected && state?.topics?.["/scout_status"]?.available === true;
  const chassis = fresh ? state?.chassis : null;
  const active = connected ? state?.active_runtime : null;
  const localization = connected ? state?.localization : null;
  const telemetry = connected && state?.vcu?.online ? state.vcu.telemetry : null;
  return <>
    <section className="instrument-grid" aria-label="关键运行指标">
      <div><span><Battery size={18} />电池电压</span><strong>{chassis ? value(chassis.battery_voltage) : "--"}<small>V</small></strong><span>{fresh ? "底盘实时反馈" : "等待有效反馈"}</span></div>
      <div><span><Gauge size={18} />实际速度</span><strong>{chassis ? value(chassis.linear_velocity, "", 2) : "--"}<small>m/s</small></strong><span>{fresh ? "底盘实时反馈" : "等待有效反馈"}</span></div>
      <div><span>运行任务</span><strong className="metric-word">{!connected ? "未确认" : active ? active.motion ? "真车运行" : "观察阶段" : "未启动"}</strong><span className="inline-status"><i />{connected ? state?.navigation?.status || "待机" : "离线"}</span></div>
      <div><span>定位状态</span><strong className="metric-word">{localization?.fusion_ready ? "已就绪" : "未就绪"}</strong><span className="inline-status"><i />RTK {localization?.fix_quality ?? "未提供"}</span></div>
    </section>
    <div className="overview-grid">
      <Band icon={Gauge} title="农机状态" status={<span className={`inline-status ${fresh ? "good" : "muted"}`}><i />{fresh ? "反馈有效" : "等待反馈"}</span>}>
        <div className="machine-visual">
          <div className="drive-reading"><span>左驱动</span><strong>{value(telemetry?.left_rpm, "", 0)}<small>rpm</small></strong></div>
          <img src={`./fieldgate/${vehicleType === "ackermann" ? "ackermann-poster" : "tracked-diagram"}.png`} alt={vehicleType === "ackermann" ? "阿克曼车辆示意" : "履带差速车辆示意"} />
          <div className="drive-reading right"><span>右驱动</span><strong>{value(telemetry?.right_rpm, "", 0)}<small>rpm</small></strong></div>
        </div>
        <div className="machine-facts">
          <div><span>实际角速度</span><strong>{value(chassis?.angular_velocity, " rad/s", 2)}</strong></div>
          <div><span>底盘故障码</span><strong>{chassis?.fault_code ?? "未提供"}</strong></div>
          <div><span>控制来源</span><strong>{active?.motion ? "导航算法" : "未启用导航输出"}</strong></div>
        </div>
      </Band>
      <Band icon={Radio} title="感知与定位">
        <div className="function-list">{CHANNELS.map(([topic, name, Icon]) => {
          const ok = connected && state?.topics?.[topic]?.available;
          return <div key={topic}><Icon size={24} strokeWidth={1.65} /><strong>{name}</strong><span className={`inline-status ${ok ? "good" : "muted"}`}><i />{ok ? "有数据" : "无数据"}</span></div>;
        })}</div>
        <div className={`operation-note ${localization?.fusion_ready ? "ready" : ""}`}><CircleCheck size={22} />{localization?.fusion_ready ? "融合定位已就绪" : "等待定位初始化"}</div>
      </Band>
      <Band icon={Navigation} title="导航任务"><Rows rows={[
        ["地图", state?.active_runtime?.map_id || "未选择"], ["导航状态", connected ? state?.navigation?.status || "待机" : "未确认"],
        ["剩余距离", value(connected ? state?.navigation?.feedback?.distance_remaining : null, " m", 2)],
        ["剩余点位", connected ? state?.navigation?.feedback?.number_of_poses_remaining : null],
      ]} /></Band>
      <Band icon={Cable} title="连接链路"><LinkSequence state={state} connected={connected} /></Band>
    </div>
  </>;
}

function Devices({ state, connected, vehicleType }) {
  const vcu = connected && state?.vcu?.online ? state.vcu : null;
  const identity = vcu?.identity;
  const telemetry = vcu?.telemetry;
  const mismatch = identity && (vehicleType === "ackermann" ? identity.profile !== "ackermann" : identity.profile !== "tracked");
  const installation = state?.vehicle?.id === vehicleType ? state?.vehicle_installation : null;
  return <div className="device-grid">
    <Band icon={Cpu} title="FieldGate VCU · HPM6750" status={<span className={`inline-status ${vcu ? "good" : "muted"}`}><i />{vcu ? "已连接" : "未连接"}</span>}>
      <Rows rows={[
        ["设备身份", identity ? "已验证" : "未验证"], ["固件版本", identity?.firmware_version],
        ["农机配置", identity?.profile === "ackermann" ? "阿克曼" : identity?.profile === "tracked" ? "履带差速" : "未提供"],
        ["机器标识", hex(identity?.machine_id)], ["协议标识", hex(identity?.protocol_id)],
        ["CAN 波特率", identity ? identity.profile === "ackermann" ? "1 Mbps" : "250 kbps" : "未提供"],
        ["硬件状态", identity ? identity.hardware_ready ? "就绪" : "初始化失败" : "未提供"],
        ["控制 / 反馈", telemetry ? `${telemetry.command_fresh ? "有效" : "无新指令"} / ${telemetry.feedback_fresh ? "有效" : "无新反馈"}` : "未提供"],
      ]} />
      {mismatch && <div className="status-message error-message">6750 固件配置与所选农机不一致，请停止运行并核对固件。</div>}
      {!vcu && <div className="status-message">{state?.vcu?.error || "未配置6750诊断串口"}</div>}
    </Band>
    <div className="device-details">
      <Band icon={Cable} title="传感器与数据通道"><Rows rows={CHANNELS.map(([topic, name]) =>
        [name, connected && state?.topics?.[topic]?.available ? "有数据" : "无数据"])} /></Band>
      <Band icon={Wrench} title="安装配置 · base_link"><Rows rows={Object.entries(installation?.mounts || {}).map(([name, mount]) =>
        [({ imu: "IMU", lidar: "C16", camera: "相机", rtk: "RTK主天线" })[name] || name,
          Array.isArray(mount?.xyz) ? `${mount.xyz.map(v => value(v, "", 4)).join(" / ")} m` : "未提供"])} />
        {!installation && <div className="status-message">未提供安装配置</div>}
      </Band>
    </div>
  </div>;
}

function SettingsView({ state, vehicleType }) {
  const config = vehicleType === "ackermann" ? state?.physical_config : null;
  const algorithms = config?.algorithms || {};
  return <div className="settings-grid">
    <Band icon={Settings} title="算法与运行配置"><Rows rows={[
      ["配置来源", vehicleType !== "ackermann" ? "履带差速程序默认" : !config ? "未提供" : config.source === "imported" ? "已导入的验证方案" : "程序默认"],
      ["方案", config?.preset], ["定位", algorithms.localization], ["初始定位", algorithms.initial_localization],
      ["感知", algorithms.perception], ["规划", algorithms.planner], ["控制", algorithms.controller],
    ]} /></Band>
    <Band icon={ShieldCheck} title="6750 部署"><Rows rows={[
      ["固件部署", "人工烧录"], ["诊断模式", "只读"], ["诊断串口", state?.vcu?.port || "未配置"],
      ["协议来源", "固件预置配置"], ["配置导入", "地图页面"],
    ]} /></Band>
    <Band icon={Wrench} title="软件信息"><Rows rows={[
      ["控制台", "农机控制台"], ["VCU 界面参考", "FieldGate VCU"], ["原作者", "刘泽禹"],
      ["ROS 域", state?.ros?.domain_id], ["ROS 范围", state?.ros ? state.ros.localhost_only ? "本机" : "局域网" : "未提供"],
    ]} /></Band>
  </div>;
}

function BridgeDiagnostics({ state, connected }) {
  const t = connected && state?.vcu?.online ? state.vcu.telemetry : null;
  return <Band icon={Cpu} title="CAN 桥接诊断"><Rows rows={[
    ["控制帧已转发", t?.command_forwarded], ["反馈帧已转发", t?.feedback_forwarded], ["队列丢弃帧", t?.dropped_frames],
    ["控制帧年龄", value(t?.command_age_ms, " ms", 0)], ["反馈帧年龄", value(t?.feedback_age_ms, " ms", 0)],
    ["桥接故障", t?.fault], ["左轮转速", value(t?.left_rpm, " RPM", 0)], ["右轮转速", value(t?.right_rpm, " RPM", 0)],
    ["目标速度", value(t?.target_speed, " m/s", 2)],
    ["目标前轮转角", value(Number.isFinite(t?.target_steering_angle) ? t.target_steering_angle * 180 / Math.PI : null, "°", 1)],
    ["左右目标百分比", t?.target_wheel_percent ? t.target_wheel_percent.join(" / ") + " %" : null],
  ]} /></Band>;
}

export default function ConsolePanels({ section, state, connected, vehicleType, diagnostics }) {
  if (section === "status") return <Overview state={state} connected={connected} vehicleType={vehicleType} />;
  if (section === "devices") return <Devices state={state} connected={connected} vehicleType={vehicleType} />;
  if (section === "settings") return <SettingsView state={state} vehicleType={vehicleType} />;
  return <>
    <div className={`diagnostic-health ${connected ? "connected" : ""}`}><ShieldCheck size={28} /><div><h3>{connected ? "设备诊断" : "等待有效设备反馈"}</h3><p>{connected ? state?.time_sync?.summary?.message || "未收到授时诊断" : "控制台离线，设备状态尚未验证"}</p></div></div>
    <div className="diagnostic-grid"><BridgeDiagnostics state={state} connected={connected} />{diagnostics}</div>
  </>;
}
