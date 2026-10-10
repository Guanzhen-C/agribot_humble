import { useState } from "react";
import { Activity, Battery, Cable, Cpu, Gauge, Settings, ShieldCheck, Wrench } from "lucide-react";

const SECTIONS = [
  ["overview", "总览", Gauge], ["devices", "设备", Cable],
  ["diagnostics", "诊断", Activity], ["settings", "设置", Settings],
];
const value = (number, unit = "", digits = 1) => Number.isFinite(number) ? `${number.toFixed(digits)}${unit}` : "未提供";
const hex = number => Number.isInteger(number) ? `0x${number.toString(16).toUpperCase()}` : "未提供";

function Rows({ rows }) {
  return <dl className="console-rows">{rows.map(([label, text]) => <div key={label}><dt>{label}</dt><dd>{text ?? "未提供"}</dd></div>)}</dl>;
}
function Band({ icon: Icon, title, children }) {
  return <section className="section-band"><div className="section-heading"><h2><Icon size={16} />{title}</h2></div>{children}</section>;
}
function Overview({ state, connected }) {
  const fresh = connected && state?.topics?.["/scout_status"]?.available === true;
  const chassis = fresh ? state?.chassis : null;
  const vcu = state?.vcu;
  const active = state?.active_runtime;
  const localization = state?.localization;
  return <>
    <Band icon={Gauge} title="运行总览">
      <div className="instrument-grid">
        <div><Battery size={18} /><span>电池电压</span><strong>{value(chassis?.battery_voltage, " V")}</strong></div>
        <div><Gauge size={18} /><span>实际速度</span><strong>{value(chassis?.linear_velocity, " m/s", 2)}</strong></div>
      </div>
      <Rows rows={[
        ["控制台链路", connected ? "已连接" : "离线"],
        ["运行任务", !connected ? "未确认" : active ? active.motion ? "真车运行" : "观察阶段" : "未启动"],
        ["导航状态", connected ? state?.navigation?.status || "未提供" : "未确认"],
        ["地图", active?.map_id || "未选择"],
        ["融合定位", connected && localization?.fusion_ready === true ? "就绪" : "未就绪"],
        ["底盘反馈", fresh ? "有数据" : "未收到有效反馈"],
        ["故障码", chassis?.fault_code],
      ]} />
    </Band>
    <Band icon={Cable} title="连接链路">
      <div className="link-stack">
        <span className={connected ? "link-up" : ""}>手机 / 浏览器 ↔ RDK</span>
        <span className={connected && vcu?.online ? "link-up" : ""}>RDK ↔ HPM6750 <small>{vcu?.online && connected ? "身份已验证" : "未验证"}</small></span>
        <span className={connected && vcu?.online && vcu?.telemetry?.feedback_fresh ? "link-up" : ""}>HPM6750 ↔ 执行控制器 <small>{vcu?.telemetry?.feedback_fresh && vcu?.online && connected ? "反馈有效" : "未验证"}</small></span>
      </div>
    </Band>
  </>;
}
function Devices({ state, connected, vehicleType }) {
  const vcu = connected && state?.vcu?.online ? state.vcu : null;
  const identity = vcu?.identity;
  const telemetry = vcu?.telemetry;
  const mismatch = identity && (vehicleType === "ackermann" ? identity.profile !== "ackermann" : identity.profile !== "tracked");
  const installation = state?.vehicle?.id === vehicleType ? state?.vehicle_installation : null;
  return <>
    <Band icon={Cpu} title="FieldGate VCU · HPM6750">
      <Rows rows={[
        ["设备身份", identity ? "已验证" : "未验证"],
        ["固件版本", identity?.firmware_version], ["农机配置", identity?.profile === "ackermann" ? "阿克曼" : identity?.profile === "tracked" ? "履带差速" : "未提供"],
        ["机器标识", hex(identity?.machine_id)], ["协议标识", hex(identity?.protocol_id)],
        ["CAN 波特率", identity ? identity.profile === "ackermann" ? "1 Mbps" : "250 kbps" : "未提供"],
        ["硬件状态", identity ? identity.hardware_ready ? "就绪" : "初始化失败" : "未提供"],
        ["控制 / 反馈", telemetry ? `${telemetry.command_fresh ? "有效" : "无新指令"} / ${telemetry.feedback_fresh ? "有效" : "无新反馈"}` : "未提供"],
      ]} />
      {mismatch && <div className="status-message error-message">6750 固件配置与所选农机不一致，请停止运行并核对固件。</div>}
      {!vcu && <div className="status-message">{state?.vcu?.error || "未配置6750诊断串口"}</div>}
    </Band>
    <Band icon={Wrench} title="安装配置 · base_link">
      <Rows rows={Object.entries(installation?.mounts || {}).map(([name, mount]) =>
        [({ imu: "IMU", lidar: "C16", camera: "相机", rtk: "RTK主天线" })[name] || name,
          Array.isArray(mount?.xyz) ? `${mount.xyz.map(v => value(v, "", 4)).join(" / ")} m` : "未提供"])} />
      {!installation && <div className="status-message">未提供安装配置</div>}
    </Band>
    <Band icon={Cable} title="传感器与数据通道">
      <Rows rows={[["/lidar/points", "C16"], ["/imu/data", "IMU"], ["/camera/rgb/image_raw", "单目相机"],
        ["/rtk/fix", "RTK"], ["/fastlivo_rtk/odometry", "融合定位"]].map(([topic, name]) =>
        [name, connected && state?.topics?.[topic]?.available ? "有数据" : "无数据"])} />
    </Band>
  </>;
}
function SettingsView({ state, vehicleType }) {
  const config = vehicleType === "ackermann" ? state?.physical_config : null;
  const algorithms = config?.algorithms || {};
  return <>
    <Band icon={Settings} title="运行配置">
      <Rows rows={[["配置来源", vehicleType !== "ackermann" ? "履带差速程序默认" : !config ? "未提供" : config.source === "imported" ? "已导入的验证方案" : "程序默认"],
        ["方案", config?.preset], ["定位", algorithms.localization], ["初始定位", algorithms.initial_localization],
        ["感知", algorithms.perception], ["规划", algorithms.planner], ["控制", algorithms.controller]]} />
    </Band>
    <Band icon={ShieldCheck} title="6750 部署">
      <Rows rows={[["固件部署", "人工烧录"], ["诊断模式", "只读"], ["诊断串口", state?.vcu?.port || "未配置"],
        ["协议来源", "固件预置配置"], ["配置导入", "地图页面"]]} />
    </Band>
    <Band icon={Wrench} title="软件信息">
      <Rows rows={[["控制台", "农机控制台"], ["VCU 界面参考", "FieldGate VCU"], ["原作者", "刘泽禹"],
        ["ROS 域", state?.ros?.domain_id], ["ROS 范围", state?.ros ? state.ros.localhost_only ? "本机" : "局域网" : "未提供"]]} />
    </Band>
  </>;
}
function BridgeDiagnostics({ state, connected }) {
  const t = connected && state?.vcu?.online ? state.vcu.telemetry : null;
  return <Band icon={Cpu} title="CAN 桥接诊断"><Rows rows={[
    ["控制帧已转发", t?.command_forwarded], ["反馈帧已转发", t?.feedback_forwarded], ["队列丢弃帧", t?.dropped_frames],
    ["控制帧年龄", value(t?.command_age_ms, " ms", 0)], ["反馈帧年龄", value(t?.feedback_age_ms, " ms", 0)],
    ["桥接故障", t?.fault], ["左轮转速", value(t?.left_rpm, " RPM", 0)], ["右轮转速", value(t?.right_rpm, " RPM", 0)],
    ["目标速度", value(t?.target_speed, " m/s", 2)], ["目标前轮转角", value(Number.isFinite(t?.target_steering_angle) ? t.target_steering_angle * 180 / Math.PI : null, "°", 1)],
    ["左右目标百分比", t?.target_wheel_percent ? t.target_wheel_percent.join(" / ") + " %" : null],
  ]} /></Band>;
}

export default function ConsolePanels({ state, connected, vehicleType, diagnostics }) {
  const [section, setSection] = useState("overview");
  return <div className="console-pages">
    <nav className="console-section-nav" aria-label="状态分类">{SECTIONS.map(([id, label, Icon]) =>
      <button type="button" key={id} aria-pressed={section === id} className={section === id ? "selected" : ""} onClick={() => setSection(id)}><Icon size={16} />{label}</button>)}</nav>
    <div className="panel-content">
      {section === "overview" && <Overview state={state} connected={connected} />}
      {section === "devices" && <Devices state={state} connected={connected} vehicleType={vehicleType} />}
      {section === "diagnostics" && <><BridgeDiagnostics state={state} connected={connected} />{diagnostics}</>}
      {section === "settings" && <SettingsView state={state} vehicleType={vehicleType} />}
    </div>
  </div>;
}
