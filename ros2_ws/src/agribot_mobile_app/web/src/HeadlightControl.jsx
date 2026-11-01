import { useEffect, useState } from "react";
import { Lightbulb } from "lucide-react";

export default function HeadlightControl({ state, connected, vehicleType, execute }) {
  const [busy, setBusy] = useState(false);
  const [requested, setRequested] = useState(null);
  useEffect(() => setRequested(null), [vehicleType, state?.active_runtime?.profile_id]);
  const feedback = state?.chassis_feedback;
  const fresh = connected && feedback?.fresh === true && feedback.vehicle_type === vehicleType;
  const enabled = requested ?? (fresh ? state?.chassis?.headlight : null);
  const reason = !connected ? "控制台离线"
    : !fresh ? "等待有效底盘反馈"
      : !feedback.headlight_supported ? "当前底盘协议不支持灯光控制"
        : feedback.read_only ? "观察阶段CAN只读"
          : typeof enabled !== "boolean" ? "等待灯光状态反馈" : "";
  const toggle = async () => {
    setBusy(true);
    try {
      const result = await execute("/api/v1/chassis/headlight", { enabled: !enabled, vehicle_type: vehicleType });
      if (result) setRequested(result.headlight);
    } finally {
      setBusy(false);
    }
  };
  return <button type="button" className={`headlight-control${enabled === true ? " active" : ""}`}
    aria-label={enabled === true ? "关灯" : "开灯"} aria-pressed={enabled === true}
    disabled={Boolean(reason) || busy} onClick={toggle}
    title={reason || (requested !== null ? "显示最近成功下发的灯光指令" : enabled ? "前灯已开启" : "前灯已关闭")}>
    <Lightbulb size={18} /><span>{busy ? "等待反馈" : enabled === true ? "关灯" : "开灯"}</span>
  </button>;
}
