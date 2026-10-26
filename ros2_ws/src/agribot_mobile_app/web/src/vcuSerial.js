// HPM6750 UART0 diagnostics and protocol selection over Web Serial, mirroring vcu_monitor.py.
// The 6750 maintenance UART sits on the operator's computer, so the browser
// queries it directly; this needs a desktop Chromium browser on HTTPS.
import { useCallback, useEffect, useRef, useState } from "react";

const CRC_TABLE = (() => {
  const table = new Uint32Array(256);
  for (let n = 0; n < 256; n += 1) {
    let c = n;
    for (let k = 0; k < 8; k += 1) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    table[n] = c >>> 0;
  }
  return table;
})();

function crc32(bytes) {
  let crc = 0xffffffff;
  for (const byte of bytes) crc = CRC_TABLE[(crc ^ byte) & 0xff] ^ (crc >>> 8);
  return (crc ^ 0xffffffff) >>> 0;
}

function query(opcode, sequence) {
  const frame = new Uint8Array(24);
  const view = new DataView(frame.buffer);
  frame.set([0x56, 0x43, 0x46, 0x47, 1, opcode]);
  view.setUint16(6, 8, true);
  view.setUint32(8, sequence, true);
  view.setUint32(20, crc32(frame.subarray(0, 20)), true);
  return frame;
}

function checkedFrame(data, opcode, sequence, length) {
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  if (data.length !== length || data[0] !== 0x56 || data[1] !== 0x43 || data[2] !== 0x46
    || data[3] !== 0x47 || data[4] !== 1 || data[5] !== opcode) throw new Error("6750回复格式错误");
  if (view.getUint16(6, true) !== length - 16 || view.getUint32(8, true) !== sequence) {
    throw new Error("6750回复长度或序号错误");
  }
  if (view.getUint32(length - 4, true) !== crc32(data.subarray(0, length - 4))) throw new Error("6750回复CRC错误");
  return new DataView(data.buffer, data.byteOffset + 12, length - 16);
}

const SUPPORTED = [[18, 514], [1, 257]];
const supported = (machine, protocol) => SUPPORTED.some(([m, p]) => m === machine && p === protocol);

// Selections the native-bridge firmware accepts for the agribot stack.
export const VCU_PROFILES = [
  { id: "ackermann", label: "阿克曼（转向农机）", machine_id: 0x0012, protocol_id: 0x0202, bitrate: "1 Mbps" },
  { id: "tracked", label: "履带差速（三合一）", machine_id: 0x0001, protocol_id: 0x0101, bitrate: "250 kbps" },
];
export const VCU_INTERFACES = [
  { id: 1, label: "标准 CAN" },
  { id: 2, label: "SocketCAN" },
];

// Shown while switching; each stage is a real step of the VCFG write.
export const VCU_SWITCH_STAGES = ["发送配置", "写入 Flash", "重载协议", "回读校验"];

const ACK_MESSAGES = {
  1: "6750拒绝了损坏的配置帧",
  2: "6750不支持当前配置帧版本",
  3: "6750固件不支持所选农机",
  4: "6750固件不支持所选协议",
  5: "所选农机与协议不兼容",
  6: "6750未处于停车待机：算力仍在发送控制指令，请停止导航和底盘程序1秒后再写入",
  7: "6750写入Flash失败",
  8: "6750写入后校验失败",
  9: "6750固件不支持所选算力接口",
};

function writeFrame(sequence, machine, protocol, computeInterface) {
  const frame = new Uint8Array(24);
  const view = new DataView(frame.buffer);
  frame.set([0x56, 0x43, 0x46, 0x47, 1, 0x10]);
  view.setUint16(6, 8, true);
  view.setUint32(8, sequence, true);
  view.setUint16(12, machine, true);
  view.setUint16(14, protocol, true);
  view.setUint32(16, computeInterface, true);
  view.setUint32(20, crc32(frame.subarray(0, 20)), true);
  return frame;
}

export function decodeWriteAck(data, sequence) {
  const p = checkedFrame(data, 0x90, sequence, 24);
  return {
    status: p.getUint8(0), slot: p.getUint8(1) === 0 ? "A" : "B", generation: p.getUint16(2, true),
    machine_id: p.getUint16(4, true), protocol_id: p.getUint16(6, true),
  };
}

export function decodeIdentity(data, sequence) {
  const p = checkedFrame(data, 0xa1, sequence, 24);
  const [major, minor, patch, flags] = [p.getUint8(0), p.getUint8(1), p.getUint8(2), p.getUint8(3)];
  const machine = p.getUint16(4, true);
  const protocol = p.getUint16(6, true);
  if (flags & ~3 || !supported(machine, protocol)) throw new Error("6750原生桥接配置不支持");
  return {
    firmware_version: `${major}.${minor}.${patch}`, machine_id: machine, protocol_id: protocol,
    profile: machine === 18 ? "ackermann" : "tracked",
    native_bridge: Boolean(flags & 2), hardware_ready: Boolean(flags & 1),
  };
}

export function decodeTelemetry(data, sequence) {
  const p = checkedFrame(data, 0xa0, sequence, 80);
  if (p.getUint8(0) !== 2) throw new Error("6750需使用原生桥接状态协议v2");
  const flags = p.getUint8(1);
  const fault = p.getUint16(2, true);
  const machine = p.getUint16(4, true);
  const protocol = p.getUint16(6, true);
  const iface = p.getUint8(8);
  const valid = p.getUint16(10, true);
  if (flags & ~15 || valid & ~15 || (iface !== 1 && iface !== 2) || !supported(machine, protocol)) {
    throw new Error("6750状态值错误");
  }
  const age = offset => (p.getUint32(offset, true) === 0xffffffff ? null : p.getUint32(offset, true));
  const i32 = offset => p.getInt32(offset, true);
  const commandFresh = Boolean(flags & 2);
  return {
    machine_id: machine, protocol_id: protocol, configured: Boolean(flags & 1),
    command_fresh: commandFresh, feedback_fresh: Boolean(flags & 4),
    hardware_ready: Boolean(flags & 8), fault, compute_interface: iface,
    uptime_ms: p.getUint32(12, true), command_age_ms: age(16), feedback_age_ms: age(20),
    battery_voltage: valid & 1 ? i32(24) / 1000 : null,
    linear_velocity: valid & 2 ? i32(28) / 1000 : null,
    angular_velocity: valid & 2 ? i32(32) / 1000 : null,
    target_speed: commandFresh && machine === 18 ? i32(36) / 1000 : null,
    target_steering_angle: commandFresh && machine === 18 ? i32(40) / 1000 : null,
    target_wheel_percent: commandFresh && machine === 1 ? [i32(36), i32(40)] : null,
    left_rpm: valid & 4 ? p.getInt16(44, true) : null,
    right_rpm: valid & 8 ? p.getInt16(46, true) : null,
    work_mode: p.getUint8(48) === 255 ? null : p.getUint8(48),
    drive_disabled: p.getUint8(49) === 255 ? null : Boolean(p.getUint8(49)),
    command_forwarded: p.getUint32(52, true), dropped_frames: p.getUint32(56, true),
    feedback_forwarded: p.getUint32(60, true),
  };
}

const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const PORT_LABEL = "电脑浏览器串口";

class SerialSession {
  constructor(port, publish) {
    this.port = port;
    this.publish = publish;
    this.buffer = new Uint8Array(0);
    this.stopped = false;
    this.waiter = null;
    this.queue = Promise.resolve();
    this.sequence = Math.floor(Math.random() * 0xffff);
  }

  // Polling and configuration writes share one UART; never interleave frames.
  exclusive(task) {
    const run = this.queue.then(task, task);
    this.queue = run.catch(() => {});
    return run;
  }

  nextSequence() {
    this.sequence = (this.sequence + 1) >>> 0;
    return this.sequence;
  }

  async poll() {
    const identitySequence = this.nextSequence();
    const identity = decodeIdentity(await this.request(0x21, identitySequence, 24), identitySequence);
    const telemetrySequence = this.nextSequence();
    const telemetry = decodeTelemetry(await this.request(0x20, telemetrySequence, 80), telemetrySequence);
    if (identity.machine_id !== telemetry.machine_id || identity.protocol_id !== telemetry.protocol_id) {
      throw new Error("6750身份与状态配置不一致");
    }
    return { identity, telemetry };
  }

  writeSelection({ machine_id, protocol_id, compute_interface }, onStage = () => {}) {
    return this.exclusive(async () => {
      onStage(0);
      await sleep(350);
      const sequence = this.nextSequence();
      onStage(1);
      const ack = decodeWriteAck(
        await this.request(0x10, sequence, 24, writeFrame(sequence, machine_id, protocol_id, compute_interface), 3000),
        sequence,
      );
      if (ack.status !== 0) throw new Error(ACK_MESSAGES[ack.status] || `6750返回未知错误：${ack.status}`);
      if (ack.machine_id !== machine_id || ack.protocol_id !== protocol_id) throw new Error("6750回复的配置与写入值不一致");
      // The firmware re-initialises CAN right after the ACK and does not read UART0
      // meanwhile, so give it time before querying, then retry a lost reply.
      onStage(2);
      await sleep(500);
      onStage(3);
      let reply;
      for (let attempt = 0; !reply; attempt += 1) {
        try {
          reply = await this.poll();
        } catch (error) {
          if (attempt >= 2) throw error;
          await sleep(300);
        }
      }
      // Saved only once the running firmware reports the new selection itself.
      const { identity, telemetry } = reply;
      if (identity.machine_id !== machine_id || identity.protocol_id !== protocol_id
        || telemetry.compute_interface !== compute_interface) {
        throw new Error("6750已回复写入，但运行配置未切换，请复位后重新核对");
      }
      this.publish({ configured: true, online: true, port: PORT_LABEL, identity, telemetry, error: "", received_at: Date.now() / 1000 });
      return { ...ack, identity };
    });
  }

  async run() {
    await this.port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: "none", flowControl: "none" });
    this.writer = this.port.writable.getWriter();
    this.reader = this.port.readable.getReader();
    const reading = this.readLoop();
    while (!this.stopped) {
      try {
        const { identity, telemetry } = await this.exclusive(() => this.poll());
        this.publish({ configured: true, online: true, port: PORT_LABEL, identity, telemetry, error: "", received_at: Date.now() / 1000 });
        await sleep(1000);
      } catch (error) {
        if (this.stopped) break;
        this.publish({ configured: true, online: false, port: PORT_LABEL, identity: null, telemetry: null, error: error.message });
        await sleep(2000);
      }
    }
    await reading;
  }

  async readLoop() {
    try {
      while (!this.stopped) {
        const { value, done } = await this.reader.read();
        if (done) break;
        const merged = new Uint8Array(this.buffer.length + value.length);
        merged.set(this.buffer);
        merged.set(value, this.buffer.length);
        this.buffer = merged.length > 512 ? merged.slice(-512) : merged;
        this.waiter?.();
      }
    } catch (error) {
      if (!this.stopped) this.publish({ configured: true, online: false, port: PORT_LABEL, identity: null, telemetry: null, error: `串口读取失败: ${error.message}` });
    }
  }

  async request(opcode, sequence, size, frame = query(opcode, sequence), timeoutMs = 600) {
    this.buffer = new Uint8Array(0);
    await this.writer.write(frame);
    const deadline = Date.now() + timeoutMs;
    while (!this.stopped && Date.now() < deadline) {
      let start = -1;
      for (let i = 0; i + 3 < this.buffer.length; i += 1) {
        if (this.buffer[i] === 0x56 && this.buffer[i + 1] === 0x43 && this.buffer[i + 2] === 0x46 && this.buffer[i + 3] === 0x47) { start = i; break; }
      }
      if (start >= 0 && this.buffer.length - start >= size) {
        const frame = this.buffer.slice(start, start + size);
        checkedFrame(frame, opcode | 0x80, sequence, size);
        return frame;
      }
      await new Promise(resolve => {
        this.waiter = resolve;
        setTimeout(resolve, Math.max(1, deadline - Date.now()));
      });
    }
    throw new Error("6750回复超时");
  }

  async close() {
    this.stopped = true;
    try { await this.reader?.cancel(); } catch { /* port already gone */ }
    try { this.reader?.releaseLock(); } catch { /* released */ }
    try { this.writer?.releaseLock(); } catch { /* released */ }
    try { await this.port.close(); } catch { /* closed */ }
  }
}

export function browserSerialAvailable() {
  return typeof window !== "undefined" && window.isSecureContext && typeof navigator !== "undefined" && "serial" in navigator;
}

export function useBrowserVcu() {
  const [vcu, setVcu] = useState(null);
  const [busy, setBusy] = useState(false);
  const session = useRef(null);

  const disconnect = useCallback(async () => {
    const current = session.current;
    session.current = null;
    setVcu(null);
    if (current) await current.close();
  }, []);

  const connect = useCallback(async () => {
    if (!browserSerialAvailable() || session.current) return;
    setBusy(true);
    try {
      const port = await navigator.serial.requestPort();
      const current = new SerialSession(port, values => { if (session.current === current) setVcu(values); });
      session.current = current;
      setVcu({ configured: true, online: false, port: PORT_LABEL, identity: null, telemetry: null, error: "等待身份回复" });
      current.run().catch(error => {
        if (session.current === current) {
          session.current = null;
          setVcu({ configured: true, online: false, port: PORT_LABEL, identity: null, telemetry: null, error: `无法打开串口: ${error.message}` });
        }
      });
    } catch (error) {
      if (error?.name !== "NotFoundError") {
        setVcu({ configured: true, online: false, port: PORT_LABEL, identity: null, telemetry: null, error: `无法打开串口: ${error.message}` });
      }
    } finally {
      setBusy(false);
    }
  }, []);

  useEffect(() => {
    if (!browserSerialAvailable()) return undefined;
    const onDisconnect = event => {
      if (session.current?.port === event.target || session.current?.port === event.port) disconnect();
    };
    navigator.serial.addEventListener("disconnect", onDisconnect);
    return () => {
      navigator.serial.removeEventListener("disconnect", onDisconnect);
      session.current?.close();
    };
  }, [disconnect]);

  const writeSelection = useCallback(async (selection, onStage) => {
    if (!session.current) throw new Error("请先连接 6750 诊断串口");
    return session.current.writeSelection(selection, onStage);
  }, []);

  return { available: browserSerialAvailable(), vcu, busy, active: vcu !== null, connect, disconnect, writeSelection };
}

// The console's vehicle selector decides the 6750 protocol.
export function vehicleProfile(vehicleType) {
  return VCU_PROFILES.find(item => item.id === (vehicleType === "ackermann" ? "ackermann" : "tracked"));
}

// Compares the page's vehicle with the 6750; only an explicit write() changes the 6750.
export function useVcuConfig(vcuSerial, vehicleType) {
  const [job, setJob] = useState(null);
  const { writeSelection } = vcuSerial;
  const profile = vehicleProfile(vehicleType);

  useEffect(() => setJob(current => (current?.status === "writing" ? current : null)), [vehicleType]);

  const write = useCallback(async computeInterface => {
    const target = vehicleProfile(vehicleType);
    const iface = VCU_INTERFACES.find(item => item.id === computeInterface);
    setJob({ status: "writing", stage: 0, text: `正在写入 6750：${target.label}，${iface.label}` });
    try {
      const ack = await writeSelection(
        { machine_id: target.machine_id, protocol_id: target.protocol_id, compute_interface: computeInterface },
        stage => setJob(current => (current?.status === "writing" ? { ...current, stage } : current)),
      );
      setJob({ status: "done", text: `6750 已写入并回读验证：${target.label}（CAN ${target.bitrate}），${iface.label}，槽 ${ack.slot}，代数 ${ack.generation}` });
    } catch (error) {
      setJob({ status: "failed", text: `写入 6750 失败：${error.message}` });
    }
  }, [vehicleType, writeSelection]);

  return { profile, write, ...(job || {}) };
}
