import { memo, useCallback, useEffect, useRef, useState } from "react";
import {
  centerAll,
  connectPi,
  disconnectPi,
  flushCurrentFrame,
  getLastFrame,
  getMotorChannels,
  getRuntimeState,
  getTransportStatus,
  setMotorTarget,
  type MotorChannel,
  type RuntimeState,
  type UdpControlFrame,
} from "./tauri";

const DEFAULT_ENDPOINT = "192.168.1.50:6000";
const MOTOR_COUNT = 32;
// Slider drags fire change events far faster than the IPC round-trip is
// worth; pending values are coalesced and sent at most ~30 times per second.
const SEND_INTERVAL_MS = 33;

function fallbackRuntime(): RuntimeState {
  return {
    endpoint: null,
    heartbeatHz: 100,
    disabledMotorIds: [30, 31],
    targetLogical: Array(MOTOR_COUNT).fill(0),
    targetApplied: Array(MOTOR_COUNT).fill(0),
    currentApplied: Array(MOTOR_COUNT).fill(0),
  };
}

type SliderRowProps = {
  channel: MotorChannel;
  logicalValue: number;
  appliedValue: number;
  onChange: (motorId: number, value: number) => void;
};

const SliderRow = memo(function SliderRow({
  channel,
  logicalValue,
  appliedValue,
  onChange,
}: SliderRowProps) {
  return (
    <label className={channel.enabled ? "slider-row dense" : "slider-row dense disabled"}>
      <div className="slider-meta">
        <strong>
          #{channel.id} {channel.name}
        </strong>
        <span>
          board {channel.board} / ch {channel.channel} / offset {channel.offset.toFixed(1)}
        </span>
        {!channel.enabled ? (
          <span className="channel-badge">disabled for current neck redesign</span>
        ) : null}
      </div>
      <input
        type="range"
        min={channel.minLogical}
        max={channel.maxLogical}
        step={0.5}
        value={logicalValue}
        disabled={!channel.enabled}
        onChange={(event) => onChange(channel.id, Number(event.target.value))}
      />
      <div className="value-pair">
        <span>L {logicalValue.toFixed(1)}</span>
        <span>A {appliedValue.toFixed(1)}</span>
      </div>
    </label>
  );
});

function App() {
  const [endpoint, setEndpoint] = useState(DEFAULT_ENDPOINT);
  const [channels, setChannels] = useState<MotorChannel[]>([]);
  const [runtime, setRuntime] = useState<RuntimeState>(fallbackRuntime);
  const [lastFrame, setLastFrame] = useState<UdpControlFrame | null>(null);
  const [status, setStatus] = useState("Loading config...");
  const [connected, setConnected] = useState(false);
  const pendingSendsRef = useRef(new Map<number, number>());
  const sendTimerRef = useRef<number | null>(null);

  useEffect(() => {
    async function bootstrap() {
      try {
        const [motorChannels, runtimeState, transportStatus] = await Promise.all([
          getMotorChannels(),
          getRuntimeState(),
          getTransportStatus(),
        ]);
        setChannels(motorChannels);
        setRuntime(runtimeState);
        setConnected(transportStatus.connected);
        if (transportStatus.endpoint) {
          setEndpoint(transportStatus.endpoint);
        }
        setStatus("Config loaded");
      } catch (error) {
        setStatus(String(error));
      }
    }

    bootstrap();
  }, []);

  async function refreshLastFrame() {
    try {
      setLastFrame(await getLastFrame());
    } catch (error) {
      setStatus(String(error));
    }
  }

  async function handleConnect() {
    try {
      const result = await connectPi(endpoint);
      setConnected(result.connected);
      setStatus(`UDP executor connected: ${result.endpoint}`);
    } catch (error) {
      setStatus(String(error));
    }
  }

  async function handleDisconnect() {
    try {
      await disconnectPi();
      setConnected(false);
      setStatus("UDP executor disconnected");
    } catch (error) {
      setStatus(String(error));
    }
  }

  async function handleCenterAll() {
    try {
      const next = await centerAll();
      setRuntime(next);
      setStatus("All motor targets reset to neutral");
    } catch (error) {
      setStatus(String(error));
    }
  }

  async function handleFlush() {
    try {
      const frame = await flushCurrentFrame();
      setLastFrame(frame);
      setStatus(frame ? `Frame ${frame.frameId} sent immediately` : "No UDP endpoint configured");
    } catch (error) {
      setStatus(String(error));
    }
  }

  const flushPendingSends = useCallback(async () => {
    sendTimerRef.current = null;
    const pending = [...pendingSendsRef.current.entries()];
    pendingSendsRef.current.clear();
    for (const [motorId, value] of pending) {
      try {
        const next = await setMotorTarget(motorId, value);
        setRuntime((current) => {
          // A drag may have queued newer values while this response was in
          // flight; keep those instead of the server echo.
          if (pendingSendsRef.current.size === 0) return next;
          const targetLogical = [...next.targetLogical];
          for (const [id, pendingValue] of pendingSendsRef.current) {
            targetLogical[id] = pendingValue;
          }
          return { ...next, targetLogical };
        });
      } catch (error) {
        setStatus(String(error));
      }
    }
  }, []);

  const handleSliderChange = useCallback(
    (motorId: number, value: number) => {
      setRuntime((current) => {
        const next = { ...current, targetLogical: [...current.targetLogical] };
        next.targetLogical[motorId] = value;
        return next;
      });

      pendingSendsRef.current.set(motorId, value);
      if (sendTimerRef.current === null) {
        sendTimerRef.current = window.setTimeout(flushPendingSends, SEND_INTERVAL_MS);
      }
    },
    [flushPendingSends],
  );

  useEffect(() => {
    return () => {
      if (sendTimerRef.current !== null) {
        window.clearTimeout(sendTimerRef.current);
      }
    };
  }, []);

  return (
    <main className="app-shell">
      <section className="hero-card">
        <div>
          <p className="eyebrow">BionicFace Calibration Console</p>
          <h1>32-channel direct motor control panel</h1>
          <p className="lede">
            React slider values are sent through Tauri invoke. Rust performs offset compensation,
            logical clamp, 100Hz interpolation, and UDP JSON dispatch to the Raspberry Pi dumb
            executor.
          </p>
        </div>
        <div className="hero-actions">
          <label className="endpoint-field">
            <span>UDP Endpoint</span>
            <input value={endpoint} onChange={(event) => setEndpoint(event.target.value)} />
          </label>
          <div className="button-row">
            <button onClick={handleConnect}>Connect</button>
            <button className="secondary" onClick={handleDisconnect}>
              Disconnect
            </button>
            <button className="secondary" onClick={handleCenterAll}>
              Center All
            </button>
            <button className="secondary" onClick={handleFlush}>
              Flush
            </button>
          </div>
          <p className="status-line">{connected ? "Transport: connected" : "Transport: idle"}</p>
          <p className="status-line muted">{status}</p>
        </div>
      </section>

      <section className="workspace-grid single">
        <article className="panel">
          <div className="panel-header">
            <div>
              <p className="panel-kicker">Sliders</p>
              <h2>Logical target input</h2>
            </div>
            <p className="panel-note">
              Disabled channels remain in protocol but hold their neutral values.
            </p>
          </div>

          <div className="slider-stack calibration-grid">
            {channels.map((channel) => (
              <SliderRow
                key={channel.id}
                channel={channel}
                logicalValue={runtime.targetLogical[channel.id] ?? channel.neutralLogical}
                appliedValue={runtime.currentApplied[channel.id] ?? channel.neutralApplied}
                onChange={handleSliderChange}
              />
            ))}
          </div>
        </article>

        <article className="panel">
          <div className="panel-header">
            <div>
              <p className="panel-kicker">Runtime</p>
              <h2>Transport and frame monitor</h2>
            </div>
            <button className="secondary" onClick={refreshLastFrame}>
              Refresh Frame
            </button>
          </div>

          <div className="runtime-grid">
            <div className="readout-chip">
              <span>Heartbeat</span>
              <strong>{runtime.heartbeatHz} Hz</strong>
            </div>
            <div className="readout-chip">
              <span>Disabled</span>
              <strong>{runtime.disabledMotorIds.join(", ") || "None"}</strong>
            </div>
            <div className="readout-chip">
              <span>Endpoint</span>
              <strong>{runtime.endpoint ?? "Not set"}</strong>
            </div>
          </div>

          <pre className="frame-dump">
            {lastFrame ? JSON.stringify(lastFrame, null, 2) : "No frame captured yet."}
          </pre>
        </article>
      </section>
    </main>
  );
}

export default App;
