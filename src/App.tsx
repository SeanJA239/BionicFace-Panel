import { memo, useCallback, useEffect, useRef, useState } from "react";
import {
  applyExpressionPresetScaled,
  centerAll,
  connectPi,
  disconnectPi,
  flushCurrentFrame,
  forceManualControl,
  getExternalInputStatus,
  getLastFrame,
  getMotorChannels,
  getRuntimeState,
  getSequencePlaybackStatus,
  getTransportStatus,
  listExpressionPresets,
  listSequences,
  nod,
  playSequence,
  setIdleBehaviorEnabled,
  setMotorTarget,
  stopSequence,
  wink,
  type ExpressionPresetSummary,
  type ExternalInputStatus,
  type MotorChannel,
  type RuntimeState,
  type SequencePlaybackStatus,
  type SequenceSummary,
  type UdpControlFrame,
} from "./tauri";

const DEFAULT_ENDPOINT = "192.168.1.101:6000";
const MOTOR_COUNT = 32;
// Slider drags fire change events far faster than the IPC round-trip is
// worth; pending values are coalesced and sent at most ~30 times per second.
const SEND_INTERVAL_MS = 33;
// Polls runtime state (and thus control_source) so the panel notices an
// external driver claiming control, or relinquishing it, without the user
// having to trigger a manual action first.
const RUNTIME_POLL_INTERVAL_MS = 300;

function fallbackRuntime(): RuntimeState {
  return {
    endpoint: null,
    heartbeatHz: 100,
    disabledMotorIds: [],
    targetLogical: Array(MOTOR_COUNT).fill(0),
    targetApplied: Array(MOTOR_COUNT).fill(0),
    currentApplied: Array(MOTOR_COUNT).fill(0),
    targetNorm: Array(MOTOR_COUNT).fill(0),
    currentNorm: Array(MOTOR_COUNT).fill(0),
    controlSource: "manual",
    idleBehaviorEnabled: true,
  };
}

function controlSourceLabel(source: RuntimeState["controlSource"]): string {
  switch (source) {
    case "external":
      return "External";
    case "idle":
      return "Idle";
    default:
      return "Manual";
  }
}

type SliderRowProps = {
  channel: MotorChannel;
  logicalValue: number;
  appliedValue: number;
  normValue: number;
  locked: boolean;
  onChange: (motorId: number, value: number) => void;
};

const SliderRow = memo(function SliderRow({
  channel,
  logicalValue,
  appliedValue,
  normValue,
  locked,
  onChange,
}: SliderRowProps) {
  const disabled = !channel.enabled || locked;
  return (
    <label className={disabled ? "slider-row dense disabled" : "slider-row dense"}>
      <div className="slider-meta">
        <strong>
          #{channel.id} {channel.name}
        </strong>
        <span>
          board {channel.board} / ch {channel.channel} / offset {channel.offset.toFixed(1)}
        </span>
        {!channel.enabled ? (
          <span className="channel-badge">disabled in config</span>
        ) : null}
        {channel.enabled && locked ? (
          <span className="channel-badge">external control active</span>
        ) : null}
      </div>
      <input
        type="range"
        min={channel.minLogical}
        max={channel.maxLogical}
        step={0.5}
        value={logicalValue}
        disabled={disabled}
        onChange={(event) => onChange(channel.id, Number(event.target.value))}
      />
      <div className="value-pair">
        <span>L {logicalValue.toFixed(1)}</span>
        <span>A {appliedValue.toFixed(1)}</span>
        <span>N {normValue.toFixed(2)}</span>
      </div>
    </label>
  );
});

function App() {
  const [endpoint, setEndpoint] = useState(DEFAULT_ENDPOINT);
  const [channels, setChannels] = useState<MotorChannel[]>([]);
  const [runtime, setRuntime] = useState<RuntimeState>(fallbackRuntime);
  const [expressionPresets, setExpressionPresets] = useState<ExpressionPresetSummary[]>([]);
  const [lastFrame, setLastFrame] = useState<UdpControlFrame | null>(null);
  const [status, setStatus] = useState("Loading config...");
  const [connected, setConnected] = useState(false);
  const [activePresetId, setActivePresetId] = useState<string | null>(null);
  const [presetIntensity, setPresetIntensity] = useState(1.0);
  const [externalStatus, setExternalStatus] = useState<ExternalInputStatus | null>(null);
  const [sequences, setSequences] = useState<SequenceSummary[]>([]);
  const [playbackStatus, setPlaybackStatus] = useState<SequencePlaybackStatus>({
    playing: false,
    sequenceId: null,
    label: null,
    stepIndex: null,
    totalSteps: null,
  });
  const isExternal = runtime.controlSource === "external";
  const pendingSendsRef = useRef(new Map<number, number>());
  const sendTimerRef = useRef<number | null>(null);

  useEffect(() => {
    async function bootstrap() {
      try {
        const [motorChannels, runtimeState, transportStatus, presets, sequenceList] = await Promise.all([
          getMotorChannels(),
          getRuntimeState(),
          getTransportStatus(),
          listExpressionPresets(),
          listSequences(),
        ]);
        setChannels(motorChannels);
        setRuntime(runtimeState);
        setConnected(transportStatus.connected);
        setExpressionPresets(presets);
        setSequences(sequenceList);
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

  useEffect(() => {
    const timer = window.setInterval(async () => {
      try {
        const [runtimeState, status, playback] = await Promise.all([
          getRuntimeState(),
          getExternalInputStatus(),
          getSequencePlaybackStatus(),
        ]);
        setRuntime(runtimeState);
        setExternalStatus(status);
        setPlaybackStatus(playback);
        if (runtimeState.controlSource === "external") {
          setActivePresetId(null);
        }
      } catch {
        // Transient IPC hiccups shouldn't spam the status line on every poll.
      }
    }, RUNTIME_POLL_INTERVAL_MS);
    return () => window.clearInterval(timer);
  }, []);

  async function handleForceManualControl() {
    try {
      const next = await forceManualControl();
      setRuntime(next);
      setStatus("Forced control source back to Manual");
    } catch (error) {
      setStatus(String(error));
    }
  }

  async function handlePlaySequence(sequence: SequenceSummary) {
    try {
      await playSequence(sequence.id);
      setActivePresetId(null);
      setStatus(`Playing sequence: ${sequence.label}`);
    } catch (error) {
      setStatus(String(error));
    }
  }

  async function handleToggleIdleBehavior() {
    try {
      const next = await setIdleBehaviorEnabled(!runtime.idleBehaviorEnabled);
      setRuntime(next);
      setStatus(`Idle behavior ${next.idleBehaviorEnabled ? "enabled" : "disabled"}`);
    } catch (error) {
      setStatus(String(error));
    }
  }

  async function handleStopSequence() {
    try {
      const next = await stopSequence();
      setRuntime(next);
      setPlaybackStatus((current) => ({ ...current, playing: false }));
      setStatus("Sequence playback stopped");
    } catch (error) {
      setStatus(String(error));
    }
  }

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
      setActivePresetId(null);
      setStatus("All motor targets reset to neutral");
    } catch (error) {
      setStatus(String(error));
    }
  }

  async function handleNod() {
    try {
      setStatus("Nodding...");
      const next = await nod();
      setRuntime(next);
      setStatus("Nod complete");
    } catch (error) {
      setStatus(String(error));
    }
  }

  async function handleWink() {
    try {
      setStatus("Winking...");
      const next = await wink();
      setRuntime(next);
      setActivePresetId(null);
      setStatus("Wink complete (returned to center)");
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
      setActivePresetId(null);
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

  async function handleApplyPreset(preset: ExpressionPresetSummary, intensity: number) {
    try {
      const next = await applyExpressionPresetScaled(preset.id, intensity);
      setRuntime(next);
      setActivePresetId(preset.id);
      setStatus(`Expression preset applied: ${preset.label} @ ${intensity.toFixed(2)}`);
    } catch (error) {
      setStatus(String(error));
    }
  }

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
            <button className="secondary" onClick={handleCenterAll} disabled={isExternal}>
              Center All
            </button>
            <button className="secondary" onClick={handleNod} disabled={isExternal}>
              Nod
            </button>
            <button className="secondary" onClick={handleFlush}>
              Flush
            </button>
            {isExternal ? (
              <button className="secondary" onClick={handleForceManualControl}>
                Force Manual
              </button>
            ) : null}
          </div>
          {expressionPresets.length > 0 ? (
            <>
              <label className="endpoint-field">
                <span>Preset intensity ({presetIntensity.toFixed(2)})</span>
                <input
                  type="range"
                  min={0}
                  max={1}
                  step={0.01}
                  value={presetIntensity}
                  disabled={isExternal}
                  onChange={(event) => setPresetIntensity(Number(event.target.value))}
                />
              </label>
              <div className="button-row">
                {expressionPresets.map((preset) => (
                  <button
                    className={activePresetId === preset.id ? "" : "secondary"}
                    key={preset.id}
                    disabled={isExternal}
                    onClick={() =>
                      preset.id === "wink" ? handleWink() : handleApplyPreset(preset, presetIntensity)
                    }
                  >
                    {preset.label}
                  </button>
                ))}
              </div>
            </>
          ) : null}
          {sequences.length > 0 ? (
            <>
              <div className="button-row">
                {sequences.map((sequence) => (
                  <button
                    className={playbackStatus.sequenceId === sequence.id ? "" : "secondary"}
                    key={sequence.id}
                    disabled={isExternal}
                    onClick={() => handlePlaySequence(sequence)}
                  >
                    {sequence.label}
                  </button>
                ))}
                <button className="secondary" onClick={handleStopSequence} disabled={!playbackStatus.playing}>
                  Stop Sequence
                </button>
              </div>
              {playbackStatus.playing ? (
                <p className="status-line">
                  Playing <strong>{playbackStatus.label}</strong>: step{" "}
                  {(playbackStatus.stepIndex ?? 0) + 1}/{playbackStatus.totalSteps}
                </p>
              ) : null}
            </>
          ) : null}
          <p className="status-line">{connected ? "Transport: connected" : "Transport: idle"}</p>
          <p className="status-line">
            Control source: <strong>{controlSourceLabel(runtime.controlSource)}</strong>
            {isExternal && externalStatus
              ? ` (port ${externalStatus.port}, ${externalStatus.fps.toFixed(1)} fps, seq ${externalStatus.lastSeq ?? "-"})`
              : null}
          </p>
          <label className="endpoint-field">
            <span>
              <input
                type="checkbox"
                checked={runtime.idleBehaviorEnabled}
                onChange={handleToggleIdleBehavior}
              />{" "}
              Idle noise + blink
            </span>
          </label>
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
                normValue={runtime.currentNorm[channel.id] ?? 0}
                locked={isExternal}
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
