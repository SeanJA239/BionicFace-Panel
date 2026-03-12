import { useEffect, useMemo, useRef, useState } from "react";
import {
  connectPi,
  pingPi,
  sendBlendshapeFrame,
  setSingleMotor,
  updateMotorConfig,
} from "./tauri";
import {
  BLENDSHAPE_MAPPING,
  BLENDSHAPE_NAMES,
  DEFAULT_MOTOR_CONFIG,
  MOTOR_NODES,
} from "./topology";

const CANVAS_WIDTH = 640;
const CANVAS_HEIGHT = 720;
const DRAG_RANGE_PX = 42;
const LOGICAL_RANGE_DEG = 45;

type DragState = {
  motorId: number;
  pointerId: number;
  startY: number;
  startLogical: number;
};

const endpointDefault = "tcp://192.168.1.50:5555";

function clamp(value: number, min: number, max: number) {
  return Math.max(min, Math.min(max, value));
}

function App() {
  const [endpoint, setEndpoint] = useState(endpointDefault);
  const [connection, setConnection] = useState("Not connected");
  const [logicalAngles, setLogicalAngles] = useState<number[]>(() => Array(32).fill(0));
  const [blendshapes, setBlendshapes] = useState<Record<string, number>>(() =>
    Object.fromEntries(BLENDSHAPE_NAMES.map((name) => [name, 0])),
  );
  const [activeMotorId, setActiveMotorId] = useState<number | null>(null);
  const [status, setStatus] = useState("Idle");
  const dragRef = useRef<DragState | null>(null);
  const sendTimerRef = useRef<number | null>(null);

  useEffect(() => {
    updateMotorConfig(DEFAULT_MOTOR_CONFIG).catch((error) => {
      setStatus(`Config sync skipped: ${String(error)}`);
    });
  }, []);

  useEffect(() => {
    return () => {
      if (sendTimerRef.current !== null) {
        window.clearTimeout(sendTimerRef.current);
      }
    };
  }, []);

  const nodePositions = useMemo(
    () =>
      MOTOR_NODES.map((node) => {
        const logical = logicalAngles[node.id] ?? 0;
        const dy = -(logical / LOGICAL_RANGE_DEG) * DRAG_RANGE_PX;
        return {
          ...node,
          logical,
          renderedX: node.x,
          renderedY: node.y + dy,
        };
      }),
    [logicalAngles],
  );

  async function handleConnect() {
    try {
      const result = await connectPi(endpoint);
      setConnection(result.connected ? `Connected: ${result.endpoint}` : "Disconnected");
      setStatus("Pi endpoint connected");
    } catch (error) {
      setConnection("Connect failed");
      setStatus(String(error));
    }
  }

  async function handlePing() {
    try {
      const result = await pingPi();
      setStatus(`Ping ok: ${JSON.stringify(result)}`);
    } catch (error) {
      setStatus(String(error));
    }
  }

  function queueMotorSend(motorId: number, logicalAngle: number) {
    if (sendTimerRef.current !== null) {
      window.clearTimeout(sendTimerRef.current);
    }
    sendTimerRef.current = window.setTimeout(async () => {
      try {
        const ack = await setSingleMotor(motorId, logicalAngle);
        setStatus(`Frame ${ack.frame_id} -> motor ${motorId} @ ${logicalAngle.toFixed(1)} deg`);
      } catch (error) {
        setStatus(String(error));
      }
    }, 16);
  }

  function beginDrag(motorId: number, pointerId: number, clientY: number) {
    dragRef.current = {
      motorId,
      pointerId,
      startY: clientY,
      startLogical: logicalAngles[motorId] ?? 0,
    };
    setActiveMotorId(motorId);
  }

  function updateDrag(clientY: number) {
    const drag = dragRef.current;
    if (!drag) {
      return;
    }

    const deltaY = drag.startY - clientY;
    const deltaLogical = (deltaY / DRAG_RANGE_PX) * LOGICAL_RANGE_DEG;
    const nextLogical = clamp(drag.startLogical + deltaLogical, -LOGICAL_RANGE_DEG, LOGICAL_RANGE_DEG);

    setLogicalAngles((current) => {
      const next = [...current];
      next[drag.motorId] = Number(nextLogical.toFixed(2));
      return next;
    });
    queueMotorSend(drag.motorId, nextLogical);
  }

  function endDrag(pointerId: number) {
    if (dragRef.current?.pointerId === pointerId) {
      dragRef.current = null;
      setActiveMotorId(null);
    }
  }

  async function handleBlendshapeChange(name: string, value: number) {
    const nextBlendshapes = { ...blendshapes, [name]: value };
    setBlendshapes(nextBlendshapes);
    const coefficients = BLENDSHAPE_NAMES.map((key) => nextBlendshapes[key]);
    const previewAngles = BLENDSHAPE_MAPPING.map((row) =>
      row.reduce((sum, weight, index) => sum + weight * coefficients[index], 0),
    );
    setLogicalAngles(previewAngles);

    try {
      const ack = await sendBlendshapeFrame(
        [...BLENDSHAPE_NAMES],
        coefficients,
        BLENDSHAPE_MAPPING,
      );
      setStatus(`Blendshape frame ${ack.frame_id} dispatched`);
    } catch (error) {
      setStatus(String(error));
    }
  }

  return (
    <main className="app-shell">
      <section className="hero-card">
        <div>
          <p className="eyebrow">BionicFace Panel</p>
          <h1>2D motor topology and live facial drive testbed</h1>
          <p className="lede">
            Drag any anchor point for logical-angle micro tuning. Blendshape sliders are routed
            through the Rust matrix layer and logged with nanosecond timestamps before dispatch.
          </p>
        </div>
        <div className="hero-actions">
          <label className="endpoint-field">
            <span>Pi Endpoint</span>
            <input value={endpoint} onChange={(event) => setEndpoint(event.target.value)} />
          </label>
          <div className="button-row">
            <button onClick={handleConnect}>Connect</button>
            <button className="secondary" onClick={handlePing}>
              Ping
            </button>
          </div>
          <p className="status-line">{connection}</p>
          <p className="status-line muted">{status}</p>
        </div>
      </section>

      <section className="workspace-grid">
        <article className="panel topology-panel">
          <div className="panel-header">
            <div>
              <p className="panel-kicker">Topology</p>
              <h2>32-motor facial anchor map</h2>
            </div>
            <p className="panel-note">Vertical drag range ±45 logical degrees</p>
          </div>

          <svg
            className="topology-canvas"
            viewBox={`0 0 ${CANVAS_WIDTH} ${CANVAS_HEIGHT}`}
            onPointerMove={(event) => updateDrag(event.clientY)}
            onPointerUp={(event) => endDrag(event.pointerId)}
            onPointerLeave={(event) => endDrag(event.pointerId)}
          >
            <defs>
              <linearGradient id="faceGradient" x1="0%" x2="100%" y1="0%" y2="100%">
                <stop offset="0%" stopColor="#1a2940" />
                <stop offset="100%" stopColor="#09111d" />
              </linearGradient>
              <radialGradient id="signalGlow" cx="50%" cy="30%" r="70%">
                <stop offset="0%" stopColor="rgba(56,189,248,0.45)" />
                <stop offset="100%" stopColor="rgba(56,189,248,0)" />
              </radialGradient>
            </defs>

            <rect x="0" y="0" width={CANVAS_WIDTH} height={CANVAS_HEIGHT} rx="28" fill="url(#faceGradient)" />
            <ellipse cx="320" cy="320" rx="210" ry="255" fill="url(#signalGlow)" />
            <path
              d="M220 205 C255 180 285 180 320 205 C355 180 385 180 420 205"
              className="face-line"
            />
            <path d="M280 275 C300 265 340 265 360 275" className="face-line" />
            <path d="M250 360 C290 340 350 340 390 360" className="face-line" />
            <path d="M265 430 C300 455 340 455 375 430" className="face-line" />
            <path d="M320 290 L320 510" className="face-axis" />

            {MOTOR_NODES.map((node) => (
              <line
                key={`guide-${node.id}`}
                x1={node.x}
                y1={node.y - DRAG_RANGE_PX}
                x2={node.x}
                y2={node.y + DRAG_RANGE_PX}
                className="guide-line"
              />
            ))}

            {nodePositions.map((node) => (
              <g key={node.id}>
                <line x1={320} y1={320} x2={node.renderedX} y2={node.renderedY} className="tendon-line" />
                <circle
                  cx={node.renderedX}
                  cy={node.renderedY}
                  r={activeMotorId === node.id ? 17 : 13}
                  fill={node.color}
                  className="motor-node"
                  onPointerDown={(event) => {
                    beginDrag(node.id, event.pointerId, event.clientY);
                    event.currentTarget.setPointerCapture(event.pointerId);
                  }}
                />
                <text x={node.renderedX} y={node.renderedY - 22} textAnchor="middle" className="node-label">
                  {node.id}
                </text>
              </g>
            ))}
          </svg>

          <div className="motor-readout">
            {nodePositions.map((node) => (
              <div
                key={`readout-${node.id}`}
                className={activeMotorId === node.id ? "readout-chip active" : "readout-chip"}
              >
                <span>{node.name}</span>
                <strong>{node.logical.toFixed(1)}°</strong>
              </div>
            ))}
          </div>
        </article>

        <article className="panel blendshape-panel">
          <div className="panel-header">
            <div>
              <p className="panel-kicker">Blendshape</p>
              <h2>Realtime coefficient test panel</h2>
            </div>
            <p className="panel-note">Mapped in Rust before ZMQ dispatch</p>
          </div>

          <div className="slider-stack">
            {BLENDSHAPE_NAMES.map((name) => (
              <label className="slider-row" key={name}>
                <span>{name}</span>
                <input
                  type="range"
                  min={0}
                  max={1}
                  step={0.01}
                  value={blendshapes[name]}
                  onChange={(event) => handleBlendshapeChange(name, Number(event.target.value))}
                />
                <strong>{blendshapes[name].toFixed(2)}</strong>
              </label>
            ))}
          </div>
        </article>
      </section>
    </main>
  );
}

export default App;
