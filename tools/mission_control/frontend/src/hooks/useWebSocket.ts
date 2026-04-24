import { useState, useEffect, useRef, useCallback } from 'react';

export interface TelemetryData {
  speed: number;
  rpm: number;
  gear: number;
  x: number;
  y: number;
  z: number;
  throttle: number;
  steering: number;
  brake: number;
  // Regen telemetry (motor-side, pre-gearbox)
  regen_torque: number;         // Nm currently absorbed
  regen_power: number;          // W currently absorbed
  regen_avail_torque: number;   // Nm cap at the current motor ω
  regen_max_torque: number;     // Hardware motor-torque ceiling (from settings)
  regen_max_power: number;      // Hardware cell-input power ceiling (from settings)
  doo: number;
  oc: number;
  laps: number;
  required_laps: number;
  finished: boolean;
  event: string;
  fps: number;
  paused: boolean;
  res_active: boolean;
  pipeline_enabled: boolean;
  error?: string;
}

const defaultTelemetry: TelemetryData = {
  speed: 0, rpm: 0, gear: 0,
  x: 0, y: 0, z: 0,
  throttle: 0, steering: 0, brake: 0,
  regen_torque: 0, regen_power: 0, regen_avail_torque: 0,
  regen_max_torque: 0, regen_max_power: 0,
  doo: 0, oc: 0, laps: 0, required_laps: 0,
  finished: false, event: 'unknown',
  fps: 0, paused: false, res_active: false, pipeline_enabled: false,
};

export function useWebSocket(url: string) {
  const [data, setData] = useState<TelemetryData>(defaultTelemetry);
  const [connected, setConnected] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);

  const connect = useCallback(() => {
    try {
      const ws = new WebSocket(url);
      wsRef.current = ws;

      ws.onopen = () => setConnected(true);
      ws.onclose = () => {
        setConnected(false);
        setTimeout(connect, 2000); // Reconnect
      };
      ws.onerror = () => ws.close();
      ws.onmessage = (e) => {
        try {
          const parsed = JSON.parse(e.data);
          if (!parsed.error) setData(parsed);
        } catch {}
      };
    } catch {
      setTimeout(connect, 2000);
    }
  }, [url]);

  useEffect(() => {
    connect();
    return () => {
      wsRef.current?.close();
    };
  }, [connect]);

  return { data, connected };
}
