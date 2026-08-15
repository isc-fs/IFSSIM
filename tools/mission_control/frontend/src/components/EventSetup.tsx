import { useState, useEffect, useRef } from 'react'
import { apiFetch, promptForApiKey, readJsonResponse } from '../lib/api'
import type { TelemetryData } from '../hooks/useWebSocket'
import { useConfirm } from './ConfirmDialog'

export type MissionInfo = {
  name: string
  label: string
  kind: 'pipeline' | 'sim_benchmark_control'
  mission_id: number | null
  sim_event_type: string
  supports_laps: boolean
  description: string
}

const LS_RECORD_BAG = 'ifssim.mc.record_bag'

function loadRecordBagPref(): boolean {
  try {
    return localStorage.getItem(LS_RECORD_BAG) === '1'
  } catch {
    return false
  }
}

function saveRecordBagPref(v: boolean) {
  try {
    localStorage.setItem(LS_RECORD_BAG, v ? '1' : '0')
  } catch {
    /* best-effort */
  }
}

function missionFromSimEvent(ev: string, missions: MissionInfo[]): string {
  const hit = missions.find(m => m.sim_event_type === ev)
  return hit?.name ?? ev
}

export default function EventSetup({ telemetry }: { telemetry: TelemetryData }) {
  const [missions, setMissions] = useState<MissionInfo[]>([])
  const [mission, setMission] = useState('trackdrive')
  const [laps, setLaps] = useState(10)
  const [msg, setMsg] = useState('')
  const [busy, setBusy] = useState(false)
  const [recordBag, setRecordBag] = useState<boolean>(loadRecordBagPref)
  const confirm = useConfirm()
  const initialSyncDoneRef = useRef(false)

  useEffect(() => {
    let cancelled = false
    ;(async () => {
      try {
        const res = await apiFetch('/api/pipeline/missions')
        const data = await readJsonResponse<{ missions: MissionInfo[] }>(res)
        if (cancelled || !data.missions?.length) return
        setMissions(data.missions)
        setMission(prev => (
          data.missions.some(m => m.name === prev) ? prev : data.missions[0].name
        ))
      } catch {
        /* keep defaults until backend is up */
      }
    })()
    return () => { cancelled = true }
  }, [])

  useEffect(() => {
    if (initialSyncDoneRef.current || missions.length === 0) return
    if (telemetry.event && telemetry.event !== 'unknown') {
      setMission(missionFromSimEvent(telemetry.event, missions))
      initialSyncDoneRef.current = true
    }
  }, [telemetry.event, missions])

  const selected = missions.find(m => m.name === mission)

  type ApiResponse = {
    ok?: boolean
    error?: string
    mission?: string
    event?: string
    laps?: number
    mode?: string
    bag?: { error?: string; name?: string }
  }
  const api = async <T = ApiResponse>(
    url: string,
    body?: Record<string, unknown>,
  ): Promise<T> => {
    const res = await apiFetch(url, {
      method: 'POST',
      headers: body ? { 'Content-Type': 'application/json' } : {},
      body: body ? JSON.stringify(body) : undefined,
    }, promptForApiKey)
    return readJsonResponse<T>(res)
  }

  return (
    <div className="space-y-6">
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">
          Mission / mode
        </h2>
        {missions.length === 0 ? (
          <p className="text-sm text-gray-500 mb-4">Loading missions from pipeline registry…</p>
        ) : (
          <div className="flex flex-wrap gap-3 mb-4">
            {missions.map(m => (
              <button
                key={m.name}
                type="button"
                onClick={() => setMission(m.name)}
                title={m.description || m.name}
                className={`px-4 py-2 rounded-lg text-sm font-medium transition-all ${
                  mission === m.name
                    ? 'bg-[#ffb81c] text-black'
                    : 'bg-[#222] text-gray-400 hover:bg-[#333] border border-[#444]'
                }`}
              >
                {m.label}
                {m.kind === 'sim_benchmark_control' && (
                  <span className="ml-1 text-[10px] opacity-70">(sim)</span>
                )}
              </button>
            ))}
          </div>
        )}

        {selected?.description && (
          <p className="text-xs text-gray-500 mb-4">{selected.description}</p>
        )}

        {selected?.supports_laps && (
          <div className="flex items-center gap-3 mb-4">
            <label className="text-sm text-gray-400">Laps:</label>
            <input
              type="number" value={laps} onChange={e => setLaps(+e.target.value)}
              className="w-20 px-3 py-1.5 bg-[#111] border border-[#444] rounded text-white text-sm"
              min={1} max={100}
            />
          </div>
        )}

        <label className="flex items-center gap-2 mb-3 text-sm text-gray-300 select-none cursor-pointer">
          <input
            type="checkbox"
            checked={recordBag}
            disabled={busy || selected?.kind === 'sim_benchmark_control'}
            onChange={e => { setRecordBag(e.target.checked); saveRecordBagPref(e.target.checked) }}
            className="w-4 h-4 accent-[#ffb81c] disabled:opacity-50"
          />
          <span>Record bag (mcap)</span>
          <span className="text-gray-500 text-xs">
            {selected?.kind === 'sim_benchmark_control'
              ? '— not used for benchmark control (use capture_benchmark_bag.py)'
              : '— full topic dump; auto-pulled onto host bags/ when the session stops'}
          </span>
        </label>

        <div className="flex gap-3 flex-wrap items-center">
          <button
            disabled={busy || missions.length === 0}
            onClick={async () => {
              setBusy(true)
              setMsg('Starting…')
              try {
                const r = await api('/api/event/start', {
                  mission,
                  num_laps: laps,
                  record_bag: recordBag,
                })
                if (r.ok) {
                  let m = `Session started: ${r.mission ?? mission}`
                  if (r.laps != null) m += ` (${r.laps} laps)`
                  if (r.mode === 'benchmark_control') {
                    m += ' — benchmark control (GT driver + metrics)'
                  }
                  if (r.bag?.error) {
                    m += ` — recording failed: ${r.bag.error}`
                  } else if (r.bag?.name) {
                    m += ` — recording: ${r.bag.name}`
                  }
                  setMsg(m)
                } else {
                  setMsg(`Error: ${r.error ?? 'unknown'}`)
                }
              } catch (e) {
                setMsg(`Error: ${e instanceof Error ? e.message : String(e)}`)
              } finally {
                setBusy(false)
              }
            }}
            className="px-6 py-2.5 bg-green-700 text-white font-bold rounded-lg text-sm hover:bg-green-600 transition-all
                       shadow-[0_0_10px_rgba(34,197,94,0.3)] disabled:opacity-50 disabled:cursor-not-allowed
                       disabled:hover:bg-green-700"
          >
            {busy ? 'Working…' : 'Start Session'}
          </button>
          <button
            disabled={busy}
            onClick={async () => {
              const ok = await confirm({
                title: 'Stop autonomy pipeline?',
                message:
                  'This disables the autonomous driver (or stops benchmark control). The sim keeps running.\n\n' +
                  'For an emergency stop, use the RES button in the header instead.',
                confirmLabel: 'Stop pipeline',
                destructive: true,
              })
              if (!ok) return
              setBusy(true)
              setMsg('Stopping pipeline…')
              try {
                const r = await api('/api/pipeline/stop')
                setMsg(r.ok ? 'Session stopped' : `Error: ${r.error ?? 'unknown'}`)
              } catch (e) {
                setMsg(`Error: ${e instanceof Error ? e.message : String(e)}`)
              } finally {
                setBusy(false)
              }
            }}
            className="px-6 py-2.5 bg-red-700 text-white font-bold rounded-lg text-sm hover:bg-red-600 transition-all
                       shadow-[0_0_10px_rgba(239,68,68,0.3)] disabled:opacity-50 disabled:cursor-not-allowed
                       disabled:hover:bg-red-700"
          >
            {busy ? 'Working…' : 'Stop Session'}
          </button>
          {telemetry.pipeline_enabled && (
            <span className="text-green-400 text-xs font-medium animate-pulse">● PIPELINE RUNNING</span>
          )}
          {telemetry.bag_state === 'recording' && (
            <span className="text-red-400 text-xs font-medium animate-pulse">
              ● RECORDING{telemetry.bag_name ? ` — ${telemetry.bag_name}` : ''}
            </span>
          )}
          {telemetry.bag_state === 'starting' && (
            <span className="text-amber-400 text-xs font-medium">◌ starting recorder…</span>
          )}
          {telemetry.bag_state === 'stopped' && telemetry.bag_name && telemetry.bag_host_path && (
            <span className="text-green-400 text-xs" title={`Bag on host:\nbags/${telemetry.bag_name}/`}>
              ✓ bag saved to host: <code className="font-mono">bags/{telemetry.bag_name}/</code>
            </span>
          )}
          {telemetry.bag_state === 'stopped' && telemetry.bag_name && !telemetry.bag_host_path && (
            <span className="text-gray-400 text-xs">
              ◍ bag saved (in container): {telemetry.bag_name}
            </span>
          )}
          {telemetry.bag_state === 'failed' && (
            <span className="text-red-500 text-xs font-medium">✗ recording failed (see logs)</span>
          )}
        </div>

        {msg && <p className="mt-3 text-xs text-gray-400">{msg}</p>}
      </div>

      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Simulation Control</h2>
        <div className="flex gap-3">
          <button onClick={() => api('/api/sim/pause')}
            className="px-4 py-2 bg-yellow-700 text-white rounded-lg text-sm font-medium hover:bg-yellow-600">
            Pause
          </button>
          <button onClick={() => api('/api/sim/resume')}
            className="px-4 py-2 bg-green-700 text-white rounded-lg text-sm font-medium hover:bg-green-600">
            Resume
          </button>
          <button
            onClick={async () => {
              const ok = await confirm({
                title: 'Reset simulation?',
                message:
                  'Teleports the vehicle back to the start and resets lap counters.\n\n' +
                  'The autonomy pipeline keeps its current state — stop the session separately if needed.',
                confirmLabel: 'Reset',
                destructive: true,
              })
              if (ok) api('/api/sim/reset')
            }}
            className="px-4 py-2 bg-red-800 text-white rounded-lg text-sm font-medium hover:bg-red-700">
            Reset
          </button>
        </div>
      </div>

      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Live Event State</h2>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm">
          <Stat label="Sim event" value={telemetry.event} />
          <Stat label="Selected mode" value={mission} />
          <Stat label="Laps" value={`${telemetry.laps}/${telemetry.required_laps}`} />
          <Stat label="DOO" value={telemetry.doo} warn={telemetry.doo > 0} />
          <Stat label="OC" value={telemetry.oc} warn={telemetry.oc > 0} />
          <Stat label="Speed" value={`${telemetry.speed.toFixed(1)} m/s`} />
          <Stat label="Finished" value={telemetry.finished ? 'YES' : 'No'} good={telemetry.finished} />
          <Stat label="Paused" value={telemetry.paused ? 'YES' : 'No'} />
          <Stat label="FPS" value={telemetry.fps.toFixed(0)} />
        </div>
      </div>
    </div>
  )
}

function Stat({ label, value, warn, good }: { label: string; value: string | number | boolean; warn?: boolean; good?: boolean }) {
  return (
    <div className="bg-[#111] rounded-lg px-4 py-3">
      <div className="text-gray-500 text-xs uppercase mb-1">{label}</div>
      <div className={`text-lg font-semibold ${warn ? 'text-orange-400' : good ? 'text-green-400' : 'text-white'}`}>
        {String(value)}
      </div>
    </div>
  )
}
