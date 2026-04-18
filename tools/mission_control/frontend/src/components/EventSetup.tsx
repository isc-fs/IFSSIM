import { useState, useEffect } from 'react'

const EVENTS = ['trackdrive', 'autocross', 'acceleration', 'skidpad'] as const

export default function EventSetup({ telemetry }: { telemetry: any }) {
  const [event, setEvent] = useState('trackdrive')
  const [laps, setLaps] = useState(10)
  const [msg, setMsg] = useState('')

  useEffect(() => {
    if (telemetry.event && telemetry.event !== 'unknown' && EVENTS.includes(telemetry.event as any)) {
      setEvent(telemetry.event)
    }
  }, [telemetry.event])

  const api = async (url: string, body?: any) => {
    const res = await fetch(url, {
      method: 'POST',
      headers: body ? { 'Content-Type': 'application/json' } : {},
      body: body ? JSON.stringify(body) : undefined,
    })
    return res.json()
  }

  return (
    <div className="space-y-6">
      {/* Event Selector */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Event Configuration</h2>
        <div className="flex flex-wrap gap-3 mb-4">
          {EVENTS.map(e => (
            <button
              key={e}
              onClick={() => setEvent(e)}
              className={`px-4 py-2 rounded-lg text-sm font-medium capitalize transition-all ${
                event === e
                  ? 'bg-[#ffb81c] text-black'
                  : 'bg-[#222] text-gray-400 hover:bg-[#333] border border-[#444]'
              }`}
            >
              {e}
            </button>
          ))}
        </div>

        {event === 'trackdrive' && (
          <div className="flex items-center gap-3 mb-4">
            <label className="text-sm text-gray-400">Laps:</label>
            <input
              type="number" value={laps} onChange={e => setLaps(+e.target.value)}
              className="w-20 px-3 py-1.5 bg-[#111] border border-[#444] rounded text-white text-sm"
              min={1} max={100}
            />
          </div>
        )}

        <div className="flex gap-3 flex-wrap items-center">
          <button
            onClick={async () => {
              setMsg('Starting…')
              const r = await api('/api/event/start', { event_type: event, num_laps: laps })
              setMsg(r.ok ? `Session started: ${r.event} (${r.laps} laps)` : `Error: ${r.error}`)
            }}
            className="px-6 py-2.5 bg-green-700 text-white font-bold rounded-lg text-sm hover:bg-green-600 transition-all
                       shadow-[0_0_10px_rgba(34,197,94,0.3)]"
          >
            Start Session
          </button>
          <button
            onClick={async () => {
              await api('/api/res/activate')
              setMsg('Session stopped')
            }}
            className="px-6 py-2.5 bg-red-700 text-white font-bold rounded-lg text-sm hover:bg-red-600 transition-all
                       shadow-[0_0_10px_rgba(239,68,68,0.3)]"
          >
            Stop Session
          </button>
          {telemetry.pipeline_enabled && (
            <span className="text-green-400 text-xs font-medium animate-pulse">● PIPELINE RUNNING</span>
          )}
        </div>

        {msg && <p className="mt-3 text-xs text-gray-400">{msg}</p>}
      </div>

      {/* Sim Controls */}
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
          <button onClick={() => { if (confirm('Reset simulation?')) api('/api/sim/reset') }}
            className="px-4 py-2 bg-red-800 text-white rounded-lg text-sm font-medium hover:bg-red-700">
            Reset
          </button>
        </div>
      </div>

      {/* Live State */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Live Event State</h2>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm">
          <Stat label="Event" value={telemetry.event} />
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

function Stat({ label, value, warn, good }: { label: string; value: any; warn?: boolean; good?: boolean }) {
  return (
    <div className="bg-[#111] rounded-lg px-4 py-3">
      <div className="text-gray-500 text-xs uppercase mb-1">{label}</div>
      <div className={`text-lg font-semibold ${warn ? 'text-orange-400' : good ? 'text-green-400' : 'text-white'}`}>
        {String(value)}
      </div>
    </div>
  )
}
