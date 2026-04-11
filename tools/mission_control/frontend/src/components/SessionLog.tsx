import { useState, useEffect } from 'react'

interface LogEntry {
  timestamp: string;
  type: string;
  message: string;
}

export default function SessionLog() {
  const [log, setLog] = useState<LogEntry[]>([])

  const refresh = async () => {
    const r = await fetch('/api/session/log')
    setLog(await r.json())
  }

  useEffect(() => {
    refresh()
    const interval = setInterval(refresh, 3000)
    return () => clearInterval(interval)
  }, [])

  const exportLog = () => {
    window.open('/api/session/export', '_blank')
  }

  const typeColors: Record<string, string> = {
    event_set: 'text-[#ffb81c]',
    event_start: 'text-green-400',
    res: 'text-red-400',
    reset: 'text-blue-400',
    track_load: 'text-purple-400',
    track_generate: 'text-cyan-400',
  }

  return (
    <div className="space-y-4">
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <div className="flex justify-between items-center mb-4">
          <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold">Session Log</h2>
          <div className="flex gap-2">
            <button onClick={refresh}
              className="px-3 py-1.5 bg-[#333] text-gray-300 rounded text-xs hover:bg-[#444]">
              Refresh
            </button>
            <button onClick={exportLog}
              className="px-3 py-1.5 bg-[#ffb81c] text-black font-medium rounded text-xs hover:bg-[#e6a619]">
              Export JSON
            </button>
          </div>
        </div>

        {log.length > 0 ? (
          <div className="space-y-1 max-h-[500px] overflow-y-auto">
            {[...log].reverse().map((entry, i) => (
              <div key={i} className="flex items-start gap-3 px-3 py-2 rounded hover:bg-[#111] text-sm">
                <span className="text-gray-600 font-mono text-xs whitespace-nowrap">
                  {new Date(entry.timestamp).toLocaleTimeString()}
                </span>
                <span className={`font-medium text-xs uppercase w-24 ${typeColors[entry.type] || 'text-gray-400'}`}>
                  {entry.type}
                </span>
                <span className="text-gray-300">{entry.message}</span>
              </div>
            ))}
          </div>
        ) : (
          <p className="text-gray-500 text-sm">No events logged yet. Actions will appear here.</p>
        )}
      </div>
    </div>
  )
}
