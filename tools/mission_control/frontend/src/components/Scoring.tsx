import { useState, useEffect } from 'react'
import { apiFetch, promptForApiKey } from '../lib/api'

interface ScoringData {
  event: string;
  laps_completed: number;
  lap_times: number[];
  best_lap: number;
  total_elapsed: number;
  doo_count: number;
  doo_penalty_s: number;
  oc_count: number;
  oc_penalty_s: number;
  corrected_time: number;
  // Backend returns null until at least one lap is completed (no reference
  // time yet). Anything that calls .toFixed on this MUST null-check.
  t_best_reference: number | null;
  max_points: number;
  score: number;
}

export default function Scoring() {
  const [scoring, setScoring] = useState<ScoringData | null>(null)
  const [tBest, setTBest] = useState('')

  const refresh = async () => {
    const url = tBest ? `/api/scoring/summary?t_best=${tBest}` : '/api/scoring/summary'
    const r = await apiFetch(url, {}, promptForApiKey)
    const d = await r.json()
    if (!d.error) setScoring(d)
  }

  useEffect(() => {
    refresh()
    const interval = setInterval(refresh, 2000)
    return () => clearInterval(interval)
  }, [tBest])

  if (!scoring) return <p className="text-gray-500">Loading scoring data...</p>

  return (
    <div className="space-y-4">
      {/* Score Header */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 text-center">
        <div className="text-gray-500 text-xs uppercase tracking-wider mb-2">Event Score</div>
        <div className="text-6xl font-bold text-[#ffb81c]">{scoring.score.toFixed(1)}</div>
        <div className="text-gray-500 text-sm">/ {scoring.max_points} points</div>
        <div className="text-lg text-white capitalize mt-2">{scoring.event}</div>
      </div>

      {/* Time Breakdown */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 text-center">
          <div className="text-xs text-gray-500 uppercase mb-2">Total Elapsed</div>
          <div className="text-3xl font-bold text-white">{scoring.total_elapsed.toFixed(3)}s</div>
        </div>
        <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 text-center">
          <div className="text-xs text-gray-500 uppercase mb-2">Penalties</div>
          <div className="text-2xl font-bold text-orange-400">
            +{(scoring.doo_penalty_s + scoring.oc_penalty_s).toFixed(1)}s
          </div>
          <div className="text-xs text-gray-500 mt-1">
            DOO: +{scoring.doo_penalty_s}s ({scoring.doo_count} hits) |
            OC: +{scoring.oc_penalty_s}s ({scoring.oc_count} off-track)
          </div>
        </div>
        <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 text-center">
          <div className="text-xs text-gray-500 uppercase mb-2">Corrected Time</div>
          <div className="text-3xl font-bold text-[#ffb81c]">{scoring.corrected_time.toFixed(3)}s</div>
        </div>
      </div>

      {/* Best Reference */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Reference Time (T_best)</h2>
        <div className="flex items-center gap-3">
          <input
            type="number" step="0.1" placeholder="Auto (best lap)"
            value={tBest} onChange={e => setTBest(e.target.value)}
            className="w-40 px-3 py-1.5 bg-[#111] border border-[#444] rounded text-white text-sm"
          />
          <span className="text-xs text-gray-500">
            {tBest
              ? `Using ${tBest}s`
              : scoring.t_best_reference != null
                ? `Auto: ${scoring.t_best_reference.toFixed(3)}s (best lap)`
                : 'Auto: (waiting for first lap)'}
          </span>
        </div>
      </div>

      {/* Lap Times Table */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">
          Lap Times ({scoring.laps_completed} laps)
        </h2>
        {scoring.lap_times.length > 0 ? (
          <table className="w-full text-sm">
            <thead>
              <tr className="text-gray-500 text-xs uppercase">
                <th className="text-left pb-2">Lap</th>
                <th className="text-right pb-2">Time</th>
                <th className="text-right pb-2">Delta</th>
              </tr>
            </thead>
            <tbody>
              {scoring.lap_times.map((t, i) => (
                <tr key={i} className="border-t border-[#222]">
                  <td className="py-2 text-gray-300">{i + 1}</td>
                  <td className="py-2 text-right font-mono text-white">{t.toFixed(3)}s</td>
                  <td className={`py-2 text-right font-mono ${
                    t === scoring.best_lap ? 'text-green-400' : 'text-gray-500'
                  }`}>
                    {t === scoring.best_lap ? 'BEST' : `+${(t - scoring.best_lap).toFixed(3)}`}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        ) : (
          <p className="text-gray-500 text-sm">No laps completed yet</p>
        )}
      </div>
    </div>
  )
}
