import { useEffect, useMemo, useRef, useState } from 'react'
import type { PointerEvent as ReactPointerEvent, WheelEvent as ReactWheelEvent } from 'react'
import { apiFetch, promptForApiKey, readJsonResponse } from '../lib/api'
import { useConfirm } from './ConfirmDialog'
import { EMPTY_SPEC } from '../lib/trackSpec'
import type { Section, TrackSpec, XY } from '../lib/trackSpec'

/*
 * Track editor: draw a path, get a track.
 *
 * The geometry is the backend's (tools/envgen via /api/track/preview_spec);
 * this component only edits the spec and draws what comes back. Coordinates
 * are metres in the track CSV's frame (+x right, +y up on screen, the car
 * drives from point 1 toward point 2). The backend recentres the cones on the
 * origin; the editor shifts them back by `offset` so the drawn points stay put.
 *
 * Mouse: in Click mode a click adds a point (after the selected one, if any),
 * a drag on empty space pans, a drag on a point moves it. In Freehand mode a
 * drag draws the whole path, simplified to control points. Wheel zooms.
 * Delete removes the selected point; Escape deselects.
 */

type Side = NonNullable<Section['side']>

interface Issue { code: string; severity: 'error' | 'warn'; message: string; s?: number; xy?: XY }
interface Preview {
  ok: boolean; length_m: number; cones: { type: string; x: number; y: number }[]
  by_type: Record<string, number>; offset: XY; issues: Issue[]
  centreline: XY[]; left: XY[]; right: XY[]
  start: { spawn: XY; forward: XY } | null
  error?: string
}
interface CreateResponse extends Partial<Preview> { name?: string; error?: string }

const CONE_COLOUR: Record<string, string> = {
  blue: '#3b82f6', yellow: '#facc15', big_orange: '#f97316', small_orange: '#fdba74',
}

/** Ramer-Douglas-Peucker: a freehand stroke down to its control points. */
function simplify(pts: XY[], tol: number): XY[] {
  if (pts.length < 3) return pts
  const [a, b] = [pts[0], pts[pts.length - 1]]
  const dx = b[0] - a[0], dy = b[1] - a[1]
  const len = Math.hypot(dx, dy) || 1e-9
  let worst = 0, idx = 0
  for (let i = 1; i < pts.length - 1; i++) {
    const d = Math.abs(dy * pts[i][0] - dx * pts[i][1] + b[0] * a[1] - b[1] * a[0]) / len
    if (d > worst) { worst = d; idx = i }
  }
  if (worst <= tol) return [a, b]
  return [...simplify(pts.slice(0, idx + 1), tol).slice(0, -1), ...simplify(pts.slice(idx), tol)]
}

const round1 = (v: number) => Math.round(v * 10) / 10

/** Sections are by point number: keep them on the same stretch when a point
 *  is inserted (delta +1) or deleted (delta -1) at index `at`. */
function shiftSections(sections: Section[] | undefined, at: number, delta: 1 | -1): Section[] | undefined {
  if (!sections) return sections
  const moved = sections
    .map(sec => ({
      ...sec,
      from: sec.from > at || (delta > 0 && sec.from === at) ? sec.from + delta : sec.from,
      to: sec.to > at || (delta > 0 && sec.to === at) ? sec.to + delta : sec.to,
    }))
    .filter(sec => sec.to !== sec.from)
  return moved.length ? moved : undefined
}

interface Props {
  initialName?: string
  initialSpec?: TrackSpec
  onSaved?: (name: string) => void
}

export default function TrackEditor({ initialName = '', initialSpec = EMPTY_SPEC, onSaved }: Props) {
  const [spec, setSpec] = useState<TrackSpec>(initialSpec)
  const [name, setName] = useState(initialName.replace(/\.csv$/, ''))
  const [mode, setMode] = useState<'click' | 'freehand'>('click')
  const [selected, setSelected] = useState<number | null>(null)
  const [preview, setPreview] = useState<Preview | null>(null)
  const [previewError, setPreviewError] = useState('')
  const [msg, setMsg] = useState('')
  const [saving, setSaving] = useState(false)
  const [view, setView] = useState({ cx: 0, cy: 0, half: 70 })   // metres
  const [stroke, setStroke] = useState<XY[] | null>(null)
  const svgRef = useRef<SVGSVGElement>(null)
  const drag = useRef<{ kind: 'point' | 'pan' | 'pending'; index?: number; start: XY; view?: typeof view } | null>(null)
  const confirm = useConfirm()

  const enough = spec.points.length >= (spec.closed ? 3 : 2)

  // Live preview, debounced; a stale reply never overwrites a newer one.
  useEffect(() => {
    if (!enough) return
    const ctrl = new AbortController()
    const t = setTimeout(async () => {
      try {
        const r = await fetch('/api/track/preview_spec', {
          method: 'POST', signal: ctrl.signal,
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ spec }),
        })
        const d = await readJsonResponse<Preview>(r)
        if (d.error) { setPreviewError(d.error); setPreview(null) }
        else { setPreviewError(''); setPreview(d) }
      } catch (e) {
        if (!ctrl.signal.aborted) setPreviewError(e instanceof Error ? e.message : String(e))
      }
    }, 250)
    return () => { clearTimeout(t); ctrl.abort() }
  }, [spec, enough])

  const shown = enough ? preview : null
  const off: XY = shown?.offset ?? [0, 0]
  const un = (p: XY): XY => [p[0] - off[0], p[1] - off[1]]

  // ---- pointer handling ----------------------------------------------------
  const toWorld = (e: { clientX: number; clientY: number }): XY => {
    const svg = svgRef.current
    if (!svg) return [0, 0]
    const pt = svg.createSVGPoint()
    pt.x = e.clientX; pt.y = e.clientY
    const m = svg.getScreenCTM()
    if (!m) return [0, 0]
    const w = pt.matrixTransform(m.inverse())
    return [round1(w.x), round1(-w.y)]
  }

  const setPoints = (points: XY[]) => setSpec(s => ({ ...s, points }))

  const onPointerDown = (e: ReactPointerEvent<SVGSVGElement>) => {
    const target = e.target as Element
    const idxAttr = target.getAttribute('data-point')
    const w = toWorld(e)
    svgRef.current?.setPointerCapture(e.pointerId)
    if (idxAttr !== null) {
      const i = Number(idxAttr)
      setSelected(i)
      drag.current = { kind: 'point', index: i, start: w }
      return
    }
    if (mode === 'freehand') {
      setStroke([w])
      return
    }
    drag.current = { kind: 'pending', start: [e.clientX, e.clientY], view }
  }

  const onPointerMove = (e: ReactPointerEvent<SVGSVGElement>) => {
    if (stroke) {
      const w = toWorld(e)
      const last = stroke[stroke.length - 1]
      if (Math.hypot(w[0] - last[0], w[1] - last[1]) > 0.5) setStroke([...stroke, w])
      return
    }
    const d = drag.current
    if (!d) return
    if (d.kind === 'point' && d.index !== undefined) {
      const w = toWorld(e)
      setPoints(spec.points.map((p, i) => (i === d.index ? w : p)))
    } else if ((d.kind === 'pending' || d.kind === 'pan') && d.view) {
      const dxPx = e.clientX - d.start[0], dyPx = e.clientY - d.start[1]
      if (d.kind === 'pending' && Math.hypot(dxPx, dyPx) < 4) return
      d.kind = 'pan'
      const svg = svgRef.current
      const px = svg ? svg.clientWidth : 600
      const mPerPx = (2 * d.view.half) / px
      setView({ ...d.view, cx: d.view.cx - dxPx * mPerPx, cy: d.view.cy + dyPx * mPerPx })
    }
  }

  const onPointerUp = (e: ReactPointerEvent<SVGSVGElement>) => {
    if (stroke) {
      // Simplify, then merge points a hand puts too close together: two
      // control points 2 m apart make a kink the spline turns into a fold.
      const pts: XY[] = []
      for (const p of simplify(stroke, 1.5)) {
        const q = pts[pts.length - 1]
        if (!q || Math.hypot(p[0] - q[0], p[1] - q[1]) >= 4) pts.push(p)
      }
      // A closed loop is drawn back to where it began, usually a little past
      // it: the loop closes itself, so drop the tail near the start.
      while (spec.closed && pts.length > 3) {
        const [a, b] = [pts[0], pts[pts.length - 1]]
        if (Math.hypot(a[0] - b[0], a[1] - b[1]) >= 15) break
        pts.pop()
      }
      // a new stroke is a new path: sections by point number no longer apply
      if (pts.length >= 2) { setSpec(s => ({ ...s, points: pts, sections: undefined })); setSelected(null) }
      setStroke(null)
      return
    }
    const d = drag.current
    drag.current = null
    if (d?.kind === 'pending') {
      const w = toWorld(e)
      const at = selected === null ? spec.points.length : selected + 1
      setSpec(s => ({
        ...s,
        points: [...s.points.slice(0, at), w, ...s.points.slice(at)],
        sections: shiftSections(s.sections, at, 1),
      }))
      setSelected(at)
    }
  }

  const onWheel = (e: ReactWheelEvent<SVGSVGElement>) => {
    const f = Math.exp(e.deltaY * 0.001)
    setView(v => ({ ...v, half: Math.min(400, Math.max(10, v.half * f)) }))
  }

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const tag = (e.target as HTMLElement | null)?.tagName
      if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return
      if ((e.key === 'Delete' || e.key === 'Backspace') && selected !== null) {
        e.preventDefault()
        setSpec(s => ({
          ...s,
          points: s.points.filter((_, i) => i !== selected),
          sections: shiftSections(s.sections, selected, -1),
        }))
        setSelected(null)
      } else if (e.key === 'Escape') {
        setSelected(null)
      }
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [selected])

  // ---- sections ---------------------------------------------------------------
  const sections = spec.sections ?? []
  const setSections = (next: Section[]) => setSpec(s => ({ ...s, sections: next.length ? next : undefined }))
  const addSection = () => {
    const n = spec.points.length
    const from = selected ?? 1
    const to = spec.closed ? Math.min(from + 2, n) : Math.min(from + 2, n - 1)
    setSections([...sections, { from, to, drop: 0.5, side: 'left' }])
  }
  const patchSection = (k: number, patch: Partial<Section>) =>
    setSections(sections.map((s, i) => {
      if (i !== k) return s
      const next: Section = { ...s, ...patch }
      // an empty field means "not set", not zero
      for (const key of Object.keys(next) as (keyof Section)[]) {
        if (next[key] === undefined || (next[key] as unknown) === '' || Number.isNaN(next[key] as number)) delete next[key]
      }
      return next
    }))

  // ---- save -----------------------------------------------------------------
  const save = async (overwrite = false): Promise<void> => {
    if (!name.trim()) { setMsg('Give the track a name first'); return }
    setSaving(true)
    try {
      const r = await apiFetch('/api/track/create', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name: name.trim(), spec, overwrite }),
      }, promptForApiKey)
      const d = await readJsonResponse<CreateResponse>(r)
      if (r.status === 409) {
        setSaving(false)
        const ok = await confirm({
          title: 'Replace track?', message: `${name.trim()}.csv already exists. Replace it with this one?`,
          confirmLabel: 'Replace', destructive: true,
        })
        if (ok) return save(true)
        return
      }
      if (d.name && r.ok) {
        setMsg(`Saved ${d.name}: ${d.cones} cones, ${d.length_m} m`)
        onSaved?.(d.name)
      } else {
        setMsg(`Not saved: ${d.error ?? r.statusText}`)
      }
    } catch (e) {
      setMsg(`Error: ${e instanceof Error ? e.message : String(e)}`)
    } finally {
      setSaving(false)
    }
  }

  const download = () => {
    const blob = new Blob([JSON.stringify(spec, null, 2) + '\n'], { type: 'application/json' })
    const a = document.createElement('a')
    a.href = URL.createObjectURL(blob)
    a.download = `${name.trim() || 'track'}.track.json`
    a.click()
    URL.revokeObjectURL(a.href)
  }

  // ---- drawing ----------------------------------------------------------------
  const vb = `${view.cx - view.half} ${-(view.cy + view.half)} ${2 * view.half} ${2 * view.half}`
  const grid = useMemo(() => {
    const step = view.half > 150 ? 50 : 10
    const lines: number[] = []
    const lo = Math.floor((Math.min(view.cx, view.cy) - view.half * 1.5) / step) * step
    const hi = Math.ceil((Math.max(view.cx, view.cy) + view.half * 1.5) / step) * step
    for (let v = lo; v <= hi; v += step) lines.push(v)
    return { step, lines }
  }, [view])
  const poly = (pts: XY[], close: boolean) =>
    pts.map(un).map(p => `${p[0]},${-p[1]}`).join(' ') + (close && pts.length ? ` ${un(pts[0])[0]},${-un(pts[0])[1]}` : '')
  const errors = shown?.issues.filter(i => i.severity === 'error') ?? []
  const warns = shown?.issues.filter(i => i.severity === 'warn') ?? []
  const r = view.half / 90   // a few pixels, in metres

  return (
    <div className="grid grid-cols-1 lg:grid-cols-[1fr_320px] gap-4">
      <div className="relative">
        <svg
          ref={svgRef} viewBox={vb} role="img" aria-label="Track editor canvas"
          className={`w-full aspect-square bg-[#0d0d0d] rounded-lg border border-[#333] touch-none select-none ${mode === 'freehand' ? 'cursor-crosshair' : 'cursor-pointer'}`}
          onPointerDown={onPointerDown} onPointerMove={onPointerMove} onPointerUp={onPointerUp} onWheel={onWheel}
        >
          {grid.lines.map(v => (
            <g key={v}>
              <line x1={v} x2={v} y1={-(view.cy + view.half * 2)} y2={-(view.cy - view.half * 2)} stroke={v === 0 ? '#3a3a3a' : '#1c1c1c'} strokeWidth={1} vectorEffect="non-scaling-stroke" />
              <line y1={-v} y2={-v} x1={view.cx - view.half * 2} x2={view.cx + view.half * 2} stroke={v === 0 ? '#3a3a3a' : '#1c1c1c'} strokeWidth={1} vectorEffect="non-scaling-stroke" />
            </g>
          ))}
          {/* customMap's floor (about 200 m x 250 m) as the safe square, centred where the track will be */}
          <rect x={-100 - off[0]} y={-100 + off[1]} width={200} height={200} fill="none" stroke="#2f4f2f" strokeDasharray="6 4" strokeWidth={1} vectorEffect="non-scaling-stroke" />

          {shown && <>
            <polyline points={poly(shown.left, spec.closed)} fill="none" stroke="#1e3a8a" strokeWidth={1} vectorEffect="non-scaling-stroke" />
            <polyline points={poly(shown.right, spec.closed)} fill="none" stroke="#713f12" strokeWidth={1} vectorEffect="non-scaling-stroke" />
            {shown.cones.map((c, i) => {
              const [x, y] = un([c.x, c.y])
              return <circle key={i} cx={x} cy={-y} r={c.type === 'big_orange' ? 0.55 : 0.4} fill={CONE_COLOUR[c.type] ?? '#888'} />
            })}
            {shown.start && (() => {
              const [sx, sy] = un(shown.start.spawn)
              const [fx, fy] = shown.start.forward
              return <line x1={sx} y1={-sy} x2={sx + fx * 6} y2={-(sy + fy * 6)} stroke="#ef4444" strokeWidth={3} vectorEffect="non-scaling-stroke" markerEnd="url(#arrow)" />
            })()}
            {shown.issues.filter(i => i.xy).map((i, k) => {
              const [x, y] = un(i.xy as XY)
              return <circle key={k} cx={x} cy={-y} r={4} fill="none" stroke={i.severity === 'error' ? '#ef4444' : '#f59e0b'} strokeWidth={2} vectorEffect="non-scaling-stroke"><title>{i.message}</title></circle>
            })}
          </>}
          <defs>
            <marker id="arrow" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse">
              <path d="M 0 0 L 10 5 L 0 10 z" fill="#ef4444" />
            </marker>
          </defs>

          {/* the drawn path and its control points */}
          {spec.points.length > 1 && (
            <polyline points={spec.points.map(p => `${p[0]},${-p[1]}`).join(' ') + (spec.closed ? ` ${spec.points[0][0]},${-spec.points[0][1]}` : '')}
              fill="none" stroke="#555" strokeDasharray="3 3" strokeWidth={1} vectorEffect="non-scaling-stroke" />
          )}
          {stroke && <polyline points={stroke.map(p => `${p[0]},${-p[1]}`).join(' ')} fill="none" stroke="#ffb81c" strokeWidth={2} vectorEffect="non-scaling-stroke" />}
          {spec.points.map((p, i) => (
            <g key={i}>
              <circle data-point={i} cx={p[0]} cy={-p[1]} r={r * 2.2}
                fill={i === selected ? '#ffb81c' : i === 0 ? '#22c55e' : '#e5e5e5'} stroke="#000" strokeWidth={1} vectorEffect="non-scaling-stroke" className="cursor-move" />
              <text x={p[0] + r * 3} y={-p[1] - r * 3} fontSize={r * 4} fill="#9ca3af" pointerEvents="none">{i}</text>
            </g>
          ))}
        </svg>
        <div className="absolute top-2 left-2 flex gap-1">
          {(['click', 'freehand'] as const).map(m => (
            <button key={m} onClick={() => setMode(m)}
              className={`px-2 py-1 rounded text-xs border ${mode === m ? 'bg-[#ffb81c] text-black border-[#ffb81c]' : 'bg-[#1a1a1a] text-gray-300 border-[#444]'}`}>
              {m === 'click' ? 'Click points' : 'Freehand'}
            </button>
          ))}
          <button onClick={() => { setSpec(s => ({ ...s, points: [], sections: undefined })); setSelected(null); setPreview(null) }} className="px-2 py-1 rounded text-xs border bg-[#1a1a1a] text-gray-300 border-[#444]">Clear</button>
        </div>
        <p className="mt-2 text-xs text-gray-500">
          {mode === 'click'
            ? 'Click to add a point (after the selected one). Drag a point to move it, drag the background to pan, scroll to zoom. Delete removes the selected point. Point 0 (green) is the start; the car drives toward point 1.'
            : 'Drag to draw the whole path in one stroke; it is simplified to control points you can then edit in Click mode.'}
        </p>
      </div>

      <div className="space-y-3 text-sm">
        <div className="flex flex-wrap gap-2 items-end">
          <Num label="Width (m)" value={spec.width} step={0.1} onChange={v => setSpec(s => ({ ...s, width: v }))} />
          <Num label="Spacing (m)" value={spec.spacing.straight} step={0.5} onChange={v => setSpec(s => ({ ...s, spacing: { ...s.spacing, straight: v } }))} />
          <Num label="Corner min (m)" value={spec.spacing.corner_min} step={0.1} onChange={v => setSpec(s => ({ ...s, spacing: { ...s.spacing, corner_min: v } }))} />
          <Num label="Seed" value={spec.seed} step={1} onChange={v => setSpec(s => ({ ...s, seed: Math.max(0, Math.round(v)) }))} />
          <label className="flex items-center gap-2 text-xs text-gray-400 uppercase tracking-wide pb-1.5">
            <input type="checkbox" checked={spec.closed} onChange={e => setSpec(s => ({ ...s, closed: e.target.checked }))} /> Closed loop
          </label>
        </div>

        <div className="bg-[#111] border border-[#333] rounded-lg p-3" aria-live="polite">
          {!enough && <p className="text-gray-500 text-xs">Add at least {spec.closed ? 3 : 2} points.</p>}
          {previewError && <p className="text-red-400 text-xs">{previewError}</p>}
          {shown && <>
            <p className="text-xs text-gray-400">
              {shown.length_m} m · <span className="text-blue-400">{shown.by_type.blue ?? 0}</span> blue ·{' '}
              <span className="text-yellow-400">{shown.by_type.yellow ?? 0}</span> yellow ·{' '}
              <span className="text-orange-400">{(shown.by_type.big_orange ?? 0) + (shown.by_type.small_orange ?? 0)}</span> orange
            </p>
            {errors.length === 0 && warns.length === 0 && <p className="text-green-400 text-xs mt-1">Meets the FS layout rules.</p>}
            <ul className="mt-1 space-y-1">
              {[...errors, ...warns].map((i, k) => (
                <li key={k} className={`text-xs ${i.severity === 'error' ? 'text-red-400' : 'text-amber-400'}`}>
                  <span className="uppercase font-semibold">{i.severity === 'error' ? 'Error' : 'Rule'}</span> {i.message}{i.s !== undefined ? ` (${i.s} m in)` : ''}
                </li>
              ))}
            </ul>
          </>}
        </div>

        <div className="bg-[#111] border border-[#333] rounded-lg p-3 space-y-2">
          <div className="flex justify-between items-center">
            <h3 className="text-[#ffb81c] text-xs uppercase tracking-wider font-semibold">Test sections</h3>
            <button onClick={addSection} disabled={spec.points.length < 3} className="px-2 py-0.5 text-xs rounded bg-[#333] hover:bg-[#444] disabled:opacity-40">Add</button>
          </div>
          {sections.length === 0 && <p className="text-xs text-gray-500">Change one stretch of track: narrower, sparser, missing or wrong-coloured cones, misplaced cones. From and to are point numbers.</p>}
          {sections.map((s, k) => (
            <div key={k} className="border border-[#2a2a2a] rounded p-2 space-y-1">
              <div className="flex gap-2 items-end flex-wrap">
                <Num label="From" value={s.from} step={1} w="w-14" onChange={v => patchSection(k, { from: Math.round(v) })} />
                <Num label="To" value={s.to} step={1} w="w-14" onChange={v => patchSection(k, { to: Math.round(v) })} />
                <Num label="Width" value={s.width} step={0.1} w="w-16" onChange={v => patchSection(k, { width: v })} />
                <Num label="Spacing" value={s.spacing} step={0.5} w="w-16" onChange={v => patchSection(k, { spacing: v })} />
                <button onClick={() => setSections(sections.filter((_, i) => i !== k))} className="ml-auto text-xs text-red-400 px-1" aria-label={`Remove section ${k + 1}`}>Remove</button>
              </div>
              <div className="flex gap-2 items-end flex-wrap">
                <Num label="Drop 0-1" value={s.drop} step={0.1} w="w-16" onChange={v => patchSection(k, { drop: v })} />
                <Num label="Jitter (m)" value={s.jitter} step={0.1} w="w-16" onChange={v => patchSection(k, { jitter: v })} />
                <Pick label="Side" value={s.side ?? 'both'} options={['both', 'left', 'right']} onChange={v => patchSection(k, { side: v as Side })} />
                <Pick label="Colours" value={s.recolor ?? ''} options={['', 'swap', 'blue', 'yellow', 'small_orange']} onChange={v => patchSection(k, { recolor: (v || undefined) as Section['recolor'] })} />
              </div>
            </div>
          ))}
        </div>

        <div className="flex gap-2 items-end">
          <label className="flex flex-col gap-1 text-xs text-gray-500 uppercase tracking-wide flex-1">
            Name
            <input value={name} onChange={e => setName(e.target.value)} placeholder="hairpin_missing_blue"
              className="px-3 py-1.5 bg-[#111] border border-[#444] rounded text-white text-sm placeholder:text-gray-600" />
          </label>
          <button onClick={() => save()} disabled={saving || !shown || errors.length > 0}
            className="px-4 py-2 bg-[#ffb81c] text-black font-bold rounded-lg text-sm hover:bg-[#e6a619] disabled:opacity-50">
            {saving ? 'Saving…' : 'Save track'}
          </button>
        </div>
        <button onClick={download} disabled={!spec.points.length} className="text-xs text-gray-400 underline disabled:opacity-40">Download spec (.track.json)</button>
        {msg && <p className="text-xs text-gray-400">{msg}</p>}
      </div>
    </div>
  )
}

function Num({ label, value, onChange, step, w = 'w-20' }: {
  label: string; value: number | undefined; onChange: (v: number) => void; step: number; w?: string
}) {
  return (
    <label className="flex flex-col gap-1 text-[10px] text-gray-500 uppercase tracking-wide">
      {label}
      <input type="number" step={step} value={value ?? ''} onChange={e => onChange(e.target.value === '' ? NaN : +e.target.value)}
        className={`${w} px-2 py-1 bg-[#111] border border-[#444] rounded text-white text-sm`} />
    </label>
  )
}

function Pick({ label, value, options, onChange }: {
  label: string; value: string; options: string[]; onChange: (v: string) => void
}) {
  return (
    <label className="flex flex-col gap-1 text-[10px] text-gray-500 uppercase tracking-wide">
      {label}
      <select value={value} onChange={e => onChange(e.target.value)} className="px-2 py-1 bg-[#111] border border-[#444] rounded text-white text-sm">
        {options.map(o => <option key={o} value={o}>{o || '—'}</option>)}
      </select>
    </label>
  )
}
