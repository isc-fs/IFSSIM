/**
 * The track spec the editor edits and the backend builds (tools/envgen/spec.py
 * is the source of truth for the format).
 */

export type XY = [number, number]

export interface Section {
  from: number; to: number
  width?: number; spacing?: number; drop?: number
  side?: 'both' | 'left' | 'right'
  recolor?: 'swap' | 'blue' | 'yellow' | 'small_orange'
  jitter?: number; note?: string
}

export interface TrackSpec {
  format: 'ifssim-trackspec/1'
  closed: boolean
  points: XY[]
  width: number
  spacing: { straight: number; corner_min: number }
  seed: number
  sections?: Section[]
}

export const EMPTY_SPEC: TrackSpec = {
  format: 'ifssim-trackspec/1', closed: true, points: [], width: 3.5,
  spacing: { straight: 5, corner_min: 1.5 }, seed: 1,
}
