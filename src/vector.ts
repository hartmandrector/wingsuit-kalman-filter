import type { Vector3 } from "./types.js"

export function vec(x = 0, y = 0, z = 0): Vector3 {
  return { x, y, z }
}

export function add(a: Vector3, b: Vector3): Vector3 {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }
}

export function sub(a: Vector3, b: Vector3): Vector3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }
}

export function mul(a: Vector3, s: number): Vector3 {
  return { x: a.x * s, y: a.y * s, z: a.z * s }
}

export function div(a: Vector3, s: number): Vector3 {
  if (s === 0) return vec()
  return { x: a.x / s, y: a.y / s, z: a.z / s }
}

export function degToRad(deg: number): number {
  return deg * Math.PI / 180
}

export function magnitude(v: Vector3): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
}

/**
 * Vector dot product with supplied projection function for the first arg.
 * This function exists to prevent copying data.
 * Requires m1.length == m2.length. No safety.
 */
export function dotMap<T>(m1: T[], m2: number[], proj1: (d: T) => number): number {
  // if (m1.length !== m2.length) throw new Error(`length mismatch ${m1.length} != ${m2.length}`)
  const n = m1.length
  let out = 0
  for (let i = 0; i < n; i++) {
    out += proj1(m1[i]) * m2[i]
  }
  return out
}