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
