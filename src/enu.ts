import type { MLocation } from './types.js'
import type { LatLonAlt, Vector3 } from './types.js'

let referencePoint: LatLonAlt | undefined

export function setReference(latLngAlt: LatLonAlt): void {
  referencePoint = latLngAlt
}

export function getReference(): LatLonAlt | undefined {
  return referencePoint
}

export function resetReference(): void {
  referencePoint = undefined
}

/**
 * Convert GPS sample to ENU coordinates relative to reference point
 */
export function gpsToENU(gps: MLocation): Vector3 {
  if (!referencePoint) {
    throw new Error('Reference point not set')
  }
  
  const R = 6378137 // Earth radius in meters
  const cosLat = Math.cos(referencePoint.lat * Math.PI / 180)
  
  const dLat = (gps.lat - referencePoint.lat) * Math.PI / 180
  const dLon = (gps.lng - referencePoint.lng) * Math.PI / 180
  const alt = gps.alt
  const dAlt = alt - referencePoint.alt
  
  const x = dLon * R * cosLat // East
  const y = dAlt // Up
  const z = dLat * R // North
  
  return { x, y, z }
}

/**
 * Convert LatLonAlt to ENU coordinates relative to reference point
 */
export function latLonAltToENU(latLngAlt: LatLonAlt): Vector3 {
  if (!referencePoint) {
    throw new Error('Reference point not set')
  }
  
  const R = 6378137 // Earth radius in meters
  const cosLat = Math.cos(referencePoint.lat * Math.PI / 180)
  
  const dLat = (latLngAlt.lat - referencePoint.lat) * Math.PI / 180
  const dLon = (latLngAlt.lng - referencePoint.lng) * Math.PI / 180
  const dAlt = latLngAlt.alt - referencePoint.alt
  
  const x = dLon * R * cosLat // East
  const y = dAlt // Up
  const z = dLat * R // North
  
  return { x, y, z }
}

/**
 * Convert ENU coordinates back to lat/lon/alt
 */
export function ENUToLatLonAlt(x: number, y: number, z: number): LatLonAlt {
  if (!referencePoint) {
    throw new Error('Reference point not set')
  }
  
  const R = 6378137 // Earth radius in meters
  const cosLat = Math.cos(referencePoint.lat * Math.PI / 180)
  
  const dLon = x / (R * cosLat)
  const dAlt = y
  const dLat = z / R
  
  const lat = referencePoint.lat + (dLat * 180 / Math.PI)
  const lng = referencePoint.lng + (dLon * 180 / Math.PI)
  const alt = referencePoint.alt + dAlt
  
  return { lat, lng, alt }
}

/**
 * Convert ENU Vector3 back to LatLonAlt format
 */
export function ENUToLatLngAlt(enu: Vector3): LatLonAlt {
  return ENUToLatLonAlt(enu.x, enu.y, enu.z)
}