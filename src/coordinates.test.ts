import { describe, it, expect, beforeEach } from 'vitest'
import { setReference, latLonAltToENU, ENUToLatLonAlt, getReference, resetReference } from './coordinates.js'

describe('Coordinate Conversion', () => {
  beforeEach(() => {
    setReference({ lat: 37.7749, lng: -122.4194, alt: 100 }) // San Francisco
  })

  it('should set reference point correctly', () => {
    expect(getReference()).toEqual({
      lat: 37.7749,
      lng: -122.4194,
      alt: 100
    })
  })

  it('should convert reference point to origin', () => {
    const enu = latLonAltToENU({ lat: 37.7749, lng: -122.4194, alt: 100 })
    expect(enu.x).toBeCloseTo(0, 5)
    expect(enu.y).toBeCloseTo(0, 5)
    expect(enu.z).toBeCloseTo(0, 5)
  })

  it('should convert coordinates to ENU correctly', () => {
    // Point slightly north and east of reference
    const enu = latLonAltToENU({ lat: 37.7759, lng: -122.4184, alt: 110 })
    
    expect(enu.x).toBeGreaterThan(0) // East should be positive
    expect(enu.y).toBeCloseTo(10, 1) // Up should be ~10m
    expect(enu.z).toBeGreaterThan(0) // North should be positive
  })

  it('should convert ENU back to lat/lon/alt correctly', () => {
    const originalLat = 37.7759
    const originalLon = -122.4184
    const originalAlt = 110
    
    const enu = latLonAltToENU({ lat: originalLat, lng: originalLon, alt: originalAlt })
    const backToLatLon = ENUToLatLonAlt(enu.x, enu.y, enu.z)
    
    expect(backToLatLon.lat).toBeCloseTo(originalLat, 6)
    expect(backToLatLon.lng).toBeCloseTo(originalLon, 6)
    expect(backToLatLon.alt).toBeCloseTo(originalAlt, 3)
  })

  it('should throw error when reference not set', () => {
    resetReference() // Reset reference
    expect(() => latLonAltToENU({ lat: 37.7749, lng: -122.4194, alt: 100 }))
      .toThrow('Reference point not set')
    expect(() => ENUToLatLonAlt(0, 0, 0))
      .toThrow('Reference point not set')
  })
})