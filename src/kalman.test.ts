import { describe, it, expect } from 'vitest'
import { KalmanFilter3D } from './kalman.js'

describe('KalmanFilter3D', () => {
  describe('Kalman Filter Operations', () => {

    it('should initialize with zero state', () => {
      const kalman = new KalmanFilter3D()
      const state = kalman.getState()
      expect(state.position).toEqual({ x: 0, y: 0, z: 0 })
      expect(state.velocity).toEqual({ x: 0, y: 0, z: 0 })
      expect(state.acceleration).toEqual({ x: 0, y: 0, z: 0 })
    })

    it('should update state with first measurement', () => {
      const kalman = new KalmanFilter3D()
      const timestamp = Date.now()
      kalman.updateWithEnu(10, 5, 20, 0, 0, 0, 1.0, 1.0, 1.0, timestamp)
      
      const state = kalman.getState()
      // With velocity measurements, should be more accurate than before
      expect(state.position.x).toBeCloseTo(10, 0)
      expect(state.position.y).toBeCloseTo(5, 0)
      expect(state.position.z).toBeCloseTo(20, 0)
      expect(state.velocity.x).toBeCloseTo(0, 0)
      expect(state.velocity.y).toBeCloseTo(0, 0)
      expect(state.velocity.z).toBeCloseTo(0, 0)
    })

    it('should predict future state', () => {
      const kalman = new KalmanFilter3D()
      const timestamp = Date.now()
      
      // First update to establish initial state
      kalman.updateWithEnu(0, 0, 0, 0, 0, 0, 1.0, 1.0, 1.0, timestamp)
      kalman.updateWithEnu(10, 0, 0, 10, 0, 0, 1.0, 1.0, 1.0, timestamp + 1000) // Moving east at ~10 m/s
      
      // Predict 1 second into the future
      const predicted = kalman.predictAt(timestamp + 2000)
      
      expect(predicted).not.toBeNull()
      expect(predicted?.position.x).toBeGreaterThan(18) // Should be close to 20m, but filter has some variance
      expect(predicted?.position.x).toBeLessThan(25) // Reasonable upper bound
      expect(predicted?.position.y).toBeCloseTo(0, 1)
      expect(predicted?.position.z).toBeCloseTo(0, 1)
      expect(predicted?.velocity.x).toBeGreaterThan(8) // Should be close to 10 m/s
    })

    it('should return undefined for prediction before first update', () => {
      const kalman = new KalmanFilter3D()
      const predicted = kalman.predictAt(Date.now())
      expect(predicted).toBeUndefined()
    })

    it('should return current state for prediction at same time', () => {
      const kalman = new KalmanFilter3D()
      const timestamp = Date.now()
      kalman.updateWithEnu(10, 5, 20, 0, 0, 0, 1.0, 1.0, 1.0, timestamp)
      
      const predicted = kalman.predictAt(timestamp)
      const current = kalman.getState()
      
      expect(predicted?.position).toEqual(current.position)
      expect(predicted?.velocity).toEqual(current.velocity)
      expect(predicted?.acceleration).toEqual(current.acceleration)
    })

    it('should handle prediction with acceleration', () => {
      const kalman = new KalmanFilter3D()
      const timestamp = Date.now()
      
      // Create a scenario with acceleration
      kalman.updateWithEnu(0, 0, 0, 0, 0, 0, 1.0, 1.0, 1.0, timestamp)
      kalman.updateWithEnu(1, 0, 0, 1, 0, 0, 1.0, 1.0, 1.0, timestamp + 1000) // 1 m/s velocity
      kalman.updateWithEnu(4, 0, 0, 3, 0, 0, 1.0, 1.0, 1.0, timestamp + 2000) // 3 m/s velocity (2 m/s^2 acceleration)
      
      // Predict 1 second into future
      const predicted = kalman.predictAt(timestamp + 3000)
      
      // Should continue accelerating beyond the 4m position
      expect(predicted?.position.x).toBeGreaterThan(6) // Should accelerate beyond 4m
      expect(predicted?.position.x).toBeLessThan(10) // Reasonable upper bound
      expect(predicted?.velocity.x).toBeGreaterThan(3) // Should have increased velocity
    })

    it('should smooth noisy measurements', () => {
      const kalman = new KalmanFilter3D()
      const timestamp = Date.now()
      
      // Add several measurements with some noise
      const measurements = [
        { x: 0, y: 0, z: 0, t: timestamp },
        { x: 10.1, y: 0.2, z: -0.1, t: timestamp + 1000 },
        { x: 19.9, y: -0.1, z: 0.1, t: timestamp + 2000 },
        { x: 30.2, y: 0.15, z: -0.05, t: timestamp + 3000 }
      ]
      
      measurements.forEach(m => kalman.updateWithEnu(m.x, m.y, m.z, 10, 0, 0, 1.0, 1.0, 1.0, m.t))
      
      const state = kalman.getState()
      
      // The filter should smooth out the noise better with velocity measurements
      expect(Math.abs(state.position.y)).toBeLessThan(0.5)
      expect(Math.abs(state.position.z)).toBeLessThan(0.5)
      expect(state.position.x).toBeCloseTo(30, 0) // Should be reasonably close to 30
      expect(state.velocity.x).toBeCloseTo(10, 0) // Should track the consistent velocity
    })
  })

})