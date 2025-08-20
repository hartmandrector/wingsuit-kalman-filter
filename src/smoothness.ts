import type { Vector3 } from './types.js'
import { sub, magnitude, div } from './vector.js'

export function calculateJerk(velocities: Vector3[], times: number[]): number[] {
  const jerks: number[] = []
  if (velocities.length < 3) return jerks
  
  for (let i = 1; i < velocities.length - 1; i++) {
    const dt1 = (times[i] - times[i-1]) / 1000 // Convert ms to seconds
    const dt2 = (times[i+1] - times[i]) / 1000
    
    if (dt1 > 0 && dt2 > 0) {
      const acc1 = div(sub(velocities[i], velocities[i-1]), dt1)
      const acc2 = div(sub(velocities[i+1], velocities[i]), dt2)
      const jerk = magnitude(div(sub(acc2, acc1), (dt1 + dt2) / 2))
      jerks.push(jerk)
    }
  }
  return jerks
}

export function calculateCurvature(positions: Vector3[], velocities: Vector3[]): number[] {
  const curvatures: number[] = []
  if (positions.length < 2) return curvatures
  
  for (let i = 0; i < velocities.length; i++) {
    const speed = magnitude(velocities[i])
    if (speed > 0.1) { // Minimum speed threshold to avoid division by zero
      if (i > 0 && i < velocities.length - 1) {
        const v1 = velocities[i-1]
        const v2 = velocities[i+1]
        const dv = sub(v2, v1)
        const curvature = magnitude(dv) / (speed * speed)
        curvatures.push(curvature)
      }
    }
  }
  return curvatures
}

export function calculateVelocityChangeRate(velocities: Vector3[], times: number[]): number[] {
  const changeRates: number[] = []
  if (velocities.length < 2) return changeRates
  
  for (let i = 1; i < velocities.length; i++) {
    const dt = (times[i] - times[i-1]) / 1000 // Convert ms to seconds
    if (dt > 0) {
      const dv = sub(velocities[i], velocities[i-1])
      const changeRate = magnitude(dv) / dt
      changeRates.push(changeRate)
    }
  }
  return changeRates
}