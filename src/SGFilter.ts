import type { Vector3, MLocation } from './types.js'
import { vec, add, sub, mul, div, degToRad } from './vector.js'
import { calculateWingsuitAcceleration, calculateWingsuitParameters } from './wse.js'

export enum CalculationMethod {
  STANDARD = 'standard'  // SGFilter only supports STANDARD method
}

export interface MotionState {
  position: Vector3
  velocity: Vector3
  acceleration: Vector3
  // Additional acceleration components for analysis
  aMeasured?: Vector3
  aWSE?: Vector3
}

export interface SGFilterOptions {
  filterLength?: number
}

interface FilteredOutput {
  time: number
  position: Vector3
  velocity: Vector3
  acceleration: Vector3
  aMeasured: Vector3
  aWSE: Vector3
  kl: number
  kd: number
  roll: number
}

/**
 * Savitzky-Golay Filter for GPS data smoothing.
 * Implements proper two-sided non-causal convolution with delay compensation.
 * The filter delay is floor(filterLength/2) * samplePeriod.
 */
export class SGFilter {
  private filterLength: number
  private calculationMethod: CalculationMethod = CalculationMethod.STANDARD
  private gpsBuffer: MLocation[] = []
  private filteredOutputs: FilteredOutput[] = [] // Store last 2 filtered outputs for interpolation
  private halfFilterLength: number
  private lastUpdateMillis: number | undefined
  private originGps: MLocation | undefined

  // SG filter coefficients - use the pre-computed ones from savitzky-golay.ts
  private cMN: number[][] = [
    [-3, 12, 17, 12, -3],  // 5-point
    [-2, 3, 6, 7, 6, 3, -2],  // 7-point
    [-21, 14, 39, 54, 59, 54, 39, 14, -21],  // 9-point
    [-36, 9, 44, 69, 84, 89, 84, 69, 44, 9, -36],  // 11-point
    [-11, 0, 9, 16, 21, 24, 25, 24, 21, 16, 9, 0, -11],  // 13-point
    [-78, -13, 42, 87, 122, 147, 162, 167, 162, 147, 122, 87, 42, -13, -78],  // 15-point
    [-21, -6, 7, 18, 27, 34, 39, 42, 43, 42, 39, 34, 27, 18, 7, -6, -21],  // 17-point
    [-136, -51, 24, 89, 144, 189, 224, 249, 264, 269, 264, 249, 224, 189, 144, 89, 24, -51, -136],  // 19-point
    [-171, -76, 9, 84, 149, 204, 249, 284, 309, 324, 329, 324, 309, 284, 249, 204, 149, 84, 9, -76, -171],  // 21-point
    [-42, -21, -2, 15, 30, 43, 54, 63, 70, 75, 78, 79, 78, 75, 70, 63, 54, 43, 30, 15, -2, -21, -42],  // 23-point
    [-253, -138, -33, 62, 147, 222, 287, 343, 387, 422, 447, 462, 467, 462, 447, 422, 387, 343, 287, 222, 147, 62, -33, -138, -253]  // 25-point
  ]

  constructor(opts?: SGFilterOptions) {
    this.filterLength = opts?.filterLength ?? 15 // Default to 15-point filter
    
    // Ensure filter length is odd and in valid range
    if (this.filterLength % 2 === 0) {
      this.filterLength += 1
    }
    this.filterLength = Math.max(5, Math.min(25, this.filterLength))
    this.halfFilterLength = Math.floor(this.filterLength / 2)

    this.lastUpdateMillis = undefined
    this.originGps = undefined
  }

  /**
   * Converts GPS lat/lon/alt to ENU relative to the originGps
   */
  gpsToEnu(gps: MLocation): Vector3 {
    if (!this.originGps) return vec()

    const EARTH_RADIUS_METERS = 6371000
    const latRad = degToRad(gps.lat)
    const lonRad = degToRad(gps.lng)
    const originLatRad = degToRad(this.originGps.lat)
    const originLonRad = degToRad(this.originGps.lng)

    const deltaLat = latRad - originLatRad
    const deltaLon = lonRad - originLonRad

    const north = deltaLat * EARTH_RADIUS_METERS
    const east = deltaLon * EARTH_RADIUS_METERS * Math.cos(originLatRad)
    const up = gps.alt - this.originGps.alt

    return vec(east, up, north)
  }

  /**
   * Get normalized SG coefficients for current filter length
   */
  private getSGCoefficients(): number[] {
    const coeffIndex = (this.filterLength - 5) / 2
    if (coeffIndex < 0 || coeffIndex >= this.cMN.length) {
      throw new Error(`Unsupported filter length: ${this.filterLength}`)
    }
    
    const coeffs = this.cMN[coeffIndex]
    const sum = coeffs.reduce((a, b) => a + b, 0)
    return coeffs.map(c => c / sum)
  }

  /**
   * Apply SG convolution to a signal
   */
  private applySGConvolution(signal: number[], coeffs: number[]): number {
    if (signal.length !== coeffs.length) {
      throw new Error('Signal and coefficients must have same length')
    }
    return signal.reduce((sum, val, i) => sum + val * coeffs[i], 0)
  }

  /**
   * Calculate filtered output for a GPS point at the center of the filter window
   */
  private calculateFilteredOutput(centerIndex: number): FilteredOutput {
    const centerGps = this.gpsBuffer[centerIndex]
    const startIndex = centerIndex - this.halfFilterLength
    const endIndex = centerIndex + this.halfFilterLength + 1
    
    if (startIndex < 0 || endIndex > this.gpsBuffer.length) {
      throw new Error('Not enough data points for filtering')
    }

    const window = this.gpsBuffer.slice(startIndex, endIndex)
    const coeffs = this.getSGCoefficients()

    // Apply SG filtering to position (convert to ENU first)
    const enuPositions = window.map(gps => this.gpsToEnu(gps))
    const filteredPos = vec(
      this.applySGConvolution(enuPositions.map(p => p.x), coeffs),
      this.applySGConvolution(enuPositions.map(p => p.y), coeffs),
      this.applySGConvolution(enuPositions.map(p => p.z), coeffs)
    )

    // Apply SG filtering to velocity (ENU coordinates)
    const velocities = window.map(gps => vec(gps.velE, -gps.velD, gps.velN))
    const filteredVel = vec(
      this.applySGConvolution(velocities.map(v => v.x), coeffs),
      this.applySGConvolution(velocities.map(v => v.y), coeffs),
      this.applySGConvolution(velocities.map(v => v.z), coeffs)
    )

    // Calculate acceleration from velocity differences (use numerical differentiation on filtered velocities)
    let acceleration = vec()
    if (centerIndex > 0 && centerIndex < this.gpsBuffer.length - 1) {
      const prevGps = this.gpsBuffer[centerIndex - 1]
      const nextGps = this.gpsBuffer[centerIndex + 1]
      const dt = (nextGps.time - prevGps.time) / 1000 / 2 // Central difference
      
      if (dt > 0) {
        // Calculate acceleration using central difference on raw GPS velocities for aMeasured
        const prevVel = vec(prevGps.velE, -prevGps.velD, prevGps.velN)
        const nextVel = vec(nextGps.velE, -nextGps.velD, nextGps.velN)
        acceleration = div(sub(nextVel, prevVel), dt)
      }
    }

    // Calculate wingsuit acceleration using filtered velocity
    const [aWSE_x, aWSE_y, aWSE_z] = calculateWingsuitAcceleration(
      filteredVel.z, filteredVel.x, -filteredVel.y,
      0.01, 0.01, 0.0  // Initial parameters
    )
    const aWSE = vec(aWSE_x, aWSE_y, aWSE_z)

    // Update wingsuit parameters based on filtered data
    const vN = filteredVel.z, vE = filteredVel.x, vD = -filteredVel.y
    const aN = acceleration.z, aE = acceleration.x, aD = -acceleration.y
    const [kl, kd, roll] = calculateWingsuitParameters(vN, vE, vD, aN, aE, aD, 0.01, 0.01, 0.0)

    return {
      time: centerGps.time,
      position: filteredPos,
      velocity: filteredVel,
      acceleration: acceleration,
      aMeasured: acceleration,
      aWSE: aWSE,
      kl: kl,
      kd: kd,
      roll: roll
    }
  }

  /**
   * Update with GPS measurement
   */
  update(gps: MLocation): void {
    const tNow = gps.time

    // First fix sets origin
    if (this.lastUpdateMillis === undefined) {
      this.originGps = gps
      this.gpsBuffer = [gps]
      this.lastUpdateMillis = tNow
      return
    }

    // Add GPS point to buffer
    this.gpsBuffer.push(gps)

    // Check if we can generate a new filtered output
    // We need at least filterLength points, and the output will be for the point that's
    // halfFilterLength positions back from the current end
    if (this.gpsBuffer.length >= this.filterLength) {
      const outputIndex = this.gpsBuffer.length - 1 - this.halfFilterLength
      if (outputIndex >= 0) {
        try {
          const filteredOutput = this.calculateFilteredOutput(outputIndex)
          
          // Store the last 2 filtered outputs for interpolation
          this.filteredOutputs.push(filteredOutput)
          if (this.filteredOutputs.length > 2) {
            this.filteredOutputs.shift()
          }
        } catch (error) {
          console.warn('Failed to calculate filtered output:', error)
        }
      }
    }

    // Limit buffer size to prevent excessive memory usage
    const maxBufferSize = Math.max(100, this.filterLength * 3)
    if (this.gpsBuffer.length > maxBufferSize) {
      const removeCount = this.gpsBuffer.length - maxBufferSize
      this.gpsBuffer.splice(0, removeCount)
    }

    this.lastUpdateMillis = tNow
  }

  /**
   * Interpolate between two filtered outputs
   */
  private interpolateOutputs(output1: FilteredOutput, output2: FilteredOutput, targetTime: number): FilteredOutput {
    const t1 = output1.time
    const t2 = output2.time
    
    if (t2 === t1) {
      return output2
    }
    
    const alpha = (targetTime - t1) / (t2 - t1)
    const clampedAlpha = Math.max(0, Math.min(1, alpha))
    
    return {
      time: targetTime,
      position: add(mul(output1.position, 1 - clampedAlpha), mul(output2.position, clampedAlpha)),
      velocity: add(mul(output1.velocity, 1 - clampedAlpha), mul(output2.velocity, clampedAlpha)),
      acceleration: add(mul(output1.acceleration, 1 - clampedAlpha), mul(output2.acceleration, clampedAlpha)),
      aMeasured: add(mul(output1.aMeasured, 1 - clampedAlpha), mul(output2.aMeasured, clampedAlpha)),
      aWSE: add(mul(output1.aWSE, 1 - clampedAlpha), mul(output2.aWSE, clampedAlpha)),
      kl: output1.kl * (1 - clampedAlpha) + output2.kl * clampedAlpha,
      kd: output1.kd * (1 - clampedAlpha) + output2.kd * clampedAlpha,
      roll: output1.roll * (1 - clampedAlpha) + output2.roll * clampedAlpha
    }
  }

  /**
   * Uses the same integration method as KalmanFilter3D for prediction consistency
   */
  private integrateState(state: number[], dt: number): number[] {
    const [x, y, z, vx, vy, vz, ax, ay, az, kl, kd, roll] = state

    // SGFilter only supports STANDARD method
    const x_next = x + vx * dt
    const y_next = y + vy * dt
    const z_next = z + vz * dt
    
    const vx_next = vx + ax * dt
    const vy_next = vy + ay * dt
    const vz_next = vz + az * dt
    
    const [ax_next, ay_next, az_next] = calculateWingsuitAcceleration(vz_next, vx_next, -vy_next, kl, kd, roll)
    
    return [x_next, y_next, z_next, vx_next, vy_next, vz_next, ax_next, ay_next, az_next, kl, kd, roll]
  }

  /**
   * Predict state at future time
   */
  predictstate(deltaTime: number): number[] {
    // Get the latest filtered output as starting point
    const latestOutput = this.getLatestFilteredOutput()
    if (!latestOutput) {
      return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0.01, 0]
    }

    if (deltaTime <= 0) {
      return [
        latestOutput.position.x, latestOutput.position.y, latestOutput.position.z,
        latestOutput.velocity.x, latestOutput.velocity.y, latestOutput.velocity.z,
        latestOutput.acceleration.x, latestOutput.acceleration.y, latestOutput.acceleration.z,
        latestOutput.kl, latestOutput.kd, latestOutput.roll
      ]
    }

    const maxStepSize = 0.1
    let remainingTime = deltaTime

    let sstate = [
      latestOutput.position.x, latestOutput.position.y, latestOutput.position.z,
      latestOutput.velocity.x, latestOutput.velocity.y, latestOutput.velocity.z,
      latestOutput.acceleration.x, latestOutput.acceleration.y, latestOutput.acceleration.z,
      latestOutput.kl, latestOutput.kd, latestOutput.roll
    ]

    while (remainingTime > 0) {
      const stepSize = Math.min(remainingTime, maxStepSize)
      sstate = this.integrateState(sstate, stepSize)
      remainingTime -= stepSize
    }

    return sstate
  }

  /**
   * Get the latest filtered output or interpolated result
   */
  private getLatestFilteredOutput(): FilteredOutput | undefined {
    if (this.filteredOutputs.length === 0) {
      return undefined
    }

    if (this.filteredOutputs.length === 1) {
      return this.filteredOutputs[0]
    }

    // Use the most recent filtered output
    return this.filteredOutputs[this.filteredOutputs.length - 1]
  }

  /**
   * Predict at a specific query time with proper delay handling
   */
  predictAt(tQueryMillis: number): (MotionState & { kl?: number, kd?: number, roll?: number }) | undefined {
    if (this.lastUpdateMillis === undefined) return undefined

    // Calculate the filter delay
    const filterDelayMs = this.halfFilterLength * (1000 / 90) // Assuming 90Hz GPS rate
    const delayedQueryTime = tQueryMillis - filterDelayMs

    // If we have multiple filtered outputs, try to interpolate
    if (this.filteredOutputs.length >= 2) {
      const output1 = this.filteredOutputs[0]
      const output2 = this.filteredOutputs[1]
      
      // Check if the delayed query time is within our interpolation range
      if (delayedQueryTime >= output1.time && delayedQueryTime <= output2.time) {
        const interpolated = this.interpolateOutputs(output1, output2, delayedQueryTime)
        
        // Now predict forward from the interpolated point to the actual query time
        const forwardDt = (tQueryMillis - interpolated.time) / 1000
        if (forwardDt > 0) {
          const currentState = [
            interpolated.position.x, interpolated.position.y, interpolated.position.z,
            interpolated.velocity.x, interpolated.velocity.y, interpolated.velocity.z,
            interpolated.acceleration.x, interpolated.acceleration.y, interpolated.acceleration.z,
            interpolated.kl, interpolated.kd, interpolated.roll
          ]


          const [x, y, z, vx, vy, vz, ax, ay, az, kl, kd, roll] = currentState

          return {
            position: vec(x, y, z),
            velocity: vec(vx, vy, vz),
            acceleration: vec(ax, ay, az),
            aMeasured: interpolated.aMeasured,
            aWSE: interpolated.aWSE,
            kl: kl,
            kd: kd,
            roll: roll
          }
        } else {//query for past
          return {
            position: interpolated.position,
            velocity: interpolated.velocity,
            acceleration: interpolated.acceleration,
            aMeasured: interpolated.aMeasured,
            aWSE: interpolated.aWSE,
            kl: interpolated.kl,
            kd: interpolated.kd,
            roll: interpolated.roll
          }
        }
      }// not between last 2 gps points... make future prediction
      
    }

    // Fall back to prediction from latest filtered output
    const latestOutput = this.getLatestFilteredOutput()
    if (!latestOutput) {
      return undefined
    }

    const dt = (tQueryMillis - latestOutput.time) / 1000
    const predictedState = this.predictstate(dt)
    const [x, y, z, vx, vy, vz, ax, ay, az, kl, kd, roll] = predictedState

    return {
      position: vec(x, y, z),
      velocity: vec(vx, vy, vz),
      acceleration: vec(ax, ay, az),
      aMeasured: latestOutput.aMeasured,
      aWSE: latestOutput.aWSE,
      kl: kl,
      kd: kd,
      roll: roll
    }
  }

  /**
   * Predict from a given state
   */
  private predictFromState(state: number[], deltaTime: number): number[] {
    if (deltaTime <= 0) return state

    const maxStepSize = 0.1
    let remainingTime = deltaTime
    let sstate = [...state]

    while (remainingTime > 0) {
      const stepSize = Math.min(remainingTime, maxStepSize)
      sstate = this.integrateState(sstate, stepSize)
      remainingTime -= stepSize
    }

    return sstate
  }

  /**
   * Get current state (latest filtered output)
   */
  getState(): MotionState & { kl?: number, kd?: number, roll?: number } {
    const latestOutput = this.getLatestFilteredOutput()
    
    if (!latestOutput) {
      return {
        position: vec(),
        velocity: vec(),
        acceleration: vec(),
        aMeasured: vec(),
        aWSE: vec(),
        kl: 0.01,
        kd: 0.01,
        roll: 0.0
      }
    }

    return {
      position: latestOutput.position,
      velocity: latestOutput.velocity,
      acceleration: latestOutput.acceleration,
      aMeasured: latestOutput.aMeasured,
      aWSE: latestOutput.aWSE,
      kl: latestOutput.kl,
      kd: latestOutput.kd,
      roll: latestOutput.roll
    }
  }

  /**
   * Reset filter state
   */
  reset(): void {
    this.gpsBuffer = []
    this.filteredOutputs = []
    this.lastUpdateMillis = undefined
    this.originGps = undefined
  }

  /**
   * Set filter length
   */
  setFilterLength(length: number): void {
    let newLength = Math.max(5, Math.min(25, length))
    if (newLength % 2 === 0) {
      newLength += 1
    }
    this.filterLength = newLength
    this.halfFilterLength = Math.floor(newLength / 2)
    
    // Clear filtered outputs since filter characteristics changed
    this.filteredOutputs = []
  }

  getFilterLength(): number {
    return this.filterLength
  }

  setCalculationMethod(method: CalculationMethod): void {
    this.calculationMethod = CalculationMethod.STANDARD
  }

  getCalculationMethod(): CalculationMethod {
    return this.calculationMethod
  }
}
