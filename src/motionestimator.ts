import type { Vector3, MLocation } from './types.js'
import { vec, add, sub, mul, div, degToRad } from './vector.js'
import { calculateWingsuitAcceleration, calculateWingsuitParameters } from './wse.js'

export enum CalculationMethod {
  TRAPEZOIDAL = 'trapezoidal',
  STANDARD = 'standard'
}

export interface MotionState {
  position: Vector3
  velocity: Vector3
  acceleration: Vector3
  // Additional acceleration components for analysis
  aMeasured?: Vector3
  aWSE?: Vector3
}

export interface MotionEstimatorOptions {
  alpha?: number
  alphaVelocity?: number
  alphaAcceleration?: number
  estimateVelocity?: boolean
}


/**
 * Mirrors the Java MotionEstimator:
 * - alpha blend on acceleration and position
 * - velocity taken straight from GPS
 * - ENU conversion from lat/lon/alt at first fix as origin
 */
export class MotionEstimator {
  private alpha: number
  private alphaVelocity: number
  private alphaAcceleration: number
  private estimateVelocity: boolean
  private calculationMethod: CalculationMethod = CalculationMethod.TRAPEZOIDAL
  private p: Vector3           // metres ENU
  private v: Vector3           // m/s ENU
  private a: Vector3           // m/s^2 ENU
  private aMeasured: Vector3   // measured acceleration (m/s^2 ENU)
  private aWSE: Vector3        // wingsuit model acceleration (m/s^2 ENU)
  private positionDelta: Vector3  // p - lastPosition (ENU)
  private lastUpdateMillis: number | undefined
  private lastMeasuredVelocity: Vector3 | undefined  // last GPS velocity for acceleration calculation

  // Wingsuit parameters
  private kl: number = 0.01
  private kd: number = 0.01
  private roll: number = 0.0

  // GPS-origin mode (mirrors Java)
  private originGps: MLocation | undefined

  // ENU-origin mode for update(x,y,z,...)
  private originEnu: Vector3 | undefined

  constructor(opts?: MotionEstimatorOptions) {
    const alpha = opts && typeof opts.alpha === 'number' ? opts.alpha : 0.1
    this.alpha = alpha
    this.alphaVelocity = opts && typeof opts.alphaVelocity === 'number' ? opts.alphaVelocity : alpha
    this.alphaAcceleration = opts && typeof opts.alphaAcceleration === 'number' ? opts.alphaAcceleration : alpha
    this.estimateVelocity = opts?.estimateVelocity ?? false

    this.p = vec()           // metres ENU
    this.v = vec()           // m/s ENU
    this.a = vec()           // m/s^2 ENU
    this.aMeasured = vec()   // measured acceleration (m/s^2 ENU)
    this.aWSE = vec()        // wingsuit model acceleration (m/s^2 ENU)

    this.positionDelta = vec()  // p - lastPosition (ENU)
    this.lastUpdateMillis = undefined
    this.lastMeasuredVelocity = undefined

    // GPS-origin mode (mirrors Java)
    this.originGps = undefined

    // ENU-origin mode for update(x,y,z,...)
    this.originEnu = undefined
  }

  /**
   * Converts GPS lat/lon/alt to ENU relative to the originGps
   * @param gps GPS sample
   * @returns ENU vector {x:east, y:up, z:north}
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
   * Call every time a fresh GPS sample arrives (closest to the Java API).
   * @param gps GPS sample
   */
  updateFromGps(gps: MLocation): void {
    const tNow = gps.time

    // First fix sets origin and initializes state
    if (this.lastUpdateMillis === undefined) {
      this.originGps = gps
      this.originEnu = undefined  // choose GPS-origin mode
      this.p = vec()              // new origin = (0,0,0)
      this.v = vec(gps.velE, -gps.velD, gps.velN)
      this.a = vec()
      this.aMeasured = vec()
      this.aWSE = vec()
      this.positionDelta = vec()
      this.lastUpdateMillis = tNow
      return
    }

    const dt = Math.max(0, (tNow - this.lastUpdateMillis) * 1e-3)
    if (dt === 0) return // Skip if no time has passed

    // Get current GPS measurements in ENU
    const pMeasured = this.gpsToEnu(gps)
    const vMeasured = vec(gps.velE, -gps.velD, gps.velN)

    // 1. Calculate acceleration from GPS velocity change (inferred, not measured)
    let aMeasured = vec()  // Default to zero acceleration
    if (this.lastMeasuredVelocity !== undefined) {
      aMeasured = div(sub(vMeasured, this.lastMeasuredVelocity), dt)
    }
    this.aMeasured = aMeasured  // Store for plotting
    
    // 2. Calculate predicted velocity for wingsuit acceleration
    const vPredicted = add(this.v, mul(this.a, dt))
    
    // 3. Calculate wingsuit acceleration using predicted velocity and stored parameters
    const [aWSE_x, aWSE_y, aWSE_z] = calculateWingsuitAcceleration(
      vPredicted.z, vPredicted.x, -vPredicted.y,
      this.kl, this.kd, this.roll
    )
    const aWSE = vec(aWSE_x, aWSE_y, aWSE_z)
    this.aWSE = aWSE  // Store for plotting
    
    // 4. Complementary filter for acceleration using wingsuit prediction
    const aOld = this.a
    this.a = add(mul(aWSE, 1 - this.alphaAcceleration), mul(aMeasured, this.alphaAcceleration))
  // log aMeasured and aWSE
    console.log('aMeasured:', aMeasured)
    console.log('aWSE:', aWSE)
    console.log("astate:", this.a)
    
    if (this.calculationMethod === CalculationMethod.STANDARD) {
      // Standard method: position first, then velocity, then acceleration (no trapezoidal rule)
      
      // Update position using current velocity
      const pPredictedStandard = add(this.p, mul(this.v, dt))
      
      // Update velocity using current acceleration
      const vPredictedStandard = add(this.v, mul(this.a, dt))
      
      // Update acceleration using new velocity
      const [aWSE_x_new, aWSE_y_new, aWSE_z_new] = calculateWingsuitAcceleration(
        vPredictedStandard.z, vPredictedStandard.x, -vPredictedStandard.y,
        this.kl, this.kd, this.roll
      )
      this.aWSE = vec(aWSE_x_new, aWSE_y_new, aWSE_z_new)
      
      // Complementary filter for velocity (blend predicted with GPS measurement)
      if (this.estimateVelocity) {
        this.v = add(mul(vPredictedStandard, 1 - this.alphaVelocity), mul(vMeasured, this.alphaVelocity))
      } else {
        this.v = vMeasured // Use GPS velocity directly if not estimating
      }

      // Complementary filter for position (blend predicted with GPS measurement)
      this.p = add(mul(pPredictedStandard, 1 - this.alpha), mul(pMeasured, this.alpha))
    } else {
      // Trapezoidal method (existing implementation)
      
      // 5. Predict current state using trapezoidal rule from last state
      // Trapezoidal rule for velocity: v_pred = v + (a_old + a_new) * dt / 2
      const vPredictedFinal = add(this.v, mul(add(aOld, this.a), dt / 2))
      
      // Trapezoidal rule for position: p_pred = p + (v_old + v_new) * dt / 2
      const pPredictedFinal = add(this.p, mul(add(this.v, vPredictedFinal), dt / 2))

      // 6. Complementary filter for velocity (blend predicted with GPS measurement)
      if (this.estimateVelocity) {
        this.v = add(mul(vPredictedFinal, 1 - this.alphaVelocity), mul(vMeasured, this.alphaVelocity))
      } else {
        this.v = vMeasured // Use GPS velocity directly if not estimating
      }

      // 7. Complementary filter for position (blend predicted with GPS measurement)
      this.p = add(mul(pPredictedFinal, 1 - this.alpha), mul(pMeasured, this.alpha))
    }

    // 8. Update wingsuit parameters based on final state
    // Convert ENU to NDE for parameter calculation
    const vN = this.v.z, vE = this.v.x, vD = -this.v.y
    const aN = this.a.z, aE = this.a.x, aD = -this.a.y
    const [newKl, newKd, newRoll] = calculateWingsuitParameters(vN, vE, vD, aN, aE, aD, this.kl, this.kd, this.roll)
    this.kl = newKl
    this.kd = newKd
    this.roll = newRoll

    this.positionDelta = sub(this.p, pMeasured)
    
    // Store current measured velocity for next iteration's acceleration calculation
    this.lastMeasuredVelocity = vMeasured
    this.lastUpdateMillis = tNow
  }

  /**
   * Update with GPS measurement (matches KalmanFilter3D API).
   * @param gps GPS measurement
   */
  update(gps: MLocation): void {
    this.updateFromGps(gps)
  }

  
  /**
   * Delta from the LAST measurement's ENU position to a future query time.
   * This exactly matches the Java method: positionDelta + v*dt + 0.5*a*dt^2
   * @param tQueryMillis Query time in milliseconds
   * @returns Position delta vector
   */
  predictDelta(tQueryMillis: number): Vector3 {
    if (this.lastUpdateMillis === undefined) return vec()
    let dt = (tQueryMillis - this.lastUpdateMillis) * 1e-3
    if (dt < 0) dt = 0
    return add(add(this.positionDelta, mul(this.v, dt)), mul(this.a, 0.5 * dt * dt))
  }

  /**
   * Absolute prediction at a query time (convenience).
   * Derived from Java variables: lastPosition + predictDelta(dt) = p + v*dt + 0.5*a*dt^2.
   * @param tQueryMillis Query time in milliseconds
   * @returns Motion state at query time or undefined if not initialized
   */
  predictAt(tQueryMillis: number): (MotionState & { kl?: number, kd?: number, roll?: number }) | undefined {
    if (this.lastUpdateMillis === undefined) return undefined
    const dt = Math.max(0, (tQueryMillis - this.lastUpdateMillis) * 1e-3)
    const pos = add(add(this.p, mul(this.v, dt)), mul(this.a, 0.5 * dt * dt))
    const vel = add(this.v, mul(this.a, dt))
    return {
      position: pos,
      velocity: vel,
      acceleration: this.a,
      aMeasured: this.aMeasured,
      aWSE: this.aWSE,
      kl: this.kl,
      kd: this.kd,
      roll: this.roll
    }
  }

  /**
   * Current motion state at the last update time.
   * @returns Current motion state
   */
  getState(): MotionState & { kl?: number, kd?: number, roll?: number } {
    return {
      position: this.p,
      velocity: this.v,
      acceleration: this.a,
      aMeasured: this.aMeasured,
      aWSE: this.aWSE,
      kl: this.kl,
      kd: this.kd,
      roll: this.roll
    }
  }

  /**
   * Resets state and clears origins.
   */
  reset(): void {
    this.p = vec()
    this.v = vec()
    this.a = vec()
    this.aMeasured = vec()
    this.aWSE = vec()
    this.positionDelta = vec()
    this.lastUpdateMillis = undefined
    this.lastMeasuredVelocity = undefined
    this.originGps = undefined
    this.originEnu = undefined
    
    // Reset wingsuit parameters
    this.kl = 0.01
    this.kd = 0.01
    this.roll = 0.0
  }

  /**
   * Set the calculation method for state integration.
   * @param method The calculation method to use
   */
  setCalculationMethod(method: CalculationMethod): void {
    this.calculationMethod = method
  }

  /**
   * Get the current calculation method.
   * @returns The current calculation method
   */
  getCalculationMethod(): CalculationMethod {
    return this.calculationMethod
  }
}