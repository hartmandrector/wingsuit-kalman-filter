import type { Vector3, MLocation } from './types.js'
import { vec, add, sub, mul, div, degToRad } from './vector.js'


export interface MotionState {
  position: Vector3
  velocity: Vector3
  acceleration: Vector3
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
  private p: Vector3           // metres ENU
  private v: Vector3           // m/s ENU
  private a: Vector3           // m/s^2 ENU
  private positionDelta: Vector3  // p - lastPosition (ENU)
  private lastUpdateMillis: number | undefined

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

    this.positionDelta = vec()  // p - lastPosition (ENU)
    this.lastUpdateMillis = undefined

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
      this.positionDelta = vec()
      this.lastUpdateMillis = tNow
      return
    }

    const dt = Math.max(0, (tNow - this.lastUpdateMillis) * 1e-3)

    const vNew = vec(gps.velE, -gps.velD, gps.velN)
    const aRaw = dt > 0 ? div(sub(vNew, this.v), dt) : vec()
    this.a = add(mul(this.a, 1 - this.alphaAcceleration), mul(aRaw, this.alphaAcceleration))

    const lastPosition = this.gpsToEnu(gps)

    // predict (constant-acceleration)
    const pPred = add(add(this.p, mul(this.v, dt)), mul(this.a, 0.5 * dt * dt))

    // update using measurement as evidence (complementary filter)
    this.p = add(mul(pPred, 1 - this.alpha), mul(lastPosition, this.alpha))

    // use GPS velocity directly or alpha-blend it?
    if (this.estimateVelocity) {
      this.v = add(mul(this.v, 1 - this.alphaVelocity), mul(vNew, this.alphaVelocity))
    } else {
      this.v = vNew
    }

    this.positionDelta = sub(this.p, lastPosition)
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
   * ENU path: if you already have ENU position and velocity, this mirrors the same math.
   * The first call sets an ENU origin from your first (x,y,z) so that lastPosition starts at 0.
   * @param x East coordinate
   * @param y Up coordinate  
   * @param z North coordinate
   * @param vx East velocity
   * @param vy Up velocity
   * @param vz North velocity
   * @param timestampMillis Timestamp in milliseconds
   */
  updateWithEnu(x: number, y: number, z: number, vx: number, vy: number, vz: number, timestampMillis: number): void {
    const tNow = timestampMillis

    if (this.lastUpdateMillis === undefined) {
      this.originEnu = vec(x, y, z)  // set ENU origin so lastPosition = 0 on first sample
      this.originGps = undefined      // choose ENU-origin mode
      this.p = vec()
      this.v = vec(vx, vy, vz)
      this.a = vec()
      this.positionDelta = vec()
      this.lastUpdateMillis = tNow
      return
    }

    const dt = Math.max(0, (tNow - this.lastUpdateMillis) * 1e-3)
    const vNew = vec(vx, vy, vz)
    const aRaw = dt > 0 ? div(sub(vNew, this.v), dt) : vec()
    this.a = add(mul(this.a, 1 - this.alphaAcceleration), mul(aRaw, this.alphaAcceleration))

    // lastPosition is incoming ENU relative to our first-sample origin
    const lastPosition = this.originEnu
      ? sub(vec(x, y, z), this.originEnu)
      : vec()

    const pPred = add(add(this.p, mul(this.v, dt)), mul(this.a, 0.5 * dt * dt))
    this.p = add(mul(pPred, 1 - this.alpha), mul(lastPosition, this.alpha))

    this.v = vNew
    this.positionDelta = sub(this.p, lastPosition)
    this.lastUpdateMillis = tNow
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
  predictAt(tQueryMillis: number): MotionState | undefined {
    if (this.lastUpdateMillis === undefined) return undefined
    const dt = Math.max(0, (tQueryMillis - this.lastUpdateMillis) * 1e-3)
    const pos = add(add(this.p, mul(this.v, dt)), mul(this.a, 0.5 * dt * dt))
    const vel = add(this.v, mul(this.a, dt))
    return {
      position: pos,
      velocity: vel,
      acceleration: this.a
    }
  }

  /**
   * Current motion state at the last update time.
   * @returns Current motion state
   */
  getState(): MotionState {
    return {
      position: this.p,
      velocity: this.v,
      acceleration: this.a
    }
  }

  /**
   * Resets state and clears origins.
   */
  reset(): void {
    this.p = vec()
    this.v = vec()
    this.a = vec()
    this.positionDelta = vec()
    this.lastUpdateMillis = undefined
    this.originGps = undefined
    this.originEnu = undefined
  }
}