import {
  createIdentityMatrix,
  createZeroMatrix,
  matrixMultiply,
  matrixVectorMultiply,
  matrixAdd,
  matrixSubtract,
  transpose,
  matrixInverse
} from './matrix.js'
import { Vector3, MLocation, Coefficients } from './types.js'
import { MotionState } from './motionestimator.js'
import { calculateWingsuitAcceleration, calculateWingsuitParameters, gravity } from './wse.js'
import { vec, sub } from './vector.js'

export enum CalculationMethod {
  TRAPEZOIDAL = 'trapezoidal',
  STANDARD = 'standard'
}

export class KalmanFilter3D {
  private state: number[]
  private P: number[][]
  private Q: number[][]
  private R: number[][]
  private B: number[][]
  private u: number[]
  private lastUpdateTime: number | undefined
  private lastMeasuredVelocity: { vx: number, vy: number, vz: number } | undefined
  private originGps: MLocation | undefined
  private calculationMethod: CalculationMethod = CalculationMethod.TRAPEZOIDAL
  
  // Store acceleration components for plotting
  private aMeasured: Vector3 = { x: 0, y: 0, z: 0 }
  private aWSE: Vector3 = { x: 0, y: 0, z: 0 }
  
  // Measurement noise offsets for user control
  private measurementNoiseOffsets = {
    position: 0,
    velocity: 0,
    acceleration: 0
  }

  constructor() {
    // Extended state vector: [x, y, z, vx, vy, vz, ax, ay, az, kl, kd, roll]
    this.state = new Array(12).fill(0)
    this.state[9] = 0.01 // Initial kl estimate for wingsuit
    this.state[10] = 0.01 // Initial kd estimate for wingsuit
    this.state[11] = 0.0 // Initial roll estimate for wingsuit

    // Covariance matrix (12x12)
    this.P = createIdentityMatrix(12)
    for (let i = 0; i < 9; i++) {
      this.P[i][i] = 1000 // Initial uncertainty for position, velocity, acceleration
    }
    this.P[9][9] = 0.1   // kl uncertainty
    this.P[10][10] = 0.1 // kd uncertainty  
    this.P[11][11] = 0.005 // roll uncertainty

    // Process noise covariance - using default slider values
    this.Q = createIdentityMatrix(12)
    for (let i = 0; i < 3; i++) {
      this.Q[i][i] = 1.4401 // Position process noise (matches slider default)
      this.Q[i + 3][i + 3] = 0.4226 // Velocity process noise (matches slider default)
      this.Q[i + 6][i + 6] = 68.4501 // Acceleration process noise (matches slider default)
    }
    this.Q[9][9] = 0.01   // kl process noise (slow changes)
    this.Q[10][10] = 0.01 // kd process noise (slow changes)
    this.Q[11][11] = 0.001   // roll process noise

    this.B = createIdentityMatrix(12)
    this.u = new Array(12).fill(0)

    // Measurement noise covariance (position and velocity measurements only) - using default slider values
    this.R = createIdentityMatrix(6)
    for (let i = 0; i < 3; i++) {
      this.R[i][i] = 1.21 // GPS position measurement noise (matches slider default)
      this.R[i + 3][i + 3] = 2.25 // GPS velocity measurement noise (matches slider default)
    }

    this.lastUpdateTime = undefined
    this.originGps = undefined
  }

  private integrateStep(dt: number): void {
    const [x, y, z, vx, vy, vz, ax, ay, az, kl, kd, roll] = this.state

    if (this.calculationMethod === CalculationMethod.STANDARD) {
      // Standard method: position first, then velocity, then acceleration (no trapezoidal rule)
      
      // Update position using current velocity
      const x_next = x + vx * dt
      const y_next = y + vy * dt
      const z_next = z + vz * dt
      
      // Update velocity using current acceleration
      const vx_next = vx + ax * dt
      const vy_next = vy + ay * dt
      const vz_next = vz + az * dt
      
      // Calculate new acceleration using updated velocity
      const [ax_next, ay_next, az_next] = calculateWingsuitAcceleration(vz_next, vx_next, -vy_next, kl, kd, roll)
      
      // Update state
      this.state = [x_next, y_next, z_next, vx_next, vy_next, vz_next, ax_next, ay_next, az_next, kl, kd, roll]
    } else {
      // Trapezoidal method (existing implementation)
      
      // Calculate current acceleration
      const [ax_current, ay_current, az_current] = [ax,ay,az]//calculateWingsuitAcceleration(vz, vx, -vy, kl, kd, roll)

      // Update velocity using trapezoidal rule
      let vx_next = vx + (ax_current + ax) / 2 * dt
      let vy_next = vy + (ay_current + ay) / 2 * dt
      let vz_next = vz + (az_current + az) / 2 * dt

      // Update position using trapezoidal rule
      const x_next = x + (vx + vx_next) / 2 * dt
      const y_next = y + (vy + vy_next) / 2 * dt
      const z_next = z + (vz + vz_next) / 2 * dt

      // Update state (kl, kd, roll remain constant during prediction)
      this.state = [x_next, y_next, z_next, vx_next, vy_next, vz_next, ax_current, ay_current, az_current, kl, kd, roll]
    }
  }

  private integrateState(state: number[], dt: number): number[] {
    const [x, y, z, vx, vy, vz, ax, ay, az, kl, kd, roll] = state

    if (this.calculationMethod === CalculationMethod.STANDARD) {
      // Standard method: position first, then velocity, then acceleration (no trapezoidal rule)
      
      // Update position using current velocity
      const x_next = x + vx * dt
      const y_next = y + vy * dt
      const z_next = z + vz * dt
      
      // Update velocity using current acceleration
      const vx_next = vx + ax * dt
      const vy_next = vy + ay * dt
      const vz_next = vz + az * dt
      
      // Calculate new acceleration using updated velocity
      const [ax_next, ay_next, az_next] = calculateWingsuitAcceleration(vz_next, vx_next, -vy_next, kl, kd, roll)
      
      // Return updated state
      return [x_next, y_next, z_next, vx_next, vy_next, vz_next, ax_next, ay_next, az_next, kl, kd, roll]
    } else {
      // Trapezoidal method (existing implementation)
      
      // Calculate current acceleration
      const [ax_current, ay_current, az_current] = calculateWingsuitAcceleration(vz, vx, -vy, kl, kd, roll)

      // Update velocity using trapezoidal rule
      let vx_next = vx + (ax_current + ax) / 2 * dt
      let vy_next = vy + (ay_current + ay) / 2 * dt
      let vz_next = vz + (az_current + az) / 2 * dt

      // Update position using trapezoidal rule
      const x_next = x + (vx + vx_next) / 2 * dt
      const y_next = y + (vy + vy_next) / 2 * dt
      const z_next = z + (vz + vz_next) / 2 * dt

      // Return updated state
      return [x_next, y_next, z_next, vx_next, vy_next, vz_next, ax_current, ay_current, az_current, kl, kd, roll]
    }
  }

  // predict at
  predictstate(deltaTime: number): number[] {
    if (deltaTime <= 0) return this.state

    const maxStepSize = 0.1 // Maximum 0.1 second steps
    let remainingTime = deltaTime


    let sstate = [...this.state]
    while (remainingTime > 0) {
      const stepSize = Math.min(remainingTime, maxStepSize)
      sstate = this.integrateState(sstate, stepSize)
      remainingTime -= stepSize
    }

    return sstate
  }

  predict(deltaTime: number): void {
    if (deltaTime <= 0) return

    const maxStepSize = 0.1 // Maximum 0.1 second steps
    let remainingTime = deltaTime

    //save old state for restoring after prediction
   //const oldState = [...this.state]

    while (remainingTime > 0) {
      const stepSize = Math.min(remainingTime, maxStepSize)
      this.integrateStep(stepSize)
      remainingTime -= stepSize
    }

    // Create approximate Jacobian for covariance propagation
    const F = this.calculateJacobian(deltaTime)
    console.log("F:", F)
    // Predict covariance: P = F*P*F^T + Q*dt (scale process noise by time step)
    const FP = matrixMultiply(F, this.P)
    const FPFT = matrixMultiply(FP, transpose(F))
    
    // Scale Q by deltaTime for proper discrete-time process noise
    const Q_scaled = this.Q.map(row => row.map(val => val * deltaTime))
    this.P = matrixAdd(FPFT, Q_scaled)
  }

  private calculateJacobian(dt: number): number[][] {
    // More complete Jacobian for 12x12 system
    const F = createIdentityMatrix(12)

    // Position derivatives
    F[0][3] = dt // dx/dvx
    F[1][4] = dt // dy/dvy
    F[2][5] = dt // dz/dvz

    // Velocity derivatives
    F[3][6] = dt // dvx/dax
    F[4][7] = dt // dvy/day
    F[5][8] = dt // dvz/daz

    // Acceleration derivatives would be complex (wingsuit dynamics)
    // For now, keep them as identity (acceleration persists)

    // kl, kd, roll remain constant during prediction (identity already set)

    return F
  }

  update(gps: MLocation): void {
    if (this.lastUpdateTime === undefined) {
      // First measurement initializes state - FIX: use 12-element state
      this.originGps = gps
      this.state = [0, 0, 0, gps.velE, -gps.velD, gps.velN, 0, 0, 0, 0.01, 0.01, 0] // 12 elements
      this.lastUpdateTime = gps.time
      return
    }

    const pos = this.gpsToEnu(gps)
    this.updateWithEnu(pos.x, pos.y, pos.z, gps.velE, -gps.velD, gps.velN, gps.hAcc, gps.vAcc, gps.sAcc, gps.time)
  }

  private updateR(hAcc: number, vAcc: number, sAcc: number): void {
    // Update measurement noise covariance based on accuracy with user offsets
    // use error measurements from flysight plus user-controlled offsets
    this.R = createIdentityMatrix(6)
    for (let i = 0; i < 3; i++) {
      // Apply position offset, ensure positive values
      //const basePositionNoise = i === 1 ? vAcc : hAcc
      const positionNoise = Math.max(0.0001, this.measurementNoiseOffsets.position)
      this.R[i][i] = positionNoise
      
      // Apply velocity offset, ensure positive values
      const velocityNoise = Math.max(0.0001, this.measurementNoiseOffsets.velocity)
      this.R[i + 3][i + 3] = velocityNoise
    }
  }

  private gpsToEnu(gps: MLocation): Vector3 {
    if (!this.originGps) return { x: 0, y: 0, z: 0 }

    const EARTH_RADIUS_METERS = 6371000
    const toRad = (deg: number) => deg * Math.PI / 180

    const latRad = toRad(gps.lat)
    const lonRad = toRad(gps.lng)
    const originLatRad = toRad(this.originGps.lat)
    const originLonRad = toRad(this.originGps.lng)

    const deltaLat = latRad - originLatRad
    const deltaLon = lonRad - originLonRad

    const north = deltaLat * EARTH_RADIUS_METERS
    const east = deltaLon * EARTH_RADIUS_METERS * Math.cos(originLatRad)
    const up = gps.alt - this.originGps.alt

    return { x: east, y: up, z: north }
  }

  updateWithEnu(x: number, y: number, z: number, vx: number, vy: number, vz: number, hAcc: number, vAcc: number, sAcc: number, timestamp: number): void {
    // Update measurement noise covariance
    this.updateR(hAcc, vAcc, sAcc)

   

    // Extract measured acceleration from GPS data change
    let measuredAx = 0, measuredAy = 0, measuredAz = 0
    if (this.lastUpdateTime !== undefined && this.lastMeasuredVelocity !== undefined) {
      const dt = (timestamp - this.lastUpdateTime) / 1000
      if (dt > 0) {
        measuredAx = (vx - this.lastMeasuredVelocity.vx) / dt
        measuredAy = (vy - this.lastMeasuredVelocity.vy) / dt
        measuredAz = (vz - this.lastMeasuredVelocity.vz) / dt
      }
    }
    
    // Store measured acceleration for plotting
    this.aMeasured = { x: measuredAx, y: measuredAy, z: measuredAz }
    
    // Update wingsuit parameters from kalman acceleration,
    //const [ae_kalman, ad_kalman, an_kalman] = [this.state[6], -this.state[7], this.state[8]]
    //const [ve_kalman, vd_kalman, vn_kalman] = [this.state[3], -this.state[4], this.state[5]]
    //this.updateWingsuitParameters(vn_kalman, ve_kalman, vd_kalman, an_kalman, ae_kalman, ad_kalman)

 if (this.lastUpdateTime !== undefined) {
      const deltaTime = (timestamp - this.lastUpdateTime) / 1000
      this.predict(deltaTime) //advance state using updated state kl, kd, roll
    }

    // Standard Kalman update for position and velocity only
    const H = createZeroMatrix(6, 12)
    H[0][0] = 1 // x position
    H[1][1] = 1 // y position  
    H[2][2] = 1 // z position
    H[3][3] = 1 // x velocity
    H[4][4] = 1 // y velocity
    H[5][5] = 1 // z velocity

    // Innovation
    const z_measurement = [x, y, z, vx, vy, vz]
    const h_x = [this.state[0], this.state[1], this.state[2], this.state[3], this.state[4], this.state[5]]
    const y_innovation = z_measurement.map((z, i) => z - h_x[i])

    // Innovation covariance: S = H*P*H^T + R
    const HP = matrixMultiply(H, this.P)
    const HPHT = matrixMultiply(HP, transpose(H))
    const S = matrixAdd(HPHT, this.R)

    // Kalman gain: K = P*H^T*S^(-1)
    const PHT = matrixMultiply(this.P, transpose(H))
    const S_inv = matrixInverse(S)
    const K = matrixMultiply(PHT, S_inv)

    // Update state: x = x + K*y
    const Ky = matrixVectorMultiply(K, y_innovation)
    for (let i = 0; i < 12; i++) {
      this.state[i] += Ky[i]
    }

     // Update wingsuit parameters from kalman acceleration,
    const [ae_kalma, ad_kalma, an_kalma] = [this.state[6], -this.state[7], this.state[8]]
    const [ve_kalma, vd_kalma, vn_kalma] = [this.state[3], -this.state[4], this.state[5]]
    this.updateWingsuitParameters(vn_kalma, ve_kalma, vd_kalma, an_kalma, ae_kalma, ad_kalma)


 // Calculate and store WSE acceleration for plotting
    const kl = this.state[9], kd = this.state[10], roll = this.state[11]
    const [aWSE_x, aWSE_y, aWSE_z] = calculateWingsuitAcceleration(vz, vx, -vy, kl, kd, roll)
    this.aWSE = { x: aWSE_x, y: aWSE_y, z: aWSE_z }

     console.log(`Kalman State update at ${timestamp}:`, {
      position: { x: this.state[0], y: this.state[1], z: this.state[2] },
      velocity: { x: this.state[3], y: this.state[4], z: this.state[5] },
      acceleration: { x: this.state[6], y: this.state[7], z: this.state[8] },
      kl: this.state[9],
      kd: this.state[10],
      rolldeg: this.state[11] * 180 / Math.PI
    })

    // Update covariance: P = (I - K*H)*P
    const KH = matrixMultiply(K, H)
    const I_KH = matrixSubtract(createIdentityMatrix(12), KH)
    this.P = matrixMultiply(I_KH, this.P)

    // Store current measured velocity for next iteration's acceleration calculation
    this.lastMeasuredVelocity = { vx, vy, vz }
    this.lastUpdateTime = timestamp
  }

  private updateWingsuitParameters(vN: number, vE: number, vD: number, accelN: number, accelE: number, accelD: number): void {
    const vel = Math.sqrt(vN * vN + vE * vE + vD * vD)
    if (vel < 1.0) return // Skip update at low speeds

    // Use WSE function to calculate parameters
    const [kl, kd, roll] = calculateWingsuitParameters(
      vN, vE, vD, accelN, accelE, accelD,
      this.state[9], this.state[10], this.state[11]
    )

    // Update state with new parameters
    this.state[9] = kl
    this.state[10] = kd
    this.state[11] = roll
  }

  getState(): MotionState & { kl: number, kd: number, roll: number } {
    return {
      position: { x: this.state[0], y: this.state[1], z: this.state[2] },
      velocity: { x: this.state[3], y: this.state[4], z: this.state[5] },
      acceleration: { x: this.state[6], y: this.state[7], z: this.state[8] },
      aMeasured: this.aMeasured,
      aWSE: this.aWSE,
      kl: this.state[9],
      kd: this.state[10],
      roll: this.state[11]
    }
  }

  predictAt(timestamp: number): (MotionState & { kl: number, kd: number, roll: number }) | undefined {
    if (this.lastUpdateTime === undefined) return undefined
    let result
    const deltaTime = (timestamp - this.lastUpdateTime) / 1000
    if (deltaTime <= 0) return this.getState()


    // Predict forward
    const sstate = this.predictstate(deltaTime)
    result = {
      position: { x: sstate[0], y: sstate[1], z: sstate[2] },
      velocity: { x: sstate[3], y: sstate[4], z: sstate[5] },
      acceleration: { x: sstate[6], y: sstate[7], z: sstate[8] },
      aMeasured: this.aMeasured,
      aWSE: this.aWSE,
      kl: sstate[9],
      kd: sstate[10],
      roll: sstate[11]
    }

    return result
  }
  reset(): void {
    // Reset state vector
    this.state = new Array(12).fill(0)
    this.state[9] = 0.0005 // Initial kl estimate for wingsuit
    this.state[10] = 0.0005 // Initial kd estimate for wingsuit
    this.state[11] = 0.0 // Initial roll estimate for wingsuit

    // Reset covariance matrix
    this.P = createIdentityMatrix(12)
    for (let i = 0; i < 9; i++) {
      this.P[i][i] = 1000 // Initial uncertainty for position, velocity, acceleration
    }
    this.P[9][9] = 0.01   // kl uncertainty
    this.P[10][10] = 0.01 // kd uncertainty  
    this.P[11][11] = 0.005 // roll uncertainty

    // Reset timestamps and origin
    this.lastUpdateTime = undefined
    this.originGps = undefined
    this.lastMeasuredVelocity = undefined
  }

  setProcessNoise(position: number, velocity: number, acceleration: number, wingsuit: number): void {
    // Update process noise covariance matrix Q
    for (let i = 0; i < 3; i++) {
      this.Q[i][i] = position // Position process noise
      this.Q[i + 3][i + 3] = velocity // Velocity process noise
      this.Q[i + 6][i + 6] = acceleration // Acceleration process noise
    }
    this.Q[9][9] = wingsuit   // kl process noise
    this.Q[10][10] = wingsuit // kd process noise
    this.Q[11][11] = wingsuit // roll process noise
  }

  setMeasurementNoise(position: number, velocity: number): void {
    // Store the offset values that will be applied to FlySight accuracy values
    this.measurementNoiseOffsets.position = position
    this.measurementNoiseOffsets.velocity = velocity
    // Acceleration offset is not used since we removed that slider
    this.measurementNoiseOffsets.acceleration = 0
  }

  setCalculationMethod(method: CalculationMethod): void {
    this.calculationMethod = method
  }

  getCalculationMethod(): CalculationMethod {
    return this.calculationMethod
  }

  // match filter state to polar!  todo: roll first to get direction then coeffs...
public matchAerodynamicModel(
  measuredAccel: Vector3,// from filter
  velocity: Vector3,
  roll: number,
  candidates: Coefficients[],
  aoas: number[],
  rho: number,
  s: number,
  m: number,
): { bestCoeff: Coefficients; expectedAccel: Vector3, aoa:number } {
  let minResidual = Infinity;
  let best = candidates[0];
  let bestAccel = vec();
  let bestaoa = 0;

  for (let i=0; i<candidates.length; i++) {
    const candidate = candidates[i];
    const k = .5 * rho* s / m
    const kl = candidate.cl * k / gravity
    const kd = candidate.cd * k / gravity
    const predictedAccelArr = calculateWingsuitAcceleration(velocity.z, velocity.x, -velocity.y, roll, kl,kd);
    const predictedAccel: Vector3 = { x: predictedAccelArr[0], y: predictedAccelArr[1], z: predictedAccelArr[2] };
    const diff = sub(measuredAccel, predictedAccel);
    const residual = Math.sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

    if (residual < minResidual) {
      minResidual = residual;
      best = candidate;
      bestAccel = predictedAccel;
      bestaoa = aoas[i];
    }
  }

  return { bestCoeff: best, expectedAccel: bestAccel, aoa: bestaoa};
}


}

export class WindKalmanFilter {
  private state: number[] // wind velocity [wx, wy, wz]
  private covariance: number[][]
  private processNoise: number[][]
  private measurementNoise: number[][]

  constructor() {
    this.state = new Array(3).fill(0)
    this.covariance = createIdentityMatrix(3)
    this.processNoise = createIdentityMatrix(3)
    this.measurementNoise = createIdentityMatrix(3)
  
    for (let i = 0; i < 3; i++) {
      this.processNoise[i][i] = 0.1 // Wind process noise
      this.measurementNoise[i][i] = 1.0 // Wind measurement noise
    }
  }

  predict(): void {
    // Assume wind evolves slowly - add process noise to covariance
    this.covariance = matrixAdd(this.covariance, this.processNoise)
  }

  update(measuredWindAccel: Vector3): void {
    // Create measurement vector from Vector3
    const measurement = [measuredWindAccel.x, measuredWindAccel.y, measuredWindAccel.z]
    
    // H matrix is identity for direct observation
    const H = createIdentityMatrix(3)
    
    // Innovation: z - H*x
    const innovation = measurement.map((z, i) => z - this.state[i])
    
    // Innovation covariance: S = H*P*H^T + R
    const HP = matrixMultiply(H, this.covariance)
    const HPHT = matrixMultiply(HP, transpose(H))
    const S = matrixAdd(HPHT, this.measurementNoise)
    
    // Kalman gain: K = P*H^T*S^(-1)
    const PHT = matrixMultiply(this.covariance, transpose(H))
    const S_inv = matrixInverse(S)
    const K = matrixMultiply(PHT, S_inv)
    
    // Update state: x = x + K*innovation
    const Ky = matrixVectorMultiply(K, innovation)
    for (let i = 0; i < 3; i++) {
      this.state[i] += Ky[i]
    }
    
    // Update covariance: P = (I - K*H)*P
    const KH = matrixMultiply(K, H)
    const I_KH = matrixSubtract(createIdentityMatrix(3), KH)
    this.covariance = matrixMultiply(I_KH, this.covariance)
  }

  getWindVelocity(): Vector3 {
    return { x: this.state[0], y: this.state[1], z: this.state[2] }
  }

  reset(): void {
    this.state = new Array(3).fill(0)
    this.covariance = createIdentityMatrix(3)
  }
}
