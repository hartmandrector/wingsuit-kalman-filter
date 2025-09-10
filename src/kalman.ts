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
import { Vector3, MLocation, Coefficients, WSEQPolar } from './types.js'
import { MotionState } from './motionestimator.js'
import { calculateWingsuitAcceleration, calculateWingsuitParameters, gravity, computewindadjustedwsorientation, rho0 } from './wse.js'
import { vec, sub, add } from './vector.js'
import { aurafivepolar } from './polar-library.js'

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
  
  // Wind estimation components
  private windFilter: WindKalmanFilter
  private polar: WSEQPolar = aurafivepolar // Default polar
  private estimatedWind: Vector3 = { x: 0, y: 0, z: 0 }
  private windAdjustedAoA: number = 0
  private windAdjustedRoll: number = 0
  private windsustainedSpeeds: { vxs: number, vys: number } = { vxs: 0, vys: 0 }
  private windadjustedcoefficients: Coefficients = { cl: 0, cd: 0 }
  
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

    // Initialize wind filter
    this.windFilter = new WindKalmanFilter()

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

    // Update wind estimation using the time delta
    if (this.lastUpdateTime !== undefined) {
      const deltaTime = (timestamp - this.lastUpdateTime) / 1000
      this.updateWindEstimation(deltaTime)
    }

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

  getState(): MotionState & { kl: number, kd: number, roll: number, windEstimate?: Vector3, windAdjustedAoA?: number, windAdjustedRoll: number, windsustainedSpeeds: { vxs: number, vys: number }, windadjustedcoefficients: Coefficients } {
    const windData = this.getWindEstimate()
    return {
      position: { x: this.state[0], y: this.state[1], z: this.state[2] },
      velocity: { x: this.state[3], y: this.state[4], z: this.state[5] },
      acceleration: { x: this.state[6], y: this.state[7], z: this.state[8] },
      aMeasured: this.aMeasured,
      aWSE: this.aWSE,
      kl: this.state[9],
      kd: this.state[10],
      roll: this.state[11],
      windEstimate: windData.windEstimate,
      windAdjustedAoA: windData.windAdjustedAoA,
      windAdjustedRoll: windData.windAdjustedRoll,
      windsustainedSpeeds: windData.windsustainedSpeeds,
      windadjustedcoefficients: windData.windadjustedcoefficients
    }
  }

  predictAt(timestamp: number): (MotionState & { kl: number, kd: number, roll: number, windEstimate?: Vector3, windAdjustedAoA?: number, windAdjustedRoll: number, windsustainedSpeeds: { vxs: number, vys: number }, windadjustedcoefficients: Coefficients }) | undefined {
    if (this.lastUpdateTime === undefined) return undefined
    let result
    const deltaTime = (timestamp - this.lastUpdateTime) / 1000
    if (deltaTime <= 0) return this.getState()


    // Predict forward
    const sstate = this.predictstate(deltaTime)
    const windData = this.getWindEstimate()
    result = {
      position: { x: sstate[0], y: sstate[1], z: sstate[2] },
      velocity: { x: sstate[3], y: sstate[4], z: sstate[5] },
      acceleration: { x: sstate[6], y: sstate[7], z: sstate[8] },
      aMeasured: this.aMeasured,
      aWSE: this.aWSE,
      kl: sstate[9],
      kd: sstate[10],
      roll: sstate[11],
      windEstimate: windData.windEstimate,
      windAdjustedAoA: windData.windAdjustedAoA,
      windAdjustedRoll: windData.windAdjustedRoll,
      windsustainedSpeeds: windData.windsustainedSpeeds,
      windadjustedcoefficients: windData.windadjustedcoefficients
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

    // Reset wind estimation
    this.windFilter.reset()
    this.estimatedWind = { x: 0, y: 0, z: 0 }
    this.windAdjustedAoA = 0
    this.windAdjustedRoll = 0

    // Reset timestamps and origin
    this.lastUpdateTime = undefined
    this.originGps = undefined
    this.lastMeasuredVelocity = undefined
  }

  setProcessNoise(position: number, velocity: number, acceleration: number, kl: number, kd: number, roll: number): void {
    // Update process noise covariance matrix Q
    for (let i = 0; i < 3; i++) {
      this.Q[i][i] = position // Position process noise
      this.Q[i + 3][i + 3] = velocity // Velocity process noise
      this.Q[i + 6][i + 6] = acceleration // Acceleration process noise
    }
    this.Q[9][9] = kl     // kl process noise
    this.Q[10][10] = kd   // kd process noise
    this.Q[11][11] = roll // roll process noise
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

  // Wind estimation functions
  private updateWindEstimation(deltaTime: number): void {
    // Get current state from main filter
    const velocity: Vector3 = { x: this.state[3], y: this.state[4], z: this.state[5] }
    const acceleration: Vector3 = { x: this.state[6], y: this.state[7], z: this.state[8] }
    
    // Step 1: Predict wind filter state forward in time
    this.windFilter.predict()
    
    // Step 2: Find optimal wind velocity that minimizes acceleration residual
    const optimalWind = this.optimizeWindVelocity(velocity, acceleration)
    
    // Step 3: Use optimal wind as "measurement" for wind filter update
    // The innovation will be the difference between predicted wind and optimal wind
    const windMeasurement = optimalWind.windVelocity
    this.windFilter.update(windMeasurement)
    
    // Step 4: Get updated wind estimate from filter
    this.estimatedWind = this.windFilter.getWindVelocity()
    
    // Step 5: Update wind-adjusted parameters using final wind estimate
    const windAdjustedorientation = computewindadjustedwsorientation(
      velocity.z,    // vN (North)
      velocity.x,    // vE (East) 
      -velocity.y,   // vD (Down - note sign flip)
      acceleration.z, // aN
      acceleration.x, // aE
      -acceleration.y, // aD
      this.estimatedWind.z,  // wvN
      this.estimatedWind.x,  // wvE
      -this.estimatedWind.y, // wvD
      0.9, // rho
      this.polar
    )
    
    // Store wind-adjusted parameters - use AoA from optimization instead of computewindadjustedwsorientation
    this.windAdjustedAoA = optimalWind.bestAoA  // Use the AoA from matchAerodynamicModel optimization
    this.windadjustedcoefficients = optimalWind.bestCoeff
    this.windsustainedSpeeds = { vxs: windAdjustedorientation.windadjustedsustainedspeeds.vxs, vys: windAdjustedorientation.windadjustedsustainedspeeds.vys }
    this.windAdjustedRoll = windAdjustedorientation.aroll
  }

  // Optimize wind velocity using gradient descent to minimize a composite objective function
  private optimizeWindVelocity(velocity: Vector3, measuredAccel: Vector3): {
    windVelocity: Vector3,
    bestCoeff: Coefficients,
    minResidual: number,
    bestAoA: number
  } {
    // Generate candidate coefficients from polar
    const candidates: Coefficients[] = []
    const aoas: number[] = []
    
    for (let i = 0; i < this.polar.aoas.length; i++) {
      const aoa = this.polar.aoas[i]
      const c = this.polar.stallpoint[i]
      candidates.push({ cl: c.cl, cd: c.cd })
      aoas.push(aoa)
    }
    
    const rho = 0.9
    const s = this.polar.s
    const m = this.polar.m
    
    // Define objective function that balances wind speed minimization and acceleration fit
    const objectiveFunction = (wind: Vector3): { cost: number, coeff: Coefficients, residual: number, aoa: number } => {
      const airspeedVelocity = add(velocity, wind)
      
      const bestFit = this.matchAerodynamicModel(
        measuredAccel,
        airspeedVelocity,
        this.state[11], // current roll
        candidates,
        aoas,
        rho,
        s,
        m
      )
      
      const residual = Math.sqrt(
        Math.pow(measuredAccel.x - bestFit.expectedAccel.x, 2) +
        Math.pow(measuredAccel.y - bestFit.expectedAccel.y, 2) +
        Math.pow(measuredAccel.z - bestFit.expectedAccel.z, 2)
      )
      
      const windSpeed = Math.sqrt(wind.x * wind.x + wind.y * wind.y + wind.z * wind.z)
      
      // Composite cost function: prioritize good fit but penalize high wind speeds
      // Scale factors can be tuned based on your preferences
      const residualWeight = 100.0  // Strong penalty for poor acceleration fit
      const windSpeedWeight = 1.0   // Moderate penalty for high wind speeds
      const cost = residualWeight * residual * residual + windSpeedWeight * windSpeed * windSpeed
      
      return { cost, coeff: bestFit.bestCoeff, residual, aoa: bestFit.aoa }
    }
    
    // Gradient descent optimization
    let currentWind = { ...this.estimatedWind }
    let learningRate = 0.05  // Start with small learning rate
    const maxIterations = 50
    const convergenceThreshold = 1e-6
    const epsilon = 1e-4  // For numerical gradient calculation
    
    let bestResult = objectiveFunction(currentWind)
    let bestWind = { ...currentWind }
    
    // Optional: Enable debug logging (set to false for production)
    const debugLogging = false
    
    if (debugLogging) {
      console.log(`Wind optimization starting - Initial cost: ${bestResult.cost.toFixed(6)}, residual: ${bestResult.residual.toFixed(6)}`)
    }
    
    for (let iteration = 0; iteration < maxIterations; iteration++) {
      // Calculate numerical gradient
      const gradient = { x: 0, y: 0, z: 0 }
      
      // Partial derivative with respect to wind.x
      const wind_x_plus = { ...currentWind, x: currentWind.x + epsilon }
      const wind_x_minus = { ...currentWind, x: currentWind.x - epsilon }
      gradient.x = (objectiveFunction(wind_x_plus).cost - objectiveFunction(wind_x_minus).cost) / (2 * epsilon)
      
      // Partial derivative with respect to wind.y
      const wind_y_plus = { ...currentWind, y: currentWind.y + epsilon }
      const wind_y_minus = { ...currentWind, y: currentWind.y - epsilon }
      gradient.y = (objectiveFunction(wind_y_plus).cost - objectiveFunction(wind_y_minus).cost) / (2 * epsilon)
      
      // Partial derivative with respect to wind.z
      const wind_z_plus = { ...currentWind, z: currentWind.z + epsilon }
      const wind_z_minus = { ...currentWind, z: currentWind.z - epsilon }
      gradient.z = (objectiveFunction(wind_z_plus).cost - objectiveFunction(wind_z_minus).cost) / (2 * epsilon)
      
      // Update wind estimate using gradient descent
      const newWind = {
        x: currentWind.x - learningRate * gradient.x,
        y: currentWind.y - learningRate * gradient.y,
        z: currentWind.z - learningRate * gradient.z
      }
      
      // Evaluate new position
      const newResult = objectiveFunction(newWind)
      
      // Check if we improved
      if (newResult.cost < bestResult.cost) {
        bestResult = newResult
        bestWind = { ...newWind }
        currentWind = newWind
        
        // Adaptive learning rate: increase if we're improving
        learningRate = Math.min(learningRate * 1.1, 0.2)
        
        if (debugLogging) {
          const windSpeed = Math.sqrt(newWind.x * newWind.x + newWind.y * newWind.y + newWind.z * newWind.z)
          console.log(`Iter ${iteration}: Improved! Cost: ${newResult.cost.toFixed(6)}, residual: ${newResult.residual.toFixed(6)}, wind speed: ${windSpeed.toFixed(3)} m/s, LR: ${learningRate.toFixed(6)}`)
        }
      } else {
        // Reduce learning rate if we didn't improve
        learningRate *= 0.5
        
        // Stop if learning rate gets too small
        if (learningRate < 1e-6) {
          if (debugLogging) {
            console.log(`Iter ${iteration}: Learning rate too small, stopping optimization`)
          }
          break
        }
      }
      
      // Check for convergence
      const gradientMagnitude = Math.sqrt(gradient.x * gradient.x + gradient.y * gradient.y + gradient.z * gradient.z)
      if (gradientMagnitude < convergenceThreshold) {
        if (debugLogging) {
          console.log(`Iter ${iteration}: Converged! Gradient magnitude: ${gradientMagnitude.toFixed(8)}`)
        }
        break
      }
    }
    
    if (debugLogging) {
      const finalWindSpeed = Math.sqrt(bestWind.x * bestWind.x + bestWind.y * bestWind.y + bestWind.z * bestWind.z)
      console.log(`Wind optimization complete - Final cost: ${bestResult.cost.toFixed(6)}, residual: ${bestResult.residual.toFixed(6)}, wind speed: ${finalWindSpeed.toFixed(3)} m/s`)
    }
    
    return {
      windVelocity: bestWind,
      bestCoeff: bestResult.coeff,
      minResidual: bestResult.residual,
      bestAoA: bestResult.aoa
    }
  }

  // Get wind estimation results
  getWindEstimate(): { 
    windEstimate: Vector3,
    windAdjustedAoA: number,
    windAdjustedRoll: number,
    windsustainedSpeeds: { vxs: number, vys: number },
    windadjustedcoefficients: Coefficients
  } {
  
    return {
      windEstimate: this.estimatedWind,
      windAdjustedAoA: this.windAdjustedAoA,
      windAdjustedRoll: this.windAdjustedRoll,
      windsustainedSpeeds: this.windsustainedSpeeds,
      windadjustedcoefficients: this.windadjustedcoefficients
    }
  }

  // Set custom polar for wind estimation
  setPolar(polar: WSEQPolar): void {
    this.polar = polar
  }

  // Wind filter noise control methods
  setWindProcessNoise(noise: number): void {
    this.windFilter.setProcessNoise(noise)
  }

  setWindMeasurementNoise(noise: number): void {
    this.windFilter.setMeasurementNoise(noise)
  }

  getWindProcessNoise(): number {
    return this.windFilter.getProcessNoise()
  }

  getWindMeasurementNoise(): number {
    return this.windFilter.getMeasurementNoise()
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
    const predictedAccelArr = calculateWingsuitAcceleration(velocity.z, velocity.x, -velocity.y, kl,kd,roll);
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
  
    // Default noise values - can be adjusted via sliders
    for (let i = 0; i < 3; i++) {
      this.processNoise[i][i] = 0.1 // Wind process noise (how quickly wind changes)
      this.measurementNoise[i][i] = 1.0 // Wind measurement noise (trust in wind acceleration residual)
    }
  }

  predict(): void {
    // Assume wind evolves slowly - add process noise to covariance
    this.covariance = matrixAdd(this.covariance, this.processNoise)
  }

  update(measuredWindVelocity: Vector3): void {
    // Create measurement vector from Vector3
    const measurement = [measuredWindVelocity.x, measuredWindVelocity.y, measuredWindVelocity.z]
    
    // H matrix is identity for direct observation of wind velocity
    const H = createIdentityMatrix(3)
    
    // Innovation: z - H*x (measured wind velocity - predicted wind velocity)
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

  // Noise parameter setters for slider control
  setProcessNoise(noise: number): void {
    for (let i = 0; i < 3; i++) {
      this.processNoise[i][i] = noise
    }
  }

  setMeasurementNoise(noise: number): void {
    for (let i = 0; i < 3; i++) {
      this.measurementNoise[i][i] = noise
    }
  }

  // Getters for current noise values (for UI display)
  getProcessNoise(): number {
    return this.processNoise[0][0]
  }

  getMeasurementNoise(): number {
    return this.measurementNoise[0][0]
  }

  getWindVelocity(): Vector3 {
    return { x: this.state[0], y: this.state[1], z: this.state[2] }
  }

  reset(): void {
    this.state = new Array(3).fill(0)
    this.covariance = createIdentityMatrix(3)
  }
}
