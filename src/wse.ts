/**
 * Wingsuit Simulation Engine (WSE) functions
 * 
 * These functions work in North-East-Down (NED) coordinate system for wingsuit physics calculations.
 * The output is converted to ENU (East-North-Up) coordinates for compatibility with the motion estimator.
 */

/**
 * Sign function helper
 */
function signum(x: number): number {
  return x > 0 ? 1 : x < 0 ? -1 : 0
}

/**
 * Calculate wingsuit acceleration based on velocity and wingsuit parameters
 * 
 * @param vN North velocity (m/s)
 * @param vE East velocity (m/s) 
 * @param vD Down velocity (m/s)
 * @param kl Lift coefficient
 * @param kd Drag coefficient
 * @param roll Roll angle (radians)
 * @returns [aE, aU, aN] - acceleration in ENU coordinates (East, Up, North) in m/s²
 */
export function calculateWingsuitAcceleration(vN: number, vE: number, vD: number, kl: number, kd: number, roll: number): [number, number, number] {
  const g = 9.81
  
  const v = Math.sqrt(vN * vN + vD * vD + vE * vE)

  if (v < 0.1) return [0, -g, 0] // Handle near-zero velocity

  const groundSpeed = Math.sqrt(vN * vN + vE * vE)
  if (groundSpeed < 0.1) return [0, -g, 0] // Handle near-zero groundspeed

  const cosRoll = Math.cos(roll)
  const sinRoll = Math.sin(roll)

  // Calculate acceleration in NDE coordinates
  const aN = g * (kl * v / groundSpeed * (vN * vD * cosRoll - vE * v * sinRoll) - kd * vN * v)
  const aD = g * (1 - kl * v * groundSpeed * cosRoll - kd * vD * v)
  const aE = g * (kl * v / groundSpeed * (vE * vD * cosRoll + vN * v * sinRoll) - kd * vE * v)

  // Convert back to ENU coordinates
  return [aE, -aD, aN]
}

/**
 * Calculate wingsuit parameters from measured acceleration
 * 
 * @param vN North velocity (m/s)
 * @param vE East velocity (m/s)
 * @param vD Down velocity (m/s)
 * @param accelN North acceleration (m/s²)
 * @param accelE East acceleration (m/s²)
 * @param accelD Down acceleration (m/s²)
 * @param currentKl Current lift coefficient (used as fallback at low speeds)
 * @param currentKd Current drag coefficient (used as fallback at low speeds)
 * @param currentRoll Current roll angle (used as fallback at low speeds)
 * @returns [kl, kd, roll] - wingsuit parameters
 */
export function calculateWingsuitParameters(
  vN: number, 
  vE: number, 
  vD: number, 
  accelN: number, 
  accelE: number, 
  accelD: number,
  currentKl: number,
  currentKd: number, 
  currentRoll: number
): number[] {
  const gravity = 9.81
  const accelDminusG = accelD - gravity

  // Calculate acceleration due to drag (projection onto velocity)
  const vel = Math.sqrt(vN * vN + vE * vE + vD * vD)
  if (vel < 1.0) return [currentKl, currentKd, currentRoll] // Return current values at low speeds

  const proj = (accelN * vN + accelE * vE + accelDminusG * vD) / vel

  const dragN = proj * vN / vel
  const dragE = proj * vE / vel
  const dragD = proj * vD / vel
  // Calculate correct sign for drag
  const dragSign = -signum(dragN * vN + dragE * vE + dragD * vD)

  const accelDrag = dragSign * Math.sqrt(dragN * dragN + dragE * dragE + dragD * dragD)

  // Calculate acceleration due to lift (rejection from velocity)
  const liftN = accelN - dragN
  const liftE = accelE - dragE
  const liftD = accelDminusG - dragD
  const accelLift = Math.sqrt(liftN * liftN + liftE * liftE + liftD * liftD)

  // Calculate wingsuit coefficients
  const kl = accelLift / gravity / vel / vel
  const kd = accelDrag / gravity / vel / vel

  // Calculate roll angle
  const smoothGroundspeed = Math.sqrt(vN * vN + vE * vE)
  let roll = currentRoll // Default to current roll
  
  if (smoothGroundspeed > 1.0) {
    const rollArg = (1 - accelD / gravity - kd * vel * vD) / (kl * smoothGroundspeed * vel)
    if (Math.abs(rollArg) <= 1.0) {
      const rollMagnitude = Math.acos(rollArg)
      const rollSign = signum(liftN * -vE + liftE * vN)
      roll = rollSign * rollMagnitude
    }
  }

  return [kl, kd, roll]
}
