/**
 * Wingsuit Simulation Engine (WSE) functions
 * 
 * These functions work in North-East-Down (NED) coordinate system for wingsuit physics calculations.
 * The output is converted to ENU (East-North-Up) coordinates for compatibility with the motion estimator.
 */

import { Coefficients, SustainedSpeeds, Vector3, WSEQPolar } from "./types"
import { vec } from "./vector"

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



export function calculatesustainedspeeds(kl:number, kd:number){
    const denom = Math.pow(kl*kl+kd*kd,.75)
    return [kl/denom, kd/denom]
}




// Physical constants
export const gravity = 9.80665 // Acceleration due to gravity (m/s^2)
export const gasConst = 8.31447 // Universal gas constant (J/mol/K)

// ISA standard atmosphere
export const rho0 = 1.225 // kg/m^3 ISA density at sea level
export const pressure0 = 101325.0 // ISA pressure at sea level Pa
export const temp0 = 288.15 // ISA temperature 15 degrees celcius
export const lapseRate = 0.0065 // Temperature lapse rate (K/m)
export const mmAir = 0.0289644 // Molar mass of dry air (kg/mol)

// Exponent for barometric formula
const baroExp = gravity * mmAir / (gasConst * lapseRate)

/**
 * Temperature in kelvin at a given altitude.
 * https://en.wikipedia.org/wiki/Lapse_rate
 */
export function temp(altitude: number): number {
  return temp0 - lapseRate * altitude
}

/**
 * Barometric formula
 * https://en.wikipedia.org/wiki/Atmospheric_pressure#Altitude_variation
 * @return pressure in pascals (Pa)
 */
export function altitudeToPressure(altitude: number): number {
  return pressure0 * Math.pow(1 - lapseRate * altitude / temp0, baroExp)
}

/**
 * Calculate air density at given altitude and temperature offset
 */
export function getrho(altitude: number, temperatureOffset: number = 0): number {
  const pressure = altitudeToPressure(altitude)
  const temperature = temp(altitude) + temperatureOffset
  return pressure / (gasConst / mmAir) / temperature
}

export interface wsorientationstate{
    vN: number
    vE: number
    vD: number
    aN: number
    aE: number
    aD: number
    wvN: number
    wvE: number
    wvD: number
}

// expects wse coorsinates outputs filter coordinates
export function computewindadjustedwsorientation( 
    vN: number,
    vE: number,
    vD: number,
    aN: number,
    aE: number,
    aD: number,
    wvN: number,
    wvE: number,
    wvD: number,
    rho: number,
    polar: WSEQPolar
) {

    const gravity = 9.81
    
 // const dtor = Math.PI / 180
 // const trackwind = {
 //   wvn: w.speed * Math.cos(w.direction * dtor) * Math.cos(w.inclination * dtor),
 //   wve: w.speed * Math.sin(w.direction * dtor) * Math.cos(w.inclination * dtor),
 //   wvd: w.speed * Math.sin(-w.inclination * dtor)
 // }

  // airspeed
  const avN = vN + wvN
  const avE = vE + wvE
  const avD = vD + wvD

  const accelN = aN
  const accelE = aE
  const accelD = aD
  const accelDminusG = accelD - gravity

  // Calculate acceleration due to drag
  const vel = Math.sqrt(avN * avN + avE * avE + avD * avD)
  const proj = (accelN * avN + accelE * avE + accelDminusG * avD) / vel

  const dragN = proj * avN / vel
  const dragE = proj * avE / vel
  const dragD = proj * avD / vel

  const accelDrag = Math.sqrt(dragN * dragN + dragE * dragE + dragD * dragD)

  // Calculate acceleration due to lift
  const liftN = accelN - dragN
  const liftE = accelE - dragE
  const liftD = accelDminusG - dragD

  const accelLift = Math.sqrt(liftN * liftN + liftE * liftE + liftD * liftD)
  // calc correct sign. drag should be negative when in same direction as velocity (in aeroplane when climbing)
  const dragsign = -signum(dragN * avN + dragE * avE + dragD * avD)
  // Calculate magic wingsuit coefficients
  const kl = accelLift / gravity / vel / vel
  const kd = accelDrag / gravity / vel / vel * dragsign

  const divisor = Math.pow(kl * kl + kd * kd, 0.75)
  const sustained_x = kl / divisor
  const sustained_y = kd / divisor
  //ss[i] = { vxs: sustained_x, vys: sustained_y }

  // Calculate roll angle
  const smoothhairspeed = Math.sqrt(avN * avN + avE * avE)
  const rollMagnitude = Math.acos((1 - accelD / gravity - kd * vel * avD) / (kl * smoothhairspeed * vel))
  const rollSign = signum(liftN * -avE + liftE * avN)
  const roll = rollSign * rollMagnitude
  ///if (!isNaN(roll)) {
  /// point.roll = roll
  //console.log("roll",roll,"old",point.roll)
  ///}
  const vs = Math.sqrt(sustained_x * sustained_x + sustained_y * sustained_y)
  //const airPressure = altitudeToPressure(data.alt)
  //const temperature = temp(data.alt) 
  //+ 20 // temp offset
  const trho = rho//airPressure / (gasConst / mmAir) / (temperature + temperatureOffset)
  let susi = getsustainedindex(vs, polar, trho).n
  //if (susi < 0) {
  //  const maxg = getmaxg(polar, trho)
   // const mgvs = Math.sqrt(maxg.vxs * maxg.vxs + maxg.vys * maxg.vys)
   // if (vs < mgvs) {
   //   susi = 0
   // } else {
   //   susi = 1
   // }
  //}
  const wsaoa = indexaoa(susi, polar.aoaindexes, polar.aoas) * Math.PI / 180

  const p = {windadjustedsustainedspeeds:{vxs:sustained_x,vys:sustained_y},asx:avE, asy:avN, asz:-avD , aroll:roll, aswsaoa: wsaoa, wlx: liftE, wly:-liftD, wlz:liftN, wdx:dragE, wdy:-dragD, wdz:dragN }
  //console.log(p, data)
  return p
}

export function coefftoss(cl: number, cd: number, s: number, m: number, rho: number): SustainedSpeeds {
  const k = .5 * rho * s / m
  const kl = cl * k / gravity
  const kd = cd * k / gravity
  const denom = Math.pow(kl * kl + kd * kd, 0.75)
  return { vxs: kl / denom, vys: kd / denom }
}

// input index is polar index, returns aoa @ corisponding polar index
export function indexaoa(inputindex: number, aoaindexes: number[], aoas: number[]): number {
  if (inputindex >= 1) return aoas[aoas.length - 1]
  if (inputindex < 0) return aoas[0]
  const topi = aoaindexes.findIndex((i) => inputindex < i)
  const bottomi = topi - 1
  const alpha = (inputindex - aoaindexes[bottomi]) / (aoaindexes[topi] - aoaindexes[bottomi])
  return aoas[bottomi] * (1 - alpha) + aoas[topi] * alpha
}

// getsustained finds the sustained speeds for a given vs
export function getsustainedindex(vs: number, polar: WSEQPolar, rho: number): { ss: SustainedSpeeds, n: number } {
  let l = 0
  let r = 1
  const maxIterations = 100
  let iterations = 0
  
  while (l < r && iterations < maxIterations) {
    const m = l + (r - l) / 2
    const speedx = getspeed(m, polar, rho)
    const tot = Math.pow(speedx.vxs * speedx.vxs + speedx.vys * speedx.vys, .5)
    
    if (Math.abs(tot - vs) < .1) {
      return { ss: speedx, n: m }
    }
    
    if (vs > tot) {
      if (l === m) break // Prevent infinite loop
      l = m
    } else {
      r = m
    }
    
    iterations++
  }

  return { ss: { vys: -1, vxs: -1 }, n: -1 }
}

// getspeed takes a number between 0 and 1, polar, rho and returns sustained speeds
// can be used with a polar of any shape
// numbers less than 0 map to the stall polar
export function getspeed(speed: number, polar: WSEQPolar, rho: number): SustainedSpeeds {
  // can be used with a polar of any shape
  // handle lookup table first
  // linear interpolation between points
  if (polar.stallpoint) {
    if (speed < 0) {
      //check if stall point data exists
      const cf = indexstallarray(speed, polar.stallpoint)
      const speeds = coefftoss(cf.cl, cf.cd, polar.s, polar.m, rho)
      return speeds
    }

  }
 

  // continuious polars need to index stall with drag and middle and speed with lift
  let cl: number
  let cd: number
  //  Do linear polar first and overwrite values for other cases
  cl = polar.rangemaxcl - speed * (polar.rangemaxcl - polar.rangemincl)
  // const acl = polar.rangemincl*speed+polar.rangemaxcl*(1-speed)
  // console.log(cl,acl)
  cd = polar.polarslope * Math.pow(cl - polar.polarclo, 2) + polar.polarmindrag

  // speed
  if (polar.clspeed && polar.cdspeed && polar.clspeedsep && polar.cdspeedsep) {// if speed polar is defined
    const indexatsep = (polar.rangemaxcl - polar.clspeedsep) / (polar.rangemaxcl - polar.rangemincl)
    if (speed > indexatsep) {// use speed polar
      // get slope from standard form
      const speedslope = (polar.clspeedsep - polar.clspeed) / Math.pow(polar.cdspeedsep - polar.cdspeed, 2)
      // get 0 lift point
      // let maxspdcl = polar.clspeed + Math.sqrt(polar.clspeed / speedslope)// may be <0?
      const speedindex = (speed - indexatsep) / (1 - indexatsep) // 1- speed/indexatsep
      cl = polar.clspeedsep - speedindex * (polar.clspeedsep - polar.rangemincl)
      cd = speedslope * Math.pow(cl - polar.clspeed, 2) + polar.cdspeed
    }
  }
  // stall polar
  if (polar.clstallsep && polar.cdstallsep && polar.clstall && polar.cdstall && polar.stallmaxdrag) {// if stall point is defined
    // find index at seperation point
    const indexatsep = (polar.clstall - polar.clstallsep) / (polar.rangemaxcl - polar.rangemincl)
    if (speed < indexatsep) {// use stall polar
      // get slope from standard form
      const stallslope = (polar.clstallsep - polar.clstall) / Math.pow(polar.cdstallsep - polar.cdstall, 2)
      // re index stall range
      const stallindex = 1 - speed / indexatsep // 1- speed/indexatsep
      // get cd
      cd = polar.cdstallsep + (polar.stallmaxdrag - polar.cdstallsep) * stallindex
      // get cl
      cl = stallslope * Math.pow(cd - polar.cdstall, 2) + polar.clstall
      // console.log('indexes',speed,stallindex,indexatsep,stallslope,cd,polar.clstallsep,polar.clstall,polar.cdstallsep,polar.cdstall)
    }
  }
  const k = .5 * rho * polar.s / polar.m
  const kl = cl * k / gravity
  const kd = cd * k / gravity
  const speeds= {
    vys: kd / Math.pow(kl * kl + kd * kd, .75),
    vxs: kl / Math.pow(kl * kl + kd * kd, .75)
  }
  return speeds
}

function indexstallarray(speed: number, arr: Coefficients[]): Coefficients {

  if (speed <= -1) return { cl: arr[0].cl, cd: arr[0].cd }
  if (speed <= 0) return { cl: arr[arr.length - 1].cl, cd: arr[arr.length - 1].cd }
  const ifl = Math.floor((speed + 1) * (arr.length - 1))
  const icl = ifl + 1 //Math.ceil(speed * (polar.table.length - 1))
  const cb = arr[ifl]
  const ct = arr[icl]
  // const alpha = speed * (polar.table.length - 1) / (icl - ifl)
  const i = (speed + 1) * (arr.length - 1) - ifl

  return {
    cl: cb.cl + i * (ct.cl - cb.cl),
    cd: cb.cd + i * (ct.cd - cb.cd)

  }
}

// getmaxg  uses standard form of max glide from quadratic
export function getmaxg(polar: WSEQPolar, rho: number): SustainedSpeeds {

  if (polar.stallpoint) {
    const maxg = polar.stallpoint.reduce((a, b) => { return a.cl / a.cd > b.cl / b.cd ? a : b })
    return coefftoss(maxg.cl, maxg.cd, polar.s, polar.m, rho)
  }
  if (polar.table) {// discrete
    const maxg = polar.table.reduce((a, b) => { return a.cl / a.cd > b.cl / b.cd ? a : b })
    const k = .5 * rho * polar.s / polar.m
    const kl = maxg.cl * k / gravity
    const kd = maxg.cd * k / gravity
    const speeds: SustainedSpeeds = {
      vys: kd / Math.pow(kl * kl + kd * kd, .75),
      vxs: kl / Math.pow(kl * kl + kd * kd, .75)
    }

    // console.log("\t" + "getmaxg" + "\t" + speeds.vxs * 2.237 + "\t" + speeds.vys * 2.237 + "\t" + speeds.vxs / speeds.vys)
    return speeds
  }
  // continuious



  
  const cl = Math.pow(polar.polarclo * polar.polarclo + polar.polarmindrag / polar.polarslope, .5)
  const cd = polar.polarslope * Math.pow(cl - polar.polarclo, 2) + polar.polarmindrag
  const k = .5 * rho * polar.s / polar.m
  const kl = cl * k / gravity
  const kd = cd * k / gravity
  const speeds: SustainedSpeeds = {
    vys: kd / Math.pow(kl * kl + kd * kd, .75),
    vxs: kl / Math.pow(kl * kl + kd * kd, .75)
  }
  return speeds
}
/**
 const gravity = 9.81

// returns shortened [] if trackwind is shorter
export function computewindadjustedsustainedspeeds(data: Point[], trackacceleration: any, trackwind: any, startindex?: number): SustainedSpeeds[] {

  //const data = track.data
  const start = startindex ? startindex : 0
  const s = { vxs: 0, vys: 0 }
  const ss: SustainedSpeeds[] = [s, s]
  // Compute smoothed speeds
  //smoothPoints(data)
  // console.log(`Smooth processing took ${new Date().getTime() - timerStart} ms`)

  for (let i = 0; i < trackwind.length; i++) {
    if (i + start < data.length) {
      const point = data[i + start]
      // airspeed
      const avN = point.smooth.vN + trackwind[i].wvn
      const avE = point.smooth.vE + trackwind[i].wve
      const avD = point.smooth.vD + trackwind[i].wvd
      const accelN = trackacceleration[i + start].an
      const accelE = trackacceleration[i + start].ae
      const accelD = trackacceleration[i + start].ad
      const accelDminusG = accelD - gravity

      // Calculate acceleration due to drag
      const vel = Math.sqrt(avN * avN + avE * avE + avD * avD)
      const proj = (accelN * avN + accelE * avE + accelDminusG * avD) / vel

      const dragN = proj * avN / vel
      const dragE = proj * avE / vel
      const dragD = proj * avD / vel
      // point.dragN = dragN
      // point.dragE = dragE
      //point.dragD = dragD
      const accelDrag = Math.sqrt(dragN * dragN + dragE * dragE + dragD * dragD)

      // Calculate acceleration due to lift
      const liftN = accelN - dragN
      const liftE = accelE - dragE
      const liftD = accelDminusG - dragD
      // point.liftN = liftN
      // point.liftE = liftE
      // point.liftD = liftD
      const accelLift = Math.sqrt(liftN * liftN + liftE * liftE + liftD * liftD)
      // calc correct sign. drag should be negative when in same direction as velocity (in aeroplane when climbing)
      const dragsign = -signum(dragN * avN + dragE * avE + dragD * avD)
      // Calculate magic wingsuit coefficients
      const kl = accelLift / gravity / vel / vel
      const kd = accelDrag / gravity / vel / vel * dragsign

      const divisor = Math.pow(kl * kl + kd * kd, 0.75)
      const sustained_x = kl / divisor
      const sustained_y = kd / divisor
      ss[i] = { vxs: sustained_x, vys: sustained_y }

      // Calculate roll angle
      //const smoothhairspeed = Math.sqrt(avN * avN + avE * avE)
      //const rollMagnitude = Math.acos((1 - accelD / gravity - kd * vel * avD) / (kl * smoothhairspeed * vel))
      //const rollSign = signum(liftN * -avE + liftE * avN)
      //const roll = rollSign * rollMagnitude
      //if (!isNaN(roll)) {
      //point.roll = roll
      //console.log("roll",roll,"old",point.roll)
      //}
    }
  }
  //console.log(ss, start)
  return ss
}

// returns shortened [] if trackwind is shorter
export function computewindadjustedwsorientation(data: Point, trackacceleration: any, w: any) {

  const dtor = Math.PI / 180
  const trackwind = {
    wvn: w.speed * Math.cos(w.direction * dtor) * Math.cos(w.inclination * dtor),
    wve: w.speed * Math.sin(w.direction * dtor) * Math.cos(w.inclination * dtor),
    wvd: w.speed * Math.sin(-w.inclination * dtor)
  }

  // airspeed
  const avN = data.smooth.vN + trackwind.wvn
  const avE = data.smooth.vE + trackwind.wve
  const avD = data.smooth.vD + trackwind.wvd

  const accelN = trackacceleration.an
  const accelE = trackacceleration.ae
  const accelD = trackacceleration.ad
  const accelDminusG = accelD - gravity

  // Calculate acceleration due to drag
  const vel = Math.sqrt(avN * avN + avE * avE + avD * avD)
  const proj = (accelN * avN + accelE * avE + accelDminusG * avD) / vel

  const dragN = proj * avN / vel
  const dragE = proj * avE / vel
  const dragD = proj * avD / vel
  // point.dragN = dragN
  // point.dragE = dragE
  //point.dragD = dragD
  const accelDrag = Math.sqrt(dragN * dragN + dragE * dragE + dragD * dragD)

  // Calculate acceleration due to lift
  const liftN = accelN - dragN
  const liftE = accelE - dragE
  const liftD = accelDminusG - dragD
  // point.liftN = liftN
  // point.liftE = liftE
  // point.liftD = liftD
  const accelLift = Math.sqrt(liftN * liftN + liftE * liftE + liftD * liftD)
  // calc correct sign. drag should be negative when in same direction as velocity (in aeroplane when climbing)
  const dragsign = -signum(dragN * avN + dragE * avE + dragD * avD)
  // Calculate magic wingsuit coefficients
  const kl = accelLift / gravity / vel / vel
  const kd = accelDrag / gravity / vel / vel * dragsign

  const divisor = Math.pow(kl * kl + kd * kd, 0.75)
  const sustained_x = kl / divisor
  const sustained_y = kd / divisor
  //ss[i] = { vxs: sustained_x, vys: sustained_y }

  // Calculate roll angle
  const smoothhairspeed = Math.sqrt(avN * avN + avE * avE)
  const rollMagnitude = Math.acos((1 - accelD / gravity - kd * vel * avD) / (kl * smoothhairspeed * vel))
  const rollSign = signum(liftN * -avE + liftE * avN)
  const roll = rollSign * rollMagnitude
  ///if (!isNaN(roll)) {
  /// point.roll = roll
  //console.log("roll",roll,"old",point.roll)
  ///}
  const vs = Math.sqrt(sustained_x * sustained_x + sustained_y * sustained_y)
  const airPressure = altitudeToPressure(data.alt)
  const temperature = temp(data.alt) //+ 20 // temp offset
  const trho = airPressure / (gasConst / mmAir) / (temperature + 20)
  let susi = getsustainedindex(vs, corvidpolar, trho).n
  if (susi < 0) {
    const maxg = getmaxg(corvidpolar, trho)
    const mgvs = Math.sqrt(maxg.vxs * maxg.vxs + maxg.vys * maxg.vys)
    if (vs < mgvs) {
      susi = 0
    } else {
      susi = 1
    }
  }
  const wsaoa = indexaoa(susi, genericwingsuitaoaindexes, genericwingsuitaoas) * Math.PI / 180
  //point.smooth.vD = avD
  //point.smooth.vN = avN
  //point.smooth.vE = avE
  const p = { ...data, smooth: { vN: avN, vE: avE, vD: avD }, roll: roll, wsaoa: wsaoa, vN: liftN, vE: liftE, climb: liftD, lat: dragN, lng: dragE, alt: dragD }
  //console.log(p, data)
  return p
}

export function computewindadjustedcanopyorientation(data: Point[], trackacceleration: any, trackwind: any, startindex: number, fl: number, currentmillis: number) {
  const wsdata = []
  const start = startindex ? startindex : 0
  //reindex
  let editi = 0
  for (let i = 0; i < trackwind.length; i++) {
    const w = trackwind[i]
    if (i + start < data.length) {
      wsdata[i] = computewindadjustedwsorientation(data[i + start], trackacceleration[i + start], w)
      if (wsdata[i].millis == currentmillis) editi = i
    }
  }
  const cdata = []
  for (let i = 0; i < trackwind.length; i++) {
    const w = trackwind[i]
    const point = wsdata[i]
    const cpoint: Cpoint = {
      millis: 0,
      date: new Date(),
      // pilots drag
      pdragN: 0,
      pdragE: 0,
      pdragD: 0,
      // canopy position
      // relative to pilot
      cpE: 0,
      cpD: 0,
      cpN: 0,
      // global position
      cplat: 0,
      cplng: 0,
      cpalt: 0,
      // canopy speed
      vcpE: 0,
      vcpD: 0,
      vcpN: 0,
      cpdragE: 0,
      cpdragN: 0,
      cpdragD: 0,
      cpliftE: 0,
      cpliftN: 0,
      cpliftD: 0,
      cpaoa: 0,
      cpcl: 0,
      cpcd: 0,
      // local canopy rotation speed relative to pilot
      rotE: 0,
      rotD: 0,
      rotN: 0,
      // canopy normal
      cnormN: 0,
      cnormE: 0,
      cnormD: 0,
      cpvxs: 0,
      cpvys: 0,
      index: 0
    }
    const vN = point.smooth.vN
    const vE = point.smooth.vE
    const vD = point.smooth.vD
    const vel = Math.sqrt(vN * vN + vE * vE + vD * vD)
    // get exit weight from user!!
    const m = 77.5
    const s = 2
    cpoint.millis = point.millis
    cpoint.date = point.date
    //cpoint.index = i

    // calculate canopy relative position
    // calculate and subtract pilots acceldrag from total accel
    // estimate pilots drag
    // const pilotkd = 35 / Math.pow(35 * 35, 1.5)
    // const pilotacceldrag = vel * vel * pilotkd * gravity
    const dynamicPressure = point.rho * vel * vel / 2
    const pilotcd = .35
    const pilotacceldrag = pilotcd * dynamicPressure * s / m

    // pilots drag
    const pdragN = -vN / vel * pilotacceldrag
    const pdragE = -vE / vel * pilotacceldrag
    const pdragD = -vD / vel * pilotacceldrag

    cpoint.pdragN = pdragN
    cpoint.pdragE = pdragE
    cpoint.pdragD = pdragD

    // subtract pilots drag from total aerodynamic force to get canopy normal

    const cnormN = point.vN + point.lat - pdragN
    const cnormE = point.vE + point.lng - pdragE
    const cnormD = point.climb + point.alt - pdragD
    // console.log(point.millis, point.rho, dynamicPressure, pilotacceldrag, point.liftN, point.dragN, pdragN)
    // canopy position relative to pilot
    // 5.1m from pilots cg to aerodynamic center
    const linelength = 3 // 4.15//m changed from 5 to lower center of rotation
    const magf = Math.pow(cnormN * cnormN + cnormE * cnormE + cnormD * cnormD, .5)
    const cpE = linelength * cnormE / magf
    const cpD = linelength * cnormD / magf
    const cpN = linelength * cnormN / magf
    cpoint.cpE = cpE
    cpoint.cpN = cpN
    cpoint.cpD = cpD
    cpoint.cplat = point.lat + cpN / 111320
    cpoint.cplng = point.lng + cpE / (111320 * Math.cos(point.lat * Math.PI / 180))
    cpoint.cpalt = point.alt - cpD

    cdata[i] = cpoint
    // console.log(cpoint,point)
  }

  // Calculate canopy speed
  const i = editi
  const w = trackwind[i]
  const point = wsdata[i]
  const cpoint = cdata[i]
  const vN = point.smooth.vN
  const vE = point.smooth.vE
  const vD = point.smooth.vD


  if (!fl) fl = 5
  const localPoints = lslice(i, cdata, fl)
  // Normalize time, because otherwise math explodes
  const startTime = localPoints[0].millis
  const getT = (d: Cpoint) => (d.millis - startTime) * 1e-3
  // Compute rotation speed m/sec
  cpoint.rotE = getSlope(localPoints, getT, (d) => d.cpE)
  cpoint.rotD = getSlope(localPoints, getT, (d) => d.cpD)
  cpoint.rotN = getSlope(localPoints, getT, (d) => d.cpN)
  // Compute canopy speed
  cpoint.vcpE = vE + cpoint.rotE
  cpoint.vcpD = vD + cpoint.rotD
  cpoint.vcpN = vN + cpoint.rotN
  // calculate canopy AOA
  // get AOA
  // angle between 2 vectors:
  // take dot product
  const dot = cpoint.cpE * cpoint.vcpE + cpoint.cpD * cpoint.vcpD + cpoint.cpN * cpoint.vcpN
  // subtract from pi rad to get anglenv
  const anglenv = Math.PI - Math.acos(dot / (Math.pow(cpoint.cpE * cpoint.cpE + cpoint.cpD * cpoint.cpD + cpoint.cpN * cpoint.cpN, .5) * Math.pow(cpoint.vcpE * cpoint.vcpE + cpoint.vcpD * cpoint.vcpD + cpoint.vcpN * cpoint.vcpN, .5)))
  const trimangle = Math.PI / 2 + 3 * Math.PI / 180// 3deg
  cpoint.cpaoa = Math.PI - trimangle - anglenv
  //point.cpaoa = Math.PI - trimangle - anglenv
  cpoint.rotN = wsdata[i].roll
  return cpoint

}

*/