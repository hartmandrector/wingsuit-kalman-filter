import { MLocation } from "./types"
import { dotMap } from "./vector"
import { getSlope } from "./utils"

// Legacy Point interface for backward compatibility
interface Point {
  climb: number
  vN: number
  vE: number
  lat: number
  lng: number
  alt: number
  smooth: {
    vD: number
    vN: number
    vE: number
  }
}

// Convolution matrix
// Simple moving average filter
// const c = new Array(size).fill(1 / size)
// Savitzky and Golay 1964. Smoothing and Differentiation of Data by Simplified Least-Squares Procedures.
// Quadratic cubic coefficients
const c = [-78, -13, 42, 87, 122, 147, 162, 167, 162, 147, 122, 87, 42, -13, -78]
// First derivative quadratic coefficients
// const cDerive1 = [-7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7]
// Second derivative quadratic coefficients
// const cDerive2 = [91, 52, 19, -8, -29, -48, -53, -56, -53, -48, -29, -8, 19, 52, 91]
const size = c.length
const half = Math.floor(size / 2)
const sum = c.reduce((a, b) => a + b)
// Normalize C
const cNorm = c.map((x) => x / sum)

// Project out the velocity component
// If you wanted to adjust for altitude, apply trueToIndicated here.
const vD = (point: Point) => -point.climb
const vN = (point: Point) => point.vN
const vE = (point: Point) => point.vE

/**
 * Savitzky-Golay filter for point velocity smoothing
 */
export function smoothPoints(data: Point[]): void {
  // Copy head and tail, apply convolution filter to middle
  for (let i = 0; i < data.length; i++) {
    if (i <= half || data.length - half <= i) {
      // Copy head and tail data
      data[i].smooth = {
        vD: vD(data[i]),
        vN: vN(data[i]),
        vE: vE(data[i])
      }
    } else {
      // Apply convolution filter
      const start = i - half
      const slice = data.slice(start, start + size)
      data[i].smooth = {
        vD: dotMap(slice, cNorm, vD),
        vN: dotMap(slice, cNorm, vN),
        vE: dotMap(slice, cNorm, vE)
      }
      // TODO: Get acceleration from filter
    }
  }
}

// all sg coefficients
// filter lengths: 5,7,9,11,13,15,17,19,21,23,25
const cMX = [
  [-3, 12, 17, 12, -3],
  [-2, 3, 6, 7, 6, 3, -2],
  [-21, 14, 39, 54, 59, 54, 39, 14, -21],
  [-36, 9, 44, 69, 84, 89, 84, 69, 44, 9, -36],
  [-11, 0, 9, 16, 21, 24, 25, 24, 21, 16, 9, 0, -11],
  [-78, -13, 42, 87, 122, 147, 162, 167, 162, 147, 122, 87, 42, -13, -78],
  [-21, -6, 7, 18, 27, 34, 39, 42, 43, 42, 39, 34, 27, 18, 7, -6, -21],
  [-136, -51, 24, 89, 144, 189, 224, 249, 264, 269, 264, 249, 224, 189, 144, 89, 24, -51, -136],
  [-171, -76, 9, 84, 149, 204, 249, 284, 309, 324, 329, 324, 309, 284, 249, 204, 149, 84, 9, -76, -171],
  [-42, -21, -2, 15, 30, 43, 54, 63, 70, 75, 78, 79, 78, 75, 70, 63, 54, 43, 30, 15, -2, -21, -42],
  [-253, -138, -33, 62, 147, 222, 287, 343, 387, 422, 447, 462, 467, 462, 447, 422, 387, 343, 287, 222, 147, 62, -33, -138, -253]
]
const cMN: number[][] = cMX.map((xar) => {
  const psum = xar.reduce((a, b) => a + b)//b!=xar[0]?a + 2*b:a+b)
  //console.log("psum, ",psum)
  return xar.map((x) => x / psum)
})

// smooth with filter length 5 - 25
export function smoothPointsN(data: Point[], filterlength: number, secondfilterlength?: number, thirdfilterlength?: number): void {
  // todo error check length
  const nsize = filterlength
  const nhalf = Math.floor(nsize / 2)
  const ncNorm = cMN.find((xar) => xar.length == filterlength)
  if (!ncNorm) {
    console.error("bad filter length")
  } else {
    // Copy head and tail, apply convolution filter to middle
    for (let i = 0; i < data.length; i++) {
      if (i <= nhalf || data.length - nhalf <= i) {
        // Copy head and tail data
        data[i].smooth = {
          vD: vD(data[i]),
          vN: vN(data[i]),
          vE: vE(data[i])
        }
      } else {
        // Apply convolution filter
        const start = i - nhalf
        const slice = data.slice(start, start + nsize)
        data[i].smooth = {
          vD: dotMap(slice, ncNorm, vD),
          vN: dotMap(slice, ncNorm, vN),
          vE: dotMap(slice, ncNorm, vE)
        }
      }
    }
  }
  if (secondfilterlength) {
    const tsize = secondfilterlength
    const thalf = Math.floor(tsize / 2)
    const tcNorm = cMN.find((xar) => xar.length == secondfilterlength)
    const tvD = (point: Point) => point.smooth.vD
    const tvN = (point: Point) => point.smooth.vN
    const tvE = (point: Point) => point.smooth.vE
    if (!tcNorm) {
      console.error("bad filter length")
    } else {
      // Copy head and tail, apply convolution filter to middle
      for (let i = 0; i < data.length; i++) {
        if (i <= thalf || data.length - thalf <= i) {
          // Copy head and tail data
          data[i].smooth = {
            vD: tvD(data[i]),
            vN: tvN(data[i]),
            vE: tvE(data[i])
          }
        } else {
          // Apply convolution filter
          const start = i - thalf
          const slice = data.slice(start, start + tsize)
          data[i].smooth = {
            vD: dotMap(slice, tcNorm, tvD),
            vN: dotMap(slice, tcNorm, tvN),
            vE: dotMap(slice, tcNorm, tvE)
          }
        }
      }
    }
  }
  if (thirdfilterlength) {
    const thsize = thirdfilterlength
    const thhalf = Math.floor(thsize / 2)
    const thcNorm = cMN.find((xar) => xar.length == thirdfilterlength)
    const thvD = (point: Point) => point.smooth.vD
    const thvN = (point: Point) => point.smooth.vN
    const thvE = (point: Point) => point.smooth.vE
    if (!thcNorm) {
      console.error("bad filter length")
    } else {
      // Copy head and tail, apply convolution filter to middle
      for (let i = 0; i < data.length; i++) {
        if (i <= thhalf || data.length - thhalf <= i) {
          // Copy head and tail data
          data[i].smooth = {
            vD: thvD(data[i]),
            vN: thvN(data[i]),
            vE: thvE(data[i])
          }
        } else {
          // Apply convolution filter
          const start = i - thhalf
          const slice = data.slice(start, start + thsize)
          data[i].smooth = {
            vD: dotMap(slice, thcNorm, thvD),
            vN: dotMap(slice, thcNorm, thvN),
            vE: dotMap(slice, thcNorm, thvE)
          }
        }
      }
    }
  }


}


export function smoothPositionN(data: Point[], filterlength: number, secondfilterlength?: number, thirdfilterlength?: number): void {
  // todo error check length
  const nsize = filterlength
  const nhalf = Math.floor(nsize / 2)
  const ncNorm = cMN.find((xar) => xar.length == filterlength)
  const lAT = (point: Point) => point.lat
  const lNG = (point: Point) => point.lng
  const aLT = (point: Point) => point.alt
  if (!ncNorm) {
    console.error("bad filter length")
  } else {
    // Copy head and tail, apply convolution filter to middle
    for (let i = 0; i < data.length; i++) {

      if (i <= nhalf || data.length - nhalf <= i) {
        // Copy head and tail data
        data[i].lat = lAT(data[i])
        data[i].lng = lNG(data[i])
        data[i].alt = aLT(data[i])
      } else {
        // Apply convolution filter
        const start = i - nhalf
        const slice = data.slice(start, start + nsize)
        data[i].lat = dotMap(slice, ncNorm, lAT),
        data[i].lng = dotMap(slice, ncNorm, lNG),
        data[i].alt = dotMap(slice, ncNorm, aLT)
      }
    }
  }
  if (secondfilterlength) {
    const ssize = secondfilterlength
    const shalf = Math.floor(ssize / 2)
    const scNorm = cMN.find((xar) => xar.length == secondfilterlength)

    if (!scNorm) {
      console.error("bad filter length")
    } else {
      // Copy head and tail, apply convolution filter to middle
      for (let i = 0; i < data.length; i++) {
        if (i <= shalf || data.length - shalf <= i) {
        // Copy head and tail data
          data[i].lat = lAT(data[i])
          data[i].lng = lNG(data[i])
          data[i].alt = aLT(data[i])
        } else {
        // Apply convolution filter
          const start = i - shalf
          const slice = data.slice(start, start + ssize)
          data[i].lat = dotMap(slice, scNorm, lAT),
          data[i].lng = dotMap(slice, scNorm, lNG),
          data[i].alt = dotMap(slice, scNorm, aLT)
        }
      }
    }
  }
  if (thirdfilterlength) {
    const tsize = thirdfilterlength
    const thalf = Math.floor(tsize / 2)
    const tcNorm = cMN.find((xar) => xar.length == thirdfilterlength)
    if (!tcNorm) {
      console.error("bad filter length")
    } else {
      // Copy head and tail, apply convolution filter to middle
      for (let i = 0; i < data.length; i++) {
        if (i <= thalf || data.length - thalf <= i) {
          // Copy head and tail data
          data[i].lat = lAT(data[i])
          data[i].lng = lNG(data[i])
          data[i].alt = aLT(data[i])
        } else {
        // Apply convolution filter
          const start = i - thalf
          const slice = data.slice(start, start + tsize)
          data[i].lat = dotMap(slice, tcNorm, lAT),
          data[i].lng = dotMap(slice, tcNorm, lNG),
          data[i].alt = dotMap(slice, tcNorm, aLT)
        }
      }
    }
  }
}

/**
 * Calculate smoothed GPS speeds from CSV data using Savitzky-Golay filter
 * Returns smoothed velocities in NED coordinates that can be added to PlotPoints
 */
export interface SmoothedSpeed {
  velN: number  // North velocity (m/s)
  velE: number  // East velocity (m/s) 
  velD: number  // Down velocity (m/s)
}

export function calculateSmoothedSpeeds(gpsData: MLocation[], filterLength: number, secondFilterLength: number, thirdFilterLength: number): SmoothedSpeed[] {
  
  
  if (gpsData.length === 0) return []
  
  // Find the appropriate filter coefficients
  const ncNorm = cMN.find(xar => xar.length === filterLength)
  if (!ncNorm) {
    console.error(`Unsupported filter length: ${filterLength}. Available lengths: ${cMN.map(arr => arr.length).join(', ')}`)
    // Fall back to simple moving average
    const fallbackSize = Math.min(filterLength, gpsData.length)
    const fallbackCoeffs = new Array(fallbackSize).fill(1 / fallbackSize)
    return applySmoothingFilter(gpsData, fallbackCoeffs)
  }
  
  return applySmoothingFilter(gpsData, ncNorm)
}

function applySmoothingFilter(gpsData: MLocation[], coefficients: number[]): SmoothedSpeed[] {
  const result: SmoothedSpeed[] = []
  const filterSize = coefficients.length
  const halfSize = Math.floor(filterSize / 2)
  
  // Extract velocity components from GPS data
  const velN = (point: MLocation) => point.velN
  const velE = (point: MLocation) => point.velE  
  const velD = (point: MLocation) => point.velD
  
  for (let i = 0; i < gpsData.length; i++) {
    if (i <= halfSize || gpsData.length - halfSize <= i) {
      // Copy head and tail data without smoothing
      result.push({
        velN: velN(gpsData[i]),
        velE: velE(gpsData[i]),
        velD: velD(gpsData[i])
      })
    } else {
      // Apply Savitzky-Golay filter
      const start = i - halfSize
      const slice = gpsData.slice(start, start + filterSize)
      result.push({
        velN: dotMap(slice, coefficients, velN),
        velE: dotMap(slice, coefficients, velE),
        velD: dotMap(slice, coefficients, velD)
      })
    }
  }
  
  return result
}

export interface SmoothedAcceleration {
  accelN: number  // North acceleration (m/s²)
  accelE: number  // East acceleration (m/s²)
  accelD: number  // Down acceleration (m/s²)
}

/**
 * Calculate smoothed acceleration from smoothed velocities using linear regression
 */
export function calculateSmoothedAcceleration(gpsData: MLocation[], smoothedSpeeds: SmoothedSpeed[], windowSize: number = 7): SmoothedAcceleration[] {
  const result: SmoothedAcceleration[] = []
  const halfWindow = Math.floor(windowSize / 2)
  
  if (gpsData.length === 0 || smoothedSpeeds.length === 0) {
    return result
  }
  
  for (let i = 0; i < gpsData.length; i++) {
    if (i < halfWindow || i >= gpsData.length - halfWindow) {
      // For points near edges, use simple numerical difference
      if (i === 0) {
        // Forward difference for first point
        if (gpsData.length > 1) {
          const dt = (gpsData[1].time - gpsData[0].time) / 1000 // Convert ms to seconds
          if (dt > 0) {
            result.push({
              accelN: (smoothedSpeeds[1].velN - smoothedSpeeds[0].velN) / dt,
              accelE: (smoothedSpeeds[1].velE - smoothedSpeeds[0].velE) / dt,
              accelD: (smoothedSpeeds[1].velD - smoothedSpeeds[0].velD) / dt
            })
          } else {
            result.push({ accelN: 0, accelE: 0, accelD: 0 })
          }
        } else {
          result.push({ accelN: 0, accelE: 0, accelD: 0 })
        }
      } else if (i === gpsData.length - 1) {
        // Backward difference for last point
        const dt = (gpsData[i].time - gpsData[i - 1].time) / 1000 // Convert ms to seconds
        if (dt > 0) {
          result.push({
            accelN: (smoothedSpeeds[i].velN - smoothedSpeeds[i - 1].velN) / dt,
            accelE: (smoothedSpeeds[i].velE - smoothedSpeeds[i - 1].velE) / dt,
            accelD: (smoothedSpeeds[i].velD - smoothedSpeeds[i - 1].velD) / dt
          })
        } else {
          result.push({ accelN: 0, accelE: 0, accelD: 0 })
        }
      } else {
        // Central difference for other edge points
        const dt = (gpsData[i + 1].time - gpsData[i - 1].time) / 1000 / 2 // Convert ms to seconds, divide by 2 for centered difference
        if (dt > 0) {
          result.push({
            accelN: (smoothedSpeeds[i + 1].velN - smoothedSpeeds[i - 1].velN) / dt,
            accelE: (smoothedSpeeds[i + 1].velE - smoothedSpeeds[i - 1].velE) / dt,
            accelD: (smoothedSpeeds[i + 1].velD - smoothedSpeeds[i - 1].velD) / dt
          })
        } else {
          result.push({ accelN: 0, accelE: 0, accelD: 0 })
        }
      }
    } else {
      // Use linear regression over the window for interior points
      const start = i - halfWindow
      const end = i + halfWindow + 1
      const windowData = gpsData.slice(start, end)
      const windowSpeeds = smoothedSpeeds.slice(start, end)
      
      // Create time-velocity pairs for regression
      // Key fix: use relative time from the first point in window to avoid large numbers
      const baseTime = windowData[0].time / 1000 // Convert to seconds
      const timeVelocityPairs = windowData.map((point, idx) => ({
        time: (point.time / 1000) - baseTime,  // Relative time in seconds from window start
        velN: windowSpeeds[idx].velN,
        velE: windowSpeeds[idx].velE,
        velD: windowSpeeds[idx].velD
      }))
      
      // Calculate acceleration as slope of velocity vs time using linear regression
      const accelN = getSlope(timeVelocityPairs, d => d.time, d => d.velN)
      const accelE = getSlope(timeVelocityPairs, d => d.time, d => d.velE)  
      const accelD = getSlope(timeVelocityPairs, d => d.time, d => d.velD)
      
      result.push({
        accelN: isFinite(accelN) ? accelN : 0,
        accelE: isFinite(accelE) ? accelE : 0,
        accelD: isFinite(accelD) ? accelD : 0
      })
    }
  }
  
  return result
}
