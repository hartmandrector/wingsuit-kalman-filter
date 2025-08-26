import { MotionEstimator } from './motionestimator.js'
import { setReference, ENUToLatLngAlt, latLonAltToENU } from './enu.js'
import type { MLocation, PlotPoint, Vector3 } from './types.js'
import { KalmanFilter3D } from './kalman.js'
import { sub, magnitude } from './vector.js'
import { calculateJerk, calculateCurvature, calculateVelocityChangeRate } from './smoothness.js'

// Which filter to use
let filterType: 'motionestimator' | 'kalman' = 'motionestimator'

// Motion Estimator parameters
let currentAlpha = 0.9
let currentAlphaVelocity = 0.9
let currentAlphaAcceleration = 0.9

// Kalman Filter parameters
let kalmanProcessNoise = {
  position: 0.1,
  velocity: 2.0,
  acceleration: 10.0
}
let kalmanMeasurementNoise = {
  position: 25.0,
  velocity: 1.0
}

let estimator = createEstimator()

function createEstimator() {
  if (filterType === 'motionestimator') {
    return new MotionEstimator({ 
      alpha: currentAlpha, 
      alphaVelocity: currentAlphaVelocity, 
      alphaAcceleration: currentAlphaAcceleration, 
      estimateVelocity: true 
    })
  } else {
    const kalman = new KalmanFilter3D()
    updateKalmanParameters(kalman)
    return kalman
  }
}

function updateKalmanParameters(kalman: KalmanFilter3D) {
  // Update process noise covariance (Q matrix)
  kalman.setProcessNoise(
    kalmanProcessNoise.position,
    kalmanProcessNoise.velocity,
    kalmanProcessNoise.acceleration,
    0.01 // Fixed wingsuit parameter
  )
  
  // Update measurement noise covariance (R matrix)  
  kalman.setMeasurementNoise(
    kalmanMeasurementNoise.position,
    kalmanMeasurementNoise.velocity
  )
}

const refreshRate = 90 // Hz

export function setFilterType(type: 'motionestimator' | 'kalman'): void {
  filterType = type
  estimator = createEstimator()
}

export function getFilterType(): 'motionestimator' | 'kalman' {
  return filterType
}

// Motion Estimator controls
export function setAlpha(alpha: number): void {
  currentAlpha = alpha
  if (filterType === 'motionestimator') {
    estimator = createEstimator()
  }
}

export function setAlphaVelocity(alphaVelocity: number): void {
  currentAlphaVelocity = alphaVelocity
  if (filterType === 'motionestimator') {
    estimator = createEstimator()
  }
}

export function setAlphaAcceleration(alphaAcceleration: number): void {
  currentAlphaAcceleration = alphaAcceleration
  if (filterType === 'motionestimator') {
    estimator = createEstimator()
  }
}

// Kalman Filter controls with logarithmic scaling
function quadraticScale(value: number): number {
  // Maps 0-1 to 0.0001-25 using quadratic scaling
  return 0.0001 + (value * value) * (25 - 0.0001)
}

function inverseQuadraticScale(scaledValue: number): number {
  // Maps 0.0001-25 back to 0-1
  return Math.sqrt((scaledValue - 0.0001) / (25 - 0.0001))
}

export function signedQuadraticScale(value: number): number {
  // Maps -1 to 1 to -25 to 25 using quadratic scaling that preserves sign
  const sign = Math.sign(value)
  const absValue = Math.abs(value)
  const scaled = absValue * absValue * 25
  return sign * scaled
}

export function inverseSignedQuadraticScale(scaledValue: number): number {
  // Maps -25 to 25 back to -1 to 1
  const sign = Math.sign(scaledValue)
  const absValue = Math.abs(scaledValue)
  const unscaled = Math.sqrt(absValue / 25)
  return sign * unscaled
}

export function setKalmanProcessNoisePosition(value: number): void {
  kalmanProcessNoise.position = quadraticScale(value)
  if (filterType === 'kalman') {
    updateKalmanParameters(estimator as KalmanFilter3D)
  }
}

export function setKalmanProcessNoiseVelocity(value: number): void {
  kalmanProcessNoise.velocity = quadraticScale(value)
  if (filterType === 'kalman') {
    updateKalmanParameters(estimator as KalmanFilter3D)
  }
}

export function setKalmanProcessNoiseAcceleration(value: number): void {
  kalmanProcessNoise.acceleration = quadraticScale(value)
  if (filterType === 'kalman') {
    updateKalmanParameters(estimator as KalmanFilter3D)
  }
}

export function setKalmanMeasurementNoisePosition(value: number): void {
  kalmanMeasurementNoise.position = signedQuadraticScale(value)
  if (filterType === 'kalman') {
    updateKalmanParameters(estimator as KalmanFilter3D)
  }
}

export function setKalmanMeasurementNoiseVelocity(value: number): void {
  kalmanMeasurementNoise.velocity = signedQuadraticScale(value)
  if (filterType === 'kalman') {
    updateKalmanParameters(estimator as KalmanFilter3D)
  }
}

export interface ErrorStats {
  position: {
    min: number
    avg: number
    max: number
    count: number
  }
  velocity: {
    min: number
    avg: number
    max: number
    count: number
  }
  smoothness: {
    jerk: {
      min: number
      avg: number
      max: number
      count: number
    }
    curvature: {
      min: number
      avg: number
      max: number
      count: number
    }
    velocityChangeRate: {
      min: number
      avg: number
      max: number
      count: number
    }
  }
}

let globalErrorStats: ErrorStats | undefined

export function getErrorStats(): ErrorStats | undefined {
  return globalErrorStats
}

export function generatePredictedPoints(gpsPoints: MLocation[]): PlotPoint[] {
  if (gpsPoints.length === 0) return []

  // Initialize MotionEstimator with first GPS point as reference
  estimator.reset()
  const firstPoint = gpsPoints[0]
  setReference({ lat: firstPoint.lat, lng: firstPoint.lng, alt: firstPoint.alt })

  const interpolatedPoints: PlotPoint[] = []
  const positionErrors: number[] = []
  const velocityErrors: number[] = []
  
  // Data for smoothness calculations
  const filteredPositions: Vector3[] = []
  const filteredVelocities: Vector3[] = []
  const filteredTimes: number[] = []

  // Track the next prediction time to maintain even spacing
  let nextPredictionTime = firstPoint.time + 1000 / refreshRate

  for (let index = 0; index < gpsPoints.length; index++) {
    const point = gpsPoints[index]

    // If this is not the first point, compute error between prediction and actual GPS
    if (index > 0) {
      const predicted = estimator.predictAt(point.time)
      if (predicted) {
        const actualENU = latLonAltToENU({ lat: point.lat, lng: point.lng, alt: point.alt })
        const positionError = magnitude(sub(predicted.position, actualENU))
        positionErrors.push(positionError)

        // Calculate velocity prediction error
        const actualVelocityENU = { x: point.velE, y: -point.velD, z: point.velN }
        const velocityError = magnitude(sub(predicted.velocity, actualVelocityENU))
        velocityErrors.push(velocityError)
      }
    }

    estimator.update(point)
    
    // Collect filtered data for smoothness calculations
    const filteredState = estimator.predictAt(point.time)
    if (filteredState) {
      filteredPositions.push(filteredState.position)
      filteredVelocities.push(filteredState.velocity)
      filteredTimes.push(point.time)
    }

    // Generate interpolated points until next GPS point (or for 2 seconds after last point)
    const nextPoint = gpsPoints[index + 1]
    const endTime = nextPoint ? nextPoint.time : point.time + 2000 // 2 seconds after last GPS

    // Generate predicted points at even intervals, maintaining spacing across GPS points
    while (nextPredictionTime < endTime) {
      const predicted = estimator.predictAt(nextPredictionTime)
      if (predicted) {
        const predLatLon = ENUToLatLngAlt(predicted.position)
        
        // Calculate sustained speeds from wingsuit coefficients
        let vxs: number | undefined = undefined
        let vys: number | undefined = undefined
        
        if (predicted.kl !== undefined && predicted.kd !== undefined) {
          const kl = predicted.kl
          const kd = predicted.kd
          const klkdSquared = kl * kl + kd * kd
          
          if (klkdSquared > 0) {
            // Sustained speeds formula: v = coefficient / (kl² + kd²)^0.75
            // Calculate in NED coordinates (same as wingsuit physics)
            const denominator = Math.pow(klkdSquared, 0.75)
            vxs = kl / denominator  // Horizontal sustained speed (NED)
            vys = kd / denominator  // Vertical sustained speed (NED, positive down)
          }
        }
        
        // Find the closest GPS point to get smoothed speeds
        let closestGpsIndex = 0
        let minTimeDiff = Math.abs(gpsPoints[0].time - nextPredictionTime)
        for (let i = 1; i < gpsPoints.length; i++) {
          const timeDiff = Math.abs(gpsPoints[i].time - nextPredictionTime)
          if (timeDiff < minTimeDiff) {
            minTimeDiff = timeDiff
            closestGpsIndex = i
          }
        }
        const closestGpsPoint = gpsPoints[closestGpsIndex]
        
        interpolatedPoints.push({
          ...predLatLon,
          time: nextPredictionTime,
          // Store ENU coordinates for display (x=East, y=Up, z=North)
          x: predicted.position.x,
          y: predicted.position.y,
          z: predicted.position.z,
          // Store ENU velocity components (m/s)
          velX: predicted.velocity.x,
          velY: predicted.velocity.y,
          velZ: predicted.velocity.z,
          // Store ENU acceleration components (m/s²)
          accelX: predicted.acceleration.x,
          accelY: predicted.acceleration.y,
          accelZ: predicted.acceleration.z,
          kl: predicted.kl,
          kd: predicted.kd,
          roll: predicted.roll,
          // Store sustained speeds
          vxs: vxs,
          vys: filterType === 'kalman' ? (vys ?? 0) : (vys ?? 0), // Convert to ENU (positive up)
          // Add smoothed GPS speeds from closest GPS point
          smoothVelN: closestGpsPoint.smoothVelN,
          smoothVelE: closestGpsPoint.smoothVelE,
          smoothVelD: closestGpsPoint.smoothVelD
        })
      }
      nextPredictionTime += 1000 / refreshRate
    }
  }

  // Calculate error statistics
  if (positionErrors.length && velocityErrors.length) {
    const positionMin = Math.min(...positionErrors)
    const positionMax = Math.max(...positionErrors)
    const positionAvg = positionErrors.reduce((sum, err) => sum + err, 0) / positionErrors.length

    const velocityMin = Math.min(...velocityErrors)
    const velocityMax = Math.max(...velocityErrors)
    const velocityAvg = velocityErrors.reduce((sum, err) => sum + err, 0) / velocityErrors.length

    // Calculate smoothness metrics using filtered data
    const jerks = calculateJerk(filteredVelocities, filteredTimes)
    const curvatures = calculateCurvature(filteredPositions, filteredVelocities)
    const velocityChangeRates = calculateVelocityChangeRate(filteredVelocities, filteredTimes)

    const jerkStats = jerks.length > 0 ? {
      min: Math.min(...jerks),
      avg: jerks.reduce((sum, j) => sum + j, 0) / jerks.length,
      max: Math.max(...jerks),
      count: jerks.length
    } : { min: 0, avg: 0, max: 0, count: 0 }

    const curvatureStats = curvatures.length > 0 ? {
      min: Math.min(...curvatures),
      avg: curvatures.reduce((sum, c) => sum + c, 0) / curvatures.length,
      max: Math.max(...curvatures),
      count: curvatures.length
    } : { min: 0, avg: 0, max: 0, count: 0 }

    const velocityChangeRateStats = velocityChangeRates.length > 0 ? {
      min: Math.min(...velocityChangeRates),
      avg: velocityChangeRates.reduce((sum, r) => sum + r, 0) / velocityChangeRates.length,
      max: Math.max(...velocityChangeRates),
      count: velocityChangeRates.length
    } : { min: 0, avg: 0, max: 0, count: 0 }

    globalErrorStats = {
      position: {
        min: positionMin,
        avg: positionAvg,
        max: positionMax,
        count: positionErrors.length
      },
      velocity: {
        min: velocityMin,
        avg: velocityAvg,
        max: velocityMax,
        count: velocityErrors.length
      },
      smoothness: {
        jerk: jerkStats,
        curvature: curvatureStats,
        velocityChangeRate: velocityChangeRateStats
      }
    }
  } else {
    globalErrorStats = undefined
  }

  return interpolatedPoints
}