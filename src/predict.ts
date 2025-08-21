import { MotionEstimator } from './motionestimator.js'
import { setReference, ENUToLatLngAlt, latLonAltToENU } from './enu.js'
import type { MLocation, PlotPoint, Vector3 } from './types.js'
import { KalmanFilter3D } from './kalman.js'
import { sub, magnitude } from './vector.js'
import { calculateJerk, calculateCurvature, calculateVelocityChangeRate } from './smoothness.js'

// Which filter to use
let filterType: 'motionestimator' | 'kalman' = 'motionestimator' // Change to 'kalman' to use Kalman filter
let currentAlpha = 0.9
let currentAlphaVelocity = 0.9
let currentAlphaAcceleration = 0.9
let estimator = filterType === 'motionestimator'
  ? new MotionEstimator({ alpha: currentAlpha, alphaVelocity: currentAlphaVelocity, alphaAcceleration: currentAlphaAcceleration, estimateVelocity: true })
  : new KalmanFilter3D()

const refreshRate = 90 // Hz

export function setAlpha(alpha: number): void {
  currentAlpha = alpha
  if (filterType === 'motionestimator') {
    estimator = new MotionEstimator({ alpha: currentAlpha, alphaVelocity: currentAlphaVelocity, alphaAcceleration: currentAlphaAcceleration, estimateVelocity: true })
  }
}

export function setAlphaVelocity(alphaVelocity: number): void {
  currentAlphaVelocity = alphaVelocity
  if (filterType === 'motionestimator') {
    estimator = new MotionEstimator({ alpha: currentAlpha, alphaVelocity: currentAlphaVelocity, alphaAcceleration: currentAlphaAcceleration, estimateVelocity: true })
  }
}

export function setAlphaAcceleration(alphaAcceleration: number): void {
  currentAlphaAcceleration = alphaAcceleration
  if (filterType === 'motionestimator') {
    estimator = new MotionEstimator({ alpha: currentAlpha, alphaVelocity: currentAlphaVelocity, alphaAcceleration: currentAlphaAcceleration, estimateVelocity: true })
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

    for (let t = point.time ; t < endTime; t += 1000 / refreshRate) {
      const predicted = estimator.predictAt(t)
      if (predicted) {
        const predLatLon = ENUToLatLngAlt(predicted.position)
        interpolatedPoints.push({
          ...predLatLon,
          time: t
        })
      }
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