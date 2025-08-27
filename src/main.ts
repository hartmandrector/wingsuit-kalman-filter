import './style.css'
import { parseCSV } from './csvParser.js'
import { plotData, speedComparisonView } from './plotter.js'
import { 
  generatePredictedPoints, 
  setAlpha, 
  setAlphaVelocity, 
  setAlphaAcceleration,
  setFilterType,
  getFilterType,
  setKalmanProcessNoisePosition,
  setKalmanProcessNoiseVelocity,
  setKalmanProcessNoiseAcceleration,
  setKalmanMeasurementNoisePosition,
  setKalmanMeasurementNoiseVelocity,
  signedQuadraticScale
} from './predict.js'
import { setReference, latLonAltToENU } from './enu.js'
import { MLocation, PlotSeries, PlotPoint } from './types.js'
import { calculateSmoothedSpeeds, calculateSmoothedAcceleration, calculateSmoothSustainedSpeeds } from './savitzky-golay.js'

let currentGpsPoints: MLocation[] = []
let currentPredictedPoints: PlotPoint[] = []
let smoothedSpeeds: ReturnType<typeof calculateSmoothedSpeeds> = []

const dropZone = document.getElementById('drop-zone')
const fileInput = document.getElementById('file-input') as HTMLInputElement | null

if (dropZone && fileInput) {
  dropZone.addEventListener('click', () => fileInput.click())
  fileInput.addEventListener('change', handleFileSelect)
}

// Setup filter type switcher
const motionEstimatorRadio = document.getElementById('motion-estimator') as HTMLInputElement | null
const kalmanFilterRadio = document.getElementById('kalman-filter') as HTMLInputElement | null
const motionEstimatorControls = document.getElementById('motion-estimator-controls') as HTMLElement | null
const kalmanFilterControls = document.getElementById('kalman-filter-controls') as HTMLElement | null

function updateControlsVisibility(): void {
  const filterType = getFilterType()
  if (motionEstimatorControls && kalmanFilterControls) {
    if (filterType === 'motionestimator') {
      motionEstimatorControls.style.display = 'block'
      kalmanFilterControls.style.display = 'none'
    } else {
      motionEstimatorControls.style.display = 'none'
      kalmanFilterControls.style.display = 'block'
    }
  }
}

if (motionEstimatorRadio && kalmanFilterRadio) {
  motionEstimatorRadio.addEventListener('change', () => {
    if (motionEstimatorRadio.checked) {
      setFilterType('motionestimator')
      updateControlsVisibility()
      if (currentGpsPoints.length > 0) {
        regeneratePlot()
      }
    }
  })

  kalmanFilterRadio.addEventListener('change', () => {
    if (kalmanFilterRadio.checked) {
      setFilterType('kalman')
      updateControlsVisibility()
      if (currentGpsPoints.length > 0) {
        regeneratePlot()
      }
    }
  })
}

// Setup download button
const downloadCsvBtn = document.getElementById('download-csv-btn') as HTMLButtonElement | null

if (downloadCsvBtn) {
  downloadCsvBtn.addEventListener('click', downloadCSV)
}

function downloadCSV(): void {
  if (currentGpsPoints.length === 0 || currentPredictedPoints.length === 0) {
    alert('No data available to download. Please load a CSV file first.')
    return
  }

  // Create CSV content
  const csvContent = generateCSVContent(currentGpsPoints, currentPredictedPoints)
  
  // Create and download file
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' })
  const link = document.createElement('a')
  
  if (link.download !== undefined) {
    const url = URL.createObjectURL(blob)
    link.setAttribute('href', url)
    link.setAttribute('download', `flysight_filtered_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.csv`)
    link.style.visibility = 'hidden'
    document.body.appendChild(link)
    link.click()
    document.body.removeChild(link)
  }
}

function generateCSVContent(gpsPoints: MLocation[], predictedPoints: PlotPoint[]): string {
  // CSV headers
  const headers = [
    'timestamp_ms',
    'time_iso',
    'data_type',
    'lat',
    'lng', 
    'alt_m',
    'x_east_m',
    'y_up_m',
    'z_north_m',
    'horizontal_distance_m',
    'vel_n_ms',
    'vel_e_ms',
    'vel_d_ms',
    'speed_ms',
    'speed_mph',
    'accel_n_ms2',
    'accel_e_ms2', 
    'accel_d_ms2',
    'accel_magnitude_ms2',
    'h_acc_m',
    'v_acc_m',
    's_acc_ms',
    'kl',
    'kd',
    'roll_rad',
    'roll_deg',
    'vxs_ms',
    'vxs_mph',
    'vys_ms',
    'vys_mph'
  ]

  let csvContent = headers.join(',') + '\n'

  // Helper function to format timestamp
  const formatTimestamp = (timestamp: number): string => {
    return new Date(timestamp).toISOString()
  }

  // Helper function to calculate acceleration (numerical derivative of velocity)
  const calculateAcceleration = (points: any[], index: number): {n: number, e: number, d: number} => {
    if (index === 0) return {n: 0, e: 0, d: 0}
    
    const current = points[index]
    const previous = points[index - 1]
    const dt = (current.time - previous.time) / 1000 // Convert to seconds
    
    if (dt <= 0) return {n: 0, e: 0, d: 0}
    
    return {
      n: (current.velN - previous.velN) / dt,
      e: (current.velE - previous.velE) / dt,
      d: (current.velD - previous.velD) / dt
    }
  }

  // Add GPS measurement rows
  for (let i = 0; i < gpsPoints.length; i++) {
    const point = gpsPoints[i]
    const accel = calculateAcceleration(gpsPoints, i)
    const accelMagnitude = Math.sqrt(accel.n * accel.n + accel.e * accel.e + accel.d * accel.d)
    const speed = Math.sqrt(point.velN * point.velN + point.velE * point.velE + point.velD * point.velD)
    const horizontalDistance = point.x !== undefined && point.z !== undefined ? 
      Math.sqrt(point.x * point.x + point.z * point.z) : 0

    const row = [
      point.time,
      formatTimestamp(point.time),
      'GPS_MEASUREMENT',
      point.lat.toFixed(8),
      point.lng.toFixed(8),
      point.alt.toFixed(3),
      (point.x !== undefined ? point.x.toFixed(3) : ''),
      (point.y !== undefined ? point.y.toFixed(3) : ''),
      (point.z !== undefined ? point.z.toFixed(3) : ''),
      horizontalDistance.toFixed(3),
      point.velN.toFixed(3),
      point.velE.toFixed(3),
      point.velD.toFixed(3),
      speed.toFixed(3),
      (speed * 2.23694).toFixed(3),
      accel.n.toFixed(3),
      accel.e.toFixed(3),
      accel.d.toFixed(3),
      accelMagnitude.toFixed(3),
      point.hAcc.toFixed(3),
      point.vAcc.toFixed(3),
      point.sAcc.toFixed(3),
      '', // kl - not available for GPS measurements
      '', // kd
      '', // roll_rad
      '', // roll_deg
      '', // vxs_ms
      '', // vxs_mph
      '', // vys_ms
      ''  // vys_mph
    ]
    
    csvContent += row.join(',') + '\n'
  }

  // Add predicted/filtered rows
  for (const point of predictedPoints) {
    const horizontalDistance = point.x !== undefined && point.z !== undefined ? 
      Math.sqrt(point.x * point.x + point.z * point.z) : 0

    // Calculate total speed and acceleration if velocity/acceleration components are available
    let speed_ms = '', speed_mph = '', accel_magnitude = ''
    let vel_n = '', vel_e = '', vel_d = ''
    let accel_n = '', accel_e = '', accel_d = ''
    
    if (point.velX !== undefined && point.velY !== undefined && point.velZ !== undefined) {
      // Convert ENU velocity back to NED for consistency with GPS data
      vel_n = point.velZ.toFixed(3)  // North = ENU Z
      vel_e = point.velX.toFixed(3)  // East = ENU X
      vel_d = (-point.velY).toFixed(3) // Down = -ENU Y
      
      const speed = Math.sqrt(point.velX * point.velX + point.velY * point.velY + point.velZ * point.velZ)
      speed_ms = speed.toFixed(3)
      speed_mph = (speed * 2.23694).toFixed(3)
    }
    
    if (point.accelX !== undefined && point.accelY !== undefined && point.accelZ !== undefined) {
      // Convert ENU acceleration back to NED for consistency with GPS data
      accel_n = point.accelZ.toFixed(3)  // North = ENU Z
      accel_e = point.accelX.toFixed(3)  // East = ENU X  
      accel_d = (-point.accelY).toFixed(3) // Down = -ENU Y
      
      const accelMag = Math.sqrt(point.accelX * point.accelX + point.accelY * point.accelY + point.accelZ * point.accelZ)
      accel_magnitude = accelMag.toFixed(3)
    }

    // Calculate wingsuit performance metrics if available
    let vxs_ms = '', vxs_mph = '', vys_ms = '', vys_mph = ''
    if (point.kl !== undefined && point.kd !== undefined) {
      const kl = point.kl
      const kd = point.kd
      const klkdSquared = kl * kl + kd * kd
      const vxs = kl / Math.pow(klkdSquared, 0.75)
      const vys = kd / Math.pow(klkdSquared, 0.75)
      vxs_ms = vxs.toFixed(6)
      vxs_mph = (vxs * 2.23694).toFixed(3)
      vys_ms = vys.toFixed(6)
      vys_mph = (vys * 2.23694).toFixed(3)
    }

    const row = [
      point.time,
      formatTimestamp(point.time),
      'FILTER_OUTPUT',
      point.lat.toFixed(8),
      point.lng.toFixed(8),
      point.alt.toFixed(3),
      (point.x !== undefined ? point.x.toFixed(3) : ''),
      (point.y !== undefined ? point.y.toFixed(3) : ''),
      (point.z !== undefined ? point.z.toFixed(3) : ''),
      horizontalDistance.toFixed(3),
      vel_n,  // vel_n from filter
      vel_e,  // vel_e from filter
      vel_d,  // vel_d from filter
      speed_ms, // speed from filter
      speed_mph, // speed_mph from filter
      accel_n,  // accel_n from filter
      accel_e,  // accel_e from filter
      accel_d,  // accel_d from filter
      accel_magnitude, // accel_magnitude from filter
      '', // h_acc - not available for predictions
      '', // v_acc
      '', // s_acc
      (point.kl !== undefined ? point.kl.toExponential(6) : ''),
      (point.kd !== undefined ? point.kd.toExponential(6) : ''),
      (point.roll !== undefined ? point.roll.toFixed(6) : ''),
      (point.roll !== undefined ? (point.roll * 180 / Math.PI).toFixed(3) : ''),
      vxs_ms,
      vxs_mph,
      vys_ms,
      vys_mph
    ]
    
    csvContent += row.join(',') + '\n'
  }

  return csvContent
}

// Helper function for quadratic scaling display
function quadraticScale(value: number): number {
  return 0.0001 + (value * value) * (25 - 0.0001)
}

// Setup alpha sliders (Motion Estimator)
const alphaSlider = document.getElementById('alpha-slider') as HTMLInputElement | null
const alphaValue = document.getElementById('alpha-value') as HTMLElement | null
const alphaVelocitySlider = document.getElementById('alpha-velocity-slider') as HTMLInputElement | null
const alphaVelocityValue = document.getElementById('alpha-velocity-value') as HTMLElement | null
const alphaAccelerationSlider = document.getElementById('alpha-acceleration-slider') as HTMLInputElement | null
const alphaAccelerationValue = document.getElementById('alpha-acceleration-value') as HTMLElement | null

if (alphaSlider && alphaValue) {
  alphaSlider.addEventListener('input', (e) => {
    const target = e.target as HTMLInputElement
    const alpha = parseFloat(target.value)
    alphaValue.textContent = alpha.toFixed(2)
    setAlpha(alpha)
    
    if (currentGpsPoints.length > 0) {
      regeneratePlot()
    }
  })
}

if (alphaVelocitySlider && alphaVelocityValue) {
  alphaVelocitySlider.addEventListener('input', (e) => {
    const target = e.target as HTMLInputElement
    const alphaVelocity = parseFloat(target.value)
    alphaVelocityValue.textContent = alphaVelocity.toFixed(2)
    setAlphaVelocity(alphaVelocity)
    
    if (currentGpsPoints.length > 0) {
      regeneratePlot()
    }
  })
}

if (alphaAccelerationSlider && alphaAccelerationValue) {
  alphaAccelerationSlider.addEventListener('input', (e) => {
    const target = e.target as HTMLInputElement
    const alphaAcceleration = parseFloat(target.value)
    alphaAccelerationValue.textContent = alphaAcceleration.toFixed(2)
    setAlphaAcceleration(alphaAcceleration)
    
    if (currentGpsPoints.length > 0) {
      regeneratePlot()
    }
  })
}

// Setup Kalman Filter sliders
// Process Noise (Q Matrix)
const kalmanQPositionSlider = document.getElementById('kalman-q-position-slider') as HTMLInputElement | null
const kalmanQPositionValue = document.getElementById('kalman-q-position-value') as HTMLElement | null
const kalmanQVelocitySlider = document.getElementById('kalman-q-velocity-slider') as HTMLInputElement | null
const kalmanQVelocityValue = document.getElementById('kalman-q-velocity-value') as HTMLElement | null
const kalmanQAccelerationSlider = document.getElementById('kalman-q-acceleration-slider') as HTMLInputElement | null
const kalmanQAccelerationValue = document.getElementById('kalman-q-acceleration-value') as HTMLElement | null

// Measurement Noise (R Matrix)
const kalmanRPositionSlider = document.getElementById('kalman-r-position-slider') as HTMLInputElement | null
const kalmanRPositionValue = document.getElementById('kalman-r-position-value') as HTMLElement | null
const kalmanRVelocitySlider = document.getElementById('kalman-r-velocity-slider') as HTMLInputElement | null
const kalmanRVelocityValue = document.getElementById('kalman-r-velocity-value') as HTMLElement | null

if (kalmanQPositionSlider && kalmanQPositionValue) {
  kalmanQPositionSlider.addEventListener('input', (e) => {
    const target = e.target as HTMLInputElement
    const value = parseFloat(target.value)
    const scaledValue = quadraticScale(value)
    kalmanQPositionValue.textContent = scaledValue.toFixed(4)
    setKalmanProcessNoisePosition(value)
    
    if (currentGpsPoints.length > 0) {
      regeneratePlot()
    }
  })
}

if (kalmanQVelocitySlider && kalmanQVelocityValue) {
  kalmanQVelocitySlider.addEventListener('input', (e) => {
    const target = e.target as HTMLInputElement
    const value = parseFloat(target.value)
    const scaledValue = quadraticScale(value)
    kalmanQVelocityValue.textContent = scaledValue.toFixed(4)
    setKalmanProcessNoiseVelocity(value)
    
    if (currentGpsPoints.length > 0) {
      regeneratePlot()
    }
  })
}

if (kalmanQAccelerationSlider && kalmanQAccelerationValue) {
  kalmanQAccelerationSlider.addEventListener('input', (e) => {
    const target = e.target as HTMLInputElement
    const value = parseFloat(target.value)
    const scaledValue = quadraticScale(value)
    kalmanQAccelerationValue.textContent = scaledValue.toFixed(4)
    setKalmanProcessNoiseAcceleration(value)
    
    if (currentGpsPoints.length > 0) {
      regeneratePlot()
    }
  })
}

if (kalmanRPositionSlider && kalmanRPositionValue) {
  kalmanRPositionSlider.addEventListener('input', (e) => {
    const target = e.target as HTMLInputElement
    const value = parseFloat(target.value)
    const scaledValue = signedQuadraticScale(value)
    kalmanRPositionValue.textContent = scaledValue.toFixed(2)
    setKalmanMeasurementNoisePosition(value)
    
    if (currentGpsPoints.length > 0) {
      regeneratePlot()
    }
  })
}

if (kalmanRVelocitySlider && kalmanRVelocityValue) {
  kalmanRVelocitySlider.addEventListener('input', (e) => {
    const target = e.target as HTMLInputElement
    const value = parseFloat(target.value)
    const scaledValue = signedQuadraticScale(value)
    kalmanRVelocityValue.textContent = scaledValue.toFixed(2)
    setKalmanMeasurementNoiseVelocity(value)
    
    if (currentGpsPoints.length > 0) {
      regeneratePlot()
    }
  })
}

// Make entire window accept drag and drop
document.body.addEventListener('dragover', handleDragOver)
document.body.addEventListener('drop', handleDrop)
document.body.addEventListener('dragleave', handleDragLeave)

function handleDragOver(e: DragEvent): void {
  e.preventDefault()
  const quadViewContainer = document.getElementById('quad-view-container')
  const overlay = document.getElementById('drag-overlay')
  
  if (quadViewContainer?.classList.contains('show')) {
    overlay?.classList.add('show')
  } else {
    dropZone?.classList.add('dragover')
  }
}

function handleDragLeave(e: DragEvent): void {
  const quadViewContainer = document.getElementById('quad-view-container')
  const overlay = document.getElementById('drag-overlay')
  
  // Only remove dragover when actually leaving the window
  if (e.clientX === 0 && e.clientY === 0) {
    if (quadViewContainer?.classList.contains('show')) {
      overlay?.classList.remove('show')
    } else {
      dropZone?.classList.remove('dragover')
    }
  }
}

function handleDrop(e: DragEvent): void {
  e.preventDefault()
  const overlay = document.getElementById('drag-overlay')
  
  dropZone?.classList.remove('dragover')
  overlay?.classList.remove('show')
  
  const files = e.dataTransfer?.files
  if (files?.length) {
    processFile(files[0])
  }
}

function handleFileSelect(e: Event): void {
  const target = e.target as HTMLInputElement
  if (target.files && target.files.length > 0) {
    processFile(target.files[0])
  }
}

function processFile(file: File): void {
  if (!file.name.toLowerCase().endsWith('.csv')) {
    alert('Please select a CSV file')
    return
  }

  const reader = new FileReader()
  reader.onload = function(e: ProgressEvent<FileReader>) {
    const csv = e.target?.result as string
    if (csv) {
      processCSVData(csv)
    }
  }
  reader.readAsText(file)
}

function regeneratePlot(): void {
  if (currentGpsPoints.length === 0) return
  
  // Generate new predicted points with current filter settings
  currentPredictedPoints = generatePredictedPoints(currentGpsPoints)

  // Create plot series using the same format as initialPlot
  const series: PlotSeries[] = [
    {
      name: 'GPS Points',
      data: currentGpsPoints,
      style: { color: '#646cff', radius: 4 }
    },
    {
      name: 'Predicted Points',
      data: currentPredictedPoints,
      style: { color: '#ff4444', radius: 2 }
    }
  ]
  
  // Preserve zoom when regenerating from slider changes
  plotData(series, true)
}

function initialPlot(): void {
  if (currentGpsPoints.length === 0) return
  
  // Generate initial predicted points
  currentPredictedPoints = generatePredictedPoints(currentGpsPoints)

  // Create plot series
  const series: PlotSeries[] = [
    {
      name: 'GPS Points',
      data: currentGpsPoints,
      style: { color: '#646cff', radius: 4 }
    },
    {
      name: 'Predicted Points',
      data: currentPredictedPoints,
      style: { color: '#ff4444', radius: 2 }
    }
  ]
  
  // Reset zoom for new data
  plotData(series, false)
}

function processCSVData(csv: string): void {
  const data = parseCSV(csv)
  console.log('Parsed CSV data:', data)
  
  // Parse GPS data with timestamps and altitude
  const latField = 'lat'
  const lonField = 'lon'
  const altField = 'hMSL'
  const timeField = 'time'

  // Store GPS points for regeneration
  currentGpsPoints = data.map((row, index) => ({
    lat: parseFloat(row[latField]),
    lng: parseFloat(row[lonField]),
    alt: altField ? parseFloat(row[altField]) || 0 : 0,
    time: timeField ? new Date(row[timeField]).getTime() : index * 1000, // Parse Zulu time
    velN: parseFloat(row['velN']) || 0,
    velE: parseFloat(row['velE']) || 0,
    velD: parseFloat(row['velD']) || 0,
    hAcc: parseFloat(row['hAcc']) || 0,
    vAcc: parseFloat(row['vAcc']) || 0,
    sAcc: parseFloat(row['sAcc']) || 0
  })).filter(point => !isNaN(point.lat) && !isNaN(point.lng))

  if (currentGpsPoints.length === 0) {
    alert('No valid coordinates found')
    return
  }

  // Set ENU reference to first GPS point and calculate ENU coordinates for all points
  const firstPoint = currentGpsPoints[0]
  setReference({ lat: firstPoint.lat, lng: firstPoint.lng, alt: firstPoint.alt })
  
  // Add ENU coordinates to all GPS points for display
  currentGpsPoints = currentGpsPoints.map(point => {
    const enu = latLonAltToENU({ lat: point.lat, lng: point.lng, alt: point.alt })
    return {
      ...point,
      x: enu.x,  // East
      y: enu.y,  // Up  
      z: enu.z   // North
    }
  })

  // Calculate smoothed GPS speeds (computed once when CSV is loaded)
  console.log('Calculating smoothed GPS speeds...')
  smoothedSpeeds = calculateSmoothedSpeeds(currentGpsPoints, 25,25,25) 
  console.log('Smoothed speeds calculated:', smoothedSpeeds.length, 'points')
  
  // Calculate smoothed acceleration from smoothed velocities
  console.log('Calculating smoothed accelerations...')
  const smoothedAccelerations = calculateSmoothedAcceleration(currentGpsPoints, smoothedSpeeds)  // Using default window size of 7
  console.log('Smoothed accelerations calculated:', smoothedAccelerations.length, 'points')
  console.log('First 5 smoothed accelerations:', smoothedAccelerations.slice(0, 5))
  console.log('Sample smoothed speeds (first 3):', smoothedSpeeds.slice(0, 3))
  console.log('Sample original speeds (first 3):', currentGpsPoints.slice(0, 3).map(p => ({ velN: p.velN, velE: p.velE, velD: p.velD })))
  console.log('Sample time differences (first 3):', currentGpsPoints.slice(0, 3).map((p, i) => i > 0 ? (p.time - currentGpsPoints[i-1].time) / 1000 : 0))
  
  // Calculate smooth sustained speeds
  console.log('Calculating smooth sustained speeds...')
  const smoothSustainedSpeeds = calculateSmoothSustainedSpeeds(currentGpsPoints, smoothedSpeeds, smoothedAccelerations)
  console.log('Smooth sustained speeds calculated:', smoothSustainedSpeeds.length, 'points')
  console.log('First 5 smooth sustained speeds:', smoothSustainedSpeeds.slice(0, 5))
  
  // Debug: Log first few smoothed speeds
 // console.log('First 5 smoothed speeds:', smoothedSpeeds.slice(0, 5))
 // console.log('First 5 original GPS speeds:', currentGpsPoints.slice(0, 5).map(p => ({ velN: p.velN, velE: p.velE, velD: p.velD })))

  // Add smoothed speeds and accelerations to GPS points
  currentGpsPoints = currentGpsPoints.map((point, index) => ({
    ...point,
    smoothVelN: smoothedSpeeds[index]?.velN,
    smoothVelE: smoothedSpeeds[index]?.velE,
    smoothVelD: smoothedSpeeds[index]?.velD,
    smoothAccelN: smoothedAccelerations[index]?.accelN,
    smoothAccelE: smoothedAccelerations[index]?.accelE,
    smoothAccelD: smoothedAccelerations[index]?.accelD,
    smoothVxs: smoothSustainedSpeeds[index]?.vxs,
    smoothVys: smoothSustainedSpeeds[index]?.vys,
    smoothRoll: smoothSustainedSpeeds[index]?.roll
  }))
  
  // Debug: Log first few GPS points with smoothed speeds
//  console.log('First 3 GPS points with smoothed speeds:', currentGpsPoints.slice(0, 3).map(p => ({
//    original: { velN: p.velN, velE: p.velE, velD: p.velD },
 //   smoothed: { velN: p.smoothVelN, velE: p.smoothVelE, velD: p.smoothVelD }
 // })))

  // Hide dropzone and show controls
  const app = document.getElementById('app')
  const quadViewContainer = document.getElementById('quad-view-container') as HTMLElement | null
  
  if (!app || !quadViewContainer) return
  
  app.classList.add('hidden')
  
  // Update controls visibility for filter type
  updateControlsVisibility()
  
  // Show quad view container
  quadViewContainer.classList.add('show')
  
  // Set up speed component selector
  setupSpeedSelector()
  
  // Generate initial plot (resets zoom)
  initialPlot()
}

async function loadDefaultFile(): Promise<void> {
  try {
    const response = await fetch('/TRACK.CSV')
    if (response.ok) {
      const csv = await response.text()
      processCSVData(csv)
    }
  } catch (error) {
    console.log('Default TRACK.CSV not found, waiting for user input')
  }
}

// Load default file when page loads
loadDefaultFile()

// Set up speed component selector
function setupSpeedSelector() {
  const selector = document.getElementById('speed-component-selector') as HTMLSelectElement
  if (selector) {
    selector.addEventListener('change', () => {
      const selectedComponent = selector.value as 'vn' | 've' | 'vd' | 'an' | 'ae' | 'ad'
      speedComparisonView.setSelectedComponent(selectedComponent)
    })
  }
}

// Initialize speed selector when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
  setupSpeedSelector()
})
