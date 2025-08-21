import './style.css'
import { parseCSV } from './csvParser.js'
import { plotData } from './plotter.js'
import { generatePredictedPoints, setAlpha, setAlphaVelocity, setAlphaAcceleration } from './predict.js'
import { MLocation, PlotSeries } from './types.js'

let currentGpsPoints: MLocation[] = []

const dropZone = document.getElementById('drop-zone')
const fileInput = document.getElementById('file-input') as HTMLInputElement | null

if (dropZone && fileInput) {
  dropZone.addEventListener('click', () => fileInput.click())
  fileInput.addEventListener('change', handleFileSelect)
}

// Setup alpha sliders
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
    
    // Regenerate plot if we have data
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
    
    // Regenerate plot if we have data
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
    
    // Regenerate plot if we have data
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
  const canvas = document.getElementById('plot-canvas')
  const overlay = document.getElementById('drag-overlay')
  
  if (canvas?.classList.contains('fullscreen')) {
    overlay?.classList.add('show')
  } else {
    dropZone?.classList.add('dragover')
  }
}

function handleDragLeave(e: DragEvent): void {
  const canvas = document.getElementById('plot-canvas')
  const overlay = document.getElementById('drag-overlay')
  
  // Only remove dragover when actually leaving the window
  if (e.clientX === 0 && e.clientY === 0) {
    if (canvas?.classList.contains('fullscreen')) {
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
  
  // Generate new predicted points with current alpha
  const predictedPoints = generatePredictedPoints(currentGpsPoints)

  // Create plot series
  const series: PlotSeries[] = [
    {
      name: 'GPS Points',
      data: currentGpsPoints,
      style: { color: '#646cff', radius: 4 }
    },
    {
      name: 'Predicted Points',
      data: predictedPoints,
      style: { color: '#ff4444', radius: 2 }
    }
  ]
  
  plotData(series)
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

  // Hide dropzone and show controls
  const app = document.getElementById('app')
  const canvas = document.getElementById('plot-canvas') as HTMLCanvasElement | null
  const controlsPanel = document.getElementById('controls-panel') as HTMLElement | null
  
  if (!app || !canvas) return
  
  app.classList.add('hidden')
  
  if (controlsPanel) {
    controlsPanel.classList.add('show')
  }
  
  // Make canvas fullscreen
  canvas.classList.remove('fullscreen') // Remove any existing state first
  canvas.classList.add('fullscreen')
  canvas.width = window.innerWidth
  canvas.height = window.innerHeight
  
  // Generate initial plot
  regeneratePlot()
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
