import './style.css'
import { parseCSV } from './csvParser.js'
import { plotData } from './plotter.js'
import { generatePredictedPoints } from './predict.js'
import { MLocation, PlotSeries } from './types.js'

const dropZone = document.getElementById('drop-zone')
const fileInput = document.getElementById('file-input') as HTMLInputElement | null

if (dropZone && fileInput) {
  dropZone.addEventListener('click', () => fileInput.click())
  fileInput.addEventListener('change', handleFileSelect)
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

function processCSVData(csv: string): void {
  const data = parseCSV(csv)
  console.log('Parsed CSV data:', data)
  
  // Parse GPS data with timestamps and altitude
  const latField = 'lat'
  const lonField = 'lon'
  const altField = 'hMSL'
  const timeField = 'time'

  const gpsPoints: MLocation[] = data.map((row, index) => ({
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

  if (gpsPoints.length === 0) {
    alert('No valid coordinates found')
    return
  }

  // Generate filtered GPS points and predicted points
  const predictedPoints = generatePredictedPoints(gpsPoints)

  // Create plot series
  const series: PlotSeries[] = [
    {
      name: 'GPS Points',
      data: gpsPoints,
      style: { color: '#646cff', radius: 4 }
    },
    {
      name: 'Predicted Points',
      data: predictedPoints,
      style: { color: '#ff4444', radius: 2 }
    }
  ]
  
  // Hide dropzone and make canvas fullscreen
  const app = document.getElementById('app')
  const canvas = document.getElementById('plot-canvas') as HTMLCanvasElement | null
  
  if (!app || !canvas) return
  
  app.classList.add('hidden')
  canvas.classList.remove('fullscreen') // Remove any existing state first
  canvas.classList.add('fullscreen')
  canvas.width = window.innerWidth
  canvas.height = window.innerHeight
  
  plotData(series)
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
