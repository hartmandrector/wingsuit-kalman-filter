import { PlotSeries, PlotPoint, Bounds } from './types.js'
import { getErrorStats } from './predict.js'

let zoom = 1
let panX = 0
let panY = 0
let allSeries: PlotSeries[] = []
let allPoints: PlotPoint[] = []
let originalBounds: Bounds | undefined
let canvas: HTMLCanvasElement
let ctx: CanvasRenderingContext2D | undefined

let isDragging = false
let lastMouseX = 0
let lastMouseY = 0
let hoveredPoint: PlotPoint | null = null

// Functions to save and restore zoom state
export function getZoomState() {
  return { zoom, panX, panY }
}

export function setZoomState(state: { zoom: number, panX: number, panY: number }) {
  zoom = state.zoom
  panX = state.panX
  panY = state.panY
}

export function plotData(series: PlotSeries[], preserveZoom: boolean = false): void {
  canvas = document.getElementById('plot-canvas') as HTMLCanvasElement
  ctx = canvas.getContext('2d') ?? undefined

  // Reset zoom state for new data (only if not preserving zoom)
  if (!preserveZoom) {
    zoom = 1
    panX = 0
    panY = 0
  }

  if (!ctx || !canvas) {
    console.error('Canvas context not available')
    return
  }

  ctx.clearRect(0, 0, canvas.width, canvas.height)

  // Store series data
  allSeries = series

  // Combine all points from all series for plotting bounds
  allPoints = series.flatMap(s => s.data)
  // Store original bounds
  originalBounds = {
    minLat: Math.min(...allPoints.map(p => p.lat)),
    maxLat: Math.max(...allPoints.map(p => p.lat)),
    minLng: Math.min(...allPoints.map(p => p.lng)),
    maxLng: Math.max(...allPoints.map(p => p.lng))
  }

  // Set up mouse event handlers for zoom and pan
  setupZoomControls()

  // Set initial cursor style
  canvas.style.cursor = 'grab'

  // Initialize point info
  updatePointInfo(null)

  // Initial render
  renderPlot(allSeries)
}

function setupZoomControls(): void {
  if (!canvas) return

  // Remove existing event listeners to avoid duplicates
  canvas.removeEventListener('wheel', handleWheel)
  canvas.removeEventListener('mousedown', handleMouseDown)
  canvas.removeEventListener('mousemove', handleMouseMove)
  canvas.removeEventListener('mouseup', handleMouseUp)
  canvas.removeEventListener('mouseleave', handleMouseUp)

  // Add wheel event for zooming
  canvas.addEventListener('wheel', handleWheel, { passive: false })

  // Add mouse events for panning
  canvas.addEventListener('mousedown', handleMouseDown)
  canvas.addEventListener('mousemove', handleMouseMove)
  canvas.addEventListener('mouseup', handleMouseUp)
  canvas.addEventListener('mouseleave', handleMouseUp)
}

function handleWheel(e: WheelEvent): void {
  e.preventDefault()

  if (!canvas) return

  const rect = canvas.getBoundingClientRect()
  const mouseX = e.clientX - rect.left
  const mouseY = e.clientY - rect.top

  // Get mouse position in world coordinates before zoom
  const worldBefore = screenToWorld(mouseX, mouseY)

  // Update zoom
  const zoomFactor = e.deltaY < 0 ? 1.2 : 1/1.2
  const oldZoom = zoom
  zoom *= zoomFactor

  // Clamp zoom to reasonable limits
  zoom = Math.max(0.1, Math.min(zoom, 50))
  const actualZoomFactor = zoom / oldZoom

  // Get mouse position in world coordinates after zoom
  const worldAfter = screenToWorld(mouseX, mouseY)

  // Adjust pan to keep mouse position stable
  panX += (worldBefore.lng - worldAfter.lng)
  panY += (worldBefore.lat - worldAfter.lat)

  // Re-render with new zoom and pan
  renderPlot(allSeries)
}

function handleMouseDown(e: MouseEvent): void {
  if (!canvas) return

  const rect = canvas.getBoundingClientRect()
  isDragging = true
  lastMouseX = e.clientX - rect.left
  lastMouseY = e.clientY - rect.top
  canvas.style.cursor = 'grabbing'
}

function handleMouseMove(e: MouseEvent): void {
  if (!canvas) return

  const rect = canvas.getBoundingClientRect()
  const mouseX = e.clientX - rect.left
  const mouseY = e.clientY - rect.top

  if (isDragging) {
    const deltaX = mouseX - lastMouseX
    const deltaY = mouseY - lastMouseY

    // Convert pixel delta to world coordinates
    const viewWidth = getViewWidth()
    const viewHeight = getViewHeight()
    const leftMargin = 200
    const rightMargin = 20
    const topMargin = 20
    const bottomMargin = 20
    const plotWidth = canvas.width - leftMargin - rightMargin
    const plotHeight = canvas.height - topMargin - bottomMargin

    panX -= (deltaX / plotWidth) * viewWidth
    panY += (deltaY / plotHeight) * viewHeight

    lastMouseX = mouseX
    lastMouseY = mouseY

    // Re-render with new pan
    renderPlot(allSeries)
  } else {
    // Handle hover detection
    const nearestPoint = findNearestPoint(mouseX, mouseY)
    if (nearestPoint !== hoveredPoint) {
      hoveredPoint = nearestPoint
      updatePointInfo(hoveredPoint)
      renderPlot(allSeries) // Re-render to show/hide hover highlight
    }
    canvas.style.cursor = 'grab'
  }
}

function handleMouseUp(): void {
  isDragging = false
  if (canvas) canvas.style.cursor = 'grab'
}

function screenToWorld(screenX: number, screenY: number): {lng: number, lat: number} {
  if (!canvas || !originalBounds) {
    return { lng: 0, lat: 0 }
  }

  const leftMargin = 200
  const rightMargin = 20
  const topMargin = 20
  const bottomMargin = 20
  const plotWidth = canvas.width - leftMargin - rightMargin
  const plotHeight = canvas.height - topMargin - bottomMargin

  const viewWidth = getViewWidth()
  const viewHeight = getViewHeight()
  const centerLng = (originalBounds.minLng + originalBounds.maxLng) / 2
  const centerLat = (originalBounds.minLat + originalBounds.maxLat) / 2

  const lng = centerLng + panX + ((screenX - leftMargin) / plotWidth - 0.5) * viewWidth
  const lat = centerLat + panY + (0.5 - (screenY - topMargin) / plotHeight) * viewHeight

  return { lng, lat }
}

function getViewWidth(): number {
  if (!originalBounds) return 0
  return (originalBounds.maxLng - originalBounds.minLng) / zoom
}

function getViewHeight(): number {
  if (!originalBounds) return 0
  return (originalBounds.maxLat - originalBounds.minLat) / zoom
}

function findNearestPoint(screenX: number, screenY: number): PlotPoint | null {
  if (!canvas || !originalBounds || allPoints.length === 0) return null

  const leftMargin = 200
  const rightMargin = 20
  const topMargin = 20
  const bottomMargin = 20
  const plotWidth = canvas.width - leftMargin - rightMargin
  const plotHeight = canvas.height - topMargin - bottomMargin

  // Calculate current view bounds
  const viewWidth = getViewWidth()
  const viewHeight = getViewHeight()
  const centerLng = (originalBounds.minLng + originalBounds.maxLng) / 2
  const centerLat = (originalBounds.minLat + originalBounds.maxLat) / 2

  const minLng = centerLng + panX - viewWidth / 2
  const maxLng = centerLng + panX + viewWidth / 2
  const minLat = centerLat + panY - viewHeight / 2
  const maxLat = centerLat + panY + viewHeight / 2

  let nearestPoint: PlotPoint | null = null
  let minDistance = Infinity
  const hoverRadius = 15 // pixels

  for (const point of allPoints) {
    const x = leftMargin + ((point.lng - minLng) / (maxLng - minLng)) * plotWidth
    const y = topMargin + plotHeight - ((point.lat - minLat) / (maxLat - minLat)) * plotHeight

    // Only consider points within view
    if (x >= leftMargin && x <= leftMargin + plotWidth && y >= topMargin && y <= topMargin + plotHeight) {
      const distance = Math.sqrt((x - screenX) ** 2 + (y - screenY) ** 2)
      if (distance < hoverRadius && distance < minDistance) {
        minDistance = distance
        nearestPoint = point
      }
    }
  }

  return nearestPoint
}

function updatePointInfo(point: PlotPoint | null): void {
  const pointDetails = document.getElementById('point-details')
  if (!pointDetails) return

  if (!point) {
    pointDetails.innerHTML = '<div>Hover over a point to see details</div>'
    return
  }

  let time = 'N/A'
  if (point.time) {
    const date = new Date(point.time)
    const hours = date.getHours().toString().padStart(2, '0')
    const minutes = date.getMinutes().toString().padStart(2, '0')
    const seconds = date.getSeconds().toString().padStart(2, '0')
    const milliseconds = date.getMilliseconds().toString().padStart(3, '0')
    time = `${hours}:${minutes}:${seconds}.${milliseconds}`
  }
  const altitude = point.alt ? `${point.alt.toFixed(1)} m` : 'N/A'
  
  let velocityInfo = ''
  // Check if this is an MLocation (has velocity data)
  const mlocation = point as any
  if (mlocation.velN !== undefined && mlocation.velE !== undefined && mlocation.velD !== undefined) {
    const speed = Math.sqrt(mlocation.velN ** 2 + mlocation.velE ** 2 + mlocation.velD ** 2)
    velocityInfo = `
      <div><strong>Speed:</strong> ${speed.toFixed(1)} m/s (${(speed * 2.23694).toFixed(1)} mph)</div>
      <div><strong>Vel N:</strong> ${mlocation.velN.toFixed(1)} m/s</div>
      <div><strong>Vel E:</strong> ${mlocation.velE.toFixed(1)} m/s</div>
      <div><strong>Vel D:</strong> ${mlocation.velD.toFixed(1)} m/s</div>
    `
  }

  pointDetails.innerHTML = `
    <div><strong>Position:</strong></div>
    <div>Lat: ${point.lat.toFixed(6)}°</div>
    <div>Lng: ${point.lng.toFixed(6)}°</div>
    <div>Alt: ${altitude}</div>
    <div><strong>Time:</strong> ${time}</div>
    ${velocityInfo}
  `
}

function renderPlot(series: PlotSeries[]): void {
  if (!originalBounds || !canvas || !ctx) return

  const leftMargin = 200
  const rightMargin = 20
  const topMargin = 20
  const bottomMargin = 20
  const plotWidth = canvas.width - leftMargin - rightMargin
  const plotHeight = canvas.height - topMargin - bottomMargin

  // Calculate current view bounds
  const viewWidth = getViewWidth()
  const viewHeight = getViewHeight()
  const centerLng = (originalBounds.minLng + originalBounds.maxLng) / 2
  const centerLat = (originalBounds.minLat + originalBounds.maxLat) / 2

  const minLng = centerLng + panX - viewWidth / 2
  const maxLng = centerLng + panX + viewWidth / 2
  const minLat = centerLat + panY - viewHeight / 2
  const maxLat = centerLat + panY + viewHeight / 2

  // Clear canvas
  ctx.clearRect(0, 0, canvas.width, canvas.height)

  // Draw background
  ctx.fillStyle = '#333'
  ctx.fillRect(leftMargin, topMargin, plotWidth, plotHeight)

  ctx.strokeStyle = '#666'
  ctx.strokeRect(leftMargin, topMargin, plotWidth, plotHeight)

  // Draw all series
  for (const plotSeries of series) {
    ctx.fillStyle = plotSeries.style.color

    for (const point of plotSeries.data) {
      const x = leftMargin + ((point.lng - minLng) / (maxLng - minLng)) * plotWidth
      const y = topMargin + plotHeight - ((point.lat - minLat) / (maxLat - minLat)) * plotHeight

      // Only draw if point is within view
      if (x >= leftMargin && x <= leftMargin + plotWidth && y >= topMargin && y <= topMargin + plotHeight) {
        ctx.beginPath()
        
        // Highlight hovered point
        if (hoveredPoint === point) {
          ctx.strokeStyle = '#ffffff'
          ctx.lineWidth = 2
          ctx.arc(x, y, plotSeries.style.radius + 2, 0, 2 * Math.PI)
          ctx.stroke()
          ctx.beginPath()
          ctx.fillStyle = plotSeries.style.color
        }
        
        ctx.arc(x, y, plotSeries.style.radius, 0, 2 * Math.PI)
        ctx.fill()
      }
    }
  }

  // Draw info text
  ctx.fillStyle = '#fff'
  ctx.font = '12px system-ui'

  // Draw series info on left side
  let yOffset = topMargin + 20
  for (const plotSeries of series) {
    ctx.fillText(`${plotSeries.name}: ${plotSeries.data.length}`, 20, yOffset)
    yOffset += 15
  }

  // Draw error statistics on left side
  const errorStats = getErrorStats()
  if (errorStats) {
    yOffset += 10
    ctx.fillText(`Position Prediction Error (m):`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Min: ${errorStats.position.min.toFixed(2)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Avg: ${errorStats.position.avg.toFixed(2)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Max: ${errorStats.position.max.toFixed(2)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Count: ${errorStats.position.count}`, 20, yOffset)
    yOffset += 20
    ctx.fillText(`Velocity Prediction Error (mph):`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Min: ${(errorStats.velocity.min * 2.23694).toFixed(2)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Avg: ${(errorStats.velocity.avg * 2.23694).toFixed(2)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Max: ${(errorStats.velocity.max * 2.23694).toFixed(2)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Count: ${errorStats.velocity.count}`, 20, yOffset)
    
    // Draw smoothness metrics
    yOffset += 20
    ctx.fillText(`Movement Smoothness:`, 20, yOffset)
    
    yOffset += 15
    ctx.fillText(`Jerk (m/s³):`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Min: ${errorStats.smoothness.jerk.min.toFixed(2)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Avg: ${errorStats.smoothness.jerk.avg.toFixed(2)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Max: ${errorStats.smoothness.jerk.max.toFixed(2)}`, 20, yOffset)
    
    yOffset += 15
    ctx.fillText(`Curvature (1/m):`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Min: ${errorStats.smoothness.curvature.min.toFixed(4)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Avg: ${errorStats.smoothness.curvature.avg.toFixed(4)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Max: ${errorStats.smoothness.curvature.max.toFixed(4)}`, 20, yOffset)
    
    yOffset += 15
    ctx.fillText(`Velocity Change Rate (m/s²):`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Min: ${errorStats.smoothness.velocityChangeRate.min.toFixed(2)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Avg: ${errorStats.smoothness.velocityChangeRate.avg.toFixed(2)}`, 20, yOffset)
    yOffset += 15
    ctx.fillText(`  Max: ${errorStats.smoothness.velocityChangeRate.max.toFixed(2)}`, 20, yOffset)
  }
}
