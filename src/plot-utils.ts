import { PlotPoint } from './types.js'

// Screen coordinate transformation utilities
export function getScreenCoords(
  x: number, y: number, z: number,
  bounds: any, zoom: number, panX: number, panY: number,
  canvas: HTMLCanvasElement
): { x: number, y: number } {
  if (!bounds) return { x: 0, y: 0 }

  const leftMargin = 200
  const rightMargin = 20
  const topMargin = 20
  const bottomMargin = 20
  const plotWidth = canvas.width - leftMargin - rightMargin
  const plotHeight = canvas.height - topMargin - bottomMargin

  const viewWidth = (bounds.maxX - bounds.minX) / zoom
  const viewHeight = (bounds.maxY - bounds.minY) / zoom
  const viewCenterX = bounds.minX + (bounds.maxX - bounds.minX) / 2 + panX
  const viewCenterY = bounds.minY + (bounds.maxY - bounds.minY) / 2 + panY

  const screenX = leftMargin + ((x - (viewCenterX - viewWidth / 2)) / viewWidth) * plotWidth
  const screenY = topMargin + (1 - (y - (viewCenterY - viewHeight / 2)) / viewHeight) * plotHeight

  return { x: screenX, y: screenY }
}

export function getSpeedScreenCoords(
  v: number, idx: number,
  bounds: any, zoom: number, panX: number, panY: number,
  canvas: HTMLCanvasElement,
  maxIdx: number
): { x: number, y: number } {
  if (!bounds) return { x: 0, y: 0 }

  const leftMargin = 200
  const rightMargin = 20
  const topMargin = 20
  const bottomMargin = 20
  const plotWidth = canvas.width - leftMargin - rightMargin
  const plotHeight = canvas.height - topMargin - bottomMargin

  const viewWidth = maxIdx / zoom
  const viewHeight = (bounds.maxSpeed - bounds.minSpeed) / zoom
  const viewCenterX = maxIdx / 2 + panX
  const viewCenterY = bounds.minSpeed + (bounds.maxSpeed - bounds.minSpeed) / 2 + panY

  const screenX = leftMargin + ((idx - (viewCenterX - viewWidth / 2)) / viewWidth) * plotWidth
  const screenY = topMargin + (1 - (v - (viewCenterY - viewHeight / 2)) / viewHeight) * plotHeight

  return { x: screenX, y: screenY }
}

export function getSmoothSustainedSpeedScreenCoords(
  vx: number, vy: number,
  bounds: any, zoom: number, panX: number, panY: number,
  canvas: HTMLCanvasElement
): { x: number, y: number } {
  if (!bounds) return { x: 0, y: 0 }

  const leftMargin = 200
  const rightMargin = 20
  const topMargin = 20
  const bottomMargin = 20
  const plotWidth = canvas.width - leftMargin - rightMargin
  const plotHeight = canvas.height - topMargin - bottomMargin

  const viewWidth = (bounds.maxVx - bounds.minVx) / zoom
  const viewHeight = (bounds.maxVy - bounds.minVy) / zoom
  const viewCenterX = bounds.minVx + (bounds.maxVx - bounds.minVx) / 2 + panX
  const viewCenterY = bounds.minVy + (bounds.maxVy - bounds.minVy) / 2 + panY

  const screenX = leftMargin + ((vx - (viewCenterX - viewWidth / 2)) / viewWidth) * plotWidth
  const screenY = topMargin + (1 - (-vy - (viewCenterY - viewHeight / 2)) / viewHeight) * plotHeight

  return { x: screenX, y: screenY }
}

export function drawAxes(
  ctx: CanvasRenderingContext2D, canvas: HTMLCanvasElement,
  bounds: any, zoom: number, panX: number, panY: number,
  xLabel: string, yLabel: string,
  isSmoothSustainedSpeed: boolean = false
): void {
  if (!bounds) return

  const leftMargin = 200
  const rightMargin = 20
  const topMargin = 20
  const bottomMargin = 20
  const plotWidth = canvas.width - leftMargin - rightMargin
  const plotHeight = canvas.height - topMargin - bottomMargin

  ctx.strokeStyle = '#ddd'
  ctx.lineWidth = 1

  // Determine axis values based on view type
  let minX: number, maxX: number, minY: number, maxY: number
  if (isSmoothSustainedSpeed) {
    const viewWidth = (bounds.maxVx - bounds.minVx) / zoom
    const viewHeight = (bounds.maxVy - bounds.minVy) / zoom
    const viewCenterX = bounds.minVx + (bounds.maxVx - bounds.minVx) / 2 + panX
    const viewCenterY = bounds.minVy + (bounds.maxVy - bounds.minVy) / 2 + panY
    
    minX = viewCenterX - viewWidth / 2
    maxX = viewCenterX + viewWidth / 2
    minY = viewCenterY - viewHeight / 2
    maxY = viewCenterY + viewHeight / 2
  } else {
    const viewWidth = (bounds.maxX - bounds.minX) / zoom
    const viewHeight = (bounds.maxY - bounds.minY) / zoom
    const viewCenterX = bounds.minX + (bounds.maxX - bounds.minX) / 2 + panX
    const viewCenterY = bounds.minY + (bounds.maxY - bounds.minY) / 2 + panY
    
    minX = viewCenterX - viewWidth / 2
    maxX = viewCenterX + viewWidth / 2
    minY = viewCenterY - viewHeight / 2
    maxY = viewCenterY + viewHeight / 2
  }

  // Draw grid lines and axis labels
  const numGridLines = 10
  
  // Vertical grid lines
  for (let i = 0; i <= numGridLines; i++) {
    const x = minX + (maxX - minX) * (i / numGridLines)
    const screenX = leftMargin + (i / numGridLines) * plotWidth
    
    ctx.beginPath()
    ctx.moveTo(screenX, topMargin)
    ctx.lineTo(screenX, topMargin + plotHeight)
    ctx.stroke()
    
    // X-axis labels
    ctx.fillStyle = '#666'
    ctx.font = '12px Arial'
    ctx.textAlign = 'center'
    ctx.fillText(x.toFixed(1), screenX, canvas.height - 5)
  }

  // Horizontal grid lines
  for (let i = 0; i <= numGridLines; i++) {
    const y = minY + (maxY - minY) * (i / numGridLines)
    const screenY = topMargin + plotHeight - (i / numGridLines) * plotHeight
    
    ctx.beginPath()
    ctx.moveTo(leftMargin, screenY)
    ctx.lineTo(leftMargin + plotWidth, screenY)
    ctx.stroke()
    
    // Y-axis labels
    ctx.fillStyle = '#666'
    ctx.font = '12px Arial'
    ctx.textAlign = 'right'
    const displayY = isSmoothSustainedSpeed ? -y : y
    ctx.fillText(displayY.toFixed(1), leftMargin - 10, screenY + 4)
  }

  // Axis labels
  ctx.fillStyle = '#333'
  ctx.font = '14px Arial'
  ctx.textAlign = 'center'
  
  // X-axis label
  ctx.fillText(xLabel, leftMargin + plotWidth / 2, canvas.height - bottomMargin + 15)
  
  // Y-axis label (rotated)
  ctx.save()
  ctx.translate(15, topMargin + plotHeight / 2)
  ctx.rotate(-Math.PI / 2)
  ctx.fillText(yLabel, 0, 0)
  ctx.restore()

  // Draw main axes
  ctx.strokeStyle = '#333'
  ctx.lineWidth = 2

  // Y-axis
  ctx.beginPath()
  ctx.moveTo(leftMargin, topMargin)
  ctx.lineTo(leftMargin, topMargin + plotHeight)
  ctx.stroke()

  // X-axis
  ctx.beginPath()
  ctx.moveTo(leftMargin, topMargin + plotHeight)
  ctx.lineTo(leftMargin + plotWidth, topMargin + plotHeight)
  ctx.stroke()
}

export function drawSpeedAxes(
  ctx: CanvasRenderingContext2D, canvas: HTMLCanvasElement,
  bounds: any, zoom: number, panX: number, panY: number,
  maxIdx: number
): void {
  if (!bounds) return

  const leftMargin = 200
  const rightMargin = 20
  const topMargin = 20
  const bottomMargin = 20
  const plotWidth = canvas.width - leftMargin - rightMargin
  const plotHeight = canvas.height - topMargin - bottomMargin

  ctx.strokeStyle = '#ddd'
  ctx.lineWidth = 1

  const viewWidth = maxIdx / zoom
  const viewHeight = (bounds.maxSpeed - bounds.minSpeed) / zoom
  const viewCenterX = maxIdx / 2 + panX
  const viewCenterY = bounds.minSpeed + (bounds.maxSpeed - bounds.minSpeed) / 2 + panY

  const minX = viewCenterX - viewWidth / 2
  const maxX = viewCenterX + viewWidth / 2
  const minY = viewCenterY - viewHeight / 2
  const maxY = viewCenterY + viewHeight / 2

  // Draw grid lines and axis labels
  const numGridLines = 10
  
  // Vertical grid lines
  for (let i = 0; i <= numGridLines; i++) {
    const x = minX + (maxX - minX) * (i / numGridLines)
    const screenX = leftMargin + (i / numGridLines) * plotWidth
    
    ctx.beginPath()
    ctx.moveTo(screenX, topMargin)
    ctx.lineTo(screenX, topMargin + plotHeight)
    ctx.stroke()
    
    // X-axis labels
    ctx.fillStyle = '#666'
    ctx.font = '12px Arial'
    ctx.textAlign = 'center'
    ctx.fillText(Math.round(x).toString(), screenX, canvas.height - 5)
  }

  // Horizontal grid lines
  for (let i = 0; i <= numGridLines; i++) {
    const y = minY + (maxY - minY) * (i / numGridLines)
    const screenY = topMargin + plotHeight - (i / numGridLines) * plotHeight
    
    ctx.beginPath()
    ctx.moveTo(leftMargin, screenY)
    ctx.lineTo(leftMargin + plotWidth, screenY)
    ctx.stroke()
    
    // Y-axis labels
    ctx.fillStyle = '#666'
    ctx.font = '12px Arial'
    ctx.textAlign = 'right'
    ctx.fillText(y.toFixed(1), leftMargin - 10, screenY + 4)
  }

  // Axis labels
  ctx.fillStyle = '#333'
  ctx.font = '14px Arial'
  ctx.textAlign = 'center'
  
  // X-axis label
  ctx.fillText('Time Index', leftMargin + plotWidth / 2, canvas.height - bottomMargin + 15)
  
  // Y-axis label (rotated)
  ctx.save()
  ctx.translate(15, topMargin + plotHeight / 2)
  ctx.rotate(-Math.PI / 2)
  ctx.fillText('Speed (m/s)', 0, 0)
  ctx.restore()

  // Draw main axes
  ctx.strokeStyle = '#333'
  ctx.lineWidth = 2

  // Y-axis
  ctx.beginPath()
  ctx.moveTo(leftMargin, topMargin)
  ctx.lineTo(leftMargin, topMargin + plotHeight)
  ctx.stroke()

  // X-axis
  ctx.beginPath()
  ctx.moveTo(leftMargin, topMargin + plotHeight)
  ctx.lineTo(leftMargin + plotWidth, topMargin + plotHeight)
  ctx.stroke()
}

export function drawLegend(ctx: CanvasRenderingContext2D, canvas: HTMLCanvasElement): void {
  const legendX = canvas.width - 180
  const legendY = 30
  
  ctx.fillStyle = 'rgba(0, 0, 0, 0.8)'
  ctx.fillRect(legendX - 10, legendY - 10, 170, 200)

  const items = [
    { color: '#ff0000', label: 'GPS Track' },
    { color: '#00ff00', label: 'Smooth Position' },
    { color: '#0000ff', label: 'Smooth Velocity' },
    { color: '#ff00ff', label: 'Smooth Acceleration' },
    { color: '#800080', label: 'Smooth Sustained Speeds' }
  ]

  items.forEach((item, index) => {
    const y = legendY + index * 25
    
    // Draw colored circle
    ctx.fillStyle = item.color
    ctx.beginPath()
    ctx.arc(legendX, y, 5, 0, 2 * Math.PI)
    ctx.fill()
    
    // Draw label
    ctx.fillStyle = '#fff'
    ctx.font = '12px system-ui'
    ctx.textAlign = 'left'
    ctx.fillText(item.label, legendX + 15, y + 4)
  })
}
