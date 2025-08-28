import { PlotView } from './plot-view-base.js'
import { PlotSeries, PlotPoint } from './types.js'
import { getScreenCoords, drawAxes } from './plot-utils.js'

export class ProfileView extends PlotView {
  constructor(canvasId: string) {
    super(canvasId)
  }

  calculateBounds(points: PlotPoint[]) {
    if (points.length === 0) return undefined

    let minX = Infinity, maxX = -Infinity
    let minY = Infinity, maxY = -Infinity

    for (const point of points) {
      // For profile view, we use z (North) as X and y (Up) as Y
      if (point.z !== undefined && point.z < minX) minX = point.z
      if (point.z !== undefined && point.z > maxX) maxX = point.z
      if (point.y !== undefined && point.y < minY) minY = point.y
      if (point.y !== undefined && point.y > maxY) maxY = point.y
    }

    // Add 10% padding
    const paddingX = (maxX - minX) * 0.1
    const paddingY = (maxY - minY) * 0.1

    return {
      minX: minX - paddingX,
      maxX: maxX + paddingX,
      minY: minY - paddingY,
      maxY: maxY + paddingY
    }
  }

  screenToWorld(screenX: number, screenY: number) {
    if (!this.canvas || !this.originalBounds) return { x: 0, y: 0 }

    const leftMargin = 200
    const rightMargin = 20
    const topMargin = 20
    const bottomMargin = 20
    const plotWidth = this.canvas.width - leftMargin - rightMargin
    const plotHeight = this.canvas.height - topMargin - bottomMargin

    const viewWidth = (this.originalBounds.maxX - this.originalBounds.minX) / this.zoom
    const viewHeight = (this.originalBounds.maxY - this.originalBounds.minY) / this.zoom
    const viewCenterX = this.originalBounds.minX + (this.originalBounds.maxX - this.originalBounds.minX) / 2 + this.panX
    const viewCenterY = this.originalBounds.minY + (this.originalBounds.maxY - this.originalBounds.minY) / 2 + this.panY

    const normalizedX = (screenX - leftMargin) / plotWidth
    const normalizedY = 1 - (screenY - topMargin) / plotHeight

    const worldX = (viewCenterX - viewWidth / 2) + normalizedX * viewWidth
    const worldY = (viewCenterY - viewHeight / 2) + normalizedY * viewHeight

    return { x: worldX, y: worldY }
  }

  getViewWidth(): number {
    if (!this.originalBounds) return 0
    return (this.originalBounds.maxX - this.originalBounds.minX) / this.zoom
  }

  getViewHeight(): number {
    if (!this.originalBounds) return 0
    return (this.originalBounds.maxY - this.originalBounds.minY) / this.zoom
  }

  worldToScreen(point: PlotPoint): { x: number, y: number } {
    if (!this.canvas || point.z === undefined || point.y === undefined) return { x: 0, y: 0 }
    // For profile view: X=North (z), Y=Up (y), Z=East (x) 
    return getScreenCoords(point.z, point.y, point.x ?? 0, this.originalBounds, this.zoom, this.panX, this.panY, this.canvas)
  }

  renderPlot(): void {
    if (!this.ctx || !this.canvas || !this.originalBounds) return

    // Clear canvas
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

    // Draw axes
    drawAxes(this.ctx, this.canvas, this.originalBounds, this.zoom, this.panX, this.panY, 'North (m)', 'Up (m)')

    // Draw series
    const plotColors = ['#ff0000', '#00ff00'] // Red and green colors
    for (let i = 0; i < this.allSeries.length; i++) {
      const series = this.allSeries[i]
      
      // Use consistent colors for both plot and legend
      const color = plotColors[i % plotColors.length]
      this.ctx.strokeStyle = color
      this.ctx.fillStyle = color
      this.ctx.lineWidth = 2

      if (series.data.length === 0) continue

      this.ctx.beginPath()
      let first = true

      for (const point of series.data) {
        if (point.z === undefined || point.y === undefined) continue
        // For profile view: X=North (z), Y=Up (y), Z=East (x)
        const screenPos = getScreenCoords(point.z, point.y, point.x ?? 0, this.originalBounds, this.zoom, this.panX, this.panY, this.canvas)

        if (first) {
          this.ctx.moveTo(screenPos.x, screenPos.y)
          first = false
        } else {
          this.ctx.lineTo(screenPos.x, screenPos.y)
        }
      }

      this.ctx.stroke()

      // Draw points
      for (const point of series.data) {
        if (point.z === undefined || point.y === undefined) continue
        const screenPos = getScreenCoords(point.z, point.y, point.x ?? 0, this.originalBounds, this.zoom, this.panX, this.panY, this.canvas)
        
        this.ctx.beginPath()
        this.ctx.arc(screenPos.x, screenPos.y, 3, 0, 2 * Math.PI)
        this.ctx.fill()
      }
    }

    // Highlight hovered point
    if (this.hoveredPoint && this.hoveredPoint.z !== undefined && this.hoveredPoint.y !== undefined) {
      const screenPos = getScreenCoords(this.hoveredPoint.z, this.hoveredPoint.y, this.hoveredPoint.x ?? 0, this.originalBounds, this.zoom, this.panX, this.panY, this.canvas)
      
      this.ctx.strokeStyle = '#000'
      this.ctx.lineWidth = 3
      this.ctx.beginPath()
      this.ctx.arc(screenPos.x, screenPos.y, 8, 0, 2 * Math.PI)
      this.ctx.stroke()

      // Show tooltip
      this.showTooltip(screenPos.x, screenPos.y, this.hoveredPoint)
    }

    // Draw legend
    this.drawCustomLegend()
  }

  private drawCustomLegend(): void {
    if (!this.ctx || !this.canvas) return

    // Only show position-related series in the legend
    const positionSeries = this.allSeries.filter(series => 
      series.data.length > 0 && (
        series.name.toLowerCase().includes('gps') || 
        series.name.toLowerCase().includes('position') ||
        series.name.toLowerCase().includes('points')
      )
    )

    if (positionSeries.length === 0) return

    const legendX = this.canvas.width - 180
    const legendY = 30
    const lineHeight = 25
    const legendHeight = positionSeries.length * lineHeight + 20

    // Draw legend background
    this.ctx.fillStyle = 'rgba(0, 0, 0, 0.8)'
    this.ctx.fillRect(legendX - 10, legendY - 10, 170, legendHeight)

    // Draw legend items
    this.ctx.font = '12px system-ui'
    this.ctx.textAlign = 'left'

    // Define the actual colors used in the plot
    const actualColors = ['#ff0000', '#00ff00'] // Red and green colors

    positionSeries.forEach((series, index) => {
      const y = legendY + index * lineHeight
      
      // Use the actual color from the plot, not from series.style
      const actualColor = actualColors[index % actualColors.length]
      
      // Draw colored line and circle matching the actual plot colors
      this.ctx!.strokeStyle = actualColor
      this.ctx!.fillStyle = actualColor
      this.ctx!.lineWidth = 2
      
      // Draw line segment
      this.ctx!.beginPath()
      this.ctx!.moveTo(legendX - 8, y)
      this.ctx!.lineTo(legendX + 8, y)
      this.ctx!.stroke()
      
      // Draw circle (point)
      this.ctx!.beginPath()
      this.ctx!.arc(legendX, y, 3, 0, 2 * Math.PI)
      this.ctx!.fill()
      
      // Draw label
      this.ctx!.fillStyle = '#fff'
      this.ctx!.fillText(series.name, legendX + 20, y + 4)
    })
  }

  private showTooltip(x: number, y: number, point: PlotPoint): void {
    if (!this.ctx) return

    const timeStr = new Date(point.time * 1000).toLocaleTimeString()
    const text = `Time: ${timeStr}\nNorth: ${(point.z ?? 0).toFixed(2)}m\nUp: ${(point.y ?? 0).toFixed(2)}m\nEast: ${(point.x ?? 0).toFixed(2)}m`
    const lines = text.split('\n')
    
    const padding = 8
    const lineHeight = 16
    const maxWidth = Math.max(...lines.map(line => this.ctx!.measureText(line).width))
    const tooltipWidth = maxWidth + padding * 2
    const tooltipHeight = lines.length * lineHeight + padding * 2

    // Position tooltip to avoid going off screen
    let tooltipX = x + 10
    let tooltipY = y - tooltipHeight - 10

    if (tooltipX + tooltipWidth > this.canvas!.width) {
      tooltipX = x - tooltipWidth - 10
    }
    if (tooltipY < 0) {
      tooltipY = y + 10
    }

    // Draw tooltip background
    this.ctx.fillStyle = 'rgba(0, 0, 0, 0.8)'
    this.ctx.fillRect(tooltipX, tooltipY, tooltipWidth, tooltipHeight)

    // Draw tooltip text
    this.ctx.fillStyle = '#fff'
    this.ctx.font = '12px Arial'
    this.ctx.textAlign = 'left'

    lines.forEach((line, index) => {
      this.ctx!.fillText(line, tooltipX + padding, tooltipY + padding + (index + 1) * lineHeight)
    })
  }
}
