import { PlotSeries, PlotPoint, Bounds } from './types.js'
import { getErrorStats } from './predict.js'

// Global hover synchronization
let globalSyncHover: ((hoveredPoint: PlotPoint | null, sourceView?: PlotView) => void) | null = null

// Base class for view-specific plotting functionality
abstract class PlotView {
  protected zoom = 1
  protected panX = 0
  protected panY = 0
  protected allSeries: PlotSeries[] = []
  protected allPoints: PlotPoint[] = []
  protected originalBounds: any = undefined
  protected canvas: HTMLCanvasElement | undefined
  protected ctx: CanvasRenderingContext2D | undefined
  protected isDragging = false
  protected lastMouseX = 0
  protected lastMouseY = 0
  public hoveredPoint: PlotPoint | null = null

  constructor(protected canvasId: string) {}

  abstract calculateBounds(points: PlotPoint[]): any
  abstract screenToWorld(screenX: number, screenY: number): any
  abstract getViewWidth(): number
  abstract getViewHeight(): number
  abstract worldToScreen(point: PlotPoint): { x: number, y: number }

  public getZoomState() {
    return { zoom: this.zoom, panX: this.panX, panY: this.panY }
  }

  public setZoomState(state: { zoom: number, panX: number, panY: number }) {
    this.zoom = state.zoom
    this.panX = state.panX
    this.panY = state.panY
  }

  public plotData(series: PlotSeries[], preserveZoom: boolean = false): void {
    this.canvas = document.getElementById(this.canvasId) as HTMLCanvasElement
    this.ctx = this.canvas?.getContext('2d') ?? undefined

    if (!preserveZoom) {
      this.zoom = 1
      this.panX = 0
      this.panY = 0
    }

    if (!this.ctx || !this.canvas) {
      console.error(`Canvas context not available for ${this.canvasId}`)
      return
    }

    // Store series data
    this.allSeries = series
    this.allPoints = series.flatMap(s => s.data)
    this.originalBounds = this.calculateBounds(this.allPoints)

    // Set up mouse event handlers
    this.setupControls()
    this.canvas.style.cursor = 'grab'

    // Initial render
    this.renderPlot()
  }

  public setupControls(): void {
    if (!this.canvas) return

    // Remove existing event listeners
    this.canvas.removeEventListener('wheel', this.handleWheel)
    this.canvas.removeEventListener('mousedown', this.handleMouseDown)
    this.canvas.removeEventListener('mousemove', this.handleMouseMove)
    this.canvas.removeEventListener('mouseup', this.handleMouseUp)
    this.canvas.removeEventListener('mouseleave', this.handleMouseUp)

    // Add new event listeners
    this.canvas.addEventListener('wheel', this.handleWheel.bind(this), { passive: false })
    this.canvas.addEventListener('mousedown', this.handleMouseDown.bind(this))
    this.canvas.addEventListener('mousemove', this.handleMouseMove.bind(this))
    this.canvas.addEventListener('mouseup', this.handleMouseUp.bind(this))
    this.canvas.addEventListener('mouseleave', this.handleMouseUp.bind(this))
  }

  protected handleWheel(e: WheelEvent): void {
    e.preventDefault()
    if (!this.canvas) return

    const rect = this.canvas.getBoundingClientRect()
    const mouseX = e.clientX - rect.left
    const mouseY = e.clientY - rect.top

    const worldBefore = this.screenToWorld(mouseX, mouseY)
    const zoomFactor = e.deltaY < 0 ? 1.2 : 1/1.2
    const oldZoom = this.zoom
    this.zoom *= zoomFactor
    this.zoom = Math.max(0.1, Math.min(this.zoom, 50))
    
    const worldAfter = this.screenToWorld(mouseX, mouseY)
    this.panX += (worldBefore.x - worldAfter.x)
    this.panY += (worldBefore.y - worldAfter.y)

    this.renderPlot()
  }

  protected handleMouseDown(e: MouseEvent): void {
    if (!this.canvas) return
    const rect = this.canvas.getBoundingClientRect()
    this.isDragging = true
    this.lastMouseX = e.clientX - rect.left
    this.lastMouseY = e.clientY - rect.top
    this.canvas.style.cursor = 'grabbing'
  }

  public handleMouseMove(e: MouseEvent): void {
    if (!this.canvas) return
    const rect = this.canvas.getBoundingClientRect()
    const mouseX = e.clientX - rect.left
    const mouseY = e.clientY - rect.top

    if (this.isDragging) {
      const deltaX = mouseX - this.lastMouseX
      const deltaY = mouseY - this.lastMouseY

      const viewWidth = this.getViewWidth()
      const viewHeight = this.getViewHeight()
      const leftMargin = 200
      const rightMargin = 20
      const topMargin = 20
      const bottomMargin = 20
      const plotWidth = this.canvas.width - leftMargin - rightMargin
      const plotHeight = this.canvas.height - topMargin - bottomMargin

      this.panX -= (deltaX / plotWidth) * viewWidth
      this.panY += (deltaY / plotHeight) * viewHeight

      this.lastMouseX = mouseX
      this.lastMouseY = mouseY
      this.renderPlot()
    } else {
      const nearestPoint = this.findNearestPoint(mouseX, mouseY)
      if (nearestPoint !== this.hoveredPoint) {
        this.hoveredPoint = nearestPoint
        // Use global hover synchronization function if available
        if (globalSyncHover) {
          globalSyncHover(nearestPoint, this)
        } else {
          updatePointInfo(nearestPoint) // Fallback
          this.renderPlot()
        }
      }
      this.canvas.style.cursor = 'grab'
    }
  }

  protected handleMouseUp(): void {
    this.isDragging = false
    if (this.canvas) this.canvas.style.cursor = 'grab'
  }

  protected findNearestPoint(screenX: number, screenY: number): PlotPoint | null {
    if (!this.canvas || !this.originalBounds || this.allPoints.length === 0) return null

    let nearestPoint: PlotPoint | null = null
    let minDistance = Infinity
    const hoverRadius = 15

    for (const point of this.allPoints) {
      const screenPos = this.worldToScreen(point)
      const distance = Math.sqrt((screenPos.x - screenX) ** 2 + (screenPos.y - screenY) ** 2)
      
      if (distance < hoverRadius && distance < minDistance) {
        minDistance = distance
        nearestPoint = point
      }
    }

    return nearestPoint
  }

  public renderPlot(): void {
    if (!this.originalBounds || !this.canvas || !this.ctx) return

    const leftMargin = 200
    const rightMargin = 20
    const topMargin = 50
    const bottomMargin = 20
    const plotWidth = this.canvas.width - leftMargin - rightMargin
    const plotHeight = this.canvas.height - topMargin - bottomMargin

    // Clear canvas
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

    // Draw background
    this.ctx.fillStyle = '#333'
    this.ctx.fillRect(leftMargin, topMargin, plotWidth, plotHeight)
    this.ctx.strokeStyle = '#666'
    this.ctx.strokeRect(leftMargin, topMargin, plotWidth, plotHeight)

    // Draw all series
    for (const plotSeries of this.allSeries) {
      this.ctx.fillStyle = plotSeries.style.color

      for (const point of plotSeries.data) {
        const screenPos = this.worldToScreen(point)
        
        if (screenPos.x >= leftMargin && screenPos.x <= leftMargin + plotWidth && 
            screenPos.y >= topMargin && screenPos.y <= topMargin + plotHeight) {
          
          this.ctx.beginPath()
          
          if (this.hoveredPoint === point) {
            this.ctx.strokeStyle = '#ffffff'
            this.ctx.lineWidth = 2
            this.ctx.arc(screenPos.x, screenPos.y, plotSeries.style.radius + 2, 0, 2 * Math.PI)
            this.ctx.stroke()
            this.ctx.beginPath()
            this.ctx.fillStyle = plotSeries.style.color
          }
          
          this.ctx.arc(screenPos.x, screenPos.y, plotSeries.style.radius, 0, 2 * Math.PI)
          this.ctx.fill()
        }
      }
    }

    // Draw info text
    this.ctx.fillStyle = '#fff'
    this.ctx.font = '12px system-ui'

    // Draw series info on left side
    let yOffset = topMargin + 20
    for (const plotSeries of this.allSeries) {
      this.ctx.fillText(`${plotSeries.name}: ${plotSeries.data.length}`, 20, yOffset)
      yOffset += 15
    }

    // Draw error statistics on left side
    const errorStats = getErrorStats()
    if (errorStats) {
      yOffset += 10
      this.ctx.fillText(`Position Prediction Error (m):`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Min: ${errorStats.position.min.toFixed(2)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Avg: ${errorStats.position.avg.toFixed(2)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Max: ${errorStats.position.max.toFixed(2)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Count: ${errorStats.position.count}`, 20, yOffset)
      yOffset += 20
      this.ctx.fillText(`Velocity Prediction Error (mph):`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Min: ${(errorStats.velocity.min * 2.23694).toFixed(2)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Avg: ${(errorStats.velocity.avg * 2.23694).toFixed(2)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Max: ${(errorStats.velocity.max * 2.23694).toFixed(2)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Count: ${errorStats.velocity.count}`, 20, yOffset)
      
      // Draw smoothness metrics
      yOffset += 20
      this.ctx.fillText(`Movement Smoothness:`, 20, yOffset)
      
      yOffset += 15
      this.ctx.fillText(`Jerk (m/s³):`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Min: ${errorStats.smoothness.jerk.min.toFixed(2)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Avg: ${errorStats.smoothness.jerk.avg.toFixed(2)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Max: ${errorStats.smoothness.jerk.max.toFixed(2)}`, 20, yOffset)
      
      yOffset += 15
      this.ctx.fillText(`Curvature (1/m):`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Min: ${errorStats.smoothness.curvature.min.toFixed(4)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Avg: ${errorStats.smoothness.curvature.avg.toFixed(4)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Max: ${errorStats.smoothness.curvature.max.toFixed(4)}`, 20, yOffset)
      
      yOffset += 15
      this.ctx.fillText(`Velocity Change Rate (m/s²):`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Min: ${errorStats.smoothness.velocityChangeRate.min.toFixed(2)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Avg: ${errorStats.smoothness.velocityChangeRate.avg.toFixed(2)}`, 20, yOffset)
      yOffset += 15
      this.ctx.fillText(`  Max: ${errorStats.smoothness.velocityChangeRate.max.toFixed(2)}`, 20, yOffset)
    }
  }
}

// Top-down view (East vs North)
class TopDownView extends PlotView {
  calculateBounds(points: PlotPoint[]) {
    return {
      minX: Math.min(...points.map(p => p.x || p.lng)),
      maxX: Math.max(...points.map(p => p.x || p.lng)),
      minY: Math.min(...points.map(p => p.z || p.lat)),
      maxY: Math.max(...points.map(p => p.z || p.lat))
    }
  }

  screenToWorld(screenX: number, screenY: number) {
    if (!this.canvas || !this.originalBounds) return { x: 0, y: 0 }

    const leftMargin = 200
    const rightMargin = 20
    const topMargin = 50
    const bottomMargin = 20
    const plotWidth = this.canvas.width - leftMargin - rightMargin
    const plotHeight = this.canvas.height - topMargin - bottomMargin

    const viewWidth = this.getViewWidth()
    const viewHeight = this.getViewHeight()
    const centerX = (this.originalBounds.minX + this.originalBounds.maxX) / 2
    const centerY = (this.originalBounds.minY + this.originalBounds.maxY) / 2

    const x = centerX + this.panX + ((screenX - leftMargin) / plotWidth - 0.5) * viewWidth
    const y = centerY + this.panY + (0.5 - (screenY - topMargin) / plotHeight) * viewHeight

    return { x, y }
  }

  getViewWidth(): number {
    return this.originalBounds ? (this.originalBounds.maxX - this.originalBounds.minX) / this.zoom : 0
  }

  getViewHeight(): number {
    return this.originalBounds ? (this.originalBounds.maxY - this.originalBounds.minY) / this.zoom : 0
  }

  worldToScreen(point: PlotPoint): { x: number, y: number } {
    if (!this.canvas || !this.originalBounds) return { x: 0, y: 0 }

    const leftMargin = 200
    const topMargin = 50
    const rightMargin = 20
    const bottomMargin = 20
    const plotWidth = this.canvas.width - leftMargin - rightMargin
    const plotHeight = this.canvas.height - topMargin - bottomMargin

    const viewWidth = this.getViewWidth()
    const viewHeight = this.getViewHeight()
    const centerX = (this.originalBounds.minX + this.originalBounds.maxX) / 2
    const centerY = (this.originalBounds.minY + this.originalBounds.maxY) / 2

    const minX = centerX + this.panX - viewWidth / 2
    const maxX = centerX + this.panX + viewWidth / 2
    const minY = centerY + this.panY - viewHeight / 2
    const maxY = centerY + this.panY + viewHeight / 2

    const pointX = point.x !== undefined ? point.x : point.lng
    const pointY = point.z !== undefined ? point.z : point.lat

    const x = leftMargin + ((pointX - minX) / (maxX - minX)) * plotWidth
    const y = topMargin + plotHeight - ((pointY - minY) / (maxY - minY)) * plotHeight

    return { x, y }
  }
}

// Profile view (Horizontal Distance vs Altitude)
class ProfileView extends PlotView {
  calculateBounds(points: PlotPoint[]) {
    // Calculate horizontal distance from origin for each point
    const horizontalDistances = points.map(p => {
      const x = p.x !== undefined ? p.x : 0
      const z = p.z !== undefined ? p.z : 0
      return Math.sqrt(x * x + z * z)
    })

    const altitudes = points.map(p => p.alt)

    return {
      minHorizontalDistance: Math.min(...horizontalDistances),
      maxHorizontalDistance: Math.max(...horizontalDistances),
      minAltitude: Math.min(...altitudes),
      maxAltitude: Math.max(...altitudes)
    }
  }

  screenToWorld(screenX: number, screenY: number) {
    if (!this.canvas || !this.originalBounds) return { x: 0, y: 0 }

    const leftMargin = 200
    const rightMargin = 20
    const topMargin = 50
    const bottomMargin = 20
    const plotWidth = this.canvas.width - leftMargin - rightMargin
    const plotHeight = this.canvas.height - topMargin - bottomMargin

    const viewWidth = this.getViewWidth()
    const viewHeight = this.getViewHeight()
    const centerX = (this.originalBounds.minHorizontalDistance + this.originalBounds.maxHorizontalDistance) / 2
    const centerY = (this.originalBounds.minAltitude + this.originalBounds.maxAltitude) / 2

    const x = centerX + this.panX + ((screenX - leftMargin) / plotWidth - 0.5) * viewWidth
    const y = centerY + this.panY + (0.5 - (screenY - topMargin) / plotHeight) * viewHeight

    return { x, y }
  }

  getViewWidth(): number {
    return this.originalBounds ? (this.originalBounds.maxHorizontalDistance - this.originalBounds.minHorizontalDistance) / this.zoom : 0
  }

  getViewHeight(): number {
    return this.originalBounds ? (this.originalBounds.maxAltitude - this.originalBounds.minAltitude) / this.zoom : 0
  }

  worldToScreen(point: PlotPoint): { x: number, y: number } {
    if (!this.canvas || !this.originalBounds) return { x: 0, y: 0 }

    const leftMargin = 200
    const topMargin = 50
    const rightMargin = 20
    const bottomMargin = 20
    const plotWidth = this.canvas.width - leftMargin - rightMargin
    const plotHeight = this.canvas.height - topMargin - bottomMargin

    const viewWidth = this.getViewWidth()
    const viewHeight = this.getViewHeight()
    const centerX = (this.originalBounds.minHorizontalDistance + this.originalBounds.maxHorizontalDistance) / 2
    const centerY = (this.originalBounds.minAltitude + this.originalBounds.maxAltitude) / 2

    const minX = centerX + this.panX - viewWidth / 2
    const maxX = centerX + this.panX + viewWidth / 2
    const minY = centerY + this.panY - viewHeight / 2
    const maxY = centerY + this.panY + viewHeight / 2

    // Calculate horizontal distance for this point
    const pointEastX = point.x !== undefined ? point.x : 0
    const pointNorthZ = point.z !== undefined ? point.z : 0
    const pointHorizontalDistance = Math.sqrt(pointEastX * pointEastX + pointNorthZ * pointNorthZ)
    const pointAltitude = point.alt

    const x = leftMargin + ((pointHorizontalDistance - minX) / (maxX - minX)) * plotWidth
    const y = topMargin + plotHeight - ((pointAltitude - minY) / (maxY - minY)) * plotHeight

    return { x, y }
  }
}

// Horizontal vs Vertical Speed view for speed component analysis
class PolarView extends PlotView {
  private maxHorizontalSpeed = 100 // m/s
  private maxVerticalSpeed = 100   // m/s (absolute value)
  private minVerticalSpeed = -100  // m/s (negative for descent)

  calculateBounds(points: PlotPoint[]) {
    // Calculate max horizontal and vertical speeds for scaling
    let maxHSpeed = 0
    let maxVSpeed = 0
    let minVSpeed = 0
    
    for (const point of points) {
      // Check GPS speeds (MLocation)
      const mlocation = point as any
      if (mlocation.velN !== undefined && mlocation.velE !== undefined && mlocation.velD !== undefined) {
        const horizontalSpeed = Math.sqrt(mlocation.velN ** 2 + mlocation.velE ** 2)
        const verticalSpeed = -mlocation.velD // Convert NED down to up (negative down becomes positive up)
        
        maxHSpeed = Math.max(maxHSpeed, horizontalSpeed)
        maxVSpeed = Math.max(maxVSpeed, verticalSpeed)
        minVSpeed = Math.min(minVSpeed, verticalSpeed)
      }
      
      // Check filter speeds (ENU coordinates)
      if (point.velX !== undefined && point.velY !== undefined && point.velZ !== undefined) {
        const horizontalSpeed = Math.sqrt(point.velX ** 2 + point.velZ ** 2) // East² + North²
        const verticalSpeed = point.velY // ENU Y is up
        
        maxHSpeed = Math.max(maxHSpeed, horizontalSpeed)
        maxVSpeed = Math.max(maxVSpeed, verticalSpeed)
        minVSpeed = Math.min(minVSpeed, verticalSpeed)
      }
      
      // Check sustained speeds (vxs, vys)
      if (point.vxs !== undefined && point.vys !== undefined) {
        const sustainedHorizontalSpeed = Math.abs(point.vxs) // vxs can be negative
        // vys is stored in NED (positive down), convert to ENU (positive up) for bounds
        const sustainedVerticalSpeed = -point.vys // Convert NED down to ENU up
        
        maxHSpeed = Math.max(maxHSpeed, sustainedHorizontalSpeed)
        maxVSpeed = Math.max(maxVSpeed, sustainedVerticalSpeed)
        minVSpeed = Math.min(minVSpeed, sustainedVerticalSpeed)
      }
      
      // Check smoothed GPS speeds
      if (point.smoothVelN !== undefined && point.smoothVelE !== undefined && point.smoothVelD !== undefined) {
        const smoothedHorizontalSpeed = Math.sqrt(point.smoothVelN * point.smoothVelN + point.smoothVelE * point.smoothVelE)
        const smoothedVerticalSpeed = -point.smoothVelD // Convert NED down to ENU up
        
        maxHSpeed = Math.max(maxHSpeed, smoothedHorizontalSpeed)
        maxVSpeed = Math.max(maxVSpeed, smoothedVerticalSpeed)
        minVSpeed = Math.min(minVSpeed, smoothedVerticalSpeed)
      }
    }
    
    // Add some margin and round to nice numbers
    this.maxHorizontalSpeed = Math.max(20, Math.ceil(maxHSpeed / 10) * 10)
    this.maxVerticalSpeed = Math.max(20, Math.ceil(maxVSpeed / 10) * 10)
    this.minVerticalSpeed = Math.min(-20, Math.floor(minVSpeed / 10) * 10)
    
    return { 
      maxHorizontalSpeed: this.maxHorizontalSpeed,
      maxVerticalSpeed: this.maxVerticalSpeed,
      minVerticalSpeed: this.minVerticalSpeed
    }
  }

  screenToWorld(screenX: number, screenY: number) {
    const margin = 60
    const plotWidth = this.canvas!.width - 2 * margin
    const plotHeight = this.canvas!.height - 2 * margin
    
    // Convert screen coordinates to world coordinates
    const horizontalSpeed = (screenX - margin) / plotWidth * this.maxHorizontalSpeed
    const verticalSpeed = this.maxVerticalSpeed - (screenY - margin) / plotHeight * (this.maxVerticalSpeed - this.minVerticalSpeed)
    
    return { x: horizontalSpeed, y: verticalSpeed }
  }

  getViewWidth(): number {
    return this.maxHorizontalSpeed
  }

  getViewHeight(): number {
    return this.maxVerticalSpeed - this.minVerticalSpeed
  }

  worldToScreen(point: PlotPoint): { x: number, y: number } {
    const margin = 60
    const plotWidth = this.canvas!.width - 2 * margin
    const plotHeight = this.canvas!.height - 2 * margin
    
    // Calculate horizontal and vertical speeds for this point
    let horizontalSpeed = 0
    let verticalSpeed = 0
    
    // Check if it's GPS data (MLocation)
    const mlocation = point as any
    if (mlocation.velN !== undefined && mlocation.velE !== undefined && mlocation.velD !== undefined) {
      horizontalSpeed = Math.sqrt(mlocation.velN ** 2 + mlocation.velE ** 2)
      verticalSpeed = -mlocation.velD // Convert NED down to up
    } 
    // Check if it's filter data (ENU)
    else if (point.velX !== undefined && point.velY !== undefined && point.velZ !== undefined) {
      horizontalSpeed = Math.sqrt(point.velX ** 2 + point.velZ ** 2) // East² + North²
      verticalSpeed = point.velY // ENU Y is up
    }
    
    // Convert to screen coordinates
    const x = margin + (horizontalSpeed / this.maxHorizontalSpeed) * plotWidth
    const y = margin + (this.maxVerticalSpeed - verticalSpeed) / (this.maxVerticalSpeed - this.minVerticalSpeed) * plotHeight
    
    return { x, y }
  }

  // Helper method to get sustained speed screen coordinates
  getSustainedSpeedScreenCoords(point: PlotPoint): { x: number, y: number } | null {
    if (point.vxs === undefined || point.vys === undefined) return null
    
    const margin = 60
    const plotWidth = this.canvas!.width - 2 * margin
    const plotHeight = this.canvas!.height - 2 * margin
    
    const horizontalSpeed = Math.abs(point.vxs) // vxs can be negative
    // vys is stored in NED (positive down), convert to ENU (positive up) for display
    const verticalSpeed = -point.vys // Convert NED down to ENU up
    
    // Convert to screen coordinates
    const x = margin + (horizontalSpeed / this.maxHorizontalSpeed) * plotWidth
    const y = margin + (this.maxVerticalSpeed - verticalSpeed) / (this.maxVerticalSpeed - this.minVerticalSpeed) * plotHeight
    
    return { x, y }
  }

  // Helper method to get smoothed GPS speed screen coordinates
  getSmoothedSpeedScreenCoords(point: PlotPoint): { x: number, y: number } | null {
    if (point.smoothVelN === undefined || point.smoothVelE === undefined || point.smoothVelD === undefined) return null
    
    const margin = 60
    const plotWidth = this.canvas!.width - 2 * margin
    const plotHeight = this.canvas!.height - 2 * margin
    
    const horizontalSpeed = Math.sqrt(point.smoothVelN * point.smoothVelN + point.smoothVelE * point.smoothVelE)
    const verticalSpeed = -point.smoothVelD // Convert NED down to ENU up
    
    // Convert to screen coordinates
    const x = margin + (horizontalSpeed / this.maxHorizontalSpeed) * plotWidth
    const y = margin + (this.maxVerticalSpeed - verticalSpeed) / (this.maxVerticalSpeed - this.minVerticalSpeed) * plotHeight
    
    return { x, y }
  }

  public renderPlot(): void {
    if (!this.originalBounds || !this.canvas || !this.ctx) return

    const margin = 60
    const plotWidth = this.canvas.width - 2 * margin
    const plotHeight = this.canvas.height - 2 * margin

    // Clear canvas
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

    // Draw axes and grid
    this.drawAxesAndGrid(margin, plotWidth, plotHeight)

    // Draw speed data
    this.drawSpeedData(margin, plotWidth, plotHeight)
  }

  private drawAxesAndGrid(margin: number, plotWidth: number, plotHeight: number): void {
    if (!this.ctx) return

    this.ctx.strokeStyle = '#666'
    this.ctx.lineWidth = 1
    this.ctx.fillStyle = '#ccc'
    this.ctx.font = '12px system-ui'

    // Draw plot background
    this.ctx.fillStyle = '#222'
    this.ctx.fillRect(margin, margin, plotWidth, plotHeight)
    
    // Draw border
    this.ctx.strokeStyle = '#666'
    this.ctx.strokeRect(margin, margin, plotWidth, plotHeight)

    // Draw horizontal grid lines (vertical speed)
    const verticalRange = this.maxVerticalSpeed - this.minVerticalSpeed
    const verticalStep = 20 // 20 m/s steps
    for (let v = Math.ceil(this.minVerticalSpeed / verticalStep) * verticalStep; v <= this.maxVerticalSpeed; v += verticalStep) {
      const y = margin + (this.maxVerticalSpeed - v) / verticalRange * plotHeight
      
      this.ctx.strokeStyle = v === 0 ? '#888' : '#444'
      this.ctx.lineWidth = v === 0 ? 2 : 1
      this.ctx.beginPath()
      this.ctx.moveTo(margin, y)
      this.ctx.lineTo(margin + plotWidth, y)
      this.ctx.stroke()
      
      // Y-axis labels (vertical speed)
      this.ctx.fillStyle = '#ccc'
      this.ctx.textAlign = 'right'
      this.ctx.fillText(`${v.toFixed(0)}`, margin - 5, y + 4)
    }

    // Draw vertical grid lines (horizontal speed)
    const horizontalStep = 20 // 20 m/s steps
    for (let h = 0; h <= this.maxHorizontalSpeed; h += horizontalStep) {
      const x = margin + (h / this.maxHorizontalSpeed) * plotWidth
      
      this.ctx.strokeStyle = h === 0 ? '#888' : '#444'
      this.ctx.lineWidth = h === 0 ? 2 : 1
      this.ctx.beginPath()
      this.ctx.moveTo(x, margin)
      this.ctx.lineTo(x, margin + plotHeight)
      this.ctx.stroke()
      
      // X-axis labels (horizontal speed)
      this.ctx.fillStyle = '#ccc'
      this.ctx.textAlign = 'center'
      this.ctx.fillText(`${h.toFixed(0)}`, x, margin + plotHeight + 15)
    }

    // Draw axis labels
    this.ctx.fillStyle = '#fff'
    this.ctx.font = '14px system-ui'
    this.ctx.textAlign = 'center'
    
    // X-axis label
    if (this.canvas) {
      this.ctx.fillText('Horizontal Speed (m/s)', margin + plotWidth / 2, this.canvas.height - 10)
    }
    
    // Y-axis label (rotated)
    this.ctx.save()
    this.ctx.translate(15, margin + plotHeight / 2)
    this.ctx.rotate(-Math.PI / 2)
    this.ctx.fillText('Vertical Speed (m/s)', 0, 0)
    this.ctx.restore()
  }

  private drawSpeedData(margin: number, plotWidth: number, plotHeight: number): void {
    if (!this.ctx || !this.allSeries) return

    // Draw all series (GPS and filter velocity data)
    for (const plotSeries of this.allSeries) {
      this.ctx.fillStyle = plotSeries.style.color

      for (const point of plotSeries.data) {
        const screenPos = this.worldToScreen(point)
        
        // Only draw points within the plot area
        if (screenPos.x >= margin && screenPos.x <= margin + plotWidth && 
            screenPos.y >= margin && screenPos.y <= margin + plotHeight) {
          
          this.ctx.beginPath()
          
          // Highlight hovered point
          if (this.hoveredPoint === point) {
            this.ctx.strokeStyle = '#ffffff'
            this.ctx.lineWidth = 2
            this.ctx.arc(screenPos.x, screenPos.y, plotSeries.style.radius + 2, 0, 2 * Math.PI)
            this.ctx.stroke()
            this.ctx.beginPath()
            this.ctx.fillStyle = plotSeries.style.color
          }
          
          this.ctx.arc(screenPos.x, screenPos.y, plotSeries.style.radius, 0, 2 * Math.PI)
          this.ctx.fill()
        }
      }
    }

    // Draw sustained speeds as small green points
    this.ctx.fillStyle = '#00ff00'
    let sustainedSpeedCount = 0
    
    for (const plotSeries of this.allSeries) {
      for (const point of plotSeries.data) {
        const sustainedPos = this.getSustainedSpeedScreenCoords(point)
        
        if (sustainedPos && 
            sustainedPos.x >= margin && sustainedPos.x <= margin + plotWidth && 
            sustainedPos.y >= margin && sustainedPos.y <= margin + plotHeight) {
          
          sustainedSpeedCount++
          this.ctx.beginPath()
          
          // Highlight hovered point for sustained speeds too
          if (this.hoveredPoint === point) {
            this.ctx.strokeStyle = '#ffffff'
            this.ctx.lineWidth = 2
            this.ctx.arc(sustainedPos.x, sustainedPos.y, 3, 0, 2 * Math.PI)
            this.ctx.stroke()
            this.ctx.beginPath()
            this.ctx.fillStyle = '#00ff00'
          }
          
          this.ctx.arc(sustainedPos.x, sustainedPos.y, 1.5, 0, 2 * Math.PI)
          this.ctx.fill()
        }
      }
    }

    // Draw smoothed GPS speeds as small blue points
    this.ctx.fillStyle = '#0099ff'
    let smoothedSpeedCount = 0
    
    console.log('Drawing smoothed speeds, total series:', this.allSeries.length)
    
    for (const plotSeries of this.allSeries) {
      console.log(`Checking series: ${plotSeries.name}, points: ${plotSeries.data.length}`)
      for (const point of plotSeries.data) {
        const smoothedPos = this.getSmoothedSpeedScreenCoords(point)
        
        if (smoothedPos && 
            smoothedPos.x >= margin && smoothedPos.x <= margin + plotWidth && 
            smoothedPos.y >= margin && smoothedPos.y <= margin + plotHeight) {
          
          smoothedSpeedCount++
          if (smoothedSpeedCount <= 5) { // Log first few
            console.log(`Drawing smoothed speed point ${smoothedSpeedCount}:`, {
              originalSpeeds: { velN: point.smoothVelN, velE: point.smoothVelE, velD: point.smoothVelD },
              screenPos: smoothedPos
            })
          }
          
          this.ctx.beginPath()
          
          // Highlight hovered point for smoothed speeds too
          if (this.hoveredPoint === point) {
            this.ctx.strokeStyle = '#ffffff'
            this.ctx.lineWidth = 2
            this.ctx.arc(smoothedPos.x, smoothedPos.y, 3, 0, 2 * Math.PI)
            this.ctx.stroke()
            this.ctx.beginPath()
            this.ctx.fillStyle = '#0099ff'
          }
          
          this.ctx.arc(smoothedPos.x, smoothedPos.y, 1.5, 0, 2 * Math.PI)
          this.ctx.fill()
        }
      }
    }
    
    console.log('Total smoothed speed points drawn:', smoothedSpeedCount)

    // Draw legend and info
    this.ctx.fillStyle = '#fff'
    this.ctx.font = '12px system-ui'
    this.ctx.textAlign = 'left'
    
    let yOffset = 20
    this.ctx.fillText('Horizontal vs Vertical Speed', 10, yOffset)
    yOffset += 15
    this.ctx.fillText(`H-Speed: 0-${this.maxHorizontalSpeed} m/s`, 10, yOffset)
    yOffset += 15
    this.ctx.fillText(`V-Speed: ${this.minVerticalSpeed}-${this.maxVerticalSpeed} m/s`, 10, yOffset)
    yOffset += 20
    
    // Series info
    for (const plotSeries of this.allSeries) {
      this.ctx.fillStyle = plotSeries.style.color
      this.ctx.fillRect(10, yOffset - 8, 8, 8)
      this.ctx.fillStyle = '#fff'
      this.ctx.fillText(`${plotSeries.name}: ${plotSeries.data.length}`, 25, yOffset)
      yOffset += 15
    }
    
    // Sustained speeds info
    if (sustainedSpeedCount > 0) {
      this.ctx.fillStyle = '#00ff00'
      this.ctx.fillRect(10, yOffset - 8, 8, 8)
      this.ctx.fillStyle = '#fff'
      this.ctx.fillText(`Sustained Speeds: ${sustainedSpeedCount}`, 25, yOffset)
      yOffset += 15
    }
    
    // Smoothed GPS speeds info
    if (smoothedSpeedCount > 0) {
      this.ctx.fillStyle = '#0099ff'
      this.ctx.fillRect(10, yOffset - 8, 8, 8)
      this.ctx.fillStyle = '#fff'
      this.ctx.fillText(`Smoothed GPS Speeds: ${smoothedSpeedCount}`, 25, yOffset)
      yOffset += 15
    }
  }

  // Override mouse handling for Cartesian coordinates
  protected findNearestPoint(screenX: number, screenY: number): PlotPoint | null {
    if (!this.canvas || !this.originalBounds || this.allPoints.length === 0) return null

    let nearestPoint: PlotPoint | null = null
    let minDistance = Infinity
    const hoverRadius = 15

    for (const point of this.allPoints) {
      // Check regular velocity points
      const screenPos = this.worldToScreen(point)
      const distance = Math.sqrt((screenPos.x - screenX) ** 2 + (screenPos.y - screenY) ** 2)
      
      if (distance < hoverRadius && distance < minDistance) {
        minDistance = distance
        nearestPoint = point
      }
      
      // Also check sustained speed points
      const sustainedPos = this.getSustainedSpeedScreenCoords(point)
      if (sustainedPos) {
        const sustainedDistance = Math.sqrt((sustainedPos.x - screenX) ** 2 + (sustainedPos.y - screenY) ** 2)
        
        if (sustainedDistance < hoverRadius && sustainedDistance < minDistance) {
          minDistance = sustainedDistance
          nearestPoint = point
        }
      }
      
      // Also check smoothed speed points
      const smoothedPos = this.getSmoothedSpeedScreenCoords(point)
      if (smoothedPos) {
        const smoothedDistance = Math.sqrt((smoothedPos.x - screenX) ** 2 + (smoothedPos.y - screenY) ** 2)
        
        if (smoothedDistance < hoverRadius && smoothedDistance < minDistance) {
          minDistance = smoothedDistance
          nearestPoint = point
        }
      }
    }

    return nearestPoint
  }
}

// Speed Comparison Chart View - shows velocity components over time
class SpeedComparisonView extends PlotView {
  private selectedComponent: 'vn' | 've' | 'vd' = 'vn'
  private minTime: number = 0
  private maxTime: number = 0
  private minSpeed: number = 0
  private maxSpeed: number = 0

  calculateBounds(points: PlotPoint[]): { minTime: number, maxTime: number, minSpeed: number, maxSpeed: number } {
    if (points.length === 0) {
      return { minTime: 0, maxTime: 0, minSpeed: 0, maxSpeed: 0 }
    }

    const times = points.filter(p => p.time).map(p => p.time!)
    const velocities: number[] = []
    const smoothVelocities: number[] = []

    points.forEach(point => {
      const mlocation = point as any
      let vel: number | undefined
      let smoothVel: number | undefined

      switch (this.selectedComponent) {
        case 'vn':
          vel = mlocation.velN
          smoothVel = mlocation.smoothVelN
          break
        case 've':
          vel = mlocation.velE
          smoothVel = mlocation.smoothVelE
          break
        case 'vd':
          vel = mlocation.velD
          smoothVel = mlocation.smoothVelD
          break
      }

      if (vel !== undefined) velocities.push(vel)
      if (smoothVel !== undefined) smoothVelocities.push(smoothVel)
    })

    const allSpeeds = [...velocities, ...smoothVelocities]
    
    return {
      minTime: Math.min(...times),
      maxTime: Math.max(...times),
      minSpeed: allSpeeds.length > 0 ? Math.min(...allSpeeds) : 0,
      maxSpeed: allSpeeds.length > 0 ? Math.max(...allSpeeds) : 0
    }
  }

  screenToWorld(screenX: number, screenY: number): { time: number, speed: number } {
    if (!this.canvas) return { time: 0, speed: 0 }

    const margin = 60
    const plotWidth = this.canvas.width - 2 * margin
    const plotHeight = this.canvas.height - 2 * margin

    const timeRange = this.maxTime - this.minTime
    const speedRange = this.maxSpeed - this.minSpeed

    const time = this.minTime + ((screenX - margin) / plotWidth) * timeRange
    const speed = this.maxSpeed - ((screenY - margin) / plotHeight) * speedRange

    return { time, speed }
  }

  getViewWidth(): number {
    return this.maxTime - this.minTime
  }

  getViewHeight(): number {
    return this.maxSpeed - this.minSpeed
  }

  worldToScreen(point: PlotPoint): { x: number, y: number } {
    if (!this.canvas || !point.time) return { x: 0, y: 0 }

    const margin = 60
    const plotWidth = this.canvas.width - 2 * margin
    const plotHeight = this.canvas.height - 2 * margin

    const timeRange = this.maxTime - this.minTime
    const speedRange = this.maxSpeed - this.minSpeed

    const x = margin + ((point.time - this.minTime) / timeRange) * plotWidth
    
    // Get velocity based on selected component
    const mlocation = point as any
    let velocity: number = 0
    switch (this.selectedComponent) {
      case 'vn':
        velocity = mlocation.velN || 0
        break
      case 've':
        velocity = mlocation.velE || 0
        break
      case 'vd':
        velocity = mlocation.velD || 0
        break
    }
    
    const y = margin + plotHeight - ((velocity - this.minSpeed) / speedRange) * plotHeight

    return { x, y }
  }

  setSelectedComponent(component: 'vn' | 've' | 'vd') {
    this.selectedComponent = component
    this.renderPlot()
  }

  public renderPlot(): void {
    if (!this.canvas || !this.ctx || this.allSeries.length === 0) return

    const margin = 60
    const plotWidth = this.canvas.width - 2 * margin
    const plotHeight = this.canvas.height - 2 * margin

    // Calculate bounds
    const bounds = this.calculateBounds(this.allPoints)
    this.minTime = bounds.minTime
    this.maxTime = bounds.maxTime
    this.minSpeed = bounds.minSpeed
    this.maxSpeed = bounds.maxSpeed

    // Add some padding to speed range
    const speedPadding = (this.maxSpeed - this.minSpeed) * 0.1
    this.minSpeed -= speedPadding
    this.maxSpeed += speedPadding

    // Clear canvas
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

    // Draw axes and grid
    this.drawAxesAndGrid(margin, plotWidth, plotHeight)

    // Draw speed data
    this.drawSpeedData(margin, plotWidth, plotHeight)
  }

  private drawAxesAndGrid(margin: number, plotWidth: number, plotHeight: number): void {
    if (!this.ctx) return

    this.ctx.strokeStyle = '#666'
    this.ctx.lineWidth = 1
    this.ctx.fillStyle = '#ccc'
    this.ctx.font = '12px system-ui'

    // Draw plot background
    this.ctx.fillStyle = '#222'
    this.ctx.fillRect(margin, margin, plotWidth, plotHeight)
    
    // Draw border
    this.ctx.strokeStyle = '#666'
    this.ctx.strokeRect(margin, margin, plotWidth, plotHeight)

    // Draw horizontal grid lines (speed)
    const speedRange = this.maxSpeed - this.minSpeed
    const speedStep = Math.max(1, Math.round(speedRange / 10))
    for (let speed = Math.ceil(this.minSpeed / speedStep) * speedStep; speed <= this.maxSpeed; speed += speedStep) {
      const y = margin + plotHeight - ((speed - this.minSpeed) / speedRange) * plotHeight
      
      this.ctx.strokeStyle = speed === 0 ? '#888' : '#444'
      this.ctx.lineWidth = speed === 0 ? 2 : 1
      this.ctx.beginPath()
      this.ctx.moveTo(margin, y)
      this.ctx.lineTo(margin + plotWidth, y)
      this.ctx.stroke()
      
      // Y-axis labels (speed)
      this.ctx.fillStyle = '#ccc'
      this.ctx.textAlign = 'right'
      this.ctx.fillText(speed.toFixed(1), margin - 10, y + 4)
    }

    // Draw vertical grid lines (time)
    const timeRange = this.maxTime - this.minTime
    const timeStep = Math.max(1000, Math.round(timeRange / 10)) // At least 1 second
    for (let time = Math.ceil(this.minTime / timeStep) * timeStep; time <= this.maxTime; time += timeStep) {
      const x = margin + ((time - this.minTime) / timeRange) * plotWidth
      
      this.ctx.strokeStyle = '#444'
      this.ctx.lineWidth = 1
      this.ctx.beginPath()
      this.ctx.moveTo(x, margin)
      this.ctx.lineTo(x, margin + plotHeight)
      this.ctx.stroke()
      
      // X-axis labels (time)
      const date = new Date(time)
      const timeLabel = `${date.getMinutes()}:${date.getSeconds().toString().padStart(2, '0')}`
      this.ctx.fillStyle = '#ccc'
      this.ctx.textAlign = 'center'
      this.ctx.fillText(timeLabel, x, margin + plotHeight + 20)
    }

    // Draw axis labels
    this.ctx.fillStyle = '#fff'
    this.ctx.font = '14px system-ui'
    this.ctx.textAlign = 'center'
    
    // X-axis label
    this.ctx.fillText('Time', margin + plotWidth / 2, this.canvas!.height - 10)
    
    // Y-axis label (rotated)
    const componentName = this.selectedComponent.toUpperCase()
    this.ctx.save()
    this.ctx.translate(15, margin + plotHeight / 2)
    this.ctx.rotate(-Math.PI / 2)
    this.ctx.fillText(`${componentName} Speed (m/s)`, 0, 0)
    this.ctx.restore()
  }

  private drawSpeedData(margin: number, plotWidth: number, plotHeight: number): void {
    if (!this.ctx) return

    const timeRange = this.maxTime - this.minTime
    const speedRange = this.maxSpeed - this.minSpeed

    // Draw original velocities as red line
    this.ctx.strokeStyle = '#ff4444'
    this.ctx.lineWidth = 2
    this.ctx.beginPath()
    
    let firstPoint = true
    for (const point of this.allPoints) {
      if (!point.time) continue
      
      const mlocation = point as any
      let velocity: number | undefined
      switch (this.selectedComponent) {
        case 'vn':
          velocity = mlocation.velN
          break
        case 've':
          velocity = mlocation.velE
          break
        case 'vd':
          velocity = mlocation.velD
          break
      }
      
      if (velocity === undefined) continue
      
      const x = margin + ((point.time - this.minTime) / timeRange) * plotWidth
      const y = margin + plotHeight - ((velocity - this.minSpeed) / speedRange) * plotHeight
      
      if (firstPoint) {
        this.ctx.moveTo(x, y)
        firstPoint = false
      } else {
        this.ctx.lineTo(x, y)
      }
    }
    this.ctx.stroke()

    // Draw smoothed velocities as blue line
    this.ctx.strokeStyle = '#0099ff'
    this.ctx.lineWidth = 2
    this.ctx.beginPath()
    
    firstPoint = true
    for (const point of this.allPoints) {
      if (!point.time) continue
      
      const mlocation = point as any
      let smoothVelocity: number | undefined
      switch (this.selectedComponent) {
        case 'vn':
          smoothVelocity = mlocation.smoothVelN
          break
        case 've':
          smoothVelocity = mlocation.smoothVelE
          break
        case 'vd':
          smoothVelocity = mlocation.smoothVelD
          break
      }
      
      if (smoothVelocity === undefined) continue
      
      const x = margin + ((point.time - this.minTime) / timeRange) * plotWidth
      const y = margin + plotHeight - ((smoothVelocity - this.minSpeed) / speedRange) * plotHeight
      
      if (firstPoint) {
        this.ctx.moveTo(x, y)
        firstPoint = false
      } else {
        this.ctx.lineTo(x, y)
      }
    }
    this.ctx.stroke()

    // Draw legend
    this.ctx.fillStyle = '#fff'
    this.ctx.font = '12px system-ui'
    this.ctx.textAlign = 'left'
    
    let yOffset = 20
    this.ctx.fillText(`${this.selectedComponent.toUpperCase()} Speed Comparison`, 10, yOffset)
    yOffset += 20
    
    // Original velocity legend
    this.ctx.fillStyle = '#ff4444'
    this.ctx.fillRect(10, yOffset - 8, 15, 3)
    this.ctx.fillStyle = '#fff'
    this.ctx.fillText(`Original ${this.selectedComponent.toUpperCase()}`, 30, yOffset)
    yOffset += 20
    
    // Smoothed velocity legend
    this.ctx.fillStyle = '#0099ff'
    this.ctx.fillRect(10, yOffset - 8, 15, 3)
    this.ctx.fillStyle = '#fff'
    this.ctx.fillText(`Smoothed ${this.selectedComponent.toUpperCase()}`, 30, yOffset)
  }
}

// Global instances (updated)
const topDownView = new TopDownView('top-canvas')
const profileView = new ProfileView('profile-canvas')
const polarView = new PolarView('polar-canvas')
const speedComparisonView = new SpeedComparisonView('speed-comparison-canvas')

// Export speed comparison view for external control
export { speedComparisonView }

// Global state management
let currentZoom = 1
let currentPanX = 0
let currentPanY = 0
let globalHoveredPoint: PlotPoint | null = null

export function getZoomState() {
  return { zoom: currentZoom, panX: currentPanX, panY: currentPanY }
}

export function setZoomState(state: { zoom: number, panX: number, panY: number }) {
  currentZoom = state.zoom
  currentPanX = state.panX
  currentPanY = state.panY
}

// Global hover synchronization
function syncHover(hoveredPoint: PlotPoint | null, sourceView?: PlotView) {
  globalHoveredPoint = hoveredPoint
  
  // Update all views only if they're different from the source
  if (sourceView !== topDownView) {
    topDownView.hoveredPoint = hoveredPoint
    topDownView.renderPlot()
  }
  
  if (sourceView !== profileView) {
    profileView.hoveredPoint = hoveredPoint
    profileView.renderPlot()
  }
  
  if (sourceView !== polarView) {
    polarView.hoveredPoint = hoveredPoint
    polarView.renderPlot()
  }
  
  if (sourceView !== speedComparisonView) {
    speedComparisonView.hoveredPoint = hoveredPoint
    speedComparisonView.renderPlot()
  }
  
  updatePointInfo(hoveredPoint)
}

// Assign to global variable
globalSyncHover = syncHover

export function plotData(series: PlotSeries[], preserveZoom: boolean = false): void {
  // Set up quad view container
  const quadViewContainer = document.getElementById('quad-view-container')
  const app = document.getElementById('app')
  
  if (quadViewContainer && app) {
    app.classList.add('hidden')
    quadViewContainer.classList.add('show')
    
    // Resize canvases to fit their containers - three column layout
    const topCanvas = document.getElementById('top-canvas') as HTMLCanvasElement
    const profileCanvas = document.getElementById('profile-canvas') as HTMLCanvasElement
    const polarCanvas = document.getElementById('polar-canvas') as HTMLCanvasElement
    const accelerationCanvas = document.getElementById('acceleration-canvas') as HTMLCanvasElement
    
    if (topCanvas && profileCanvas && polarCanvas && accelerationCanvas) {
      // Each chart column gets about 40% of screen width (leaving 20% for controls)
      const containerWidth = window.innerWidth * 0.4
      const containerHeight = window.innerHeight / 2
      
      topCanvas.width = containerWidth
      topCanvas.height = containerHeight
      profileCanvas.width = containerWidth
      profileCanvas.height = containerHeight
      polarCanvas.width = containerWidth
      polarCanvas.height = containerHeight
      accelerationCanvas.width = containerWidth
      accelerationCanvas.height = containerHeight
    }
  }

  // Override hover handling for synchronized hovering
  const originalSetupControls = [topDownView.setupControls, profileView.setupControls]
  
  topDownView.setupControls = function() {
    originalSetupControls[0].call(this)
    if (this.canvas) {
      this.canvas.removeEventListener('mousemove', this.handleMouseMove)
      this.canvas.addEventListener('mousemove', (e: MouseEvent) => {
        if (!this.canvas) return
        const rect = this.canvas.getBoundingClientRect()
        const mouseX = e.clientX - rect.left
        const mouseY = e.clientY - rect.top

        if (this.isDragging) {
          // Handle dragging normally
          this.handleMouseMove(e)
        } else {
          // Handle hover with synchronization
          const nearestPoint = this.findNearestPoint(mouseX, mouseY)
          if (nearestPoint !== globalHoveredPoint) {
            syncHover(nearestPoint)
          }
          this.canvas.style.cursor = 'grab'
        }
      })
    }
  }

  profileView.setupControls = function() {
    originalSetupControls[1].call(this)
    if (this.canvas) {
      this.canvas.removeEventListener('mousemove', this.handleMouseMove)
      this.canvas.addEventListener('mousemove', (e: MouseEvent) => {
        if (!this.canvas) return
        const rect = this.canvas.getBoundingClientRect()
        const mouseX = e.clientX - rect.left
        const mouseY = e.clientY - rect.top

        if (this.isDragging) {
          // Handle dragging normally
          this.handleMouseMove(e)
        } else {
          // Handle hover with synchronization
          const nearestPoint = this.findNearestPoint(mouseX, mouseY)
          if (nearestPoint !== globalHoveredPoint) {
            syncHover(nearestPoint)
          }
          this.canvas.style.cursor = 'grab'
        }
      })
    }
  }

  // Plot on all views
  topDownView.plotData(series, preserveZoom)
  profileView.plotData(series, preserveZoom)
  polarView.plotData(series, preserveZoom)
  speedComparisonView.plotData(series, preserveZoom)
  
  // Initialize point info
  updatePointInfo(null)
}

// Global point info function (shared between views)
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

  let wingsuitInfo = ''
  if (point.kl !== undefined && point.kd !== undefined && point.roll !== undefined) {
    const rollDegrees = (point.roll * 180 / Math.PI).toFixed(1)
    
    const formatEngineering = (value: number): string => {
      const absValue = Math.abs(value)
      if (absValue >= 1e-3) {
        return `${(value * 1000).toFixed(2)}m`
      } else if (absValue >= 1e-6) {
        return `${(value * 1000000).toFixed(1)}μ`
      } else if (absValue >= 1e-9) {
        return `${(value * 1000000000).toFixed(1)}n`
      } else {
        return value.toExponential(2)
      }
    }
    
    const kl = point.kl
    const kd = point.kd
    const klkdSquared = kl * kl + kd * kd
    const vxs_ms = kl / Math.pow(klkdSquared, 0.75)
    const vys_ms = kd / Math.pow(klkdSquared, 0.75)
    const vxs_mph = vxs_ms * 2.23694
    const vys_mph = vys_ms * 2.23694
    
    wingsuitInfo = `
      <div><strong>Wingsuit Parameters:</strong></div>
      <div>KL: ${formatEngineering(point.kl)}</div>
      <div>KD: ${formatEngineering(point.kd)}</div>
      <div>Roll: ${rollDegrees}°</div>
      <div>VXS: ${vxs_mph.toFixed(1)} mph</div>
      <div>VYS: ${vys_mph.toFixed(1)} mph</div>
    `
  }

  // Calculate horizontal distance for profile view info
  const horizontalDistance = point.x !== undefined && point.z !== undefined ? 
    Math.sqrt(point.x * point.x + point.z * point.z) : 0

  pointDetails.innerHTML = `
    <div><strong>Position:</strong></div>
    <div>X (East): ${point.x !== undefined ? point.x.toFixed(2) + ' m' : 'N/A'}</div>
    <div>Z (North): ${point.z !== undefined ? point.z.toFixed(2) + ' m' : 'N/A'}</div>
    <div>Horizontal Distance: ${horizontalDistance.toFixed(2)} m</div>
    <div>Alt: ${altitude}</div>
    <div><strong>Time:</strong> ${time}</div>
    ${velocityInfo}
    ${wingsuitInfo}
  `
}
