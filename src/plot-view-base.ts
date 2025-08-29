import { PlotSeries, PlotPoint } from './types.js'

// Global hover synchronization function type
export type HoverSyncFunction = (hoveredPoint: PlotPoint | null, sourceView?: PlotView) => void

// Base class for view-specific plotting functionality
export abstract class PlotView {
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

  // Abstract methods that each view must implement
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

    console.log(`Plotting data for ${this.canvasId}:`, {
      canvasFound: !!this.canvas,
      contextFound: !!this.ctx,
      seriesCount: series.length,
      totalPoints: series.reduce((sum, s) => sum + s.data.length, 0)
    })

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
    
    // Calculate new bounds
    const newBounds = this.calculateBounds(this.allPoints)
    
    if (preserveZoom && this.originalBounds) {
      // When preserving zoom, adjust zoom/pan to maintain the same view with new bounds
      const oldBounds = this.originalBounds
      
      // Calculate scaling factors to adjust zoom and pan (with safety checks)
      const xScale = (newBounds.maxX - newBounds.minX) / Math.max(oldBounds.maxX - oldBounds.minX, 1e-10)
      const yScale = (newBounds.maxY - newBounds.minY) / Math.max(oldBounds.maxY - oldBounds.minY, 1e-10)
      
      // Only adjust if scales are reasonable (not too extreme)
      if (isFinite(xScale) && isFinite(yScale) && xScale > 0.01 && xScale < 100 && yScale > 0.01 && yScale < 100) {
        // Calculate offset changes
        const xOffset = (newBounds.minX + newBounds.maxX) / 2 - (oldBounds.minX + oldBounds.maxX) / 2
        const yOffset = (newBounds.minY + newBounds.maxY) / 2 - (oldBounds.minY + oldBounds.maxY) / 2
        
        // Adjust zoom and pan to maintain the same visual area
        this.zoom = this.zoom / Math.max(xScale, yScale) // Use max to prevent distortion
        this.panX = this.panX * xScale - xOffset
        this.panY = this.panY * yScale - yOffset
        
        // Clamp zoom to reasonable range
        this.zoom = Math.max(0.1, Math.min(this.zoom, 100))
      }
    }
    
    // Update bounds
    this.originalBounds = newBounds

    console.log(`Bounds for ${this.canvasId}:`, this.originalBounds, `(preserveZoom: ${preserveZoom})`, {zoom: this.zoom, panX: this.panX, panY: this.panY})

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
    
    // Much more gradual zoom factors for better control
    const baseZoomFactor = e.deltaY < 0 ? 1.08 : 1/1.08
    // Even gentler zoom at higher zoom levels
    const adaptiveZoomFactor = this.zoom > 3 ? 
      (e.deltaY < 0 ? 1.04 : 1/1.04) : baseZoomFactor
    
    this.zoom *= adaptiveZoomFactor
    // Increased max zoom limit and more reasonable min zoom
    this.zoom = Math.max(0.2, Math.min(this.zoom, 100))
    
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
        this.renderPlot()
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

  public abstract renderPlot(): void
}
