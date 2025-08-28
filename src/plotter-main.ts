import { PlotSeries, PlotPoint } from './types.js'
import { TopDownView } from './top-down-view.js'
import { ProfileView } from './profile-view.js'
import { PolarView } from './polar-view.js'
import { SpeedComparisonView } from './speed-comparison-view.js'
import { HoverSyncFunction } from './plot-view-base.js'

// Create persistent view instances
const topDownView = new TopDownView('top-canvas')
const profileView = new ProfileView('profile-canvas')
const polarView = new PolarView('polar-canvas')
const speedComparisonView = new SpeedComparisonView('speed-comparison-canvas')

// Global hover synchronization
let globalHoverSync: HoverSyncFunction | null = null
let globalHoveredPoint: PlotPoint | null = null

export function setGlobalHoverSync(syncFunction: HoverSyncFunction): void {
  globalHoverSync = syncFunction
}

export function getGlobalHoverSync(): HoverSyncFunction | null {
  return globalHoverSync
}

// Global hover sync function
function syncHover(hoveredPoint: PlotPoint | null): void {
  globalHoveredPoint = hoveredPoint
  
  // Update all views with the same hovered point
  topDownView.hoveredPoint = hoveredPoint
  profileView.hoveredPoint = hoveredPoint
  polarView.hoveredPoint = hoveredPoint
  speedComparisonView.hoveredPoint = hoveredPoint
  
  // Re-render all views
  topDownView.renderPlot()
  profileView.renderPlot()
  polarView.renderPlot()
  speedComparisonView.renderPlot()
}

// Main plotting function - matches original interface
export function plotData(series: PlotSeries[], preserveZoom: boolean = false): void {
  // Set up quad view container
  const quadViewContainer = document.getElementById('quad-view-container')
  const app = document.getElementById('app')
  
  if (quadViewContainer && app) {
    app.classList.add('hidden')
    quadViewContainer.classList.add('show')
    
    // Only resize canvases if they haven't been properly sized yet (to avoid disrupting zoom)
    const topCanvas = document.getElementById('top-canvas') as HTMLCanvasElement
    const profileCanvas = document.getElementById('profile-canvas') as HTMLCanvasElement
    const polarCanvas = document.getElementById('polar-canvas') as HTMLCanvasElement
    const speedComparisonCanvas = document.getElementById('speed-comparison-canvas') as HTMLCanvasElement
    
    if (topCanvas && profileCanvas && polarCanvas && speedComparisonCanvas) {
      // Only resize if canvas is still default size or if this is the initial load
      const shouldResize = !preserveZoom || topCanvas.width <= 300
      
      if (shouldResize) {
        // Each chart column gets about 40% of screen width (leaving 20% for controls)
        const containerWidth = window.innerWidth * 0.4
        const containerHeight = window.innerHeight / 2
        
        topCanvas.width = containerWidth
        topCanvas.height = containerHeight
        profileCanvas.width = containerWidth
        profileCanvas.height = containerHeight
        polarCanvas.width = containerWidth
        polarCanvas.height = containerHeight
        speedComparisonCanvas.width = containerWidth
        speedComparisonCanvas.height = containerHeight
      }
    }
  }

  console.log('Plotting data with series:', series.map(s => ({name: s.name, dataLength: s.data.length})))

  // Set up synchronized hover handling
  const originalSetupControls = [
    topDownView.setupControls.bind(topDownView), 
    profileView.setupControls.bind(profileView),
    polarView.setupControls.bind(polarView),
    speedComparisonView.setupControls.bind(speedComparisonView)
  ]
  
  // Override each view's setupControls to add synchronized hovering
  topDownView.setupControls = function() {
    originalSetupControls[0]()
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
    originalSetupControls[1]()
    if (this.canvas) {
      this.canvas.removeEventListener('mousemove', this.handleMouseMove)
      this.canvas.addEventListener('mousemove', (e: MouseEvent) => {
        if (!this.canvas) return
        const rect = this.canvas.getBoundingClientRect()
        const mouseX = e.clientX - rect.left
        const mouseY = e.clientY - rect.top

        if (this.isDragging) {
          this.handleMouseMove(e)
        } else {
          const nearestPoint = this.findNearestPoint(mouseX, mouseY)
          if (nearestPoint !== globalHoveredPoint) {
            syncHover(nearestPoint)
          }
          this.canvas.style.cursor = 'grab'
        }
      })
    }
  }

  polarView.setupControls = function() {
    originalSetupControls[2]()
    if (this.canvas) {
      this.canvas.removeEventListener('mousemove', this.handleMouseMove)
      this.canvas.addEventListener('mousemove', (e: MouseEvent) => {
        if (!this.canvas) return
        const rect = this.canvas.getBoundingClientRect()
        const mouseX = e.clientX - rect.left
        const mouseY = e.clientY - rect.top

        if (this.isDragging) {
          this.handleMouseMove(e)
        } else {
          const nearestPoint = this.findNearestPoint(mouseX, mouseY)
          if (nearestPoint !== globalHoveredPoint) {
            syncHover(nearestPoint)
          }
          this.canvas.style.cursor = 'grab'
        }
      })
    }
  }

  speedComparisonView.setupControls = function() {
    originalSetupControls[3]()
    if (this.canvas) {
      this.canvas.removeEventListener('mousemove', this.handleMouseMove)
      this.canvas.addEventListener('mousemove', (e: MouseEvent) => {
        if (!this.canvas) return
        const rect = this.canvas.getBoundingClientRect()
        const mouseX = e.clientX - rect.left
        const mouseY = e.clientY - rect.top

        if (this.isDragging) {
          this.handleMouseMove(e)
        } else {
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
}

// Legacy functions for backward compatibility
export function plotAllViews(
  gpsSeries: PlotSeries,
  smoothPositionSeries: PlotSeries,
  smoothVelocitySeries: PlotSeries,
  smoothAccelerationSeries: PlotSeries,
  smoothSustainedSpeedsSeries: PlotSeries,
  preserveZoom: boolean = false
): void {
  const allSeries = [
    gpsSeries,
    smoothPositionSeries,
    smoothVelocitySeries,
    smoothAccelerationSeries,
    smoothSustainedSpeedsSeries
  ]

  plotData(allSeries, preserveZoom)
}
