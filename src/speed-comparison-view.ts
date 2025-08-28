import { PlotView } from './plot-view-base.js'
import { PlotSeries, PlotPoint } from './types.js'

export class SpeedComparisonView extends PlotView {
  private selectedComponent: 'vn' | 've' | 'vd' | 'an' | 'ae' | 'ad' = 'vn'
  private minTime: number = 0
  private maxTime: number = 0
  private minSpeed: number = 0
  private maxSpeed: number = 0
  private boundsInitialized: boolean = false

  constructor(canvasId: string) {
    super(canvasId)
    this.setupComponentSelector()
  }

  private setupComponentSelector(): void {
    const selector = document.getElementById('speed-component-selector') as HTMLSelectElement
    if (selector) {
      selector.addEventListener('change', (e) => {
        this.selectedComponent = (e.target as HTMLSelectElement).value as any
        this.boundsInitialized = false // Force recalculation of bounds
        this.renderPlot()
      })
    }
  }

  public plotData(series: PlotSeries[], preserveZoom: boolean = false): void {
    // Call parent plotData first
    super.plotData(series, preserveZoom)
    
    // Reset bounds initialization flag if not preserving zoom
    if (!preserveZoom) {
      this.boundsInitialized = false
    }
  }

  // Override to match the signature expected by the base class
  calculateBounds(points: PlotPoint[]): any {
    return this.calculateBoundsFromSeries(this.allSeries)
  }

  private calculateBoundsFromSeries(allSeries: PlotSeries[]): { minTime: number, maxTime: number, minSpeed: number, maxSpeed: number } {
    // For speed comparison, only use GPS data (first series) if available
    const dataToUse = allSeries.length > 0 ? allSeries[0].data : []
    // Also include filter data (second series) if available
    const filterData = allSeries.length > 1 ? allSeries[1].data : []
    
    if (dataToUse.length === 0 && filterData.length === 0) {
      return { minTime: 0, maxTime: 0, minSpeed: 0, maxSpeed: 0 }
    }

    const times = dataToUse.filter(p => p.time).map(p => p.time!)
    const filterTimes = filterData.filter(p => p.time).map(p => p.time!)
    const allTimes = [...times, ...filterTimes]
    
    const velocities: number[] = []
    const smoothVelocities: number[] = []
    const filterVelocities: number[] = []

    dataToUse.forEach(point => {
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
        case 'an':
        case 'ae':
        case 'ad':
          // Acceleration components are only available in filter data, not GPS data
          vel = undefined
          smoothVel = undefined
          break
      }

      if (vel !== undefined) velocities.push(vel)
      if (smoothVel !== undefined) smoothVelocities.push(smoothVel)
    })

    // Extract filter velocities
    filterData.forEach(point => {
      const mlocation = point as any
      let filterVel: number | undefined

      switch (this.selectedComponent) {
        case 'vn':
          filterVel = mlocation.velZ // North = ENU Z
          break
        case 've':
          filterVel = mlocation.velX // East = ENU X
          break
        case 'vd':
          filterVel = mlocation.velY ? -mlocation.velY : undefined // Down = -ENU Y
          break
        case 'an':
          filterVel = mlocation.accelZ // North acceleration = ENU Z (state)
          // Also include measured, WSE, and smoothed accelerations for bounds
          if (mlocation.aMeasuredZ !== undefined) filterVelocities.push(mlocation.aMeasuredZ)
          if (mlocation.aWSEZ !== undefined) filterVelocities.push(mlocation.aWSEZ)
          if (mlocation.smoothAccelN !== undefined) filterVelocities.push(mlocation.smoothAccelN)
          break
        case 'ae':
          filterVel = mlocation.accelX // East acceleration = ENU X (state)
          // Also include measured, WSE, and smoothed accelerations for bounds
          if (mlocation.aMeasuredX !== undefined) filterVelocities.push(mlocation.aMeasuredX)
          if (mlocation.aWSEX !== undefined) filterVelocities.push(mlocation.aWSEX)
          if (mlocation.smoothAccelE !== undefined) filterVelocities.push(mlocation.smoothAccelE)
          break
        case 'ad':
          filterVel = mlocation.accelY ? -mlocation.accelY : undefined // Down acceleration = -ENU Y (state)
          // Also include measured, WSE, and smoothed accelerations for bounds
          if (mlocation.aMeasuredY !== undefined) filterVelocities.push(-mlocation.aMeasuredY)
          if (mlocation.aWSEY !== undefined) filterVelocities.push(-mlocation.aWSEY)
          if (mlocation.smoothAccelD !== undefined) filterVelocities.push(mlocation.smoothAccelD) // smoothAccelD is already in down direction
          break
      }

      if (filterVel !== undefined) filterVelocities.push(filterVel)
    })

    const allSpeeds = [...velocities, ...smoothVelocities, ...filterVelocities]
    
    return {
      minTime: allTimes.length > 0 ? Math.min(...allTimes) : 0,
      maxTime: allTimes.length > 0 ? Math.max(...allTimes) : 0,
      minSpeed: allSpeeds.length > 0 ? Math.min(...allSpeeds) : 0,
      maxSpeed: allSpeeds.length > 0 ? Math.max(...allSpeeds) : 0
    }
  }

  public renderPlot(): void {
    if (!this.canvas || !this.ctx || this.allSeries.length === 0) return

    const margin = 60
    const plotWidth = this.canvas.width - 2 * margin
    const plotHeight = this.canvas.height - 2 * margin

    // Only recalculate bounds if not initialized yet (preserves zoom)
    if (!this.boundsInitialized) {
      const bounds = this.calculateBoundsFromSeries(this.allSeries)
      this.minTime = bounds.minTime
      this.maxTime = bounds.maxTime
      this.minSpeed = bounds.minSpeed
      this.maxSpeed = bounds.maxSpeed

      // Add some padding to speed range
      const speedPadding = (this.maxSpeed - this.minSpeed) * 0.1
      this.minSpeed -= speedPadding
      this.maxSpeed += speedPadding
      
      this.boundsInitialized = true
    }

    // Clear canvas
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)

    // Draw title
    this.ctx.fillStyle = '#fff'
    this.ctx.font = '16px system-ui'
    this.ctx.textAlign = 'center'
    this.ctx.fillText(`${this.selectedComponent.toUpperCase()} Speed Comparison`, this.canvas.width / 2, 20)

    // Draw axes and grid
    this.drawAxesAndGrid(margin, plotWidth, plotHeight)

    // Draw speed data
    this.drawSpeedData(margin, plotWidth, plotHeight)
  }

  screenToWorld(screenX: number, screenY: number): { x: number, y: number } {
    if (!this.canvas) return { x: 0, y: 0 }

    const margin = 60
    const plotWidth = this.canvas.width - 2 * margin
    const plotHeight = this.canvas.height - 2 * margin

    const viewTimeRange = this.getViewWidth()
    const viewSpeedRange = this.getViewHeight()
    
    // Calculate current view bounds with zoom and pan
    const centerTime = (this.maxTime + this.minTime) / 2
    const centerSpeed = (this.maxSpeed + this.minSpeed) / 2
    
    const minViewTime = centerTime + this.panX - viewTimeRange / 2
    const maxViewTime = centerTime + this.panX + viewTimeRange / 2
    const minViewSpeed = centerSpeed + this.panY - viewSpeedRange / 2
    const maxViewSpeed = centerSpeed + this.panY + viewSpeedRange / 2

    const time = minViewTime + ((screenX - margin) / plotWidth) * (maxViewTime - minViewTime)
    const speed = maxViewSpeed - ((screenY - margin) / plotHeight) * (maxViewSpeed - minViewSpeed)

    // Return as x,y for compatibility with base class zoom handling
    return { x: time, y: speed }
  }

  getViewWidth(): number {
    const baseTimeRange = this.maxTime - this.minTime
    return baseTimeRange / this.zoom
  }

  getViewHeight(): number {
    const baseSpeedRange = this.maxSpeed - this.minSpeed
    return baseSpeedRange / this.zoom
  }

  worldToScreen(point: PlotPoint): { x: number, y: number } {
    if (!this.canvas || !point.time) return { x: 0, y: 0 }

    const margin = 60
    const plotWidth = this.canvas.width - 2 * margin
    const plotHeight = this.canvas.height - 2 * margin

    const viewTimeRange = this.getViewWidth()
    const viewSpeedRange = this.getViewHeight()
    
    // Calculate current view bounds with zoom and pan
    const centerTime = (this.maxTime + this.minTime) / 2
    const centerSpeed = (this.maxSpeed + this.minSpeed) / 2
    
    const minViewTime = centerTime + this.panX - viewTimeRange / 2
    const maxViewTime = centerTime + this.panX + viewTimeRange / 2
    const minViewSpeed = centerSpeed + this.panY - viewSpeedRange / 2
    const maxViewSpeed = centerSpeed + this.panY + viewSpeedRange / 2

    const x = margin + ((point.time - minViewTime) / (maxViewTime - minViewTime)) * plotWidth
    
    // Get value based on selected component - use the same logic as getValueForType
    const mlocation = point as any
    let value: number = 0
    
    // Try to get the primary value for the selected component
    switch (this.selectedComponent) {
      case 'vn':
        value = mlocation.velN || mlocation.velZ || 0
        break
      case 've':
        value = mlocation.velE || mlocation.velX || 0
        break
      case 'vd':
        value = mlocation.velD || (mlocation.velY ? -mlocation.velY : 0) || 0
        break
      case 'an':
        value = mlocation.accelZ || mlocation.aMeasuredZ || 0
        break
      case 'ae':
        value = mlocation.accelX || mlocation.aMeasuredX || 0
        break
      case 'ad':
        value = (mlocation.accelY ? -mlocation.accelY : 0) || (mlocation.aMeasuredY ? -mlocation.aMeasuredY : 0) || 0
        break
    }
    
    const y = margin + plotHeight - ((value - minViewSpeed) / (maxViewSpeed - minViewSpeed)) * plotHeight

    return { x, y }
  }

  setSelectedComponent(component: 'vn' | 've' | 'vd' | 'an' | 'ae' | 'ad') {
    this.selectedComponent = component
    this.boundsInitialized = false
    this.renderPlot()
  }

  private drawAxesAndGrid(margin: number, plotWidth: number, plotHeight: number): void {
    if (!this.ctx) return

    // Calculate current view bounds with zoom and pan
    const viewTimeRange = this.getViewWidth()
    const viewSpeedRange = this.getViewHeight()
    const centerTime = (this.maxTime + this.minTime) / 2
    const centerSpeed = (this.maxSpeed + this.minSpeed) / 2
    
    const minViewTime = centerTime + this.panX - viewTimeRange / 2
    const maxViewTime = centerTime + this.panX + viewTimeRange / 2
    const minViewSpeed = centerSpeed + this.panY - viewSpeedRange / 2
    const maxViewSpeed = centerSpeed + this.panY + viewSpeedRange / 2

    // Draw plot background
    this.ctx.fillStyle = '#222'
    this.ctx.fillRect(margin, margin, plotWidth, plotHeight)
    
    // Draw border
    this.ctx.strokeStyle = '#666'
    this.ctx.strokeRect(margin, margin, plotWidth, plotHeight)

    // Draw horizontal grid lines (speed)
    const speedStep = Math.max(0.1, viewSpeedRange / 10)
    for (let speed = Math.ceil(minViewSpeed / speedStep) * speedStep; speed <= maxViewSpeed; speed += speedStep) {
      const y = margin + plotHeight - ((speed - minViewSpeed) / viewSpeedRange) * plotHeight
      
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
    const timeStep = Math.max(1000, viewTimeRange / 10) // At least 1 second
    for (let time = Math.ceil(minViewTime / timeStep) * timeStep; time <= maxViewTime; time += timeStep) {
      const x = margin + ((time - minViewTime) / viewTimeRange) * plotWidth
      
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
    const isAcceleration = this.selectedComponent.startsWith('a')
    const units = isAcceleration ? '(m/s²)' : '(m/s)'
    const label = isAcceleration ? `${componentName} Acceleration ${units}` : `${componentName} Speed ${units}`
    
    this.ctx.save()
    this.ctx.translate(15, margin + plotHeight / 2)
    this.ctx.rotate(-Math.PI / 2)
    this.ctx.fillText(label, 0, 0)
    this.ctx.restore()
  }

  private drawSpeedData(margin: number, plotWidth: number, plotHeight: number): void {
    if (!this.ctx || this.allSeries.length === 0) return

    const gpsData = this.allSeries[0].data
    const filterData = this.allSeries.length > 1 ? this.allSeries[1].data : []

    // Calculate current view bounds
    const viewTimeRange = this.getViewWidth()
    const viewSpeedRange = this.getViewHeight()
    const centerTime = (this.maxTime + this.minTime) / 2
    const centerSpeed = (this.maxSpeed + this.minSpeed) / 2
    
    const minViewTime = centerTime + this.panX - viewTimeRange / 2
    const maxViewTime = centerTime + this.panX + viewTimeRange / 2
    const minViewSpeed = centerSpeed + this.panY - viewSpeedRange / 2
    const maxViewSpeed = centerSpeed + this.panY + viewSpeedRange / 2

    // Draw GPS velocities (red)
    this.drawDataSeries(gpsData, '#ff4444', 'gps', margin, plotWidth, plotHeight, 
      minViewTime, maxViewTime, minViewSpeed, maxViewSpeed, viewTimeRange, viewSpeedRange)

    // Draw smoothed GPS velocities (blue)
    this.drawDataSeries(gpsData, '#0099ff', 'smooth', margin, plotWidth, plotHeight, 
      minViewTime, maxViewTime, minViewSpeed, maxViewSpeed, viewTimeRange, viewSpeedRange)

    // Draw filter data (green)
    if (filterData.length > 0) {
      this.drawDataSeries(filterData, '#00ff44', 'filter', margin, plotWidth, plotHeight, 
        minViewTime, maxViewTime, minViewSpeed, maxViewSpeed, viewTimeRange, viewSpeedRange)

      // For accelerations, draw additional series
      if (this.selectedComponent.startsWith('a')) {
        this.drawDataSeries(filterData, '#ff8800', 'measured', margin, plotWidth, plotHeight, 
          minViewTime, maxViewTime, minViewSpeed, maxViewSpeed, viewTimeRange, viewSpeedRange)
        this.drawDataSeries(filterData, '#8800ff', 'wse', margin, plotWidth, plotHeight, 
          minViewTime, maxViewTime, minViewSpeed, maxViewSpeed, viewTimeRange, viewSpeedRange)
        this.drawDataSeries(filterData, '#ffaa00', 'smoothAccel', margin, plotWidth, plotHeight, 
          minViewTime, maxViewTime, minViewSpeed, maxViewSpeed, viewTimeRange, viewSpeedRange)
      }
    }

    // Draw legend and highlight
    this.drawLegend()
    if (this.hoveredPoint?.time) {
      this.highlightHoveredPoint(margin, plotWidth, plotHeight, minViewTime, maxViewTime, viewTimeRange)
    }
  }

  private drawDataSeries(data: PlotPoint[], color: string, type: string, 
    margin: number, plotWidth: number, plotHeight: number,
    minViewTime: number, maxViewTime: number, minViewSpeed: number, maxViewSpeed: number,
    viewTimeRange: number, viewSpeedRange: number): void {
    
    if (!this.ctx) return

    this.ctx.strokeStyle = color
    this.ctx.lineWidth = 2
    this.ctx.beginPath()
    
    let firstPoint = true
    for (const point of data) {
      if (!point.time) continue
      
      const value = this.getValueForType(point, type)
      if (value === undefined) continue
      
      if (point.time < minViewTime || point.time > maxViewTime || 
          value < minViewSpeed || value > maxViewSpeed) continue
      
      const x = margin + ((point.time - minViewTime) / viewTimeRange) * plotWidth
      const y = margin + plotHeight - ((value - minViewSpeed) / viewSpeedRange) * plotHeight
      
      if (firstPoint) {
        this.ctx.moveTo(x, y)
        firstPoint = false
      } else {
        this.ctx.lineTo(x, y)
      }
    }
    this.ctx.stroke()
  }

  private getValueForType(point: PlotPoint, type: string): number | undefined {
    const mlocation = point as any
    
    switch (type) {
      case 'gps':
        switch (this.selectedComponent) {
          case 'vn': return mlocation.velN
          case 've': return mlocation.velE
          case 'vd': return mlocation.velD
          default: return undefined
        }
      case 'smooth':
        switch (this.selectedComponent) {
          case 'vn': return mlocation.smoothVelN
          case 've': return mlocation.smoothVelE
          case 'vd': return mlocation.smoothVelD
          default: return undefined
        }
      case 'filter':
        switch (this.selectedComponent) {
          case 'vn': return mlocation.velZ
          case 've': return mlocation.velX
          case 'vd': return mlocation.velY ? -mlocation.velY : undefined
          case 'an': return mlocation.accelZ
          case 'ae': return mlocation.accelX
          case 'ad': return mlocation.accelY ? -mlocation.accelY : undefined
          default: return undefined
        }
      case 'measured':
        switch (this.selectedComponent) {
          case 'an': return mlocation.aMeasuredZ
          case 'ae': return mlocation.aMeasuredX
          case 'ad': return mlocation.aMeasuredY ? -mlocation.aMeasuredY : undefined
          default: return undefined
        }
      case 'wse':
        switch (this.selectedComponent) {
          case 'an': return mlocation.aWSEZ
          case 'ae': return mlocation.aWSEX
          case 'ad': return mlocation.aWSEY ? -mlocation.aWSEY : undefined
          default: return undefined
        }
      case 'smoothAccel':
        switch (this.selectedComponent) {
          case 'an': return mlocation.smoothAccelN
          case 'ae': return mlocation.smoothAccelE
          case 'ad': return mlocation.smoothAccelD
          default: return undefined
        }
      default: return undefined
    }
  }

  private drawLegend(): void {
    if (!this.ctx || !this.canvas) return

    const legendX = this.canvas.width - 200
    const legendY = 30
    const lineHeight = 20

    const isAcceleration = this.selectedComponent.startsWith('a')
    const hasFilterData = this.allSeries.length > 1

    let legendItems = []
    
    if (!isAcceleration) {
      legendItems = [
        { color: '#ff4444', label: 'GPS Velocity' },
        { color: '#0099ff', label: 'Smoothed GPS' }
      ]
      if (hasFilterData) {
        legendItems.push({ color: '#00ff44', label: 'Filter Output' })
      }
    } else {
      legendItems = [{ color: '#00ff44', label: 'Filter State' }]
      if (hasFilterData) {
        legendItems.push(
          { color: '#ff8800', label: 'Measured' },
          { color: '#8800ff', label: 'WSE' },
          { color: '#ffaa00', label: 'Smoothed' }
        )
      }
    }

    const legendHeight = legendItems.length * lineHeight + 20

    // Draw legend background
    this.ctx.fillStyle = 'rgba(0, 0, 0, 0.8)'
    this.ctx.fillRect(legendX - 10, legendY - 10, 190, legendHeight)

    // Draw legend items
    this.ctx.font = '12px system-ui'
    this.ctx.textAlign = 'left'

    legendItems.forEach((item, index) => {
      const y = legendY + index * lineHeight
      
      // Draw colored line
      this.ctx!.strokeStyle = item.color
      this.ctx!.lineWidth = 3
      this.ctx!.beginPath()
      this.ctx!.moveTo(legendX, y)
      this.ctx!.lineTo(legendX + 15, y)
      this.ctx!.stroke()
      
      // Draw label
      this.ctx!.fillStyle = '#fff'
      this.ctx!.fillText(item.label, legendX + 20, y + 4)
    })
  }

  private highlightHoveredPoint(margin: number, plotWidth: number, plotHeight: number, 
    minViewTime: number, maxViewTime: number, viewTimeRange: number): void {
    
    if (!this.ctx || !this.hoveredPoint?.time) return

    const time = this.hoveredPoint.time
    if (time < minViewTime || time > maxViewTime) return

    const x = margin + ((time - minViewTime) / viewTimeRange) * plotWidth

    // Draw vertical line
    this.ctx.strokeStyle = '#ffffff'
    this.ctx.lineWidth = 1
    this.ctx.setLineDash([5, 5])
    this.ctx.beginPath()
    this.ctx.moveTo(x, margin)
    this.ctx.lineTo(x, margin + plotHeight)
    this.ctx.stroke()
    this.ctx.setLineDash([])

    // Show tooltip
    this.showTooltip(x, margin + 10)
  }

  private showTooltip(x: number, y: number): void {
    if (!this.ctx || !this.hoveredPoint) return

    const mlocation = this.hoveredPoint as any
    const timeStr = new Date(this.hoveredPoint.time! * 1000).toLocaleTimeString()
    
    let tooltipLines = [`Time: ${timeStr}`]
    
    const isAcceleration = this.selectedComponent.startsWith('a')
    const units = isAcceleration ? 'm/s²' : 'm/s'
    const componentName = this.selectedComponent.toUpperCase()
    
    // Add relevant values based on component type
    if (!isAcceleration) {
      const gpsVal = this.getValueForType(this.hoveredPoint, 'gps')
      const smoothVal = this.getValueForType(this.hoveredPoint, 'smooth')
      const filterVal = this.getValueForType(this.hoveredPoint, 'filter')
      
      if (gpsVal !== undefined) tooltipLines.push(`GPS ${componentName}: ${gpsVal.toFixed(2)} ${units}`)
      if (smoothVal !== undefined) tooltipLines.push(`Smooth ${componentName}: ${smoothVal.toFixed(2)} ${units}`)
      if (filterVal !== undefined) tooltipLines.push(`Filter ${componentName}: ${filterVal.toFixed(2)} ${units}`)
    } else {
      const filterVal = this.getValueForType(this.hoveredPoint, 'filter')
      const measuredVal = this.getValueForType(this.hoveredPoint, 'measured')
      const wseVal = this.getValueForType(this.hoveredPoint, 'wse')
      const smoothVal = this.getValueForType(this.hoveredPoint, 'smoothAccel')
      
      if (filterVal !== undefined) tooltipLines.push(`Filter ${componentName}: ${filterVal.toFixed(2)} ${units}`)
      if (measuredVal !== undefined) tooltipLines.push(`Measured ${componentName}: ${measuredVal.toFixed(2)} ${units}`)
      if (wseVal !== undefined) tooltipLines.push(`WSE ${componentName}: ${wseVal.toFixed(2)} ${units}`)
      if (smoothVal !== undefined) tooltipLines.push(`Smooth ${componentName}: ${smoothVal.toFixed(2)} ${units}`)
    }
    
    // Draw tooltip
    const padding = 8
    const lineHeight = 16
    const maxWidth = Math.max(...tooltipLines.map(line => this.ctx!.measureText(line).width))
    const tooltipWidth = maxWidth + padding * 2
    const tooltipHeight = tooltipLines.length * lineHeight + padding * 2

    let tooltipX = x + 10
    let tooltipY = y - tooltipHeight - 10

    if (tooltipX + tooltipWidth > this.canvas!.width) tooltipX = x - tooltipWidth - 10
    if (tooltipY < 0) tooltipY = y + 10

    this.ctx.fillStyle = 'rgba(0, 0, 0, 0.9)'
    this.ctx.fillRect(tooltipX, tooltipY, tooltipWidth, tooltipHeight)
    this.ctx.strokeStyle = '#666'
    this.ctx.lineWidth = 1
    this.ctx.strokeRect(tooltipX, tooltipY, tooltipWidth, tooltipHeight)

    this.ctx.fillStyle = '#fff'
    this.ctx.font = '12px system-ui'
    this.ctx.textAlign = 'left'

    tooltipLines.forEach((line, index) => {
      this.ctx!.fillText(line, tooltipX + padding, tooltipY + padding + (index + 1) * lineHeight)
    })
  }

  protected findNearestPoint(screenX: number, screenY: number): PlotPoint | null {
    if (!this.canvas || this.allSeries.length === 0) return null

    let nearestPoint: PlotPoint | null = null
    let minDistance = Infinity
    const hoverRadius = 15

    const viewTimeRange = this.getViewWidth()
    const centerTime = (this.maxTime + this.minTime) / 2
    const minViewTime = centerTime + this.panX - viewTimeRange / 2
    const maxViewTime = centerTime + this.panX + viewTimeRange / 2

    const margin = 60
    const plotWidth = this.canvas.width - 2 * margin

    for (const series of this.allSeries) {
      for (const point of series.data) {
        if (!point.time || point.time < minViewTime || point.time > maxViewTime) continue
        
        const x = margin + ((point.time - minViewTime) / viewTimeRange) * plotWidth
        const distance = Math.abs(x - screenX)
        
        if (distance < hoverRadius && distance < minDistance) {
          minDistance = distance
          nearestPoint = point
        }
      }
    }

    return nearestPoint
  }
}