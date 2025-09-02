import { PlotView } from './plot-view-base.js'
import { PlotSeries, PlotPoint } from './types.js'
import { getSmoothSustainedSpeedScreenCoords, drawAxes, drawLegend } from './plot-utils.js'

export class PolarView extends PlotView {
  private maxHorizontalSpeed = 100 // m/s
  private maxVerticalSpeed = 100   // m/s (absolute value)
  private minVerticalSpeed = -100  // m/s (negative for descent)
  private showWindAdjustedSustainedSpeeds = true // Control visibility of wind-adjusted sustained speeds

  constructor(canvasId: string) {
    super(canvasId)
    this.setupWindAdjustedToggle()
  }

  private setupWindAdjustedToggle(): void {
    const checkbox = document.getElementById('show-wind-adjusted-checkbox') as HTMLInputElement
    if (checkbox) {
      // Set initial state
      this.showWindAdjustedSustainedSpeeds = checkbox.checked
      
      // Add event listener for changes
      checkbox.addEventListener('change', (e) => {
        this.showWindAdjustedSustainedSpeeds = (e.target as HTMLInputElement).checked
        this.renderPlot() // Re-render the plot when setting changes
      })
    }
  }

  calculateBounds(points: PlotPoint[]) {
    // Calculate max horizontal and vertical speeds for scaling - GPS speeds only
    let maxHSpeed = 0
    let maxVSpeed = 0
    let minVSpeed = 0
    
    for (const point of points) {
      // Only check GPS speeds (MLocation) - ignore filter, sustained, and smoothed speeds
      const mlocation = point as any
      if (mlocation.velN !== undefined && mlocation.velE !== undefined && mlocation.velD !== undefined) {
        const horizontalSpeed = Math.sqrt(mlocation.velN ** 2 + mlocation.velE ** 2)
        const verticalSpeed = -mlocation.velD // Convert NED down to up (negative down becomes positive up)
        
        maxHSpeed = Math.max(maxHSpeed, horizontalSpeed)
        maxVSpeed = Math.max(maxVSpeed, verticalSpeed)
        minVSpeed = Math.min(minVSpeed, verticalSpeed)
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
    if (!this.canvas) return { x: 0, y: 0 }
    
    const margin = 60
    const plotWidth = this.canvas.width - 2 * margin
    const plotHeight = this.canvas.height - 2 * margin
    
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
    if (!this.canvas) return { x: 0, y: 0 }
    
    const margin = 60
    const plotWidth = this.canvas.width - 2 * margin
    const plotHeight = this.canvas.height - 2 * margin
    
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
    if ((point as any).vxs === undefined || (point as any).vys === undefined) return null
    
    const margin = 60
    const plotWidth = this.canvas!.width - 2 * margin
    const plotHeight = this.canvas!.height - 2 * margin
    
    // Use vxs directly as horizontal speed and -vys as vertical speed
    const horizontalSpeed = (point as any).vxs
    const verticalSpeed = -(point as any).vys  // Negate vys for plotting
    
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

  // Helper method to get smooth sustained speed screen coordinates
  getSmoothSustainedSpeedScreenCoords(point: PlotPoint): { x: number, y: number } | null {
    if ((point as any).smoothVxs === undefined || (point as any).smoothVys === undefined) return null
    
    const margin = 60
    const plotWidth = this.canvas!.width - 2 * margin
    const plotHeight = this.canvas!.height - 2 * margin
    
    // Use vxs directly as horizontal speed and -vys as vertical speed
    const horizontalSpeed = (point as any).smoothVxs
    const verticalSpeed = -(point as any).smoothVys  // Negate vys for plotting
    
    // Convert to screen coordinates
    const x = margin + (horizontalSpeed / this.maxHorizontalSpeed) * plotWidth
    const y = margin + (this.maxVerticalSpeed - verticalSpeed) / (this.maxVerticalSpeed - this.minVerticalSpeed) * plotHeight
    
    return { x, y }
  }

  // Helper method to get wind-adjusted sustained speed screen coordinates
  getWindAdjustedSustainedSpeedScreenCoords(point: PlotPoint): { x: number, y: number } | null {
    const filterPoint = point as any
    
    // First, try to use pre-calculated wind-adjusted sustained speeds from Kalman filter
    if (filterPoint.windsustainedSpeeds && 
        filterPoint.windsustainedSpeeds.vxs !== undefined && 
        filterPoint.windsustainedSpeeds.vys !== undefined) {
      
      const margin = 60
      const plotWidth = this.canvas!.width - 2 * margin
      const plotHeight = this.canvas!.height - 2 * margin
      
      // Use pre-calculated wind-adjusted sustained speeds for plotting
      const horizontalSpeed = filterPoint.windsustainedSpeeds.vxs
      const verticalSpeed = -filterPoint.windsustainedSpeeds.vys  // Negate for plotting (NED to screen coordinates)
      
      // Convert to screen coordinates
      const x = margin + (horizontalSpeed / this.maxHorizontalSpeed) * plotWidth
      const y = margin + (this.maxVerticalSpeed - verticalSpeed) / (this.maxVerticalSpeed - this.minVerticalSpeed) * plotHeight
      
      return { x, y }
    }
    
    // Fallback: calculate wind-adjusted sustained speeds if not pre-calculated
    // Check if we have wind-adjusted data - look for proper acceleration components
    if (!filterPoint.windEstimate || !filterPoint.smoothVelN || !filterPoint.smoothVelE || !filterPoint.smoothVelD || 
        !filterPoint.accelZ || !filterPoint.accelX || !filterPoint.accelY) return null
    
    // Calculate wind-adjusted sustained speeds using the wind estimation
    const windAdjustedSpeeds = this.calculateWindAdjustedSustainedSpeeds(
      filterPoint.smoothVelN,
      filterPoint.smoothVelE, 
      filterPoint.smoothVelD,
      filterPoint.accelZ,  // North acceleration
      filterPoint.accelX,  // East acceleration
      -filterPoint.accelY, // Down acceleration (negate ENU Y to get NED D)
      filterPoint.windEstimate
    )
    
    if (!windAdjustedSpeeds) return null
    
    const margin = 60
    const plotWidth = this.canvas!.width - 2 * margin
    const plotHeight = this.canvas!.height - 2 * margin
    
    // Use wind-adjusted sustained speeds for plotting
    const horizontalSpeed = windAdjustedSpeeds.vxs
    const verticalSpeed = -windAdjustedSpeeds.vys  // Negate for plotting
    
    // Convert to screen coordinates
    const x = margin + (horizontalSpeed / this.maxHorizontalSpeed) * plotWidth
    const y = margin + (this.maxVerticalSpeed - verticalSpeed) / (this.maxVerticalSpeed - this.minVerticalSpeed) * plotHeight
    
    return { x, y }
  }

  // Calculate wind-adjusted sustained speeds using wind estimate
  private calculateWindAdjustedSustainedSpeeds(
    velN: number, velE: number, velD: number,
    accelN: number, accelE: number, accelD: number,
    windEstimate: { x: number, y: number, z: number }
  ): { vxs: number, vys: number } | null {
    // Convert wind estimate from ENU to NED for consistency with WSE functions
    const windN = windEstimate.z   // ENU Z -> NED N
    const windE = windEstimate.x   // ENU X -> NED E  
    const windD = -windEstimate.y  // ENU Y -> NED D (flip sign)
    
    // Calculate airspeed velocity (ground velocity + wind)
    const avN = velN + windN
    const avE = velE + windE
    const avD = velD + windD
    
    const accelDminusG = accelD - 9.81 // gravity constant
    
    // Calculate acceleration due to drag (projection along velocity)
    const vel = Math.sqrt(avN * avN + avE * avE + avD * avD)
    if (vel < 1.0) return null // Skip at very low speeds
    
    const proj = (accelN * avN + accelE * avE + accelDminusG * avD) / vel
    
    const dragN = proj * avN / vel
    const dragE = proj * avE / vel
    const dragD = proj * avD / vel
    
    const accelDrag = Math.sqrt(dragN * dragN + dragE * dragE + dragD * dragD)
    
    // Calculate acceleration due to lift (total - drag)
    const liftN = accelN - dragN
    const liftE = accelE - dragE
    const liftD = accelDminusG - dragD
    
    const accelLift = Math.sqrt(liftN * liftN + liftE * liftE + liftD * liftD)
    
    // Calculate drag sign (should be negative when opposing velocity)
    const signum = (val: number) => val > 0 ? 1 : val < 0 ? -1 : 0
    const dragsign = -signum(dragN * avN + dragE * avE + dragD * avD)
    
    // Calculate wingsuit coefficients
    const kl = accelLift / 9.81 / vel / vel
    const kd = accelDrag / 9.81 / vel / vel * dragsign
    
    // Calculate sustained speeds using the same formula as WSE
    const divisor = Math.pow(kl * kl + kd * kd, 0.75)
    if (divisor === 0) return null
    
    const sustained_x = kl / divisor
    const sustained_y = kd / divisor
    
    return { vxs: sustained_x, vys: sustained_y }
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

    // Define consistent colors matching the legend
    const plotColors = ['#ff0000', '#00ff00'] // GPS (red) and Filter (green) speeds
    const sustainedSpeedColor = '#00ff00'     // Sustained speeds (green)
    const smoothedSpeedColor = '#0099ff'      // Smoothed GPS speeds (blue) 
    const smoothSustainedColor = '#800080'    // Smooth sustained speeds (purple)
    const windAdjustedSustainedColor = '#ff8800' // Wind-adjusted sustained speeds (orange)

    // Draw all series (GPS and filter velocity data)
    for (let i = 0; i < this.allSeries.length; i++) {
      const plotSeries = this.allSeries[i]
      
      // Use consistent colors instead of plotSeries.style.color
      this.ctx.fillStyle = plotColors[i % plotColors.length]

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
            this.ctx.fillStyle = plotColors[i % plotColors.length]
          }
          
          this.ctx.arc(screenPos.x, screenPos.y, plotSeries.style.radius, 0, 2 * Math.PI)
          this.ctx.fill()
        }
      }
    }

    // Draw sustained speeds as green points
    this.ctx.fillStyle = sustainedSpeedColor
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
            this.ctx.fillStyle = sustainedSpeedColor
          }
          
          this.ctx.arc(sustainedPos.x, sustainedPos.y, 1.5, 0, 2 * Math.PI)
          this.ctx.fill()
        }
      }
    }

    // Draw smoothed GPS speeds as blue points
    this.ctx.fillStyle = smoothedSpeedColor
    let smoothedSpeedCount = 0
    
    for (const plotSeries of this.allSeries) {
      for (const point of plotSeries.data) {
        const smoothedPos = this.getSmoothedSpeedScreenCoords(point)
        
        if (smoothedPos && 
            smoothedPos.x >= margin && smoothedPos.x <= margin + plotWidth && 
            smoothedPos.y >= margin && smoothedPos.y <= margin + plotHeight) {
          
          smoothedSpeedCount++
          this.ctx.beginPath()
          
          // Highlight hovered point for smoothed speeds too
          if (this.hoveredPoint === point) {
            this.ctx.strokeStyle = '#ffffff'
            this.ctx.lineWidth = 2
            this.ctx.arc(smoothedPos.x, smoothedPos.y, 3, 0, 2 * Math.PI)
            this.ctx.stroke()
            this.ctx.beginPath()
            this.ctx.fillStyle = smoothedSpeedColor
          }
          
          this.ctx.arc(smoothedPos.x, smoothedPos.y, 1.5, 0, 2 * Math.PI)
          this.ctx.fill()
        }
      }
    }

    // Draw smooth sustained speeds as purple points
    this.ctx.fillStyle = smoothSustainedColor
    let smoothSustainedSpeedCount = 0
    
    for (const plotSeries of this.allSeries) {
      for (const point of plotSeries.data) {
        const smoothSustainedPos = this.getSmoothSustainedSpeedScreenCoords(point)
        
        if (smoothSustainedPos && 
            smoothSustainedPos.x >= margin && smoothSustainedPos.x <= margin + plotWidth && 
            smoothSustainedPos.y >= margin && smoothSustainedPos.y <= margin + plotHeight) {
          
          smoothSustainedSpeedCount++
          this.ctx.beginPath()
          
          // Highlight hovered point for smooth sustained speeds too
          if (this.hoveredPoint === point) {
            this.ctx.strokeStyle = '#ffffff'
            this.ctx.lineWidth = 2
            this.ctx.arc(smoothSustainedPos.x, smoothSustainedPos.y, 4, 0, 2 * Math.PI)
            this.ctx.stroke()
            this.ctx.beginPath()
            this.ctx.fillStyle = smoothSustainedColor
          }
          
          this.ctx.arc(smoothSustainedPos.x, smoothSustainedPos.y, 2.5, 0, 2 * Math.PI)
          this.ctx.fill()
        }
      }
    }

    // Draw wind-adjusted sustained speeds as orange points (only if enabled)
    if (this.showWindAdjustedSustainedSpeeds) {
      this.ctx.fillStyle = windAdjustedSustainedColor
      let windAdjustedSustainedSpeedCount = 0
      
      // Debug: Check what wind data is available
      let totalPoints = 0
      let pointsWithWind = 0
      let pointsWithWindSustained = 0
      
      for (const plotSeries of this.allSeries) {
        for (const point of plotSeries.data) {
          totalPoints++
          const filterPoint = point as any
          if (filterPoint.windEstimate) pointsWithWind++
          if (filterPoint.windsustainedSpeeds) pointsWithWindSustained++
          
          const windAdjustedPos = this.getWindAdjustedSustainedSpeedScreenCoords(point)
          
          if (windAdjustedPos && 
              windAdjustedPos.x >= margin && windAdjustedPos.x <= margin + plotWidth && 
              windAdjustedPos.y >= margin && windAdjustedPos.y <= margin + plotHeight) {
            
            windAdjustedSustainedSpeedCount++
            this.ctx.beginPath()
            
            // Highlight hovered point for wind-adjusted sustained speeds too
            if (this.hoveredPoint === point) {
              this.ctx.strokeStyle = '#ffffff'
              this.ctx.lineWidth = 2
              this.ctx.arc(windAdjustedPos.x, windAdjustedPos.y, 3, 0, 2 * Math.PI)
              this.ctx.stroke()
              this.ctx.beginPath()
              this.ctx.fillStyle = windAdjustedSustainedColor
            }
            
            this.ctx.arc(windAdjustedPos.x, windAdjustedPos.y, 1, 0, 2 * Math.PI)
            this.ctx.fill()
          }
        }
      }

      console.log(`Polar view: Drew ${sustainedSpeedCount} sustained speeds, ${smoothedSpeedCount} smoothed speeds, ${smoothSustainedSpeedCount} smooth sustained speeds, ${windAdjustedSustainedSpeedCount} wind-adjusted sustained speeds`)
      console.log(`Polar view wind debug: ${totalPoints} total points, ${pointsWithWind} with windEstimate, ${pointsWithWindSustained} with windsustainedSpeeds`)
    } else {
      console.log(`Polar view: Drew ${sustainedSpeedCount} sustained speeds, ${smoothedSpeedCount} smoothed speeds, ${smoothSustainedSpeedCount} smooth sustained speeds, 0 wind-adjusted sustained speeds (disabled)`)
    }
    
    // Draw legend
    this.drawPolarLegend()
  }

  private showTooltip(x: number, y: number, point: PlotPoint): void {
    if (!this.ctx) return

    const timeStr = new Date(point.time * 1000).toLocaleTimeString()
    const glideRatio = point.smoothVxs && point.smoothVys ? (point.smoothVxs / Math.abs(point.smoothVys)).toFixed(2) : 'N/A'
    
    // Calculate wind-adjusted sustained speeds for tooltip
    const filterPoint = point as any
    let windAdjustedInfo = ''
    
    // First try to use pre-calculated wind-adjusted sustained speeds
    if (filterPoint.windsustainedSpeeds && 
        filterPoint.windsustainedSpeeds.vxs !== undefined && 
        filterPoint.windsustainedSpeeds.vys !== undefined) {
      
      const windAdjustedGlideRatio = (filterPoint.windsustainedSpeeds.vxs / Math.abs(filterPoint.windsustainedSpeeds.vys)).toFixed(2)
      windAdjustedInfo = `\nWind-Adj Horizontal: ${filterPoint.windsustainedSpeeds.vxs.toFixed(2)} m/s\nWind-Adj Vertical: ${filterPoint.windsustainedSpeeds.vys.toFixed(2)} m/s\nWind-Adj Glide Ratio: ${windAdjustedGlideRatio}`
      
      if (filterPoint.windEstimate) {
        windAdjustedInfo += `\nWind Estimate: [${filterPoint.windEstimate.x.toFixed(1)}, ${filterPoint.windEstimate.y.toFixed(1)}, ${filterPoint.windEstimate.z.toFixed(1)}] m/s`
      }
    }
    // Fallback: calculate if not pre-calculated
    else if (filterPoint.windEstimate && filterPoint.smoothVelN && filterPoint.accelZ) {
      const windAdjustedSpeeds = this.calculateWindAdjustedSustainedSpeeds(
        filterPoint.smoothVelN,
        filterPoint.smoothVelE, 
        filterPoint.smoothVelD,
        filterPoint.accelZ,  // North acceleration
        filterPoint.accelX,  // East acceleration
        -filterPoint.accelY, // Down acceleration (negate ENU Y to get NED D)
        filterPoint.windEstimate
      )
      
      if (windAdjustedSpeeds) {
        const windAdjustedGlideRatio = (windAdjustedSpeeds.vxs / Math.abs(windAdjustedSpeeds.vys)).toFixed(2)
        windAdjustedInfo = `\nWind-Adj Horizontal: ${windAdjustedSpeeds.vxs.toFixed(2)} m/s\nWind-Adj Vertical: ${windAdjustedSpeeds.vys.toFixed(2)} m/s\nWind-Adj Glide Ratio: ${windAdjustedGlideRatio}\nWind Estimate: [${filterPoint.windEstimate.x.toFixed(1)}, ${filterPoint.windEstimate.y.toFixed(1)}, ${filterPoint.windEstimate.z.toFixed(1)}] m/s`
      }
    }
    
    const text = `Time: ${timeStr}\nHorizontal: ${(point.smoothVxs ?? 0).toFixed(2)} m/s\nVertical: ${(point.smoothVys ?? 0).toFixed(2)} m/s\nGlide Ratio: ${glideRatio}\nRoll: ${((point.smoothRoll ?? 0) * 180 / Math.PI).toFixed(1)}°${windAdjustedInfo}`
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

  private drawPolarLegend(): void {
    if (!this.ctx || !this.canvas) return

    // Define all the speed types being plotted
    const speedTypes = [
      { name: 'GPS Speeds', color: '#ff0000' },
      { name: 'Filter Speeds', color: '#00ff00' },
      { name: 'Sustained Speeds', color: '#00ff00' },
      { name: 'Smoothed GPS Speeds', color: '#0099ff' },
      { name: 'Smooth Sustained Speeds', color: '#800080' }
    ]

    // Only add wind-adjusted sustained speeds to legend if enabled
    if (this.showWindAdjustedSustainedSpeeds) {
      speedTypes.push({ name: 'Wind-Adjusted Sustained Speeds', color: '#ff8800' })
    }

    const legendX = this.canvas.width - 250
    const legendY = 30
    const lineHeight = 25
    const legendHeight = speedTypes.length * lineHeight + 20

    // Draw legend background - black transparent like other views
    this.ctx.fillStyle = 'rgba(0, 0, 0, 0.8)'
    this.ctx.fillRect(legendX - 10, legendY - 10, 240, legendHeight)

    // Draw legend items
    this.ctx.font = '12px system-ui'
    this.ctx.textAlign = 'left'

    speedTypes.forEach((speedType, index) => {
      if (!this.ctx) return
      
      const y = legendY + index * lineHeight
      
      // Draw colored line and circle matching the plot
      this.ctx.strokeStyle = speedType.color
      this.ctx.fillStyle = speedType.color
      this.ctx.lineWidth = 2
      
      // Draw line segment
      this.ctx.beginPath()
      this.ctx.moveTo(legendX - 8, y)
      this.ctx.lineTo(legendX + 8, y)
      this.ctx.stroke()
      
      // Draw circle (point)
      this.ctx.beginPath()
      this.ctx.arc(legendX, y, 3, 0, 2 * Math.PI)
      this.ctx.fill()
      
      // Draw label
      this.ctx.fillStyle = '#fff' // White text like other views
      this.ctx.fillText(speedType.name, legendX + 20, y + 4)
    })
  }

  protected findNearestPoint(screenX: number, screenY: number): PlotPoint | null {
    if (!this.canvas || !this.originalBounds || this.allPoints.length === 0) return null

    let nearestPoint: PlotPoint | null = null
    let minDistance = Infinity
    const hoverRadius = 15

    // Only consider points with smooth sustained speeds
    const sustainedSpeedPoints = this.allPoints.filter(point => 
      point.smoothVxs !== undefined && point.smoothVys !== undefined
    )

    for (const point of sustainedSpeedPoints) {
      const screenPos = getSmoothSustainedSpeedScreenCoords(point.smoothVxs!, point.smoothVys!, this.originalBounds, this.zoom, this.panX, this.panY, this.canvas)
      const distance = Math.sqrt((screenPos.x - screenX) ** 2 + (screenPos.y - screenY) ** 2)
      
      if (distance < hoverRadius && distance < minDistance) {
        minDistance = distance
        nearestPoint = point
      }
    }

    return nearestPoint
  }
}
