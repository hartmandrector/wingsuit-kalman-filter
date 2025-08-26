export interface MLocation {
  time: number
  lat: number
  lng: number
  alt: number
  velN: number
  velE: number
  velD: number
  hAcc: number
  vAcc: number
  sAcc: number
  // ENU coordinates added during processing
  x?: number
  y?: number  
  z?: number
  // Smoothed GPS speeds (calculated once from CSV data)
  smoothVelN?: number
  smoothVelE?: number
  smoothVelD?: number
}

export interface PlotPoint {
  lat: number
  lng: number
  alt: number
  time: number
  // ENU coordinates for display (x=East, y=Up, z=North)
  x?: number
  y?: number
  z?: number
  // ENU velocity components (m/s)
  velX?: number  // East velocity
  velY?: number  // Up velocity  
  velZ?: number  // North velocity
  // ENU acceleration components (m/s²)
  accelX?: number  // East acceleration
  accelY?: number  // Up acceleration
  accelZ?: number  // North acceleration
  // Optional wingsuit parameters for hover display
  kl?: number
  kd?: number
  roll?: number
  // Sustained speeds (wingsuit aerodynamic velocities)
  vxs?: number  // Sustained horizontal speed (m/s)
  vys?: number  // Sustained vertical speed (m/s)
  // Smoothed GPS speeds (calculated once from CSV data)
  smoothVelN?: number  // Smoothed North velocity (m/s)
  smoothVelE?: number  // Smoothed East velocity (m/s)
  smoothVelD?: number  // Smoothed Down velocity (m/s)
}

export interface PointStyle {
  color: string
  radius: number
}

export interface PlotSeries {
  name: string
  data: PlotPoint[]
  style: PointStyle
}

export interface LatLonAlt {
  lat: number
  lng: number
  alt: number
}

export interface Vector3 {
  x: number
  y: number
  z: number
}

export interface Bounds {
  minLat: number
  maxLat: number
  minLng: number
  maxLng: number
}
