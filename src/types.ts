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
}

export interface PlotPoint {
  lat: number
  lng: number
  alt: number
  time: number
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
