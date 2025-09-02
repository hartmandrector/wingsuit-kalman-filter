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
  // Smoothed GPS accelerations (calculated from smoothed velocities)
  smoothAccelN?: number
  smoothAccelE?: number
  smoothAccelD?: number
  // Smooth sustained speeds (calculated from smooth velocities and accelerations)
  smoothSustainedSpeed?: number
  smoothGlideRatio?: number
  smoothLiftToDragRatio?: number
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
  accelX?: number  // East acceleration (state/filtered)
  accelY?: number  // Up acceleration (state/filtered)
  accelZ?: number  // North acceleration (state/filtered)
  // Additional acceleration vectors for analysis
  aMeasuredX?: number  // Measured acceleration East (m/s²)
  aMeasuredY?: number  // Measured acceleration Up (m/s²)
  aMeasuredZ?: number  // Measured acceleration North (m/s²)
  aWSEX?: number      // Wingsuit model acceleration East (m/s²)
  aWSEY?: number      // Wingsuit model acceleration Up (m/s²)
  aWSEZ?: number      // Wingsuit model acceleration North (m/s²)
  // Optional wingsuit parameters for hover display
  kl?: number
  kd?: number
  roll?: number
  // Sustained speeds (wingsuit aerodynamic velocities)
  vxs?: number  // Sustained horizontal speed (m/s)
  vys?: number  // Sustained vertical speed (m/s)
  // Wind estimation data (only from Kalman filter)
  windEstimate?: Vector3  // Estimated wind velocity in ENU coordinates
  windAdjustedAoA?: number  // Wind-adjusted angle of attack (radians)
  windAdjustedRoll?: number  // Wind-adjusted roll angle (radians)
  windsustainedSpeeds?: SustainedSpeeds  // Wind-adjusted sustained speeds
  windadjustedcoefficients?: Coefficients  // Wind-adjusted aerodynamic coefficients
  // Smoothed GPS speeds (calculated once from CSV data)
  smoothVelN?: number  // Smoothed North velocity (m/s)
  smoothVelE?: number  // Smoothed East velocity (m/s)
  smoothVelD?: number  // Smoothed Down velocity (m/s)
  // Smoothed GPS accelerations (calculated from smoothed velocities)
  smoothAccelN?: number  // Smoothed North acceleration (m/s²)
  smoothAccelE?: number  // Smoothed East acceleration (m/s²)
  smoothAccelD?: number  // Smoothed Down acceleration (m/s²)
  // Smooth sustained speeds (calculated from smooth velocities and accelerations)
  smoothVxs?: number  // Smooth sustained horizontal speed (m/s)
  smoothVys?: number  // Smooth sustained vertical speed (m/s)
  smoothRoll?: number  // Smooth sustained roll angle (radians)
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


export interface Coefficients {
  cl: number,
  cd: number,
}
export interface SustainedSpeeds {
  vxs: number,
  vys: number,
}

export interface WSEQPolar {
  type: string,
  name: string,
  public: boolean,
  polarslope: number,
  polarclo: number,
  polarmindrag: number,
  stallmaxdrag?: number, // if we can fly slower than cdstall but cant fly at zero lift use this
  clstallsep?: number,
  cdstallsep?: number,
  clstall?: number,
  cdstall?: number,
  clspeedsep?: number,
  cdspeedsep?: number,
  clspeed?: number,
  cdspeed?: number,
  rangemincl: number,
  rangemaxcl: number,
  s: number,
  m: number,
  table?: Coefficients[],
  aoaindexes: number[],
  aoas: number[],
  cp?: number[],
  cm?:number[],
  cg?: number,
  i?: number,
  angledifferential?: number,
  rotationdifferential?: number,
  is_enabled?: boolean,
  suit?: string,
  stallpoint: Coefficients[],
  //guides?: Guides
  deploypolar?:WSEQPolar,
  archpolar?:WSEQPolar,//canopy=brakes,ws=arch
  dearchpolar?:WSEQPolar,
  aoagrcontrolpoints?:Coefficients[][],//aoa control points
  cpcontrolpoints?:Coefficients[][],
  stallcontrolpoints?:Coefficients[][],//coeff control points
  speedcontrolpoints?:Coefficients[],
  seperationpoint?:Coefficients,
  speedseperationpoint?:Coefficients
  //  detachmentpoint: Coefficients
  //  inflectiondrag: Coefficients,
  //  corner:Coefficients,
  //  maxdrag:Coefficients,
  //}
}

