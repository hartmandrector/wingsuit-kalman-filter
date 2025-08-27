/**
 * Linear least squares fitting
 */
export function getSlope<T>(points: T[], getX: (d: T) => number, getY: (d: T) => number): number {
  let sumx = 0
  let sumy = 0
  let sumxx = 0
  let sumxy = 0
  // tslint:disable-next-line:prefer-for-of
  for (let i = 0; i < points.length; i++) {
    const point = points[i]
    const x = getX(point)
    const y = getY(point)
    sumx += x
    sumy += y
    sumxx += x * x
    sumxy += x * y
  }
  const n = points.length
  return (sumxy - sumx * sumy / n) / (sumxx - sumx * sumx / n)
}
