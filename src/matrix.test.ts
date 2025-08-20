import { describe, it, expect } from 'vitest'
import {
  createIdentityMatrix,
  createZeroMatrix,
  matrixMultiply,
  matrixVectorMultiply,
  matrixAdd,
  matrixSubtract,
  transpose,
  matrixInverse3x3
} from './matrix.js'

describe('Matrix Operations', () => {
  it('should create identity matrix correctly', () => {
    const identity = createIdentityMatrix(3)
    expect(identity).toEqual([
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1]
    ])
  })

  it('should create zero matrix correctly', () => {
    const zero = createZeroMatrix(2, 3)
    expect(zero).toEqual([
      [0, 0, 0],
      [0, 0, 0]
    ])
  })

  it('should multiply matrices correctly', () => {
    const A = [[1, 2], [3, 4]]
    const B = [[5, 6], [7, 8]]
    const result = matrixMultiply(A, B)
    expect(result).toEqual([
      [19, 22],
      [43, 50]
    ])
  })

  it('should multiply matrix by vector correctly', () => {
    const A = [[1, 2, 3], [4, 5, 6]]
    const v = [1, 2, 3]
    const result = matrixVectorMultiply(A, v)
    expect(result).toEqual([14, 32])
  })

  it('should add matrices correctly', () => {
    const A = [[1, 2], [3, 4]]
    const B = [[5, 6], [7, 8]]
    const result = matrixAdd(A, B)
    expect(result).toEqual([
      [6, 8],
      [10, 12]
    ])
  })

  it('should subtract matrices correctly', () => {
    const A = [[5, 6], [7, 8]]
    const B = [[1, 2], [3, 4]]
    const result = matrixSubtract(A, B)
    expect(result).toEqual([
      [4, 4],
      [4, 4]
    ])
  })

  it('should transpose matrix correctly', () => {
    const A = [[1, 2, 3], [4, 5, 6]]
    const result = transpose(A)
    expect(result).toEqual([
      [1, 4],
      [2, 5],
      [3, 6]
    ])
  })

  it('should invert 3x3 matrix correctly', () => {
    const A = [
      [1, 0, 2],
      [-1, 5, 0],
      [0, 3, -9]
    ]
    const result = matrixInverse3x3(A)
    
    // Verify A * A^-1 = I (approximately)
    const product = matrixMultiply(A, result)
    const tolerance = 1e-10
    
    expect(Math.abs(product[0][0] - 1)).toBeLessThan(tolerance)
    expect(Math.abs(product[1][1] - 1)).toBeLessThan(tolerance)
    expect(Math.abs(product[2][2] - 1)).toBeLessThan(tolerance)
    expect(Math.abs(product[0][1])).toBeLessThan(tolerance)
    expect(Math.abs(product[0][2])).toBeLessThan(tolerance)
  })

  it('should throw error for singular matrix', () => {
    const singular = [
      [1, 2, 3],
      [4, 5, 6],
      [7, 8, 9] // This row is linearly dependent
    ]
    expect(() => matrixInverse3x3(singular)).toThrow('Matrix is singular')
  })
})