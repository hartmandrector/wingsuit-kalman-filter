export function createIdentityMatrix(size: number): number[][] {
  const matrix: number[][] = []
  for (let i = 0; i < size; i++) {
    matrix[i] = new Array(size).fill(0)
    matrix[i][i] = 1
  }
  return matrix
}

export function createZeroMatrix(rows: number, cols: number): number[][] {
  const matrix: number[][] = []
  for (let i = 0; i < rows; i++) {
    matrix[i] = new Array(cols).fill(0)
  }
  return matrix
}

export function matrixMultiply(A: number[][], B: number[][]): number[][] {
  const rows = A.length
  const cols = B[0].length
  const common = B.length
  const result = createZeroMatrix(rows, cols)

  for (let i = 0; i < rows; i++) {
    for (let j = 0; j < cols; j++) {
      for (let k = 0; k < common; k++) {
        result[i][j] += A[i][k] * B[k][j]
      }
    }
  }
  return result
}

export function matrixVectorMultiply(A: number[][], v: number[]): number[] {
  const result: number[] = new Array(A.length).fill(0)
  for (let i = 0; i < A.length; i++) {
    for (let j = 0; j < v.length; j++) {
      result[i] += A[i][j] * v[j]
    }
  }
  return result
}

export function matrixAdd(A: number[][], B: number[][]): number[][] {
  const result = createZeroMatrix(A.length, A[0].length)
  for (let i = 0; i < A.length; i++) {
    for (let j = 0; j < A[0].length; j++) {
      result[i][j] = A[i][j] + B[i][j]
    }
  }
  return result
}

export function matrixSubtract(A: number[][], B: number[][]): number[][] {
  const result = createZeroMatrix(A.length, A[0].length)
  for (let i = 0; i < A.length; i++) {
    for (let j = 0; j < A[0].length; j++) {
      result[i][j] = A[i][j] - B[i][j]
    }
  }
  return result
}

export function transpose(A: number[][]): number[][] {
  const result = createZeroMatrix(A[0].length, A.length)
  for (let i = 0; i < A.length; i++) {
    for (let j = 0; j < A[0].length; j++) {
      result[j][i] = A[i][j]
    }
  }
  return result
}

export function matrixInverse3x3(A: number[][]): number[][] {
  const det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
              A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
              A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])

  if (Math.abs(det) < 1e-10) {
    throw new Error('Matrix is singular')
  }

  const inv = createZeroMatrix(3, 3)
  inv[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / det
  inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det
  inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det
  inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det
  inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det
  inv[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) / det
  inv[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / det
  inv[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) / det
  inv[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / det

  return inv
}

export function matrixInverse(A: number[][]): number[][] {
  const n = A.length
  const augmented = createZeroMatrix(n, 2 * n)

  // Create augmented matrix [A | I]
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      augmented[i][j] = A[i][j]
    }
    augmented[i][n + i] = 1
  }

  // Gaussian elimination
  for (let i = 0; i < n; i++) {
    // Find pivot
    let maxRow = i
    for (let k = i + 1; k < n; k++) {
      if (Math.abs(augmented[k][i]) > Math.abs(augmented[maxRow][i])) {
        maxRow = k
      }
    }

    // Swap rows
    if (maxRow !== i) {
      const temp = augmented[i]
      augmented[i] = augmented[maxRow]
      augmented[maxRow] = temp
    }

    // Check for singular matrix
    if (Math.abs(augmented[i][i]) < 1e-10) {
      throw new Error('Matrix is singular')
    }

    // Scale pivot row
    const pivot = augmented[i][i]
    for (let j = 0; j < 2 * n; j++) {
      augmented[i][j] /= pivot
    }

    // Eliminate column
    for (let k = 0; k < n; k++) {
      if (k !== i) {
        const factor = augmented[k][i]
        for (let j = 0; j < 2 * n; j++) {
          augmented[k][j] -= factor * augmented[i][j]
        }
      }
    }
  }

  // Extract inverse matrix
  const inverse = createZeroMatrix(n, n)
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      inverse[i][j] = augmented[i][n + j]
    }
  }

  return inverse
}
