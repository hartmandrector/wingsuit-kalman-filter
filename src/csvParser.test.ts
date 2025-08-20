import { describe, it, expect } from 'vitest'
import { parseCSV } from './csvParser.js'

describe('CSV Parser', () => {
  const sampleCSV = `$FLYS,1
$VAR,FIRMWARE_VER,v2024.12.30
$VAR,DEVICE_ID,002f001d5753500520323339
$VAR,SESSION_ID,90ee0c9473d93d5f0b71ac74
$COL,GNSS,time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV
$UNIT,GNSS,,deg,deg,m,m/s,m/s,m/s,m,m,m/s,
$DATA
$GNSS,2025-08-08T23:18:34.200Z,47.2419335,-123.1427603,109.244,-6.980,-0.913,0.161,208.518,234.818,4.415,6
$GNSS,2025-08-08T23:18:34.400Z,47.2419185,-123.1427638,101.888,-7.218,-0.964,0.288,104.313,112.658,2.532,8
$GNSS,2025-08-08T23:18:34.600Z,47.2417269,-123.1427021,75.688,-6.695,-0.950,0.565,81.368,92.070,1.608,11`

  it('should skip the first 7 lines', () => {
    const result = parseCSV(sampleCSV)
    expect(result.length).toBe(3)
  })

  it('should parse GNSS data lines correctly', () => {
    const result = parseCSV(sampleCSV)
    
    expect(result[0]).toEqual({
      GNSS: '$GNSS',
      time: '2025-08-08T23:18:34.200Z',
      lat: '47.2419335',
      lon: '-123.1427603',
      hMSL: '109.244',
      velN: '-6.980',
      velE: '-0.913',
      velD: '0.161',
      hAcc: '208.518',
      vAcc: '234.818',
      sAcc: '4.415',
      numSV: '6'
    })
  })

  it('should only parse lines that start with $GNSS,', () => {
    const csvWithMixedLines = `$FLYS,1
$VAR,FIRMWARE_VER,v2024.12.30
$VAR,DEVICE_ID,002f001d5753500520323339
$VAR,SESSION_ID,90ee0c9473d93d5f0b71ac74
$COL,GNSS,time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV
$UNIT,GNSS,,deg,deg,m,m/s,m/s,m/s,m,m,m/s,
$DATA
$GNSS,2025-08-08T23:18:34.200Z,47.2419335,-123.1427603,109.244,-6.980,-0.913,0.161,208.518,234.818,4.415,6
$OTHER,some,other,data
$GNSS,2025-08-08T23:18:34.400Z,47.2419185,-123.1427638,101.888,-7.218,-0.964,0.288,104.313,112.658,2.532,8`

    const result = parseCSV(csvWithMixedLines)
    expect(result.length).toBe(2)
    expect(result.every(row => row.GNSS === '$GNSS')).toBe(true)
  })

  it('should handle empty CSV', () => {
    const result = parseCSV('')
    expect(result).toEqual([])
  })

  it('should handle CSV with only header lines (no GNSS data)', () => {
    const csvWithoutData = `$FLYS,1
$VAR,FIRMWARE_VER,v2024.12.30
$VAR,DEVICE_ID,002f001d5753500520323339
$VAR,SESSION_ID,90ee0c9473d93d5f0b71ac74
$COL,GNSS,time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV
$UNIT,GNSS,,deg,deg,m,m/s,m/s,m/s,m,m,m/s,
$DATA`

    const result = parseCSV(csvWithoutData)
    expect(result).toEqual([])
  })

  it('should trim whitespace from values', () => {
    const csvWithSpaces = `$FLYS,1
$VAR,FIRMWARE_VER,v2024.12.30
$VAR,DEVICE_ID,002f001d5753500520323339
$VAR,SESSION_ID,90ee0c9473d93d5f0b71ac74
$COL,GNSS,time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV
$UNIT,GNSS,,deg,deg,m,m/s,m/s,m/s,m,m,m/s,
$DATA
$GNSS, 2025-08-08T23:18:34.200Z , 47.2419335 , -123.1427603 , 109.244 , -6.980 , -0.913 , 0.161 , 208.518 , 234.818 , 4.415 , 6 `

    const result = parseCSV(csvWithSpaces)
    expect(result[0].lat).toBe('47.2419335')
    expect(result[0].lon).toBe('-123.1427603')
    expect(result[0].hMSL).toBe('109.244')
  })

  it('should use hardcoded headers regardless of CSV content', () => {
    const result = parseCSV(sampleCSV)
    const expectedHeaders = ['GNSS', 'time', 'lat', 'lon', 'hMSL', 'velN', 'velE', 'velD', 'hAcc', 'vAcc', 'sAcc', 'numSV']
    
    expect(Object.keys(result[0])).toEqual(expectedHeaders)
  })
})