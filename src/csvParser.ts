export function parseCSV(csv: string): Record<string, string>[] {
  const lines = csv.trim().split('\n')
  const headers = ['GNSS', 'time', 'lat', 'lon', 'hMSL', 'velN', 'velE', 'velD', 'hAcc', 'vAcc', 'sAcc', 'numSV']
  const data: Record<string, string>[] = []

  for (const line of lines.slice(7)) {
    if (line.startsWith('$GNSS,')) {
      const values = line.split(',').map(v => v.trim())
      const obj: Record<string, string> = {}
      for (let index = 0; index < headers.length; index++) {
        const header = headers[index]
        obj[header] = values[index] || ''
      }
      data.push(obj)
    }
  }
  
  return data
}
