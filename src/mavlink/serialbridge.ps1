# Serial <-> stdio bridge used by AP Tuner (no native Node modules needed).
# Usage: powershell -File serialbridge.ps1 COM5 115200
# Bytes from the serial port go to stdout, bytes from stdin go to the port. Binary safe (raw streams).
param([string]$PortName = 'COM3', [int]$Baud = 115200)
$ErrorActionPreference = 'Stop'
try {
  $port = New-Object System.IO.Ports.SerialPort $PortName, $Baud, 'None', 8, 'One'
  $port.ReadTimeout = 50
  $port.WriteTimeout = 500
  $port.DtrEnable = $true
  $port.RtsEnable = $true
  $port.ReadBufferSize = 65536
  $port.Open()
  [Console]::Error.WriteLine("open " + $PortName)
} catch {
  [Console]::Error.WriteLine("cannot open " + $PortName + ": " + $_.Exception.Message)
  exit 2
}
$in = [Console]::OpenStandardInput()
$out = [Console]::OpenStandardOutput()
$ps = $port.BaseStream
$buf = New-Object byte[] 4096
$ibuf = New-Object byte[] 4096
# async read of stdin so the loop never blocks on it
$inTask = $in.ReadAsync($ibuf, 0, $ibuf.Length)
try {
  while ($port.IsOpen) {
    $n = 0
    try { $n = $ps.Read($buf, 0, $buf.Length) } catch [System.TimeoutException] { $n = 0 }
    if ($n -gt 0) { $out.Write($buf, 0, $n); $out.Flush() }
    if ($inTask.IsCompleted) {
      $k = $inTask.Result
      if ($k -le 0) { break }           # stdin closed: the app disconnected
      # a write that times out (radio buffer full, no flow) drops this packet instead of closing the port
      try { $ps.Write($ibuf, 0, $k); $ps.Flush() } catch { [Console]::Error.WriteLine("write dropped: " + $_.Exception.Message) }
      $inTask = $in.ReadAsync($ibuf, 0, $ibuf.Length)
    }
    if ($n -eq 0) { Start-Sleep -Milliseconds 5 }
  }
} finally {
  try { $port.Close() } catch {}
}
exit 0
