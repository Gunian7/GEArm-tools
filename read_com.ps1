param([string]$Port = "COM8", [int]$Baud = 115200, [int]$Ms = 1500)
for ($i = 1; $i -le 6; $i++) {
  try {
    $p = New-Object System.IO.Ports.SerialPort $Port, $Baud, None, 8, one
    $p.ReadTimeout = 1500
    $p.Open()
    Start-Sleep -Milliseconds 300
    $p.DiscardInBuffer()
    Start-Sleep -Milliseconds $Ms
    $n = $p.BytesToRead
    $data = New-Object byte[] $n
    $got = $p.Read($data, 0, $n)
    $p.Close()
    Write-Output ("try " + $i + ": BYTES=" + $got)
    if ($got -gt 0) {
      Write-Output "TEXT_START>>>"
      Write-Output ([System.Text.Encoding]::ASCII.GetString($data, 0, $got))
      Write-Output "<<<TEXT_END"
      break
    }
  } catch {
    Write-Output ("try " + $i + " ERR: " + $_.Exception.Message)
    Start-Sleep -Milliseconds 800
  }
}
