param([string]$Port="COM10", [int]$Baud=115200, [string]$Msg="ECHO-TEST-42")
try {
  $p = New-Object System.IO.Ports.SerialPort $Port, $Baud, None, 8, one
  $p.ReadTimeout = 2000
  $p.Open()
  Start-Sleep -Milliseconds 300
  $p.DiscardInBuffer()
  $bytes = [System.Text.Encoding]::ASCII.GetBytes($Msg + "`r`n")
  $p.Write($bytes, 0, $bytes.Length)
  Start-Sleep -Milliseconds 1200
  $n = $p.BytesToRead
  $data = New-Object byte[] $n
  $got = $p.Read($data, 0, $n)
  $p.Close()
  Write-Output ("WROTE='$Msg'  READBYTES=$got")
  $text = [System.Text.Encoding]::ASCII.GetString($data, 0, $got)
  Write-Output "---RECV---"
  Write-Output $text
  Write-Output "---END---"
  if ($text -match [regex]::Escape($Msg)) { Write-Output "ECHO_OK: token came back" } else { Write-Output "ECHO_MISS: token NOT seen" }
} catch { Write-Output ("ERR: " + $_.Exception.Message) }
