param(
    [string]$Port = "COM5",
    [int]$Baud = 2000000,
    [string]$LocalFile = "vision\k230_target\main.py",
    [string]$RemoteFile = "/sdcard/k230_target.py",
    [int]$ReadSeconds = 10,
    [int]$LineDelayMs = 8,
    [int]$ChunkChars = 180
)

$ErrorActionPreference = "Stop"

function Read-SerialFor {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [int]$Milliseconds
    )

    $deadline = (Get-Date).AddMilliseconds($Milliseconds)
    $text = ""
    while ((Get-Date) -lt $deadline) {
        $text += $Serial.ReadExisting()
        Start-Sleep -Milliseconds 50
    }
    return $text
}

if (-not (Test-Path -LiteralPath $LocalFile)) {
    throw "Local file not found: $LocalFile"
}

$bytes = [IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $LocalFile).Path)
$b64 = [Convert]::ToBase64String($bytes)
$chunks = New-Object System.Collections.Generic.List[string]
for ($i = 0; $i -lt $b64.Length; $i += $ChunkChars) {
    $take = [Math]::Min($ChunkChars, $b64.Length - $i)
    $chunks.Add($b64.Substring($i, $take))
}

$lines = New-Object System.Collections.Generic.List[string]
$lines.Add("import binascii")
$lines.Add("import gc")
$lines.Add("import os")
$lines.Add("path = '$RemoteFile'")
$lines.Add("chunks = []")
foreach ($chunk in $chunks) {
    $lines.Add("chunks.append('$chunk')")
}
$lines.Add("f = open(path, 'wb')")
$lines.Add("for chunk in chunks:")
$lines.Add("    f.write(binascii.a2b_base64(chunk.encode()))")
$lines.Add("f.close()")
$lines.Add("gc.collect()")
$lines.Add("print('UPLOAD_DONE', path, os.stat(path)[6])")

$serial = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$serial.ReadTimeout = 100
$serial.WriteTimeout = 2000
$serial.DtrEnable = $true
$serial.RtsEnable = $true

try {
    $serial.Open()
    Start-Sleep -Milliseconds 300

    1..6 | ForEach-Object {
        $serial.Write([byte[]](3), 0, 1)
        Start-Sleep -Milliseconds 120
    }
    Write-Output (Read-SerialFor -Serial $serial -Milliseconds 500)

    $serial.Write([byte[]](5), 0, 1)
    Start-Sleep -Milliseconds 300
    Write-Output (Read-SerialFor -Serial $serial -Milliseconds 500)

    foreach ($line in $lines) {
        $lineBytes = [System.Text.Encoding]::ASCII.GetBytes($line + "`r`n")
        $serial.Write($lineBytes, 0, $lineBytes.Length)
        if ($LineDelayMs -gt 0) {
            Start-Sleep -Milliseconds $LineDelayMs
        }
    }

    Start-Sleep -Milliseconds 300
    $serial.Write([byte[]](4), 0, 1)
    Write-Output (Read-SerialFor -Serial $serial -Milliseconds ($ReadSeconds * 1000))
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
