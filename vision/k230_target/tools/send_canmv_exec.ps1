param(
    [string]$Port = "COM5",
    [int]$Baud = 2000000,
    [string]$File = "vision\k230_target\safe_test.py",
    [int]$ReadSeconds = 30,
    [int]$LineDelayMs = 8
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

if (-not (Test-Path -LiteralPath $File)) {
    throw "File not found: $File"
}

$code = Get-Content -LiteralPath $File -Raw
$code = $code -replace "`r`n", "`n"
$code = $code -replace "`r", "`n"
$lines = $code -split "`n", -1

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

    $discard = Read-SerialFor -Serial $serial -Milliseconds 500
    if ($discard.Length -gt 0) {
        Write-Output $discard
    }

    $serial.Write([byte[]](5), 0, 1)
    Start-Sleep -Milliseconds 300
    Write-Output (Read-SerialFor -Serial $serial -Milliseconds 500)

    foreach ($line in $lines) {
        $bytes = [System.Text.Encoding]::ASCII.GetBytes($line + "`r`n")
        $serial.Write($bytes, 0, $bytes.Length)
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
