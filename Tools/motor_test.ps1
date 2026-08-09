param(
    [Parameter(Mandatory = $true)]
    [string]$Port,

    [Parameter(Mandatory = $true)]
    [ValidateRange(1, 4)]
    [int]$Motor,

    [ValidateRange(48, 200)]
    [int]$Value = 80,

    [ValidateRange(100, 5000)]
    [int]$DurationMs = 1000,

    [Parameter(Mandatory = $true)]
    [switch]$PropsRemoved
)

$ErrorActionPreference = 'Stop'
$Command = 0x4006

function Get-Crc8DvbS2 {
    param([byte[]]$Bytes)

    [byte]$crc = 0
    foreach ($item in $Bytes) {
        $crc = $crc -bxor $item
        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($crc -band 0x80) -ne 0) {
                $crc = [byte]((($crc -shl 1) -bxor 0xD5) -band 0xFF)
            } else {
                $crc = [byte](($crc -shl 1) -band 0xFF)
            }
        }
    }
    return $crc
}

function New-Msp2MotorFrame {
    param([int[]]$Values)

    $payload = [byte[]]::new(8)
    for ($index = 0; $index -lt 4; $index++) {
        $payload[$index * 2] = [byte]($Values[$index] -band 0xFF)
        $payload[$index * 2 + 1] = [byte](($Values[$index] -shr 8) -band 0xFF)
    }
    $body = [byte[]]@(0, ($Command -band 0xFF), (($Command -shr 8) -band 0xFF), 8, 0) + $payload
    $crc = Get-Crc8DvbS2 -Bytes $body
    return [byte[]]@(0x24, 0x58, 0x3C) + $body + [byte[]]@($crc)
}

function Read-Msp2EmptyResponse {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [int]$ExpectedCommand
    )

    $frame = [byte[]]::new(9)
    $offset = 0
    while ($offset -lt $frame.Length) {
        $offset += $Serial.Read($frame, $offset, $frame.Length - $offset)
    }
    $body = [byte[]]@($frame[3], $frame[4], $frame[5], $frame[6], $frame[7])
    if (($frame[0] -ne 0x24) -or ($frame[1] -ne 0x58) -or
        ($frame[4] -ne ($ExpectedCommand -band 0xFF)) -or
        ($frame[5] -ne (($ExpectedCommand -shr 8) -band 0xFF)) -or
        ($frame[6] -ne 0) -or ($frame[7] -ne 0) -or
        ((Get-Crc8DvbS2 -Bytes $body) -ne $frame[8])) {
        throw 'Invalid MSP2 response from the flight controller.'
    }
    if ($frame[2] -eq 0x21) {
        throw 'Flight controller rejected the motor test. Check flight ready=1, dshot=1, and safety=0.'
    }
    if ($frame[2] -ne 0x3E) {
        throw 'Unexpected MSP2 response direction.'
    }
}

function Send-Msp2MotorFrame {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [byte[]]$Frame
    )

    $Serial.Write($Frame, 0, $Frame.Length)
    Read-Msp2EmptyResponse -Serial $Serial -ExpectedCommand $Command
}

if (-not $PropsRemoved) {
    throw 'Refusing motor test: pass -PropsRemoved only after removing every propeller.'
}

$values = @(0, 0, 0, 0)
$values[$Motor - 1] = $Value
$runFrame = New-Msp2MotorFrame -Values $values
$stopFrame = New-Msp2MotorFrame -Values @(0, 0, 0, 0)
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, 'None', 8, 'One')
$serial.WriteTimeout = 200
$serial.ReadTimeout = 500

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    Write-Host "Testing motor M$Motor at value $Value for $DurationMs ms on $Port..."
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $DurationMs) {
        Send-Msp2MotorFrame -Serial $serial -Frame $runFrame
        Start-Sleep -Milliseconds 100
    }
} finally {
    $wasOpen = $serial.IsOpen
    $stopWrites = 0
    $lastStopError = $null
    try {
        if ($wasOpen) {
            for ($attempt = 0; $attempt -lt 3; $attempt++) {
                try {
                    $serial.Write($stopFrame, 0, $stopFrame.Length)
                    $stopWrites++
                } catch {
                    $lastStopError = $_
                }
                Start-Sleep -Milliseconds 20
            }
            $serial.Close()
        }
    } finally {
        $serial.Dispose()
    }
    if ($wasOpen) {
        if ($stopWrites -eq 0) {
            throw "Motor test ended, but every stop write failed: $($lastStopError.Exception.Message)"
        }
        Write-Host "Stop safeguard complete; $stopWrites stop command writes completed."
    }
}
