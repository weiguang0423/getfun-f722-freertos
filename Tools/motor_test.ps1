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
$MotorTestCommand = 0x4006
$MotorStatusCommand = 0x4005

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

function New-Msp2Frame {
    param(
        [int]$Command,
        [byte[]]$Payload
    )

    if ($null -eq $Payload) {
        $Payload = [byte[]]::new(0)
    }
    $body = [byte[]]@(
        0,
        ($Command -band 0xFF),
        (($Command -shr 8) -band 0xFF),
        ($Payload.Length -band 0xFF),
        (($Payload.Length -shr 8) -band 0xFF)
    ) + $Payload
    $crc = Get-Crc8DvbS2 -Bytes $body
    return [byte[]]@(0x24, 0x58, 0x3C) + $body + [byte[]]@($crc)
}

function Read-ExactBytes {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [int]$Count
    )

    $bytes = [byte[]]::new($Count)
    $offset = 0
    while ($offset -lt $bytes.Length) {
        $offset += $Serial.Read($bytes, $offset, $bytes.Length - $offset)
    }
    return $bytes
}

function Read-Msp2Response {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [int]$ExpectedCommand
    )

    [byte[]]$header = Read-ExactBytes -Serial $Serial -Count 8
    if (($header[0] -ne 0x24) -or ($header[1] -ne 0x58) -or
        ($header[4] -ne ($ExpectedCommand -band 0xFF)) -or
        ($header[5] -ne (($ExpectedCommand -shr 8) -band 0xFF))) {
        throw 'Invalid MSP2 response from the flight controller.'
    }

    $payloadLength = [int]$header[6] -bor ([int]$header[7] -shl 8)
    if ($payloadLength -gt 1024) {
        throw "Invalid MSP2 payload length $payloadLength."
    }
    [byte[]]$tail = Read-ExactBytes -Serial $Serial -Count ($payloadLength + 1)
    [byte[]]$payload = if ($payloadLength -eq 0) {
        [byte[]]::new(0)
    } else {
        [byte[]]$tail[0..($payloadLength - 1)]
    }
    [byte[]]$body = @($header[3], $header[4], $header[5], $header[6], $header[7]) + $payload
    if ((Get-Crc8DvbS2 -Bytes $body) -ne $tail[$payloadLength]) {
        throw 'Invalid MSP2 response CRC from the flight controller.'
    }
    if ($header[2] -eq 0x21) {
        throw 'Flight controller rejected the motor test. Check flight ready=1, dshot=1, and safety=0.'
    }
    if ($header[2] -ne 0x3E) {
        throw 'Unexpected MSP2 response direction.'
    }
    return [pscustomobject]@{ Payload = $payload }
}

function Invoke-Msp2Request {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [int]$Command,
        [byte[]]$Payload
    )

    [byte[]]$frame = New-Msp2Frame -Command $Command -Payload $Payload
    $Serial.Write($frame, 0, $frame.Length)
    return Read-Msp2Response -Serial $Serial -ExpectedCommand $Command
}

function New-MotorPayload {
    param([int[]]$Values)

    $payload = [byte[]]::new(8)
    for ($index = 0; $index -lt 4; $index++) {
        $payload[$index * 2] = [byte]($Values[$index] -band 0xFF)
        $payload[$index * 2 + 1] = [byte](($Values[$index] -shr 8) -band 0xFF)
    }
    return $payload
}

function Get-UInt16Le {
    param([byte[]]$Bytes, [int]$Offset)
    return [uint16]([int]$Bytes[$Offset] -bor
        ([int]$Bytes[$Offset + 1] -shl 8))
}

function Get-UInt32Le {
    param([byte[]]$Bytes, [int]$Offset)
    return [BitConverter]::ToUInt32($Bytes, $Offset)
}

function Get-MotorStatus {
    param([System.IO.Ports.SerialPort]$Serial)

    $response = Invoke-Msp2Request -Serial $Serial `
        -Command $MotorStatusCommand -Payload ([byte[]]::new(0))
    [byte[]]$payload = $response.Payload
    if ($payload.Length -ne 64) {
        throw "Unexpected motor status size $($payload.Length); expected 64."
    }

    $requested = @()
    $output = @()
    for ($motorIndex = 0; $motorIndex -lt 4; $motorIndex++) {
        $requested += Get-UInt16Le -Bytes $payload -Offset (44 + ($motorIndex * 2))
        $output += Get-UInt16Le -Bytes $payload -Offset (52 + ($motorIndex * 2))
    }
    return [pscustomobject]@{
        DshotReady = $payload[0] -ne 0
        InputsReady = $payload[1] -ne 0
        TestActive = $payload[2] -ne 0
        Busy = $payload[3] -ne 0
        SafetyFlags = Get-UInt32Le -Bytes $payload -Offset 4
        SubmitErrors = Get-UInt32Le -Bytes $payload -Offset 28
        DmaErrors = Get-UInt32Le -Bytes $payload -Offset 32
        Requested = $requested
        Output = $output
    }
}

function Send-Msp2MotorFrame {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [byte[]]$Payload
    )

    $response = Invoke-Msp2Request -Serial $Serial `
        -Command $MotorTestCommand -Payload $Payload
    if ($response.Payload.Length -ne 0) {
        throw 'Unexpected non-empty motor-test response.'
    }
}

if (-not $PropsRemoved) {
    throw 'Refusing motor test: pass -PropsRemoved only after removing every propeller.'
}

$values = @(0, 0, 0, 0)
$values[$Motor - 1] = $Value
$runPayload = New-MotorPayload -Values $values
$stopFrame = New-Msp2Frame -Command $MotorTestCommand `
    -Payload (New-MotorPayload -Values @(0, 0, 0, 0))
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, 'None', 8, 'One')
$serial.WriteTimeout = 200
$serial.ReadTimeout = 500

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $baseline = Get-MotorStatus -Serial $serial
    Write-Host ("Initial firmware status: dshot={0} inputs={1} safety=0x{2:X8} submit_err={3} dma_err={4}" -f `
        [int]$baseline.DshotReady, [int]$baseline.InputsReady,
        $baseline.SafetyFlags, $baseline.SubmitErrors, $baseline.DmaErrors)
    Write-Host "Testing motor M$Motor at value $Value for $DurationMs ms on $Port..."
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    $runStatus = $null
    while ($timer.ElapsedMilliseconds -lt $DurationMs) {
        Send-Msp2MotorFrame -Serial $serial -Payload $runPayload
        Start-Sleep -Milliseconds 20
        $runStatus = Get-MotorStatus -Serial $serial
        Start-Sleep -Milliseconds 80
    }

    if ($null -eq $runStatus) {
        throw 'No motor-test status sample was collected.'
    }
    $submitDelta = [uint64]$runStatus.SubmitErrors - [uint64]$baseline.SubmitErrors
    $dmaDelta = [uint64]$runStatus.DmaErrors - [uint64]$baseline.DmaErrors
    $outputText = $runStatus.Output -join ','
    Write-Host ("Run status: test={0} dshot={1} out=[{2}] submit_err_delta={3} dma_err_delta={4}" -f `
        [int]$runStatus.TestActive, [int]$runStatus.DshotReady,
        $outputText, $submitDelta, $dmaDelta)
    if (-not $runStatus.TestActive -or -not $runStatus.DshotReady -or
        $runStatus.Output[$Motor - 1] -ne $Value -or
        $submitDelta -ne 0 -or $dmaDelta -ne 0) {
        throw 'Firmware accepted the command but did not sustain clean DShot submission; use the Run status line to diagnose it.'
    }
    Write-Host 'Firmware sustained clean DShot submission. If the motor stayed still, check the physical motor pad waveform, ESC signal/ground, and ESC protocol.'
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
