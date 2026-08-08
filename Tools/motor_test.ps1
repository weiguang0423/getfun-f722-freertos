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

if (-not $PropsRemoved) {
    throw 'Refusing motor test: pass -PropsRemoved only after removing every propeller.'
}

$values = @(0, 0, 0, 0)
$values[$Motor - 1] = $Value
$runFrame = New-Msp2MotorFrame -Values $values
$stopFrame = New-Msp2MotorFrame -Values @(0, 0, 0, 0)
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, 'None', 8, 'One')
$serial.WriteTimeout = 200

try {
    $serial.Open()
    $deadline = [Environment]::TickCount64 + $DurationMs
    while ([Environment]::TickCount64 -lt $deadline) {
        $serial.Write($runFrame, 0, $runFrame.Length)
        Start-Sleep -Milliseconds 100
    }
} finally {
    if ($serial.IsOpen) {
        for ($attempt = 0; $attempt -lt 3; $attempt++) {
            $serial.Write($stopFrame, 0, $stopFrame.Length)
            Start-Sleep -Milliseconds 20
        }
        $serial.Close()
    }
    $serial.Dispose()
}
