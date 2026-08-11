param(
    [string]$PortName = "COM7",
    [int]$DurationSeconds = 8,
    [ValidateSet("None", "Ping", "Safe", "SafeAndPing")]
    [string]$CommandMode = "SafeAndPing"
)

$ErrorActionPreference = "Stop"
$serial = [System.IO.Ports.SerialPort]::new(
    $PortName,
    115200,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.NewLine = "`n"
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500

$telemetryCount = 0
$invalidCount = 0
$lastTelemetryAt = $null
$maximumGapMs = 0.0
$firstFrame = $null
$lastFrame = $null
$watch = [System.Diagnostics.Stopwatch]::StartNew()
$nextPingMs = 0

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()
    if ($CommandMode -eq "Safe" -or $CommandMode -eq "SafeAndPing") {
        $serial.WriteLine("SAFE,0")
    }

    while ($watch.Elapsed.TotalSeconds -lt $DurationSeconds) {
        if (($CommandMode -eq "Ping" -or $CommandMode -eq "SafeAndPing") -and
            $watch.ElapsedMilliseconds -ge $nextPingMs) {
            $serial.WriteLine("PING")
            $nextPingMs = $watch.ElapsedMilliseconds + 250
        }

        try {
            $line = $serial.ReadLine().Trim()
            if ($line.StartsWith("TEL,")) {
                $nowMs = $watch.Elapsed.TotalMilliseconds
                if ($null -ne $lastTelemetryAt) {
                    $gapMs = $nowMs - $lastTelemetryAt
                    if ($gapMs -gt $maximumGapMs) {
                        $maximumGapMs = $gapMs
                    }
                }
                $lastTelemetryAt = $nowMs
                $telemetryCount++
                if ($null -eq $firstFrame) {
                    $firstFrame = $line
                }
                $lastFrame = $line
            }
            elseif (-not [string]::IsNullOrWhiteSpace($line)) {
                $invalidCount++
            }
        }
        catch [System.TimeoutException] {
        }
    }
}
finally {
    if ($serial.IsOpen) {
        if ($CommandMode -ne "None") {
            try { $serial.WriteLine("SAFE,0") } catch {}
        }
        $serial.Close()
    }
    $serial.Dispose()
}

Write-Output "PORT=$PortName"
Write-Output "DURATION_S=$DurationSeconds"
Write-Output "COMMAND_MODE=$CommandMode"
Write-Output "TEL_COUNT=$telemetryCount"
Write-Output ("MAX_GAP_MS={0:F1}" -f $maximumGapMs)
Write-Output "INVALID_LINES=$invalidCount"
Write-Output "FIRST=$firstFrame"
Write-Output "LAST=$lastFrame"

if ($telemetryCount -lt ($DurationSeconds * 2) -or $maximumGapMs -gt 1000.0) {
    exit 1
}
