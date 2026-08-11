param(
    [string]$PortName = "COM7",
    [int]$DurationSeconds = 10
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

$watch = [System.Diagnostics.Stopwatch]::StartNew()
$nextPingMs = 0
$nextEnableMs = 0
$enableSentMs = $null
$firstOutputMs = $null
$firstOnlineMs = @($null, $null, $null)
$onlineCount = @(0, 0, 0)
$offlineAfterOnline = @(0, 0, 0)
$wasOnline = @($false, $false, $false)
$telemetryCount = 0
$invalidCount = 0
$firstFrame = $null
$lastFrame = $null
$firstEnabledAngles = $null
$lastEnabledAngles = $null

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    # Wait for a real firmware frame before enabling. This avoids measuring the
    # debugger/programmer reset window or a USB bridge startup delay as motor startup time.
    $serial.WriteLine("PING")
    $readyDeadline = $watch.Elapsed.TotalMilliseconds + 2000
    while ($watch.Elapsed.TotalMilliseconds -lt $readyDeadline) {
        try {
            $readyLine = $serial.ReadLine().Trim()
            if ($readyLine.StartsWith("TEL,")) { break }
        }
        catch [System.TimeoutException] {
            $serial.WriteLine("PING")
        }
    }
    $serial.WriteLine("SAFE,1")
    $enableSentMs = $watch.Elapsed.TotalMilliseconds
    $nextPingMs = $watch.ElapsedMilliseconds + 250
    $nextEnableMs = $watch.ElapsedMilliseconds + 250

    while ($watch.Elapsed.TotalSeconds -lt $DurationSeconds) {
        # SAFE is a state request, so retry it until telemetry acknowledges it.
        if (($null -eq $firstOutputMs) -and $watch.ElapsedMilliseconds -ge $nextEnableMs) {
            $serial.WriteLine("SAFE,1")
            $nextEnableMs = $watch.ElapsedMilliseconds + 250
        }
        if ($watch.ElapsedMilliseconds -ge $nextPingMs) {
            $serial.WriteLine("PING")
            $nextPingMs = $watch.ElapsedMilliseconds + 250
        }

        try {
            $line = $serial.ReadLine().Trim()
            if (-not $line.StartsWith("TEL,")) {
                if (-not [string]::IsNullOrWhiteSpace($line)) { $invalidCount++ }
                continue
            }

            $fields = $line.Split(',')
            if ($fields.Count -lt 13) {
                $invalidCount++
                continue
            }

            $telemetryCount++
            if ($null -eq $firstFrame) { $firstFrame = $line }
            $lastFrame = $line
            $nowMs = $watch.Elapsed.TotalMilliseconds
            $enabled = $fields[2] -eq "1"
            $flags = @(
                ($fields[4] -eq "1"),
                ($fields[6] -eq "1"),
                ($fields[8] -eq "1")
            )

            if ($enabled -and $null -eq $firstOutputMs) {
                $firstOutputMs = $nowMs
            }
            if ($enabled) {
                $angles = @([int]$fields[3], [int]$fields[5], [int]$fields[7])
                if ($null -eq $firstEnabledAngles) { $firstEnabledAngles = $angles }
                $lastEnabledAngles = $angles
            }

            for ($axis = 0; $axis -lt 3; $axis++) {
                if ($flags[$axis]) {
                    $onlineCount[$axis]++
                    if ($null -eq $firstOnlineMs[$axis]) { $firstOnlineMs[$axis] = $nowMs }
                    $wasOnline[$axis] = $true
                }
                elseif ($wasOnline[$axis]) {
                    $offlineAfterOnline[$axis]++
                    $wasOnline[$axis] = $false
                }
            }
        }
        catch [System.TimeoutException] {
        }
    }
}
finally {
    if ($serial.IsOpen) {
        try { $serial.WriteLine("SAFE,0") } catch {}
        Start-Sleep -Milliseconds 100
        $serial.Close()
    }
    $serial.Dispose()
}

$names = @("EXT", "WINCH", "YAW")
Write-Output "PORT=$PortName"
Write-Output "DURATION_S=$DurationSeconds"
Write-Output "TEL_COUNT=$telemetryCount"
Write-Output "INVALID_LINES=$invalidCount"
if ($null -ne $firstOutputMs) {
    Write-Output ("OUTPUT_ENABLE_DELAY_MS={0:F1}" -f ($firstOutputMs - $enableSentMs))
}
else {
    Write-Output "OUTPUT_ENABLE_DELAY_MS=NEVER"
}
for ($axis = 0; $axis -lt 3; $axis++) {
    $delay = if ($null -eq $firstOnlineMs[$axis]) { "NEVER" } else { "{0:F1}" -f ($firstOnlineMs[$axis] - $enableSentMs) }
    Write-Output ("{0}_FIRST_ONLINE_MS={1}" -f $names[$axis], $delay)
    Write-Output ("{0}_ONLINE_FRAMES={1}" -f $names[$axis], $onlineCount[$axis])
    Write-Output ("{0}_OFFLINE_TRANSITIONS={1}" -f $names[$axis], $offlineAfterOnline[$axis])
}
if ($null -ne $firstEnabledAngles -and $null -ne $lastEnabledAngles) {
    Write-Output ("ANGLE_DELTA_CDEG={0},{1},{2}" -f
        ($lastEnabledAngles[0] - $firstEnabledAngles[0]),
        ($lastEnabledAngles[1] - $firstEnabledAngles[1]),
        ($lastEnabledAngles[2] - $firstEnabledAngles[2]))
}
Write-Output "FIRST=$firstFrame"
Write-Output "LAST=$lastFrame"

if ($null -eq $firstOutputMs -or $telemetryCount -lt ($DurationSeconds * 2)) {
    exit 1
}
