param(
    [ValidateRange(1, 65535)]
    [int]$Port = 8000
)

$ErrorActionPreference = "Stop"
$webRoot = $PSScriptRoot
$pagePath = Join-Path $webRoot "crane_control.html"
$pidPath = Join-Path $webRoot ".upper_computer_server.pid"
$statusPath = Join-Path $webRoot ".upper_computer_server.log"
$errorPath = Join-Path $webRoot ".upper_computer_server.error.log"
$listener = $null

function Write-HttpResponse {
    param(
        [System.IO.Stream]$Stream,
        [string]$Status,
        [string]$ContentType,
        [byte[]]$Body,
        [bool]$SendBody = $true
    )

    $headerText = "HTTP/1.1 $Status`r`n" +
        "Content-Type: $ContentType`r`n" +
        "Content-Length: $($Body.Length)`r`n" +
        "Cache-Control: no-store`r`n" +
        "X-Content-Type-Options: nosniff`r`n" +
        "Connection: close`r`n`r`n"
    $headerBytes = [System.Text.Encoding]::ASCII.GetBytes($headerText)
    $Stream.Write($headerBytes, 0, $headerBytes.Length)
    if ($SendBody -and $Body.Length -gt 0) {
        $Stream.Write($Body, 0, $Body.Length)
    }
    $Stream.Flush()
}

try {
    if (-not (Test-Path -LiteralPath $pagePath)) {
        throw "crane_control.html was not found in $webRoot"
    }

    Set-Content -LiteralPath $pidPath -Value $PID -Encoding ASCII
    Set-Content -LiteralPath $statusPath -Value "Starting on http://127.0.0.1:$Port/" -Encoding ASCII
    Set-Content -LiteralPath $errorPath -Value "" -Encoding ASCII

    $listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, $Port)
    $listener.Start()
    Set-Content -LiteralPath $statusPath -Value "Running on http://127.0.0.1:$Port/ (PID $PID)" -Encoding ASCII

    while ($true) {
        $client = $null
        $reader = $null
        try {
            $client = $listener.AcceptTcpClient()
            $client.ReceiveTimeout = 3000
            $client.SendTimeout = 3000
            $stream = $client.GetStream()
            $reader = [System.IO.StreamReader]::new(
                $stream,
                [System.Text.Encoding]::ASCII,
                $false,
                1024,
                $true
            )

            $requestLine = $reader.ReadLine()
            if ([string]::IsNullOrWhiteSpace($requestLine)) {
                continue
            }

            do {
                $headerLine = $reader.ReadLine()
            } while ($null -ne $headerLine -and $headerLine.Length -gt 0)

            $parts = $requestLine.Split(" ")
            if ($parts.Length -lt 2) {
                $badRequest = [System.Text.Encoding]::UTF8.GetBytes("Bad Request")
                Write-HttpResponse -Stream $stream -Status "400 Bad Request" -ContentType "text/plain; charset=utf-8" -Body $badRequest
                continue
            }

            $method = $parts[0].ToUpperInvariant()
            $requestPath = ($parts[1] -split "\?", 2)[0]
            if ($method -ne "GET" -and $method -ne "HEAD") {
                $notAllowed = [System.Text.Encoding]::UTF8.GetBytes("Method Not Allowed")
                Write-HttpResponse -Stream $stream -Status "405 Method Not Allowed" -ContentType "text/plain; charset=utf-8" -Body $notAllowed
                continue
            }

            if ($requestPath -eq "/" -or $requestPath -eq "/crane_control.html") {
                $pageBytes = [System.IO.File]::ReadAllBytes($pagePath)
                Write-HttpResponse `
                    -Stream $stream `
                    -Status "200 OK" `
                    -ContentType "text/html; charset=utf-8" `
                    -Body $pageBytes `
                    -SendBody ($method -ne "HEAD")
            }
            else {
                $notFound = [System.Text.Encoding]::UTF8.GetBytes("Not Found")
                Write-HttpResponse -Stream $stream -Status "404 Not Found" -ContentType "text/plain; charset=utf-8" -Body $notFound -SendBody ($method -ne "HEAD")
            }
        }
        catch {
            $message = "$(Get-Date -Format o) Client error: $($_.Exception.Message)"
            Add-Content -LiteralPath $errorPath -Value $message -Encoding UTF8
        }
        finally {
            if ($null -ne $reader) {
                $reader.Dispose()
            }
            if ($null -ne $client) {
                $client.Dispose()
            }
        }
    }
}
catch {
    $message = "$(Get-Date -Format o) Server error: $($_.Exception.ToString())"
    Set-Content -LiteralPath $errorPath -Value $message -Encoding UTF8
    exit 1
}
finally {
    if ($null -ne $listener) {
        $listener.Stop()
    }
}
