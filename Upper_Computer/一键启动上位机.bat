@echo off
chcp 65001 >nul
setlocal

cd /d "%~dp0"
set "PORT=8000"
set "PAGE_URL=http://127.0.0.1:%PORT%/crane_control.html"

rem If our local port is not active, start the dependency-free PowerShell server.
powershell.exe -NoProfile -Command "$c = New-Object Net.Sockets.TcpClient; try { $c.Connect('127.0.0.1', %PORT%); exit 0 } catch { exit 1 } finally { $c.Dispose() }" >nul 2>nul
if errorlevel 1 (
    start "Crane Web Server" /min powershell.exe -NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File "%~dp0serve_upper_computer.ps1" -Port %PORT%
    timeout /t 1 /nobreak >nul
)

rem Confirm the server is reachable before opening the browser.
powershell.exe -NoProfile -Command "$c = New-Object Net.Sockets.TcpClient; try { $c.Connect('127.0.0.1', %PORT%); exit 0 } catch { exit 1 } finally { $c.Dispose() }" >nul 2>nul
if errorlevel 1 goto server_error

if exist "%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe" (
    start "" "%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe" --new-window "%PAGE_URL%"
    goto success
)
if exist "%ProgramFiles%\Microsoft\Edge\Application\msedge.exe" (
    start "" "%ProgramFiles%\Microsoft\Edge\Application\msedge.exe" --new-window "%PAGE_URL%"
    goto success
)
if exist "%ProgramFiles%\Google\Chrome\Application\chrome.exe" (
    start "" "%ProgramFiles%\Google\Chrome\Application\chrome.exe" --new-window "%PAGE_URL%"
    goto success
)
if exist "%ProgramFiles(x86)%\Google\Chrome\Application\chrome.exe" (
    start "" "%ProgramFiles(x86)%\Google\Chrome\Application\chrome.exe" --new-window "%PAGE_URL%"
    goto success
)
if exist "%LOCALAPPDATA%\Google\Chrome\Application\chrome.exe" (
    start "" "%LOCALAPPDATA%\Google\Chrome\Application\chrome.exe" --new-window "%PAGE_URL%"
    goto success
)

echo 未找到 Microsoft Edge 或 Google Chrome，无法使用 Web Serial。
pause
exit /b 1

:server_error
echo 本地网页服务启动失败。
echo 请查看：%~dp0.upper_computer_server.error.log
pause
exit /b 1

:success
exit /b 0

endlocal
