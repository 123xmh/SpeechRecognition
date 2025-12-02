@echo off
chcp 65001 >nul
echo ========================================
echo WSL2 Port Forward Setup for Web Monitor
echo ========================================
echo.

echo Getting WSL2 IP address...
for /f "tokens=*" %%i in ('wsl hostname -I') do set WSL_IP=%%i
echo WSL2 IP Address: %WSL_IP%
echo.

echo Removing existing port forwarding rules...
netsh interface portproxy delete v4tov4 listenport=9090 listenaddress=0.0.0.0

echo Adding new port forwarding rule...
netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 connectport=9090 connectaddress=%WSL_IP%

echo Configuring Windows Firewall...
netsh advfirewall firewall add rule name="WSL2 Web Monitor" dir=in action=allow protocol=TCP localport=9090

echo.
echo ========================================
echo Port forwarding setup completed!
echo ========================================
echo.
echo You can now access web_monitor at:
echo - Windows Browser: http://localhost:9090
echo - WSL2 Internal: http://%WSL_IP%:9090
echo.
echo WSL2 IP: %WSL_IP%
echo.
pause
