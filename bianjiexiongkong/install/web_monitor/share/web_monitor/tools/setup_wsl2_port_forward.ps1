# WSL2 Port Forward Setup for Web Monitor
# Run this script as Administrator

Write-Host "========================================" -ForegroundColor Green
Write-Host "WSL2 Port Forward Setup for Web Monitor" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""

# Check if running as Administrator
if (-NOT ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {
    Write-Host "This script requires Administrator privileges. Please run as Administrator." -ForegroundColor Red
    Write-Host "Right-click on PowerShell and select 'Run as Administrator'" -ForegroundColor Yellow
    pause
    exit
}

Write-Host "Getting WSL2 IP address..." -ForegroundColor Yellow
try {
    $WSL_IP = (wsl hostname -I).Trim().Split()[0]
    Write-Host "WSL2 IP Address: $WSL_IP" -ForegroundColor Green
} catch {
    Write-Host "Failed to get WSL2 IP address. Make sure WSL2 is running." -ForegroundColor Red
    pause
    exit
}

Write-Host ""
Write-Host "Removing existing port forwarding rules..." -ForegroundColor Yellow
try {
    netsh interface portproxy delete v4tov4 listenport=9090 listenaddress=0.0.0.0 2>$null
    Write-Host "Existing rules removed." -ForegroundColor Green
} catch {
    Write-Host "No existing rules to remove." -ForegroundColor Yellow
}

Write-Host "Adding new port forwarding rule..." -ForegroundColor Yellow
try {
    netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 connectport=9090 connectaddress=$WSL_IP
    Write-Host "Port forwarding rule added successfully." -ForegroundColor Green
} catch {
    Write-Host "Failed to add port forwarding rule." -ForegroundColor Red
    pause
    exit
}

Write-Host "Configuring Windows Firewall..." -ForegroundColor Yellow
try {
    netsh advfirewall firewall add rule name="WSL2 Web Monitor" dir=in action=allow protocol=TCP localport=9090
    Write-Host "Firewall rule added successfully." -ForegroundColor Green
} catch {
    Write-Host "Failed to add firewall rule." -ForegroundColor Red
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "Port forwarding setup completed!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "You can now access web_monitor at:" -ForegroundColor Cyan
Write-Host "- Windows Browser: http://localhost:9090" -ForegroundColor White
Write-Host "- WSL2 Internal: http://$WSL_IP:9090" -ForegroundColor White
Write-Host ""
Write-Host "WSL2 IP: $WSL_IP" -ForegroundColor Yellow
Write-Host ""

# Show current port forwarding rules
Write-Host "Current port forwarding rules:" -ForegroundColor Cyan
netsh interface portproxy show all

Write-Host ""
Write-Host "Press any key to continue..." -ForegroundColor Gray
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")

