param(
    [string]$ConfigPath = "$env:APPDATA\McNeel\Rhinoceros\packages\8.0\Dlubal-plugin\8.612.12.1\Dlubal-plugin.rhp.config"
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path $ConfigPath)) {
    Write-Host "Config not found: $ConfigPath"
    exit 1
}

[xml]$cfg = Get-Content $ConfigPath
$endpoint = ($cfg.configuration.appSettings.add | Where-Object { $_.key -eq 'EndpointAddress' }).value
if (-not $endpoint) {
    Write-Host "EndpointAddress not found in config."
    exit 1
}

$uri = [Uri]$endpoint
$port = $uri.Port

Write-Host "Configured endpoint: $endpoint"

$listener = Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue |
    Where-Object { $_.LocalPort -eq $port }

if (-not $listener) {
    Write-Host "Status: NOT READY (nothing listening on port $port)"
    exit 2
}

$proc = Get-Process -Id ($listener | Select-Object -First 1).OwningProcess -ErrorAction SilentlyContinue
if ($proc) {
    Write-Host "Listener: $($proc.ProcessName) (PID $($proc.Id))"
}

try {
    $r = Invoke-WebRequest -Uri $endpoint -Method Head -TimeoutSec 4 -UseBasicParsing
    Write-Host "HTTP probe: $($r.StatusCode)"
    Write-Host "Status: READY"
    exit 0
} catch {
    $statusCode = $null
    if ($_.Exception.Response -and $_.Exception.Response.StatusCode) {
        $statusCode = [int]$_.Exception.Response.StatusCode
    }

    if ($statusCode -in 401,403) {
        Write-Host "HTTP probe: $statusCode (auth required)"
        Write-Host "Status: READY"
        exit 0
    }

    Write-Host "HTTP probe failed: $($_.Exception.Message)"
    Write-Host "Status: READY (listener detected; protocol may be non-standard for HEAD probe)"
    exit 0
}
