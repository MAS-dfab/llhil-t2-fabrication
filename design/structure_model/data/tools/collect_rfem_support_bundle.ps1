param(
    [string]$WorkspaceRoot = "c:\Users\Juste\Documents\_facade_tools",
    [string]$OutputRoot = "",
    [int]$RecentDays = 5
)

$ErrorActionPreference = "Stop"

function Write-Info([string]$msg) {
    Write-Host "[rfem-support] $msg"
}

function Safe-Run([scriptblock]$block, [string]$fallback = "") {
    try {
        & $block
    }
    catch {
        if ($fallback -ne "") {
            $fallback
        }
        else {
            "ERROR: $($_.Exception.Message)"
        }
    }
}

function Add-TextSection([string]$path, [string]$title, [scriptblock]$producer) {
    Add-Content -Path $path -Value ""
    Add-Content -Path $path -Value "===== $title ====="
    $content = Safe-Run $producer
    if ($null -eq $content) {
        Add-Content -Path $path -Value "(no output)"
    }
    else {
        Add-Content -Path $path -Value ($content | Out-String)
    }
}

$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
if (-not $OutputRoot) {
    $OutputRoot = Join-Path $WorkspaceRoot "outputs"
}

$bundleName = "rfem_support_bundle_$timestamp"
$bundleDir = Join-Path $OutputRoot $bundleName
$zipPath = "$bundleDir.zip"

New-Item -ItemType Directory -Path $bundleDir -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $bundleDir "evidence") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $bundleDir "logs") -Force | Out-Null

$summaryPath = Join-Path $bundleDir "evidence\summary.txt"
Set-Content -Path $summaryPath -Value "RFEM Support Bundle"
Add-Content -Path $summaryPath -Value "Generated: $(Get-Date -Format o)"
Add-Content -Path $summaryPath -Value "Machine: $env:COMPUTERNAME"
Add-Content -Path $summaryPath -Value "User: $env:USERNAME"
Add-Content -Path $summaryPath -Value "Workspace: $WorkspaceRoot"

$rhinoPluginRoot = Join-Path $env:LOCALAPPDATA "McNeel\Rhinoceros\8.0\Dlubal-RFEM6-plugin"
$rhinoPluginLogsRoot = Join-Path $rhinoPluginRoot "log"
$rfemRoot = Join-Path $env:LOCALAPPDATA "Dlubal\RFEM6_6.13"
$rfemLogsRoot = Join-Path $rfemRoot "logs"
$packageRoot = Join-Path $env:APPDATA "McNeel\Rhinoceros\packages\8.0\Dlubal-plugin"

$configCandidates = @()
if (Test-Path $packageRoot) {
    $configCandidates = Get-ChildItem -Path $packageRoot -Recurse -File -Filter "Dlubal-plugin.rhp.config" -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending
}
$configPath = if ($configCandidates.Count -gt 0) { $configCandidates[0].FullName } else { "" }

$endpoint = ""
$isModelEndpointAddress = ""
if ($configPath -and (Test-Path $configPath)) {
    try {
        [xml]$cfg = Get-Content $configPath
        $endpoint = ($cfg.configuration.appSettings.add | Where-Object { $_.key -eq "EndpointAddress" }).value
        $isModelEndpointAddress = ($cfg.configuration.appSettings.add | Where-Object { $_.key -eq "IsModelEndpointAddress" }).value
    }
    catch {
        $endpoint = "ERROR: $($_.Exception.Message)"
    }
}

Add-TextSection $summaryPath "Versions" {
    $rfemProc = Get-Process RFEM6 -ErrorAction SilentlyContinue | Select-Object -First 1
    $rhinoProc = Get-Process Rhino -ErrorAction SilentlyContinue | Select-Object -First 1
    $items = @()
    if ($rfemProc) {
        $items += "RFEM6 Path: $($rfemProc.Path)"
        $items += "RFEM6 FileVersion: $((Get-Item $rfemProc.Path).VersionInfo.FileVersion)"
        $items += "RFEM6 ProductVersion: $((Get-Item $rfemProc.Path).VersionInfo.ProductVersion)"
    }
    else {
        $items += "RFEM6 process not running"
    }

    if ($rhinoProc) {
        $items += "Rhino Path: $($rhinoProc.Path)"
        $items += "Rhino FileVersion: $((Get-Item $rhinoProc.Path).VersionInfo.FileVersion)"
        $items += "Rhino ProductVersion: $((Get-Item $rhinoProc.Path).VersionInfo.ProductVersion)"
    }
    else {
        $items += "Rhino process not running"
    }

    if ($configPath) {
        $items += "Plugin Config: $configPath"
    }
    else {
        $items += "Plugin Config: not found"
    }

    $items -join "`n"
}

Add-TextSection $summaryPath "Endpoint Config" {
    @(
        "EndpointAddress: $endpoint",
        "IsModelEndpointAddress: $isModelEndpointAddress"
    ) -join "`n"
}

Add-TextSection $summaryPath "WinHTTP Proxy" {
    netsh winhttp show proxy
}

Add-TextSection $summaryPath "Proxy Environment Variables" {
    Get-ChildItem Env: |
        Where-Object { $_.Name -match "HTTP_PROXY|HTTPS_PROXY|NO_PROXY|ALL_PROXY" } |
        Sort-Object Name |
        Format-Table -AutoSize
}

Add-TextSection $summaryPath "Port 8082 Listener" {
    $listener = Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue |
        Where-Object { $_.LocalPort -eq 8082 } |
        Select-Object -First 1

    if (-not $listener) {
        "No listener on port 8082"
    }
    else {
        $proc = Get-Process -Id $listener.OwningProcess -ErrorAction SilentlyContinue
        [pscustomobject]@{
            LocalAddress = $listener.LocalAddress
            LocalPort = $listener.LocalPort
            PID = $listener.OwningProcess
            ProcessName = $proc.ProcessName
            ProcessPath = $proc.Path
        } | Format-List
    }
}

Add-TextSection $summaryPath "Endpoint Probes" {
    if (-not $endpoint) {
        "No endpoint found in config"
    }
    else {
        $urls = @($endpoint, "$endpoint/wsdl", "http://localhost:8082/wsdl", "http://127.0.0.1:8082/wsdl")
        $lines = @()
        foreach ($u in $urls | Select-Object -Unique) {
            try {
                $r = Invoke-WebRequest -Uri $u -Method Get -TimeoutSec 6 -UseBasicParsing
                $lines += "$u => GET $($r.StatusCode), len=$($r.Content.Length)"
            }
            catch {
                $code = "n/a"
                if ($_.Exception.Response -and $_.Exception.Response.StatusCode) {
                    $code = [int]$_.Exception.Response.StatusCode
                }
                $lines += "$u => FAILED (status=$code): $($_.Exception.Message)"
            }
        }
        $lines -join "`n"
    }
}

if ($configPath -and (Test-Path $configPath)) {
    Copy-Item -Path $configPath -Destination (Join-Path $bundleDir "evidence\Dlubal-plugin.rhp.config") -Force
}

# Copy Rhino plugin logs from recent days.
if (Test-Path $rhinoPluginLogsRoot) {
    $target = Join-Path $bundleDir "logs\rhino_plugin"
    New-Item -ItemType Directory -Path $target -Force | Out-Null

    $cutoff = (Get-Date).Date.AddDays(-[Math]::Abs($RecentDays))
    Get-ChildItem -Path $rhinoPluginLogsRoot -Directory -ErrorAction SilentlyContinue |
        Where-Object {
            if ($_.Name -notmatch '^\d{4}-\d{2}-\d{2}$') {
                return $false
            }
            $folderDate = $null
            try {
                $folderDate = Get-Date $_.Name
            }
            catch {
                return $false
            }
            $folderDate.Date -ge $cutoff
        } |
        ForEach-Object {
            Copy-Item -Path $_.FullName -Destination $target -Recurse -Force -ErrorAction SilentlyContinue
        }
}

# Copy RFEM logs and WSDL files.
if (Test-Path $rfemLogsRoot) {
    $target = Join-Path $bundleDir "logs\rfem"
    New-Item -ItemType Directory -Path $target -Force | Out-Null
    Get-ChildItem -Path $rfemLogsRoot -File -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 20 |
        ForEach-Object {
            Copy-Item -Path $_.FullName -Destination $target -Force -ErrorAction SilentlyContinue
        }

    $apiLogs = Join-Path $rfemLogsRoot "api_logs"
    if (Test-Path $apiLogs) {
        Copy-Item -Path $apiLogs -Destination (Join-Path $target "api_logs") -Recurse -Force -ErrorAction SilentlyContinue
    }
}

$rfemWsdlRoot = Join-Path $rfemRoot "wsdl"
if (Test-Path $rfemWsdlRoot) {
    Copy-Item -Path $rfemWsdlRoot -Destination (Join-Path $bundleDir "evidence\wsdl") -Recurse -Force -ErrorAction SilentlyContinue
}

Add-TextSection $summaryPath "Recent Rhino Plugin Log Files" {
    if (Test-Path $rhinoPluginLogsRoot) {
        Get-ChildItem -Path $rhinoPluginLogsRoot -Recurse -File -Filter "*.log" -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending |
            Select-Object -First 30 FullName, LastWriteTime |
            Format-Table -AutoSize
    }
    else {
        "Rhino plugin log directory not found: $rhinoPluginLogsRoot"
    }
}

Add-TextSection $summaryPath "Recent RFEM Log Files" {
    if (Test-Path $rfemLogsRoot) {
        Get-ChildItem -Path $rfemLogsRoot -File -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending |
            Select-Object -First 30 FullName, LastWriteTime |
            Format-Table -AutoSize
    }
    else {
        "RFEM logs directory not found: $rfemLogsRoot"
    }
}

if (Test-Path $zipPath) {
    Remove-Item -Path $zipPath -Force
}
Compress-Archive -Path (Join-Path $bundleDir "*") -DestinationPath $zipPath -Force

Write-Info "Bundle folder: $bundleDir"
Write-Info "Bundle zip   : $zipPath"
Write-Info "Summary file : $summaryPath"
