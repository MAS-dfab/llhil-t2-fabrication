param(
    [ValidateSet('snapshot','disable','restore','prune-old-versions')]
    [string]$Action = 'snapshot',

    [ValidateSet('light','moderate','aggressive','custom')]
    [string]$Profile = 'moderate',

    [string[]]$Packages = @(),

    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'

$packageRoot = Join-Path $env:APPDATA 'McNeel\Rhinoceros\packages\8.0'
$quarantineRoot = Join-Path $packageRoot '_disabled_by_isolation'
$snapshotFile = Join-Path $packageRoot '_isolation_snapshot.json'

if (-not (Test-Path $packageRoot)) {
    throw "Rhino package root not found: $packageRoot"
}

$profiles = @{
    light = @(
        'WombatGH','Parakeet','CurvePlus','treeFrog','horster-cameraControl','RhinoPolyhedra'
    )
    moderate = @(
        'WombatGH','Parakeet','CurvePlus','treeFrog','horster-cameraControl','RhinoPolyhedra',
        'TTToolbox','Pufferfish','Weaverbird','human','Heteroptera','MeshEdit-Components'
    )
    aggressive = @(
        'WombatGH','Parakeet','CurvePlus','treeFrog','horster-cameraControl','RhinoPolyhedra',
        'TTToolbox','Pufferfish','Weaverbird','human','Heteroptera','MeshEdit-Components',
        'Robots','ngon','Dlubal-plugin','Parametric_FEM_Toolbox','opossum','Karamba3D',
        'COMPAS-FormFinder','compas_fab','compas_timber','compas','Ant'
    )
}

function Get-PackageFolders {
    Get-ChildItem $packageRoot -Directory |
        Where-Object { $_.Name -notlike '_disabled_by_isolation' } |
        Sort-Object Name
}

function Ensure-Quarantine {
    if (-not (Test-Path $quarantineRoot)) {
        if ($DryRun) {
            Write-Host "[dry-run] would create $quarantineRoot"
        } else {
            New-Item -ItemType Directory -Path $quarantineRoot | Out-Null
            Write-Host "created $quarantineRoot"
        }
    }
}

function Save-Snapshot {
    $rows = Get-PackageFolders | ForEach-Object {
        $versions = Get-ChildItem $_.FullName -Directory -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match '^[0-9]+' } |
            Select-Object -ExpandProperty Name
        [PSCustomObject]@{
            package = $_.Name
            fullPath = $_.FullName
            versionCount = ($versions | Measure-Object).Count
            versions = $versions
            lastWriteTime = $_.LastWriteTime
        }
    }
    $json = $rows | ConvertTo-Json -Depth 4

    if ($DryRun) {
        Write-Host "[dry-run] would write snapshot to $snapshotFile"
    } else {
        Set-Content -Path $snapshotFile -Value $json -Encoding UTF8
        Write-Host "snapshot written: $snapshotFile"
    }

    $rows | Format-Table package,versionCount,@{n='versions';e={$_.versions -join ', '}} -AutoSize
}

function Resolve-TargetPackages {
    if ($Profile -eq 'custom') {
        return $Packages
    }
    return $profiles[$Profile]
}

function Disable-Packages {
    Ensure-Quarantine
    $targets = Resolve-TargetPackages
    if (-not $targets -or $targets.Count -eq 0) {
        throw 'No target packages resolved.'
    }

    foreach ($name in $targets) {
        $src = Join-Path $packageRoot $name
        if (-not (Test-Path $src)) {
            Write-Host "skip (not installed): $name"
            continue
        }

        $stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
        $dst = Join-Path $quarantineRoot ("{0}__{1}" -f $name, $stamp)

        if ($DryRun) {
            Write-Host "[dry-run] move $src -> $dst"
        } else {
            Move-Item -Path $src -Destination $dst
            Write-Host "moved $name -> quarantine"
        }
    }
}

function Restore-Packages {
    if (-not (Test-Path $quarantineRoot)) {
        Write-Host 'nothing to restore'
        return
    }

    $entries = Get-ChildItem $quarantineRoot -Directory | Sort-Object LastWriteTime
    foreach ($entry in $entries) {
        $base = ($entry.Name -split '__')[0]
        $dst = Join-Path $packageRoot $base

        if (Test-Path $dst) {
            Write-Host "skip restore (already exists): $base"
            continue
        }

        if ($DryRun) {
            Write-Host "[dry-run] restore $($entry.FullName) -> $dst"
        } else {
            Move-Item -Path $entry.FullName -Destination $dst
            Write-Host "restored $base"
        }
    }
}

function Prune-OldVersions {
    Ensure-Quarantine
    $folders = Get-PackageFolders

    foreach ($pkg in $folders) {
        $versions = Get-ChildItem $pkg.FullName -Directory -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match '^[0-9]+' } |
            Sort-Object Name

        if ($versions.Count -le 1) {
            continue
        }

        $keep = $versions[-1]
        $remove = $versions[0..($versions.Count - 2)]
        Write-Host "package: $($pkg.Name) keep: $($keep.Name) remove: $($remove.Name -join ', ')"

        foreach ($old in $remove) {
            $stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
            $dst = Join-Path $quarantineRoot ("{0}__oldver__{1}__{2}" -f $pkg.Name, $old.Name, $stamp)
            if ($DryRun) {
                Write-Host "[dry-run] move $($old.FullName) -> $dst"
            } else {
                Move-Item -Path $old.FullName -Destination $dst
                Write-Host "moved old version: $($pkg.Name) $($old.Name)"
            }
        }
    }
}

Write-Host "Action=$Action Profile=$Profile DryRun=$DryRun"
Write-Host "packageRoot=$packageRoot"
Write-Host "quarantineRoot=$quarantineRoot"

switch ($Action) {
    'snapshot' { Save-Snapshot }
    'disable' { Disable-Packages }
    'restore' { Restore-Packages }
    'prune-old-versions' { Prune-OldVersions }
}
