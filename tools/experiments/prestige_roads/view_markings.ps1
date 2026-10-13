<#
.SYNOPSIS
Open the terrain-paint prototype or its depth-clamped decal comparison.
.DESCRIPTION
The mask backend is the current ground-paint candidate. The decal backend is a diagnostic baseline
with known close-range attachment failures. Initializes osgearth_shell.bat; does not alter source data.
#>
param(
    [ValidateSet('boston', 'nyc', 'london')][string]$City = 'boston',
    [ValidateSet('intersection', 'detail', 'grazing', 'overview')][string]$View = 'intersection',
    [ValidateSet('mask', 'decal')][string]$Backend = 'mask',
    [ValidateSet(8, 32)][int]$TerrainDetail = 8,
    [switch]$GL4,
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..')
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_viewer.exe'
$folder = Join-Path $PSScriptRoot "output/render-fixture-$City/marking-comparison"
$scene = Join-Path $folder "$Backend-$View-$TerrainDetail.earth"
foreach ($path in @($setup, $viewer, $scene, (Join-Path $folder 'manifest.json'))) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing $path" }
}
if (Test-Path -LiteralPath (Join-Path $folder '../build-in-progress.json')) {
    throw 'The vector fixture is being rebuilt; wait for compilation to finish.'
}
$viewerOptions = '--window 100 100 1200 1000 --samples 4 --logdepth2 '
$viewerOptions += '--manip-min-distance 1 --manip-terrain-avoidance false'
if ($GL4) { $viewerOptions += ' --gl4' }
& cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$scene`" $viewerOptions"
if ($LASTEXITCODE -ne 0) { throw "osgearth_viewer exited with code $LASTEXITCODE" }
