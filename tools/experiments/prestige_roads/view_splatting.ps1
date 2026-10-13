<#
.SYNOPSIS
Open one of the fixed asphalt-on-terrain fixtures.
.DESCRIPTION
Uses the installed Prestige runtime and generated scenes. Initializes osgearth_shell.bat;
does not compile vectors, build the renderer, or modify elevation.
#>
param(
    [ValidateSet('boston', 'nyc', 'london')][string]$City = 'boston',
    [ValidateSet('overview', 'intersection', 'detail', 'grazing')][string]$View = 'intersection',
    [ValidateSet('splat', 'coverage')][string]$Mode = 'splat',
    [ValidateSet(8, 32)][int]$TerrainDetail = 8,
    [switch]$GL4,
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..')
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_viewer.exe'
$output = Join-Path $PSScriptRoot "output/render-fixture-$City"
$scene = Join-Path $output "$Mode-$View-$TerrainDetail.earth"
foreach ($path in @($setup, $viewer, $scene, (Join-Path $output 'render-contract/contract.json'))) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing $path" }
}
if (Test-Path -LiteralPath (Join-Path $output 'build-in-progress.json')) {
    throw 'The vector fixture is being rebuilt; wait for compilation to finish.'
}
$viewerOptions = '--window 100 100 1200 1000 --samples 4 --logdepth2 '
$viewerOptions += '--manip-min-distance 1 --manip-terrain-avoidance false'
if ($GL4) { $viewerOptions += ' --gl4' }
& cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$scene`" $viewerOptions"
if ($LASTEXITCODE -ne 0) { throw "osgearth_viewer exited with code $LASTEXITCODE" }
