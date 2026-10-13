<#
.SYNOPSIS
Show a fixed city fixture with centerline-driven grading, asphalt, and markings.
.DESCRIPTION
Initializes the repository runtime; source LOD 16 grading is reused by finer regular terrain.
#>
param(
    [ValidateSet('boston', 'nyc', 'london')][string]$City = 'boston',
    [ValidateSet('overview', 'close', 'top')][string]$View = 'close',
    [switch]$Before,
    [switch]$GL4,
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..')
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$folder = Join-Path $PSScriptRoot "output/road-grading-$City"
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_viewer.exe'
$mode = if ($Before) { 'before' } else { 'after' }
$scene = Join-Path $folder "$mode-$View.earth"
foreach ($path in @($setup, $viewer, $scene, (Join-Path $folder 'manifest.json'))) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing $path" }
}
$viewerOptions = '--window 100 100 1200 1000 --samples 4 --logdepth2 '
$viewerOptions += '--manip-min-distance 1 --manip-terrain-avoidance false'
if ($GL4) { $viewerOptions += ' --gl4' }
& cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$scene`" $viewerOptions"
if ($LASTEXITCODE -ne 0) { throw "osgearth_viewer exited with code $LASTEXITCODE" }
