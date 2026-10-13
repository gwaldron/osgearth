<#
.SYNOPSIS
Open the Congress Street bridge slice with shared ground/deck materials.
.DESCRIPTION
Uses the retained inferred height profile, initializes osgEarth's runtime, and leaves elevation unchanged.
Fixed views establish the first attachment baseline. Dynamic views expose the unfinished LOD attachment behavior.
#>
param(
    [ValidateSet('overview', 'west-landing', 'east-landing', 'deck', 'underside')][string]$View = 'overview',
    [ValidateSet('fixed', 'dynamic', 'context')][string]$Mode = 'fixed',
    [switch]$GL4,
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..')
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$folder = Join-Path $PSScriptRoot 'output/bridge-boston-congress'
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_viewer.exe'
$scene = Join-Path $folder "$Mode-$View.earth"
if (Test-Path -LiteralPath (Join-Path $folder 'build-in-progress.json')) {
    throw 'The bridge output is incomplete; finish its build before launching.'
}
foreach ($path in @($setup, $viewer, $scene, (Join-Path $folder 'manifest.json'))) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing $path" }
}
$viewerOptions = '--window 100 100 1200 1000 --samples 4 --logdepth2 '
$viewerOptions += '--manip-min-distance 1 --manip-terrain-avoidance false'
if ($GL4) { $viewerOptions += ' --gl4' }
& cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$scene`" $viewerOptions"
if ($LASTEXITCODE -ne 0) { throw "osgearth_viewer exited with code $LASTEXITCODE" }
