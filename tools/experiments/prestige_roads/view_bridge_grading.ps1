<#
.SYNOPSIS
Open the Congress Street bridge and approaches driven by one shared elevation profile.
.DESCRIPTION
Initializes osgEarth's runtime and loads a separate deck above sparse native-LOD-16 approach grading.
The profile and support footprints are explicit demo assumptions, not surveyed bridge engineering.
#>
param(
    [ValidateSet('overview', 'west-landing', 'east-landing', 'deck')][string]$View = 'overview',
    [switch]$GL4,
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..')
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$folder = Join-Path $PSScriptRoot 'output/bridge-grading-boston'
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_viewer.exe'
$scene = Join-Path $folder "$View.earth"
if (Test-Path -LiteralPath (Join-Path $folder 'build-in-progress.json')) {
    throw 'The shared-profile bridge output is incomplete; finish its build before launching.'
}
foreach ($path in @($setup, $viewer, $scene, (Join-Path $folder 'manifest.json'))) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing $path" }
}
$viewerOptions = '--window 100 100 1200 1000 --samples 4 --logdepth2 '
$viewerOptions += '--manip-min-distance 1 --manip-terrain-avoidance false'
if ($GL4) { $viewerOptions += ' --gl4' }
& cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$scene`" $viewerOptions"
if ($LASTEXITCODE -ne 0) { throw "osgearth_viewer exited with code $LASTEXITCODE" }
