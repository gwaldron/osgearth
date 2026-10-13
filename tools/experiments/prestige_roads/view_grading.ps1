<#
.SYNOPSIS
Show the compiled Boston junction before or after Prestige polygon flattening.
.DESCRIPTION
Initializes the repository runtime and loads shared asphalt/paint above the selected elevation stack.
#>
param(
    [ValidateSet('overview', 'close', 'top')][string]$View = 'overview',
    [switch]$Before,
    [switch]$GL4,
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..')
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$folder = Join-Path $PSScriptRoot 'output/grading-boston'
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
