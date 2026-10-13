<#
.SYNOPSIS
View Boston roads on stock gridded terrain, with or without the road elevation overlay.
.DESCRIPTION
Uses an existing installed runtime and generated data. Initializes osgearth_shell.bat before launch;
missing inputs and nonzero viewer exits fail explicitly. Does not build or fetch data.
#>
param(
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..'),
    [ValidateSet('graded', 'raw')][string]$Mode = 'graded',
    [ValidateSet('fine', 'coarse')][string]$Detail = 'fine',
    [ValidateSet('overview', 'junction', 'street')][string]$View = 'junction'
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_viewer.exe'
$output = Join-Path $PSScriptRoot 'output/boston-gridded-roads'
$mapFile = Join-Path $output "$Mode-$Detail-$View.earth"
if (Test-Path -LiteralPath (Join-Path $output 'build-in-progress.json')) {
    throw 'The grid build is incomplete. Finish grid_roads.py before viewing.'
}
foreach ($inputFile in @($setup, $viewer, $mapFile, (Join-Path $output 'audit.json'))) {
    if (-not (Test-Path -LiteralPath $inputFile -PathType Leaf)) { throw "Missing $inputFile" }
}
$previousAssets = $env:OSG_FILE_PATH
try {
    $env:OSG_FILE_PATH = (Join-Path $runtime 'data') + ';' + $previousAssets
    $viewerOptions = '--window 100 100 1440 960 --samples 4 --logdepth2 '
    $viewerOptions += '--manip-min-distance 1 --manip-terrain-avoidance false'
    & cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$mapFile`" $viewerOptions"
    if ($LASTEXITCODE -ne 0) { throw "osgearth_viewer exited with code $LASTEXITCODE" }
}
finally {
    $env:OSG_FILE_PATH = $previousAssets
}
