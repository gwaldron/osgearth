<#
.SYNOPSIS
Open the Boston geodetic DDS pyramid over ReadyMap in an installed osgEarth runtime.
.DESCRIPTION
Initializes osgearth_shell.bat before launching. The pyramid must already be compiled.
#>
param(
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..'),
    [ValidateSet('overview', 'back-bay', 'intersection', 'cambridge')][string]$View = 'back-bay'
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_viewer.exe'
$mapFile = Join-Path $PSScriptRoot "output/boston-metro-raster/boston-xyz-$View.earth"
$manifest = Join-Path $PSScriptRoot 'output/boston-metro-raster/tile-manifest.json'
if (Test-Path -LiteralPath (Join-Path $PSScriptRoot 'output/boston-metro-raster/tile-build-in-progress.json')) {
    throw 'The tile bake is incomplete. Finish the raster_roads.py tiles stage before viewing.'
}
foreach ($inputFile in @($setup, $viewer, $mapFile, $manifest)) {
    if (-not (Test-Path -LiteralPath $inputFile -PathType Leaf)) { throw "Missing $inputFile" }
}
& cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$mapFile`" --window 100 100 1440 960 --samples 4 --logdepth2"
if ($LASTEXITCODE -ne 0) { throw "osgearth_viewer exited with code $LASTEXITCODE" }
