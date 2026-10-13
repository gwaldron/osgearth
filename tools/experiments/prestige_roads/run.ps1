<#
.SYNOPSIS
Launch a generated Prestige Roads map after initializing an existing osgEarth runtime.
.DESCRIPTION
Does not build or download anything. Run compile.py first. RuntimeRoot must contain
osgearth_shell.bat and install/bin/osgearth_viewer.exe; missing inputs fail before launch.
#>
param(
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..'),
    [ValidateSet('', 'replacement', 'overlay', 'raw', 'graded')][string]$Mode = '',
    [ValidateSet('synthetic', 'readymap')][string]$Terrain = 'synthetic',
    [ValidateSet('fit', 'drape', 'design')][string]$Surface = 'fit',
    [switch]$City,
    [ValidateSet(2, 8, 32)][int]$SSE = 8,
    [ValidateSet('alewife', 'manhattan', 'boston', 'pittsburgh', 'london', 'paris', 'sion',
        'boston-region', 'pittsburgh-region')][string]$Scene = 'alewife',
    [ValidatePattern('^[a-z0-9-]+$')][string]$View = 'overview'
)

$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_viewer.exe'
$outputPath = if ($Scene -eq 'alewife') { 'output' } else { "output/$Scene" }
if ($Terrain -eq 'readymap') { $outputPath = "output/$Scene-readymap-$Surface" }
if (-not $Mode) {
    $Mode = if ($Terrain -eq 'synthetic') { 'replacement' } elseif ($Surface -eq 'drape') { 'raw' } else { 'graded' }
}
$prefix = if ($City) { 'city-' } else { '' }
$mapFile = Join-Path $PSScriptRoot "$outputPath/$prefix$Mode-sse$SSE-$View.earth"
$viewFile = Join-Path $PSScriptRoot "$outputPath/${prefix}viewpoints.json"
if (Test-Path -LiteralPath (Join-Path $PSScriptRoot "$outputPath/vertical-failure.json")) {
    throw 'The latest vertical design failed. Inspect vertical-failure.json before launching.'
}
if (Test-Path -LiteralPath (Join-Path $PSScriptRoot "$outputPath/surface-failure.json")) {
    throw 'The latest terrain fit failed its geometry checks. Inspect surface-failure.json before launching.'
}
if (-not (Test-Path -LiteralPath $viewFile -PathType Leaf)) {
    throw "Missing $viewFile. Compile the experiment before selecting a view."
}
$activeViews = Get-Content -LiteralPath $viewFile -Raw | ConvertFrom-Json
if ($Mode -notin $activeViews.modes) {
    throw "Mode '$Mode' is absent from $viewFile. Check Terrain and Surface."
}
if ($View -notin $activeViews.views) {
    throw "View '$View' is absent from the latest successful compilation. See $viewFile for current views."
}
foreach ($inputFile in @($setup, $viewer, $mapFile)) {
    if (-not (Test-Path -LiteralPath $inputFile -PathType Leaf)) {
        throw "Missing $inputFile. Compile the experiment and select an installed osgEarth runtime."
    }
}

Write-Host "Prestige Roads: $Scene, $Mode, SSE $SSE, $View"
Write-Host "OSM layout, $Terrain terrain, $Surface surface; see README.md and REAL_TERRAIN.md."
$viewerOptions = '--window 100 100 1440 960 --samples 4 --shadows --logdepth2 '
$viewerOptions += '--manip-max-pitch 30 --manip-min-distance 1 --manip-terrain-avoidance false'
& cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$mapFile`" $viewerOptions"
if ($LASTEXITCODE -ne 0) {
    throw "osgearth_viewer exited with code $LASTEXITCODE."
}
