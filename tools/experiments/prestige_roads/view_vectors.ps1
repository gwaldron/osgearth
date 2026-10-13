<#
.SYNOPSIS
Open a freshly compiled road-vector fixture in osgEarth.
.DESCRIPTION
Requires a successful vector_patch.py run and an installed runtime. Initializes osgearth_shell.bat;
does not build, fetch source, or reuse the earlier metro render checkpoint.
#>
param(
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..'),
    [ValidateSet('overview', 'intersection', 'bridge')][string]$View = 'overview',
    [ValidateSet('current', 'boston', 'nyc', 'london')][string]$Fixture = 'current'
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_viewer.exe'
$outputName = if ($Fixture -eq 'current') { 'boston-vectors-current' } else { "render-fixture-$Fixture" }
$output = Join-Path $PSScriptRoot "output/$outputName"
if (Test-Path -LiteralPath (Join-Path $output 'build-in-progress.json')) {
    throw 'The vector build is incomplete. Finish vector_patch.py before viewing.'
}
$mapFile = Join-Path $output "$View.earth"
foreach ($inputFile in @($setup, $viewer, $mapFile, (Join-Path $output 'manifest.json'))) {
    if (-not (Test-Path -LiteralPath $inputFile -PathType Leaf)) { throw "Missing $inputFile" }
}
# Empty material collections are valid compiler output but have no extent for an OGR feature layer.
# Omit their display nodes in a derived scene; preserve authoritative vectors and the generated source scene.
$manifest = Get-Content -LiteralPath (Join-Path $output 'manifest.json') -Raw | ConvertFrom-Json
$emptySources = @()
foreach ($material in @('asphalt', 'curb', 'gutter', 'white', 'yellow')) {
    if ($manifest.materials.$material.polygons -eq 0) { $emptySources += "$material-source" }
    if ($material -in @('asphalt', 'white', 'yellow') -and -not $manifest.structures.materials.$material) {
        $emptySources += "structure-$material-source"
    }
}
if ($emptySources.Count -gt 0) {
    [xml]$inspectionScene = Get-Content -LiteralPath $mapFile -Raw
    foreach ($element in @($inspectionScene.map.ChildNodes)) {
        if (($element.LocalName -eq 'OGRFeatures' -and $element.GetAttribute('name') -in $emptySources) -or
            ($element.LocalName -eq 'FeatureModel' -and $element.GetAttribute('features') -in $emptySources)) {
            $null = $inspectionScene.map.RemoveChild($element)
        }
    }
    $mapFile = Join-Path $output "inspection-$View.earth"
    $inspectionScene.Save($mapFile)
}
$viewerOptions = '--window 100 100 1200 1000 --samples 4 --logdepth2 '
$viewerOptions += '--manip-min-distance 1 --manip-terrain-avoidance false'
& cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$mapFile`" $viewerOptions"
if ($LASTEXITCODE -ne 0) { throw "osgearth_viewer exited with code $LASTEXITCODE" }
