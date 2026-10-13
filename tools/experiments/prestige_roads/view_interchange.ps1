<#
.SYNOPSIS
Open a source-connected regional road package with graded ground roads and separate decks.
.DESCRIPTION
Uses the bounded generated package and inferred height solution; initializes osgEarth before launch.
#>
param(
    [ValidateSet('overview', 'stack', 'ramp', 'south', 'rhone', 'interchange', 'street', 'merge', 'railings',
        'bridge-cable-stayed', 'bridge-cable-anchors', 'bridge-truss', 'bridge-supports')]
    [string]$View = 'overview',
    [ValidateSet('springfield-interchange', 'sion-city')][string]$Scene = 'springfield-interchange',
    [switch]$GL4,
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..')
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$folder = Join-Path $PSScriptRoot "output/$Scene"
if (Test-Path -LiteralPath (Join-Path $folder 'build-in-progress.json')) {
    throw 'The interchange build is incomplete; finish validation and publication before launching.'
}
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_imgui.exe'
$projData = Join-Path $runtime 'build/vcpkg_installed/x64-windows-release/share/proj'
if (Test-Path -LiteralPath (Join-Path $projData 'proj.db')) { $env:PROJ_DATA = $projData }
$scenePath = Join-Path $folder "$View.earth"
foreach ($path in @($setup, $viewer, $scenePath, (Join-Path $folder 'manifest.json'))) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing $path" }
}
$flags = '--samples 4 --window 100 100 1400 1000 --logdepth2 --prestige-sky '
$flags += '--manip-min-distance 1 --manip-terrain-avoidance false'
if ($GL4) { $flags += ' --gl4' } else { $flags += ' --nvgl --ico' }
& cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$scenePath`" $flags"
if ($LASTEXITCODE -ne 0) { throw "osgearth_imgui exited with code $LASTEXITCODE" }
