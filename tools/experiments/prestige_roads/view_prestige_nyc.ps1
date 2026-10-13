<#
.SYNOPSIS
Show NYC graded roads with Prestige buildings, vegetation, and sky.
.DESCRIPTION
Uses the generated NYC fixture packages and initializes the repository runtime.
#>
param(
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot '../../..')
)
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$setup = Join-Path $runtime 'osgearth_shell.bat'
$viewer = Join-Path $runtime 'install/bin/osgearth_imgui.exe'
$scene = Join-Path $PSScriptRoot '../../../tests/prestige_roads_nyc.earth'
foreach ($path in @($setup, $viewer, $scene)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing $path" }
}
& cmd.exe /d /c "call `"$setup`" && `"$viewer`" `"$scene`" --nvgl --ico --prestige-sky --samples 4 --logdepth2"
if ($LASTEXITCODE -ne 0) { throw "osgearth_imgui exited with code $LASTEXITCODE" }
