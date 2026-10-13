<#
.SYNOPSIS
Build an indexed, disposable runtime copy of the attributed asphalt for each fixture.
.DESCRIPTION
GeoJSON remains authoritative. GDAL copies its polygons/attributes into a GeoPackage R-tree so each
terrain page can query a neighborhood without parsing the complete city again. No terrain is modified.
#>
param([ValidateSet('boston', 'nyc', 'london')][string[]]$City = @('boston', 'nyc', 'london'))
$ErrorActionPreference = 'Stop'
$runtime = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../../..')).Path
$setup = Join-Path $runtime 'osgearth_shell.bat'
$projData = Join-Path $runtime 'build/vcpkg_installed/x64-windows-release/share/proj'
$converter = Join-Path $runtime 'build/vcpkg_installed/x64-windows-release/tools/gdal/ogr2ogr.exe'
if (-not (Test-Path -LiteralPath $converter)) { throw 'The configured GDAL runtime is not installed.' }
foreach ($name in $City) {
    $folder = Join-Path $PSScriptRoot "output/render-fixture-$name/render-contract"
    $source = Join-Path $folder 'asphalt.geojson'
    $target = Join-Path $folder 'asphalt.gpkg'
    $stamp = Join-Path $folder 'runtime-index.json'
    $sourceHash = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash
    if ((Test-Path -LiteralPath $stamp) -and (Test-Path -LiteralPath $target)) {
        $previous = Get-Content -LiteralPath $stamp -Raw | ConvertFrom-Json
        if ($previous.source_sha256 -eq $sourceHash -and
            $previous.index_sha256 -eq (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash) {
            Write-Output "$name`: indexed vectors are current"
            continue
        }
    }
    $pending = Join-Path $folder 'asphalt-next.gpkg'
    if (Test-Path -LiteralPath $pending) { Remove-Item -LiteralPath $pending }
    $command = "call `"$setup`" && set `"PROJ_DATA=$projData`" && `"$converter`" "
    $command += "-f GPKG `"$pending`" `"$source`" -nln pavement -lco SPATIAL_INDEX=YES"
    & cmd.exe /d /c $command
    if ($LASTEXITCODE -ne 0) { throw "GDAL conversion failed for $name" }
    if ((Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash -ne $sourceHash) {
        throw 'Source changed during indexing; regenerate the index.'
    }
    Move-Item -LiteralPath $pending -Destination $target -Force
    @{
        source = 'asphalt.geojson'; source_sha256 = $sourceHash
        index = 'asphalt.gpkg'; index_sha256 = (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash
        policy = 'Disposable spatial index of unchanged GeoJSON; not a new compiler interchange format'
    } | ConvertTo-Json | Set-Content -LiteralPath $stamp -Encoding utf8
    Write-Output "$name`: indexed runtime vectors ready"
}
