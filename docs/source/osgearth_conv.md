# osgearth_conv

**osgearth_conv** is a command-line tool for converting geospatial datasets into tiled formats optimized for fast loading, streaming, and portability.

## Motivation

*osgEarth* can load many different data formats for imagery and elevation. Internally, osgEarth chops geospatial into an **optimized format** -- a pyramid of multi-resolution tiles that enables you to view the data at any level of detail without requiring the entire dataset to be resident in memory.

If we do this optimization *offline*, osgEarth can skip this step and get right to the business of rendering. This means:

 * Faster load times. Pre-optimized datasets are ready to render without any runtime processing.
 * Streaming. A pre-optimized dataset may be exported as small, independent *tiles* that are easily streamed from a standard web server.
 * Portability. Large datasets may be tiled and stored in a single database file, combining the benefits of fast loading AND portability.

**osgearth_conv** does this. The tool will read, subdivide, and tile input datasets for better load-time performance and various deployment options.

## Examples

Here are some common usage examples. Later we will describe the various options in detail.

Convert a local **GeoTIFF** into a **TMS repository** of JPEG tiles:
```
osgearth_conv --in driver gdalimage       // read using the GDAL imagery driver
              --in url myinputfile.tif    // location of the GeoTIFF to read
              --out driver tmsimage       // export to a TMS imagery repository
              --out url output/tms.xml    // location of the TMS repository metadata file
              --out format jpg            // export each tile in JPEG format
```

Convert a local DTED elevation datafile into a local MBTiles database:
```
osgearth_conv --in driver gdalelevation       // read elevation data using GDAL
              --in url mydem.dt1              // location of the input file to read (DTED1)
              --in vdatum egm96               // tell the GDAL driver that DTED uses the EGM96 vertical dataum
              --out driver mbtileselevation   // write elevation data to an MBTiles database
              --out filename mydem.mbtiles    // location of the MBTiles database to write
              --out format tiff               // internal file format to use (must be tiff for elevation data)
```
Convert an online WMS (Web Map Service) repository to an offline MBTiles file:
```
osgearth_conv --in driver wmsimage
              --in url https://basemap.nationalmap.gov/arcgis/services/USGSImageryOnly/MapServer/WMSServer
              --in layers 0
              --in format jpeg
              --in srs EPSG:4326
              --out driver mbtilesimage
              --out filename ypgImagery.mbtiles
              --out format jpg
              --min-level 6
              --max-level 13
              --extents 33.020325 -114.604442 33.557478 -114.258021
```

## General usage

The usage template for **osgearth_conv** looks like this:

```
osgearth_conv --in driver [input driver]
              --in [input property] [value] ...
              --out driver [output driver]
              --out [output property] [value] ...
              [general properties] ...
```
**--in driver** designates the osgEarth layer type of the input data. Supported input drivers are:

 * `GDALImage` / `GDALElevation` - Read from any GDAL-supported local image file (like a GeoTIFF)
 * `WMSImage` - Read imagery from an OGC Web Map Service endpoint
 * `ArcGISServerImage` - Read imagery data from an ESRI ArcGIS server layer

**--out driver** designates to target format for optimized tiled data. Supported output drivers are:

 * `TMSImage` / `TMSElevation` - Create a TileMapService repository, useful for streaming data over the web with only a standard web server (like Apache)
 * `MBTilesImage` / `MBTilesElevation` - MapBox Tiles container format, this is a local SQLite database containing tiled geodata. These are useful when you want to put the tiled dataset in a single file for portability. However, you cannot stream from an MBTiles file without additional server software.

**PLEASE BE SURE** to specify an "image" driver for imagery data, and an "elevation" driver for heightfield data. They are not interchangable!

## Inputs

### GDALImage / GDALElevation

[GDAL](https://gdal.org/drivers/raster/) is a toolkit for reading many geospatial data formats. The most common one is **GeoTIFF**. 

| Property | Description |
|----------|-------------|
| --in url *path* | URL (file location) of the input data (e.g., a **GeoTIFF** file) |
| --in vdatum *string* | Optional vertical datum; use this if you need to tell GDAL which vertical datum the input data is using (e.g, `egm96`) |

Example: Convert a local GeoTIFF into an MBTiles database in PNG format (which supports transparency):
```
osgearth_conv --in driver gdalimage
              --in url mpdata.tif
              --out driver mbtilesimage
              --out filename mapdata.mbtiles
              --out format png
```

### WMSImage
This driver reads from an OGC [Web Map Service](https://www.ogc.org/standards/wms) endpoint.

| Property | Description |
|----------|-------------|
| --in url *string* | HTTP (or HTTPS) endpoint of the WMS service. Always required. |
| --in layers *string* | WMS layers for which to generate an image; corresponds to the WMS `LAYERS` parameter. This is usually required. |
| --in srs *string* | Spatial reference system of the data to retrieve. Examples are `wgs84` or `EPSG:4326`. Usually the server will have a default, but you can override that here. |
| --in format *string* | File format to return. Usually this will be a mime-type like `image/jpeg` or `image/png`; the supported formats will be available in the WMS capatilibies. |
| --in wms_version *string* | If the service requires a specific WMS protocol version, use this to send it. Usually you can omit this. |
| --in style *string* | WMS `STYLE` parameter, if the service requires one. Usually you can omit this. |
| --in transparent true | Request images with an alpha channel, if supported. Make sure if you use this that you also request a format that support transparency (e.g. `--in format png`).|

Example:
```
osgearth_conv --in driver wmsimage
              --in url https://basemap.nationalmap.gov/arcgis/services/USGSImageryOnly/MapServer/WMSServer
              --in layers 0
              --in format image/jpeg
              --in srs EPSG:4326
              --out driver mbtilesimage
              --out filename output.mbtiles
              --out format jpg
              --min-level 6
              --max-level 13
              --extents 33.020325 -114.604442 33.557478 -114.258021
```
Notes:

 * We use a `min-level` and `max-level` to limit to zoom levels of the output data.
 * The output `format` of `jpg` is used since we do not expect to need transparency support.

### ArcGISServerImage

| Property | Description |
|----------|-------------|
| --in url *string* | HTTP (or HTTPS) endpoint of the ArcGIS service. Always required. |
| --in token *string* | Authentication token (may be required) |
| --in layers *string* | Layers designator that selects which image to download. This is usually required. |
| --in format *string* | File format to request. There will be a default, but you can use this to override it. Values like `jpeg` or `png` are common. |

Example:
```
osgearth_conv --in driver arcgisserverimage
              --in url https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer
              --out driver mbtilesimage
              --out filename usgsImagery6.mbtiles
              --out format jpg
              --min-level 0
              --max-level 6
              --extents -90 -180 90 180
              --profile global-geodetic
```
Notes:

 * `min_level` and `max_level` properties limit the scope of the conversion
 * `extents` in this case is requesting full world extents
 * A `global-geodetic` output profile will reproject the input data from Mercator to Geographic (lat/long) for the best osgEarth load performance.

## Outputs

### TMSImage / TMSElevation

A [TMS (TileMapService) repository](https://wiki.osgeo.org/wiki/Tile_Map_Service_Specification) is a simple hierarchical collection of files. Each tile in the pyramid is stored in its own file. This is a good format to use if you want to stream your dataset from a simple web server (like Apache).

| Property | Description |
|----------|-------------|
| --out url *path* | Location of the metadata file that contains the repository's geospatial information. The repository will be stored relative to the location of this XML file. |
| --out format *string* | Format for each individual tile file. This should be `jpg` for RGB imagery, `png` for RGBA imagery, or `tiff` for elevation data.|

Example (transform a local GDAL GeoTIFF imagery file into a TMS repository):
```
osgearth_conv --in driver gdalimage
              --in url myinputfile.tif
              --out driver tmsimage
              --out url output/tms.xml
              --out format jpg
```
In this example, the repository will be saved in a folder called `output`. You can transfer this folder directly to a web server to stream it over the web, or you can access it locally.

### MBTilesImage / MBTilesElevation

[MapBox Tiles](https://github.com/mapbox/mbtiles-spec) stores a geospatial tileset in a SQLite database. MBTiles gives you the optimizations of a pre-tiled dataset along with the convenience and portability of a single file.

| Property | Description |
|----------|-------------|
| --out url *path* | Location of the SQLite MBTiles database file. It is common practice (but not required) to give this an `.mbtiles` extension.|
| --out format *string* | Format for each individual tile file. This should be `jpg` or `png` for imagery, and must be `tiff` for elevation data.|

Example:
```
osgearth_conv --in driver gdalelevation
              --in url mydem.dt1
              --in vdatum egm96
              --out driver mbtileselevation
              --out filename mydem.mbtiles
              --out format tiff
```


## Common Properties

These properties apply to all **osgearth_conv** operations.

| Property | Description |
|----------|-------------|
| --min-level *integer* | Lowest zoom level for which to create tiles. Default is 0.
| --max-level *integer* | Highest zoom level for which to create tiles. The default value is computed automatically based on the input. |
| --extents&nbsp;*minlat*&nbsp;*minlong*&nbsp;*maxlat*&nbsp;*maxlong* | Geographic extents (in degrees) for which to generate tiles. |
| --profile *value* | Forces the output to a specific tiling profile. By default the output will match the geospatial profile of the input data (for example, if the input if spherical mercator, the output will be tiled in a spherical mercator profile). Use this to force something different -- typically `global-geodetic` for data that will run in a whole-earth osgEarth map. |
| --no-overwrite | By default, running `osgearth_conv` will regenerate and overwrite any existing tiles that were output on a previous run. Use this option to disable that and ignore output tiles that already exist. |
| --threads *integer* | Using multiple threads can speed up the tile generation process in some cases. A good value to try is 4. |


---
&copy; Copyright 2022 Pelican Mapping
