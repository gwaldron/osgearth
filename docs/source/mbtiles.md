# MBTiles

MBTiles (Map Box Tiles) is container format for storing tiled datasets.
It uses a SQLite database.
It is a convenient mechanism for storing a large number of geospatial tiles in a single file, making it easily transportable.

[The specification may be found here.](https://docs.mapbox.com/help/glossary/mbtiles/)

Because MBTiles contains pre-tiled data, it is quite fast to access and local caching is unnecessary.

MBTiles is not streamable without additional software on the server side.
If you wish to deploy a tiled dataset on a simple HTTP server, consider [TMS](tms.md) instead.

## MBTilesImage, MBTilesElevation

CLASS: MBTilesImageLayer (inherits from [ImageLayer](image.md))

CLASS: MBTilesElevationLayer (inherits from [ElevationLayer](elevation.md))

| Property | Description                           | Type | Default |
| ---------- | ------------------------------------- | ---- | ------- |
| url        | Location of the MBTiles database file | URI  |         |

### Example

```xml
<MBTilesImage name="Countries of the World">
    <url>../data/world_countries.mbtiles</url>
</MBTilesImage>

<MBTilesElevation name="Global elevation grid">
    <url>elevationdata.mbtiles</url>
</MBTilesElevation>
```

### Creating an MBTiles file

You can use the `osgearth_conv` tool to convert a GeoTIFF (or other data source) into an MBTiles database.
Example usage:
```
osgearth_conv
    --in driver GDALImage
    --in url my_local_file.tif
    --out driver MBTilesImage
    --out url output_file.mbtiles
    --out format png
    --threads 4
```
Just replace the `--in driver` with the appropriate source type.

Type `osgearth_conv` alone for more command-line options.