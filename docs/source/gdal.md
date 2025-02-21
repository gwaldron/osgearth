# GDAL Layers

Uses the GDAL SDK to load imagery or elevation data. This is the most common layer type to use for local data files like GeoTIFF.

GDAL has [dozens of loaders](https://gdal.org/drivers/raster/index.html) for different georeferenced file formats.

## GDALImage, GDALElevation

Class: GDALImageLayer (inherits from [ImageLayer](image.md))

Class: GDALElevationLayer (inherits from [ElevationLayer](elevation.md))

| Property      | Description                                                  | Type   | Default |
| --------------- | ------------------------------------------------------------ | ------ | ------- |
| url             | Location of data source (local or remote), e.g. a GeoTIFF file | URI    |         |
| connection      | Connection string when querying a spatial database (like PostgreSQL for example) | string |         |
| single_threaded | Force single-threaded access to the GDAL driver. Most GDAL drivers are thread-safe, but not all. If you are having issues with a GDAL driver crashing, try setting this to true. | bool   | false   |
| subdataset      | Identifier of a sub-dataset within a larger GDAL dataset. Some drivers require this in order to access sub-layers within the database. | string |         |
| vdatum | Specify a vertical datum to use (elevation only) | string | |
| | Supported values = "egm96" or "egm2008" | | |

Earth file examples:

```xml
<GDALImage name="Local Data">
    <url>world.tif</url>
</GDALImage>

<GDALElevation name="Local terrian data">
    <url>elevation.dt0</url>
    <vdatum>egm96</vdatum>
</GDALElevation>
```

NOTE: this example loads a DTED0 elevation file, which is known to use the EGM96 vertical datum. We need to specify that in the earth file since the vertical datum information in NOT included in the source data itself.


## GDALDEM (image layer)

Displays a colorized or hill-shaded representation of the elevation data in the map.

| Property       | Description                                                  | Type   | Default |
| -------------- | ------------------------------------------------------------ | ------ | ------- |
| processing     | Type of shading to use. Options are `color-relief` and `hillshade` | string | `hillshade` |
| color_filename | `.clr` file containing color-relief color map                | URI    |         |

### Examples

Blend in a hill-shaded display:

```xml
<GDALDEM name="Hillshade example">
    <processing>hillshade</processing>
    <opacity>0.50</opacity>
</GDALDEM>
```

![](/_static/images/gdaldem_hillshade.png)


Start with a color relief display:

```xml
<GDALDEM name="Relief example">
    <processing>color-relief</processing>
    <color_filename>../data/colorramps/thematic.clr</color_filename>
</GDALDEM>
```

![](/_static/images/gdaldem_colorrelief.png)


