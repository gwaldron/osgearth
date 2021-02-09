# GDAL Image Layer

Uses the GDAL SDK to load imagery data. This is the most common layer type to use for local data files like GeoTIFF.

### Properties

Inherits from: [Image Layer](ImageLayer.md)

| Earth file      | Description                                                  | Type   | Default |
| --------------- | ------------------------------------------------------------ | ------ | ------- |
| url             | Location of data source (local or remote)                    | URI    |         |
| connection      | Connection string when querying a spatial database (like PostgreSQL for example) | string |         |
| single_threaded | Force single-threaded access to the GDAL driver. Most GDAL drivers are thread-safe, but not all. If you are having issues with a GDAL driver crashing, try setting this to true. | bool   | false   |
| subdataset      | Identifier of a sub-dataset within a larger GDAL dataset. Some drivers require this in order to access sub-layers within the database. | string |         |

### Examples

Layer displaying a local GeoTIFF:

```xml
<GDALImage name="Local Data">
    <url>world.tif</url>
</GDALImage>
```

