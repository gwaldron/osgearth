# TMS (Tile Map Service)

TMS ([Tile Map Service](https://wiki.osgeo.org/wiki/Tile_Map_Service_Specification)) is an OSGeo standard for tiled geospatial raster data. 
A TMS "repository" consists of an XML metadata file (usually named `tms.xml`) and a hierarchy of individual files, each one representing a single raster tile.

TMS contains pre-tiled data, so it is quite fast to load, especially if tiled in a `global-geodetic` profile.
As a simple file hierarchy, TMS is also web-streamable with nothing more than a simple HTTP server. 

## TMSImage, TMSElevation

CLASS: TMSImageLayer (inherits from [ImageLayer](image.md))

CLASS: TMSElevationLayer (inherits from [ElevationLayer](elevation.md))

| Property | Description | Type | Default |
| ---------- | ----------- | ---- | ------- |
| url | Location of the metadata file (folder or filename, usually `tms.xml`) | URI  |         |

### Example

```xml
<TMSImage name="ReadyMap 15m Imagery">
    <url>http://readymap.org/readymap/tiles/1.0.0/7/</url>
</TMSImage>

<TMSElevation name="ReadyMap 90m Elevation">
    <url>http://readymap.org/readymap/tiles/1.0.0/116/</url>
</TMSElevation>
```
