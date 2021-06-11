# XYZ

A pre-tiled image layer that uses a generic template URL to access raster tiles.

An XYZ layer exposes no metadata, so the user is resposible for telling osgEarth about the `profile` and other aspects of the data source.

The URL template MUST include the template parameters {z} (specifying the tile level), and {x} and {y} (specifying the tile offset within the level). The {y} parameter also suppose a unary negative in the form {-y} since it's common to see datasets that are flipped in the vertical direction.

## XYZImage, XYZElevation

CLASS: XYZImageLayer (inherits from [ImageLayer](image.md))

CLASS: XYZElevationLayer (inherits from [ElevationLayer](elevation.md))

| Property | Description                                                  | Type | Default |
| ---------- | ------------------------------------------------------------ | ---- | ------- |
| url  | Location of data source (local or remote), including templated parameters {x}, {y}/{-y}, and {z} (see example). The template also supports round-robin characters by way of square brackets ([]). | URI  |         |
| profile | Tiling configuration of the data source (typically `global-geodetic` or `spherical-mercator`) | string | `global-geodetic` |

### Examples

In this example, the template URL contains both an optional "round-robin" character set and the required z/x/y tile parameters. The round-robin set will cycle between three servers that balance the load of the requests:

* a.tile.openstreetmap.org
* b.tile.openstreetmap.org
* c.tile.openstreetmap.org

osgEarth will fill in the x/y/z parameters based on the current tile request during run-time.

```xml
<XYZImage name="OSM Vector Map">
    <url>http://[abc].tile.openstreetmap.org/{z}/{x}/{y}.png</url>
    <profile>spherical-mercator</profile>
    <attribution>&#169;OpenStreetMap contributors</attribution>
</XYZImage>
```

