# MBTiles Image Layer

MBTiles is container format for storing tiled datasets. It uses a SQLite database. It is a convenient mechanism for storing a large number of geospatial tiles in a single file, making it easily transportable. [More details may be found here.](https://docs.mapbox.com/help/glossary/mbtiles/)

Because MBTiles contains pre-tiled data, it is quite fast to access and local caching is unnecessary.

MBTiles is not streamable without additional software on the server side. If you wish to deploy a tiled dataset on a simple HTTP server, consider [TMS](TMSImagLayer.md) as an alternative.

### Properties

Inherits from: [Image Layer](ImageLayer.md)

| Earth file | Description                           | Type | Default |
| ---------- | ------------------------------------- | ---- | ------- |
| url        | Location of the MBTiles database file | URI  |         |

### Example

```xml
<MBTilesImage name="Countries of the World">
    <url>../data/world_countries.mbtiles</url>
</MBTilesImage>
```

