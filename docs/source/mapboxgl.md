# Mapbox GL
Rasterizes a [Mapbox GL](https://docs.mapbox.com/mapbox-gl-js/style-spec) style into an image layer.  Most of the core Mapbox GL spec is implemented and most of the styles we have tried look pretty good, but some functionality will inevitably as this is a from scratch implementation of the spec. 

You will need to build osgEarth with Blend2D rasterization support for the Mapbox GL layer to render properly.

We have used styles from [Maputnik](https://maputnik.github.io/) as well as vector tile styles from ArcGIS online.

## MapBoxGLImage

CLASS: MapBoxGLImageLayer (inherits from: [ImageLayer](image.md))

| Property     | Description                                                        | Type    | Default |
| -------------| -------------------------------------------------------------------| ------- | ------- |
| url          | Path to the Mapbox GL style.                                       | URI     |         |
| key          | Security token for access authentication if the server requires it.| string  |         |
| disable_text | Disables text rendering for this layer                             | bool    | false   |
| pixel_scale  | Overall scaling factor for icons and text                          | float   | 1.0     |

### Example
```xml
<MapBoxGLImage name="ESRI Base">
    <url>https://www.arcgis.com/sharing/rest/content/items/5e9b3685f4c24d8781073dd928ebda50/resources/styles/root.json</url>
    <profile>spherical-mercator</profile>
    <tile_size>512</tile_size>
</MapBoxGLImage>
```
