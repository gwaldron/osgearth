# ESRI ArcGIS

Access various data sources from an ESRI ArcGIS Server instance. Licenses may be required.

## ArcGISServerImage
Loads an ESRI ArcGIS imagery layer from the server.

CLASS: `ArcGISServerImageLayer` (inherits from: [ImageLayer](image.md))

| Earth file | Description                       | Type   | Default |
| ---------- | --------------------------------- | ------ | ------- |
| url        | Base URL of server                | URI    |         |
| token      | Authentication token              | string |         |
| format     | Image format to request           | string |         |
| layers     | Comma-separated layers to request | string |         |

## ArcGISTilePackageImage

Loads an ESRI ArcGIS TilePackage format image (local or remote)

CLASS: `ArcGISTilePackageImageLayer` (inherits from: [ImageLayer](image.md))

| Property | Description                                 | Type | Default |
| ---------- | ------------------------------------------- | ---- | ------- |
| url        | Location of data source (network or folder) | URI  |         |

### Examples

```xml
<ArcGISServerImage name="demo">
    <url>http://services.arcgisonline.com/arcgis/rest/services/World_Imagery/MapServer/</url>
</ArcGISServerImage>
```

```xml
<ArcGISTilePackageImage name="Data in folder">
    <url>data/bluemarble_imagery</url>
</ArcGISTilePackageImage>
```

