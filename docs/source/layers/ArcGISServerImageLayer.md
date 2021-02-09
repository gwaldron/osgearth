# ArcGIS Server Image Layer

Image layer that displays imagery from an ESRI ArcGIS Server instance.

### Properties

Inherits from: [Image Layer](ImageLayer.md)

| Earth file | Description                       | Type   | Default |
| ---------- | --------------------------------- | ------ | ------- |
| url        | Base URL of server                | URI    |         |
| token      | Authentication token              | string |         |
| format     | Image format to request           | string |         |
| layers     | Comma-separated layers to request | string |         |

### Examples

```xml
<ArcGISServerImage name="demo">
    <url>http://services.arcgisonline.com/arcgis/rest/services/World_Imagery/MapServer/</url>
</ArcGISServerImage>
```

