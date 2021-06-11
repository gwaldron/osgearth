# Cesium ion

Loads imagery from a Cesium ion server instance. An authentication token is required.

[More information on Cesium ion may be found here.]([https://cesium.com/cesium-ion/)

## CesiumIonImage

CLASS: CesiumIonImageLayer (inherits from: [ImageLayer](image.md))

| Property | Description                                                  | Type    | Default |
| ---------- | ------------------------------------------------------------ | ------- | ------- |
| asset_id   | Identifier of the imagery asset on the Cesium ion server to load | integer |         |
| token      | Security token for access authentication                     | string  |         |

### Example

```xml
<CesiumIonImage name="cesiumion bluemarble">
    <asset_id>3845</asset_id>
    <token>YOUR-TOKEN_HERE</token>
</CesiumIonImage>
```
