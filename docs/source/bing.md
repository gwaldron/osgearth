# Microsoft Bing

Displays data from the Microsoft Bing server. Subscription / API key required.

## BingImage, BingElevation

CLASS: BingImageLayer (inherits from: [ImageLayer](image.md))

CLASS: BingElevationLayer (inherits from: [ElevationLayer](elevation.md))

| Property  | Description                                                  | Type   | Default |
| ----------- | ------------------------------------------------------------ | ------ | ------- |
| key         | API key tied to your subscription                            | string |         |
| imagery_set | Imagery set to access (BingImage only). Please see [Bing documentation](https://docs.microsoft.com/en-us/bingmaps/rest-services/imagery/get-imagery-metadata#template-parameters) for available options. | string | Aerial  |

### Example

```xml
<BingImage name="bing">
    <key>{your key here}</key>
    <imagery_set>Aerial</imagery_set>
</BingImage>
```
