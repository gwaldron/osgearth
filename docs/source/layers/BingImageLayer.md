# Microsoft Bing Image Layer

Displays imagery from the Microsoft Bing server. Subscription / API key required.

### Properties

Inherits from: [Image Layer](ImageLayer.md)

| Earth file  | Description                                                  | Type   | Default |
| ----------- | ------------------------------------------------------------ | ------ | ------- |
| key         | API key tied to your subscription                            | string |         |
| imagery_set | Bing imagery set to access. Please see [Bing documentation](https://docs.microsoft.com/en-us/bingmaps/rest-services/imagery/get-imagery-metadata#template-parameters) for available options. | string | Aerial  |

### Example

```xml
<BingImage name="bing">
    <key>{your key here}</key>
    <imagery_set>Aerial</imagery_set>
</BingImage>
```

