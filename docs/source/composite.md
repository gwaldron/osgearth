# Composite Layers

Combines two or more layers into a single layer.
This can improve render and caching performance, and also allows you to group layers together for operations like opacity and visibility.

## CompositeImage, CompositeElevation

CLASS: CompositeImageLayer (inherits from [ImageLayer](image.md)))

CLASS: CompositeElevationLayer (inherits from [ElevationLayer](elevation.md))

| Property | Description                             | Type         | Default |
| ---------- | --------------------------------------- | ------------ | ------- |
| layers     | Collection of layers to composite. | [layers] |         |

### Example

Composites three image layers into a single layer:

```xml
<CompositeImage name="Combined imagery">
    <layers>
        <TMSImage name="ReadyMap 15m Imagery">
            <url>http://readymap.org/readymap/tiles/1.0.0/7/</url>
        </TMSImage>

        <GDALImage name="Boston inset">
            <url>../data/boston-inset-wgs84.tif</url>
        </GDALImage>	

        <GDALImage name="New York inset">
            <url>../data/nyc-inset-wgs84.tif</url>
        </GDALImage>
    </layers>
</CompositeImage>
```

Note: you can only composite layers of the same base type (e.g. image layers with other image layers.)
