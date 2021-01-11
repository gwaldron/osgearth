# Composite Image Layer

Combines two or more image layers into a single image layer. This can improve render and caching performance, and also allows you to group layers together for operations like opacity and visibility.

### Properties

Inherits from: [Image Layer](ImageLayer.md)

| Earth file | Description                             | Type         | Default |
| ---------- | --------------------------------------- | ------------ | ------- |
| layers     | Collection of image layers to composite | image layers |         |

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

