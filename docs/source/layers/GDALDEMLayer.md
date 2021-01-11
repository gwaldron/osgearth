# GDAL DEM Shading Layer

Displays a colorized or hill-shared representation of the elevation data in the map.

### Properties

Inherits from: [Image Layer](ImageLayer.md)

| Earth file     | Description                                                  | Type   | Default |
| -------------- | ------------------------------------------------------------ | ------ | ------- |
| processing     | Type of shading to use. Options are `color-relief` and `hillshade` | string |         |
| layer          | Elevation layer to use for shading                           | string |         |
| color_filename | `.clr` file containing color-relief color map                | URI    |         |

### Examples

Start with a color relief display:

```xml
<GDALDEM name="Relief example">
    <processing>color-relief</processing>
    <layer>elevation</layer>
    <color_filename>../data/colorramps/thematic.clr</color_filename>
</GDALDEM>
```

![](/_static/images/gdaldem_colorrelief.png)



Blend in a hill-shaded display:

```xml
<GDALDEM name="Hillshade example">
    <processing>hillshade</processing>
    <layer>elevation</layer>
    <opacity>0.75</opacity>
</GDALDEM>
```

![](/_static/images/gdaldem_hillshade.png)



