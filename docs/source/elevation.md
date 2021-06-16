# Elevation Layer (common)

An Elevation Layer is any layer that loads and displays tiled heightfield data. 
Eleation layers are stacked, rendered in order of appearance in the Map. 
In the earth tile, an Elevation Layer usually has an `Elevation` suffix.

Inherits from: [Tile Layer](tilelayer.md)

| Property          | Description                                                  | Type   | Default              |
| ------------------- | ------------------------------------------------------------ | ------ | -------------------- |
| vdatum | Vertical datum to use (egm96, egm2008)     | string   | geodetic                 |
| offset | If true, add the height values to those from the previous layer | string | false |

