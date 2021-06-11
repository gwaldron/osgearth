# Tile Layer (common)

Any layer that draws imagery or elevation tiles.

Inherits from: [Visible Layer](visiblelayer.md)

| Property      | Description                                                  | Type   | Default |
| --------------- | ------------------------------------------------------------ | ------ | ------- |
| max_data_level  | Forces a maximum LOD at which to generate new data for this layer. Data displayed past this LOD will be upsampled by the GPU. | int    |         |
| min_level       | Lowest LOD at which to use this layer                        | int    | 0       |
| max_level       | Highest LOD at which to use this layer                       | int    | none    |
| min_resolution  | Lowest resolution (in layer units per pixel) at which to use this layer | float  | none    |
| max_resolution  | Highest resolution (in layer units per pixel) at which to use this layer | float  | none    |
| min_valid_value | Smallest valid value to accept from the underlying data source. This usually only applies to elevation data. Smaller values are converted to "NO DATA" | float  | none    |
| max_valid_value | Largest valid value to accept from the underlying data source. This usually applies to elevation data. Higher values are interpreted as "NO DATA" | float  | none    |
| no_data_value   | Specific value to interpret at "NO DATA"                     | float  | none    |
| tile_size       | Number of elements in each dimension of the tile. For image layers, default is 256. For elevation layers, default is 257. | int    | 256/257 |


