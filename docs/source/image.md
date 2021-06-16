# Image Layer (common)

An Image Layer is any layer that loads and display tiled imagery on the globe. Image layers are stacked, rendered in order of appearance in the Map. In the earth tile, an Image Layer usually has an `Image` suffix.

Inherits from: [Tile Layer](tilelayer.md)

| Property          | Description                                                  | Type   | Default              |
| ------------------- | ------------------------------------------------------------ | ------ | -------------------- |
| accept_draping      | Whether draped overlays should be rendered on this layer     | bool   | true                 |
| altitude            | Distance from the ellipsoid at which to offset the rendering of this image layer | float  | 0                    |
| mag_filter          | Mip-mapping magnification filter<br />(see `osg::Texture::FilterMode`for options) | string | LINEAR               |
| min_filter          | Mip-mapping minification filter<br />(see `osg::Texture::FilterMode` for options) | string | LINEAR_MIPMAP_LINEAR |
| nodata_image        | Location of an Image that represent "no data" for a tile.    | URL    | none                 |
| shared              | Whether to allocate a dedicated texture image unit for this layer so that its data may be shared with other layers' shader code | bool   | false                |
| shared_matrix       | When `shared` is true, name of the texture matrix uniform that applies to the shared texture for this layer | string | (auto generated)     |
| shared_sampler      | When `shared` is true, name of the sampler uniform that references the shared texture for this layer | string | (auto generated)     |
| texture_compression | Controls whether and how to compress textures for the GPU.<br />`auto` : automatically select a compression method<br />`gpu` : use hardware compression if available<br />`cpu` : use CPU compression if available<br />`none` : do not compress textures | string | none                 |

