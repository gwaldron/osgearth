# Layer (common)

A data element in the Map. All layers inherit common properties from here.

### Properties

| Earth file    | Description                                                  | Type   | Default        |
| ------------- | ------------------------------------------------------------ | ------ | -------------- |
| name          | Readable name of this layer                                  | string |                |
| enabled       | Whether to open this layer when loading an earth file        | bool   | true           |
| attribution   | Readable string that describes where this layer gets its data | string |                |
| cache_policy  | Caching control over this layer                              | custom | (inherited)    |
| shader        | Inline GLSL shader component to use when rendering this layer | string |                |
| shader_define | GLSL preprocessor `#define` to set when rendering this layer | string |                |
| proxy         | Network proxy settings to use if/when this layer tries to get data from a remote source | custom |                |
| cacheid       | Custom cache ID string to use when storing a cache for this layer | string | auto-generated |