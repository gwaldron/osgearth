# Visible Layer (common)

Any layer that renders something visible to the user.

## Properties

Inherits from: [Layer](layer.md)

| Earth file        | Description                                                  | Type   | Default     |
| ----------------- | ------------------------------------------------------------ | ------ | ----------- |
| attenuation_range | Range (meters) over which this layer fades from visible to invisible when approaching `max_range` | float  | 0           |
| blend             | How this layer blends with the previously rendered layer<br />`interpolate` : blend based on alpha channel value<br />`modulate` : blend by multiplying color values | string | interpolate |
| max_range         | Maximum camera distance (meters) at which to render this layer | float  | infinity    |
| min_range         | Minimum camera distance (meters) at which to render this layer | float  | 0           |
| opacity           | Blending factor relative to underlying map layers            | float  | 1.0         |
| visible           | Whether to render this layer at all                          | bool   | true        |