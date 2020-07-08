# osgearth_decal

The current osgearth_decal requires two images, one for the image data and one for the elevation change.  The current image sample is
crater.png which is 256x256 pixels with alpha channel of 0 for transparency.

The elevation sample is burn.png and it assumes the origin or no elevation change value is 128.  With alpha values ranging from 0 to 255,
consider 128 the midpoint where lower values 0-127 increase elevation values  and 129-255 decrease elevation values.  The max increase
in elevation is at 1 (it may allow alpha values of 0 and 255 easier in paint to not use 1-254) and max decrease in elevation at alpha
of 254.  The elevation range is set to the decal size / 15 so if the default size is 250, the range will be +/- 16.6m.

## General tips for painting elevation image:
1. Start with a blank canvas and fill it with alpha of 128.  This is the zero change base image.
2. Most paint tools default to additive mode which is not inutitive here.  To increase elevations, you need to reduce alpha values.
I used overlay mode to set values rather than try to do additive.
3. Watch for overspray on the edges, small alpha values above and below 128 will be noticeable.
