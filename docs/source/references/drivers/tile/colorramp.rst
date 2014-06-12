Color Ramp
==========================================
The Color Ramp plugin uses an underlying heightfield in addition to a color ramp
file to generate RGBA images from single band datasets such as elevation or temperature.

Example usage::

    <image name="color ramp" driver="colorramp">
        <elevation name="readymap_elevation" driver="tms">
            <url>http://readymap.org/readymap/tiles/1.0.0/9/</url>
        </elevation>
        <ramp>..\data\colorramps\elevation.clr</ramp>
    </image>

Ramp files:

A file that defines how values match to colors.  Each line should contain
a value and the RGB color it's mapped to with values in the range 0-255

For example::

    0 255 0 0
    1000 255 255 0
    5000 0 0 255

    
Properties:

    :elevation:         Definition of an elevation layer to sample.

    :ramp:              Path to the ramp file to use to color the layer.
    
Also see:

    ``colorramp.earth`` sample in the repo ``tests`` folder.