Color Filter Reference
======================
A *color fitler* is an inline, GLSL processor for an ImageLayer. 
The osgEarth terrain engine runs each image tile through its layer's
color filter as it's being rendered on the GPU. You can chain color
filter together to form an image processing pipeline.

osgEarth comes with several stock filters; you can create your own
by implementing the ``osgEarth::ColorFilter`` interface.

Here is how to use a color filter in an earth file::

    <image driver="gdal" name="world">
        <color_filters>
            <chroma_key r="1" g="1" b="1" distance=".1"/>
        </color_filters>
    </image>


Stock color filters:

 * BrightnessContrast_
 * ChromaKey_
 * CMYK_
 * Gamma_
 * GLSL_
 * HSL_
 * RGB_


BrightnessContrast
------------------
This filter adjusts the brightness and contrast of the image::

    <brightness_contrast b="0.7" c="1.2"/>
    
The ``b`` and ``c`` properties are *percentages* of the incoming value.
For example, ``c="1.2"`` means to increase the contrast by 20%.


ChromaKey
---------
This filter matches color values are makes fragments to transparent,
providing a kind of "green-screen" effect::

    <chroma_key r="1.0" g="0.0" b="0.0" distance="0.1"/>
    
In this example, we find all red pixels and turn them transparent. 
The ``distance`` property searches for colors close to the specified
color. Set it to Zero for exact matches only.


CMYK
----
This filter offsets the CMYK (cyan, magenta, yellow, black) color levels::

    <cmyk y="-0.1"/>
    
Here we are lowering the "yellowness" of the fragment by 0.1. Valid range
is [-1..1] for each of ``c``, ``m``, ``y``, and ``k``.


Gamma
-----
This filter performs gamma correction. You can specify a *gamma* value for
each of ``r``, ``g``, or ``b``, or you can adjust them all together::

    <gamma rgb="1.3"/>


GLSL
----
The GLSL filter lets you embed custom GLSL code so you can adjust the 
color value in any way you like. Simply write a GLSL code block
that operates on the RGBA color variable ``inout vec4 color``::

    <glsl>
        color.rgb *= pow(color.rgb, 1.0/vec3(1.3));
    </glsl>

This example does exactly the same thing as the Gamma_ filter but 
using directly GLSL code.


HSL
---
This filter offsets the HSL (hue, saturation, lightness) levels::

    <hsl s="0.1" l="0.1"/>
    
This example adds a little more color saturation and brightens the
fragment a bit as well. Valid range is [-1..1] for each of ``h``, ``s``, and ``l``.


RGB
---
This filter offsets the RGB (red, green, blue) color levels::

    <rgb r="0.1" b="-0.5"/>
    
This example adds a little bit of red and reduces the blue channel. Valid
range is [-1..1] for each of ``r``, ``g``, and ``b``.
