Noise
==========================================
The noise plugin procedurally generates fractal terrain based on a Perlin noise generator called `libnoise`_.
We will explain how it works here, but you can also refer the the libnoise documentation for the meaning and
application of the properties below.

There are lots of ways to use the ``noise`` driver. After the properties list there are 
a few examples of how to use it.
       
Basic Properties:

    :resolution:        The linear distance (usually meters) over which to generate one cycle of
                        noise data.
    :scale:             The amount of offset to apply to noise values within a cycle. The default is
                        1.0, which means you will get noise data between [-1...1].
    :octaves:           Number of times to refine the noise data by adding levels of detail,
                        i.e. how deep the noise generator will recurse within the resolution span.
                        A higher number will create more detail as you zoom in closer. Default is 4.
    :offset:            For heightfields, set this to true to generate offset values instead of absolute
                        elevation heights. They will be added to the heights from another absolute
                        elevation layer.

Advanced Properties:

    :frequency:         The reciprocal of the *resolution* above. (Since osgEarth is a mapping SDK,
                        it is usually more intuitive to specifiy the resolution and leave this empty.)
    :persistence:       Rate at which the *scale* decreases as the noise function traverses each
                        higher octave. Scale(octave N+1) = Scale(octave N) * Persistence.
    :lacunarity:        Rate at which the *frequency* increases as the noise function traverses each
                        higher octave of detail. Freq(octave N+1) = Freq(octave N) * Lacunarity.
    :seed:              Seeds the random number generator. The noise driver is "coherent", meaning that
                        (among other things) it generates the same values given the same random
                        seed. Alter this to alter the pattern.
    :min_elevation:     The minimum elevation value to generate when creating height fields. This clamps
                        height data to create a "floor".
    :max_elevation:     The maximum elevation value to generate when createing height fields. This clamps
                        height data to create a "ceiling".
    :normal_map:        Set this to true (for an image layer) to create a bump map normal texture that you
                        can use with the ``NormalMap`` terrain effect.

Also see:

    ``noise.earth``, ``fractal_detail.earth``, and ``normalmap.earth`` samples in the repo ``tests`` folder.


Examples
--------

Create a worldwide procedural elevation layer::

    <elevation driver="noise">
        <resolution>3185500</resolution>   <!-- 1/4 earth's diameter -->
        <scale>5000</scale>                <!-- vary heights by +/- 5000m over the resolution -->
        <octaves>12</octaves>              <!-- detail recursion level -->
    </elevation>

Make it a little more interesting by tweaking the recursion properties::

    <elevation driver="noise">
        <resolution>3185500</resolution>   <!-- 1/4 earth's diameter -->
        <scale>5000</scale>                <!-- vary heights by +/- 5000m over the resolution -->
        <octaves>12</octaves>              <!-- detail recursion level -->
        <persistence>0.49</persistence>    <!-- don't reduce the scale as quickly = noisier -->
        <lacunarity>3.0</lacunarity>       <!-- increase the frequency faster = lumpier -->
    </elevation>

Look at the noise itself by creating an image layer. Looks like clouds::

    <image driver="noise">
        <resolution>3185500</resolution>   <!-- 1/4 earth's diameter -->
        <octaves>12</octaves>              <!-- detail recursion level -->
    </image>

Use ``noise`` to create an offset layer to add detail to real elevation data::

    <!-- Real elevation data -->
    <elevation name="readymap_elevation" driver="tms" enabled="true">
        <url>http://readymap.org/readymap/tiles/1.0.0/9/</url>
    </elevation>
    
    <elevation driver="noise" name="detail">
        <offset>true</offset>             <!-- treat this as offset data -->
        <tile_size>31</tile_size>         <!-- size of the tiles to create -->
        <resolution>250</resolution>      <!-- not far from the resolution of our real data -->
        <scale>20</scale>                 <!-- vary heights by 20m over 250m -->
        <octaves>4</octaves>              <!-- add some additional detail -->
    </elevation>

Instead of creating offset elevation data, we can fake it with a *normal map*. A normal map
is an invisible texture that simulates the normal vectors you'd get if you used real 
elevation data::

    <image name="normalmap" driver="noise">
        <shared>true</shared>             <!-- share this layer so our effect can find it -->
        <visible>false</visible>          <!-- we don't want to see the actual texture -->
        <normal_map>true</normal_map>     <!-- create a normal map please -->
        <tile_size>128</tile_size>        <!-- 128x128 texture -->
        <resolution>250</resolution>      <!-- resolution of the noise function -->
        <scale>20</scale>                 <!-- maximum height offset -->
        <octaves>4</octaves>              <!-- level of detail -->
    </image>
    
    ...
    <external>
       <normal_map layer="normalmap"/>    <!-- Install the terrain effect so we can see it -->
       <sky hours="17"/>                  <!-- Must have lighting as well -->
    </external>



.. _libnoise:           http://libnoise.sourceforge.net/
