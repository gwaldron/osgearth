Noise
==========================================
The noise plugin procedurally generates fractal terrain based on a perlin noise generator from `libnoise`_.
Please refer to the libnoise documentation for the meaning and application of the properties below!

Example usage::

    <elevation driver="noise">        
    </elevation>
       
Properties:

    :min_elevation:     The minimum elevation value to generate
    :max_elevation:     The maximum elevation value to generate
    :octaves:           Number of 'octaves', i.e. how deep the recursion can go. A higher number will 
                        increase variation at a wide range of LODs but will degrade performance.
    :frequency:         Spacing of the coherent noise generation pattern relative to the extent of
                        the dataset.
    :persistence:       Rate at which the frequency increases as you go to higher LODs (octaves).
    :lacunarity:        Read the libnoise docs for information on lacunarity.                                                
    :seed:              Seeds the random number generator. The noise driver is "coherent", meaning that
                        (among other things) it generates the same values given the same random
                        seed. Alter this to alter the pattern.
    
Also see:

    ``noise.earth`` and ``normalmap.earth`` samples in the repo ``tests`` folder.



.. _libnoise:           http://libnoise.sourceforge.net/
