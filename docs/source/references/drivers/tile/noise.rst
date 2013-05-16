Noise
==========================================
The noise plugin procedurally generates fractal terrain based on a perlin noise generator from `libnoise`_.

Example usage::

    <heightfield driver="noise">        
    </heightfield>
       
Properties:

    :min_elevation:     The minimum elevation to generate
    :max_elevation:     The maximum elevation to generate
    :octaves:           The number of octaves in the noise generator.
    :frequency:         The frequency of the noise generator                         
    :persistence:       The persistence of the noise generator 
    :lacunarity:        The lacunarity of the noise generator                                                
    :seed:              The seed of the noise generator
    
Also see:

    ``noise.earth`` sample in the repo ``tests`` folder.



.. _libnoise:           http://libnoise.sourceforge.net/