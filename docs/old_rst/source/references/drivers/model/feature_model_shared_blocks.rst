----

Fading
~~~~~~
When fading is supported on a model layer, you can control it like so::

    <model ...
        <fading duration  = "1.0"
                max_range = "6000"
                attenuation_distance = "1000" />
                
Properties:

    :duration:              Time over which to fade in (seconds)
    :max_range:             Distance at which to start the fade-in
    :attenuation_distance:  Distance over which to fade in


Shader Policy
~~~~~~~~~~~~~
Some drivers support a *shader policy* that lets you control how (or whether)
to generate shaders for external geometry. For example, if you want to load
an external model via a stylesheet, but do NOT want osgEarth to generate
shaders for it::

    <model ...
        <shader_policy>disable</shader_policy>
