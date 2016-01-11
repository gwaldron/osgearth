Shader Composition
==================

osgEarth uses GLSL shaders in several of its rendering modes. By default
osgEarth will detect the capabilities of your graphics hardware and
automatically select an appropriate mode to use.

Since osgEarth relies on shaders, you as a developer may wish to customize
the rendering or add your own effects and features in GLSL. Anyone who has
worked with shaders has run into the same challenges:

* Shader programs are monolithic. Adding new shader code requires you to
  copy, modify, and replace the existing code so you don't lose its
  functionality.
* Keeping your changes in sync with changes to the original code's 
  shaders is a maintenance nightmare.
* Maintaining multiple versions of shader main()s is cumbersome and
  difficult.
* Maintaining the dreaded "uber shader" becomes unmanageable as the 
  GLSL code base grows in complexity and you add more features.
  
*Shader Composition* solves these problems by *modularizing* the shader
pipeline. You can add and remove *functions* at any point in the program
without copying, pasting, or hacking other people's GLSL code.

Next we will discuss the structure of osgEarth's shader composition framework.


Framework Basics
----------------

The Shader Composition framework provides the main() functions automatically.
You do not need to write them. Instead, you write modular functions and tell the 
framework when and where to execute them.

Below you can see the main() functions that osgEarth creates.
The ``LOCATION_*`` designators allow you to inject functions at
various points in the shader's execution pipeline.

Here is the pseudo-code for osgEarth's built-in shaders mains::

    // VERTEX SHADER:

    void main(void)
    {
        vec4 vertex = gl_Vertex;

        // "LOCATION_VERTEX_MODEL" user functions are called here:
        model_func_1(vertex);
        model_func_2(vertex);
        ...

        vertex = gl_ModelViewMatrix * vertex;

        // "LOCATION_VERTEX_VIEW" user functions are called here:
        view_func_1(vertex);
        ...

        vertex = gl_ProjectionMatrix * vertex;
        
        // "LOCATION_VERTEX_CLIP" user functions are called last:
        clip_func_1(vertex);
        ...
        
        gl_Position = vertex;
    }  


    // FRAGMENT SHADER:

    void main(void)
    {
        vec4 color = gl_Color;
        ...

        // "LOCATION_FRAGMENT_COLORING" user functions are called here:
        coloring_func_1(color);
        ...

        // "LOCATION_FRAGMENT_LIGHTING" user functions are called here:
        lighting_func_1(color);
        ...

        gl_FragColor = color;
    }
    
As you can see, we have made the design decision to designate function
injection points that make sense for *most* applications. That is not to say
that they are perfect for everything, rather that we believe this approach
makes the Framework easy to use and not too "low-level".

*Important*: The Shader Composition Framework at this time only supports VERTEX and FRAGMENT
shaders. It does not support GEOMETRY or TESSELLATION shaders yet. We are planning
to add this in the future.


VirtualProgram
--------------

osgEarth introduces a new OSG state attribute called ``VirtualProgram`` that performs
the runtime shader composition. Since ``VirtualProgram`` is an ``osg::StateAttribute``,
you can attach one to any node in the scene graph. Shaders that belong to a
``VirtualProgram`` can override shaders higher up in the scene graph.
In this way you can add, combine, and override individual shader functions in osgEarth.

At run time, a ``VirtualProgram`` will look at the current state and assemble a full
``osg::Program`` that uses the built-in main()s and calls all the functions that you
have injected via ``VirtualProgram``.

 
Adding Functions
~~~~~~~~~~~~~~~~

From the generated mains we saw earlier, osgEarth calls into user functions.
These don't exist in the default shaders that osgEarth generates;
rather, they represent code that you as the developer can "inject"
into various locations in the shader pipeline.

For example, let's use user functions to create a simple "haze" effect::

    // haze_vertex:
    varying vec3 v_pos;
    void setup_haze(inout vec4 vertexView)
    {
        v_pos = vertexView.xyz;
    }
    
    // haze_fragment:
    varying vec3 v_pos;
    void apply_haze(inout vec4 color)
    {
        float dist = clamp( length(v_pos)/10000000.0, 0, 0.75 );
        color = mix(color, vec4(0.5, 0.5, 0.5, 1.0), dist);
    }
    
    // C++:
    VirtualProgram* vp = VirtualProgram::getOrCreate( stateSet );

    vp->setFunction( "setup_haze", haze_vertex,   ShaderComp::LOCATION_VERTEX_VIEW);
    vp->setFunction( "apply_haze", haze_fragment, ShaderComp::LOCATION_FRAGMENT_LIGHTING);


In this example, the function ``setup_haze`` is called from the built-in vertex shader
main() after the built-in vertex functions. The ``apply_haze`` function gets called from
the core fragment shader main() after the built-in fragment functions.

There are SIX injection points, as follows:

+----------------------------------------+-------------+------------------------------+
| Location                               | Shader Type | Signature                    |
+========================================+=============+==============================+
| ShaderComp::LOCATION_VERTEX_MODEL      | VERTEX      | void func(inout vec4 vertex) |
+----------------------------------------+-------------+------------------------------+
| ShaderComp::LOCATION_VERTEX_VIEW       | VERTEX      | void func(inout vec4 vertex) |
+----------------------------------------+-------------+------------------------------+
| ShaderComp::LOCATION_VERTEX_CLIP       | VERTEX      | void func(inout vec4 vertex) |
+----------------------------------------+-------------+------------------------------+
| ShaderComp::LOCATION_FRAGMENT_COLORING | FRAGMENT    | void func(inout vec4 color)  |
+----------------------------------------+-------------+------------------------------+
| ShaderComp::LOCATION_FRAGMENT_LIGHTING | FRAGMENT    | void func(inout vec4 color)  |
+----------------------------------------+-------------+------------------------------+
| ShaderComp::LOCATION_FRAGMENT_OUTPUT   | FRAGMENT    | void func(inout vec4 color)  |
+----------------------------------------+-------------+------------------------------+

Each VERTEX locations let you operate on the vertex in a particular *coordinate space*. 
You can alter the vertex, but you *must* leave it in the same space.

:MODEL:  Vertex is the raw, untransformed values from the geometry.
:VIEW:   Vertex is relative to the eyepoint, which lies at the origin (0,0,0) and 
         points down the -Z axis. In VIEW space, the orginal vertex has been
         transformed by ``gl_ModelViewMatrix``.
:CLIP:   Post-projected clip space. CLIP space lies in the [-w..w] range along all
         three axis, and is the result of transforming the original vertex by
         ``gl_ModelViewProjectionMatrix``.
         
The FRAGMENT locations are as follows.

:COLORING:  Functions here are called when resolving the fragment color before
            lighting is applied. Texturing or color adjustments typically 
            happen during this stage.
:LIGHTING:  Functions here affect the lighting applied to a fragment color. This is 
            where things like sun lighting, bump mapping or normal mapping would
            typically occur.
:OUTPUT:    This is where gl_FragColor is set. By default, the built-in fragment
            main() will set it for you. But you can set an OUTPUT shader to 
            replace this behavior with your own. A typical reason to do this would
            be to implement MRT rendering (see the osgearth_mrt example).


Shader Packages
---------------

Earlier we showed you how to inject functions using ``VirtualProgram``. 
The Shader Composition Framework also provides the concept of a ``ShaderPackage`` that supports
more advanced methods of shader management. We will talk about some of those now.


VirtualProgram Metadata
~~~~~~~~~~~~~~~~~~~~~~~

As we have seen, when you add a shader function to the pipeline using ``VirtualProgram``
you need to tell osgEarth the name of the GLSL function to call, and the location in
the pipeline at which to call it, like so::

    VirtualProgram* vp;
    ....
    vp->setFunction( "color_it_red", shaderSource, ShaderComp::LOCATION_FRAGMENT_COLORING );

That works. But if the function name or the inject location changes, you need to remember
to keep the GLSL code in sync with the ``setFunction()`` parameters.

It would be easier to specify this all in once place. A ``ShaderPackage`` lets you do just that.
Here is an example::

    #version 110
    
    #pragma vp_entryPoint  color_it_red
    #pragma vp_location    fragment_coloring
    #pragma vp_order       1.0
    
    void color_it_red(inout vec4 color)
    {
        color.r = 1.0;
    }
    
Now instead of calling ``VirtualProgram::setFunction()`` directory, you can create a
``ShaderPackage``, add your code, and call load to create the function on the ``VirtualProgram``::

    ShaderPackage package;
    package.add( shaderFileName, shaderSource );
    package.load( virtualProgram, shaderFileName );
    
It takes a "file name" because the shader can be in an external file.
But that is not a requirement. Read on for more details.

The ``vp_location`` values follow the code-based values, and are as follows::

    vertex_model
    vertex_view
    vertex_clip
    fragment_coloring
    fragment_lighting
    fragment_output


External GLSL Files
~~~~~~~~~~~~~~~~~~~

The ``ShaderPackage`` lets you load GLSL code from either a file or a string.
When you call the ``add`` method as show above, this tells the package to 
(a) first look for a file by that name and load from that file; and 
(b) if the file doesn't exist, use the code in the source string.

So let's look at this example::

    ShaderPackage package;
    package.add( "myshader.frag.glsl", backupSourceCode );
    ...
    package.load( virtualProgram, "myshader.frag.glsl" );

The package will try to load the shader from the GLSL file. It will search for it in the ``OSG_FILE_PATH``.
If it cannot find the file, it will load the shader from the backup source code associated with
that shader in the package.

osgEarth uses this technique internally to "inline" its stock shader code.
That gives you the option of deploying GLSL files with your application OR 
keeping them inline -- the application will still work either way.


Include Files
~~~~~~~~~~~~~

The ``ShaderPackage`` support the concept if *include files*. Your GLSL code
can *include* any other shaders in the same package by referencing their file names.
Use a custom ``#pragma`` to include another file::

    #pragma include myCode.vertex.glsl

Just as in C++, the *include* will load the other file (or source code) directly
inline. So the file you are including must be structured as if you had placed it right
in the including file. (That means it cannot have its own ``#version`` string, for example.)

Again: the *includer* and the *includee* must be registered with the same ``ShaderPackage``.

----

Concepts Specific to osgEarth
-----------------------------

Even though the VirtualProgram framework is included in the osgEarth SDK,
it really has nothing to do with map rendering. In this section we will go over some
of the things that osgEarth does with shader composition.

         
Terrain Variables
~~~~~~~~~~~~~~~~~

There are some built-in shader ``uniforms`` and ``variables`` that the osgEarth terrain
engine uses and that are available to the developer.

    *Important: Shader variables starting with the prefix ``oe_`` or ``osgearth_``
    are reserved for osgEarth internal use.*

Uniforms:

  :oe_tile_key:          (vec4) elements 0-2 hold the x, y, and LOD tile key values;
                         element 3 holds the tile's bounding sphere radius (in meters)
  :oe_layer_tex:         (sampler2D) texture applied to the current layer of the current tile
  :oe_layer_texc:        (vec4) texture coordinates for current tile
  :oe_layer_tilec:       (vec4) unit coordinates for the current tile (0..1 in x and y)
  :oe_layer_uid:         (int) Unique ID of the active layer
  :oe_layer_order:       (int) Render order of the active layer
  :oe_layer_opacity:     (float) Opacity [0..1] of the active layer

Vertex attributes:

  :oe_terrain_attr:      (vec4) elements 0-2 hold the unit height vector for a terrain
                         vertex, and element 3 holds the raw terrain elevation value
  :oe_terrain_attr2:     (vec4) element 0 holds the *parent* tile's elevation value;
                         elements 1-3 are currently unused.


Shared Image Layers
~~~~~~~~~~~~~~~~~~~

Sometimes you want to access more than one image layer at a time.
For example, you might have a masking layer that indicates land vs. water.
You may not actually want to *draw* this layer, but you want to use it to modulate
another visible layer.

You can do this using *shared image layers*. In the ``Map``, mark an image layer as
*shared* (using ``ImageLayerOptions::shared()``) and the renderer will make it available
to all the other layers in a secondary sampler.

    Please refer to ``osgearth_sharedlayer.cpp`` for a usage example!

