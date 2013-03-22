Shader Composition
==================

osgEarth uses GLSL shaders in several of its rendering modes. By default,
osgEarth will detect the capabilities of your graphics hardware and
automatically select an appropriate mode to use.

Since osgEarth relies on shaders, and since you as the developer may wish
to use your own shader code as well, osgEarth provides a *shader composition*
framework. This allows you a great deal of flexibility when incorporating
your own shaders into osgEarth.

There are several ways to integrate your own shader code into osgEarth.
We discuss these below. But first it is important to understand the basics of
osgEarth's shader composition framework.

Framework Basics
----------------

osgEarth installs default shaders for rendering. The default shaders are shown
below. The ``LOCATION_*`` designators allow you to inject functions at
various points in the shader's execution.

Here is the pseudo-code for osgEarth's built-in shaders::

    // VERTEX SHADER:

    void main(void)
    {
        vec4 vertex = gl_Vertex;

        // "LOCATION_VERTEX_MODEL" user functions are called here:
        model_func_1(vertex);
        ...

        vertex = gl_ModelViewMatrix * vertex;

        // "LOCATION_VERTEX_VIEW" user functions are called here:
        view_func_1(vertex);
        ...

        vertes = gl_ProjectionMatrix * vertex;
        
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


VirtualProgram
--------------

osgEarth include an OSG state attribute called ``VirtualProgram`` that performs
the runtime shader composition. Since ``VirtualProgram`` is an ``osg::StateAttribute``,
you can attach one to any node in the scene graph. Shaders that belong to a
``VirtualProgram`` can override shaders lower down on the attribute stack
(i.e., higher up in the scene graph). In the way you can override individual shader
functions in osgEarth.

The sections below on integration will demonstrate how to use ``VirtualProgram``.


Integrating Custom Shaders
--------------------------

There are two ways to use shader composition in osgEarth.

* Injecting user functions
* Overriding osgEarth's built-in functions with a custom ``ShaderFactory``

 
Injecting User Functions
~~~~~~~~~~~~~~~~~~~~~~~~

In the core shader code above, osgEarth calls into user functions.
These don't exist in the default shaders that osgEarth generates;
rather, they represent code that you as the developer can "inject"
into various locations in the built-in shaders.

For example, let's use User Functions to create a simple "haze" effect.
(NOTE: see this example in its entirety in osgearth_shadercomp.cpp)::

    static char s_hazeVertShader[] =
        "varying vec3 v_pos; \n"
        "void setup_haze(inout vec4 vertexVIEW) \n"
        "{ \n"
        "    v_pos = vec3(vertexVIEW); \n"
        "} \n";

    static char s_hazeFragShader[] =
        "varying vec3 v_pos; \n"
        "void apply_haze(inout vec4 color) \n"
        "{ \n"
        "    float dist = clamp( length(v_pos)/10000000.0, 0, 0.75 ); \n"
        "    color = mix(color, vec4(0.5, 0.5, 0.5, 1.0), dist); \n"
        "} \n";

    osg::StateAttribute*
    createHaze()
    {
        osgEarth::VirtualProgram* vp = new osgEarth::VirtualProgram();

        vp->setFunction( "setup_haze", s_hazeVertShader, osgEarth::ShaderComp::LOCATION_VERTEX_VIEW);
        vp->setFunction( "apply_haze", s_hazeFragShader, osgEarth::ShaderComp::LOCATION_FRAGMENT_LIGHTING);

        return vp;
    }

    ...
    sceneGraph->getOrCreateStateSet()->setAttributeAndModes( createHaze() );

In this example, the function ``setup_haze`` is called from the core vertex shader
after the built-in vertex functions. The ``apply_haze`` function gets called from
the core fragment shader after the built-in fragment functions.

There are FIVE injection points, as follows:

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

Each VERTEX locations let you operate on the vertex in a particular *coordinate space*. 
You can alter the vertex, but you *must* leave it in the same space.

:MODEL:  Vertex is the raw, untransformed values from the geometry.
:VIEW:   Vertex is relative to the eyepoint, which lies at the origin (0,0,0) and 
         points down the -Z axis. In VIEW space, the orginal vertex has been
         transformed by ``gl_ModelViewMatrix``.
:CLIP:   Post-projected clip space. CLIP space lies in the [-w..w] range along all
         three axis, and is the result of transforming the original vertex by
         ``gl_ModelViewProjectionMatrix``.


Customizing the Shader Factory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is amore advanced topic.
If you want to replace osgEarth's built-in shader functions, you can install a custom
``ShaderFactory``. The ``ShaderFactory`` is stored in the osgEarth ``Registry`` and contains
all the methods for creating the built-in functions. You can install your own ``ShaderFactory``
like so::

    #include <osgEarth/ShaderFactory>
    ...

    class CustomShaderFactory : public osgEarth::ShaderFactory
    {
        ... override desired methods here ...
    };
    ...

    osgEarth::Registry::instance()->setShaderFactory( new CustomShaderFactory() );

This method is good for replacing osgEarth's built-in lighting shader code.
HOWEVER: be aware that override the built-in texturing functions may not work.
This is because osgEarth's image layer composition mechanisms override these methods
themselves to perform layer rendering.


Sampling Image Layers
---------------------

What if you want to access one of the Map's image layers from your shader?
Since osgEarth internally manages image layers, texture units, and composition,
it is not as simple of calling GLSL's ``texture2D()`` function. Here's how to do it.

Use the ``TextureCompositor`` to create a sampler function for the layer you want
to query. You can then call this sampler function from your shader. Here's an example::

    // assume "layer" is the image layer you want to sample, and "vp" is a VirtualProgram state attribute:
    osgEarth::ImageLayer* layer;
    osgEarth::VirtualProgram* vp;

    // first get a reference to the texture compositor.
    osgEarth::TerrainEngineNode* engine = mapNode->getTerrainEngine();
    osgEarth::TextureCompositor* comp = engine->getTextureCompositor();

	// next, find our layer
	layer = mapNode->getMap()->getImageLayerByName("MyLayer");

    // next, request a sampling shader for the layer in question.
    osg::Shader* sampler = comp->createSamplerFunction( layer->getUID(), "sampleMyLayer", osg::Shader::FRAGMENT );

    // add it to your VirtualProgram:
    vp->setShader( "sampleMyLayer", sampler );

Then in your shader code, you can call the "sampleMyLayer" function::

    // FRAGMENT SHADER
    void sampleMyLayer(void);  // declaration
    ...
    void someFunction()
    {
        ...
        vec4 texel = sampleMyLayer();
    }

The sampler function will automatically sample the proper sampler with the current
texture coordinate.


System Uniforms
---------------

In addition the the OSG system uniforms (which all start with "osg_"), osgEarth
provides various uniforms. They are:

  :osgearth_LightingEnabled:     whether GL lighting is enabled (bool)
  :osgearth_ImageLayerEnabled:   whether image layer N is enabled (bool[])
  :osgearth_CameraElevation:     distance from camera to ellipsoid/Z=0 (float)

