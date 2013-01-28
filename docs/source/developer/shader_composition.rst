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
below. Note the following function types:

* **built-in functions**: functions that osgEarth installs by default
  (but that you can override)
* **user functions**: functions that you "inject" into the shader either
  before (pre) or after (post) the built-ins.

Here is the pseudo-code for osgEarth's built-in shaders::

    // VERTEX SHADER:

    void main(void)
    {
        gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
        ...

        // "LOCATION_VERTEX_PRE_TEXTURING" user functions are called here:
        pre_tex_func_1(...);
        ...

        // the built-in functions are called next:
        osgearth_vert_setupTexturing();

        // "LOCATION_VERTEX_PRE_LIGHTING" user functions are called here:
        pre_light_func_1(...);
        ...

        if ( lighting_enabled )
            osgearth_vert_setupLighting();

        // "LOCATION_VERTEX_POST_LIGHTING" user functions are called last:
        post_light_func_1(...);
    }  


    // FRAGMENT SHADER:

    void main(void)
    {
        vec4 color = vec4(1,1,1,1);
        ...

        // "LOCATION_FRAGMENT_PRE_TEXTURING" user functions are called here:
        pre_tex_func_1(color);
        ...

        // then the built-in osgEarth functions are called:
        osgearth_frag_applyTexturing(color);

        // "LOCATION_FRAGMENT_PRE_LIGHTING" user functions are called here:
        pre_light_func_1(...);
        ...

        if (osgearth_lighting_enabled)
            osgearth_frag_applyLighting(color);
        ...

        // "LOCATION_FRAGMENT_POST_LIGHTING" user functions are called last:
        post_light_func_1(color);
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
* Overriding osgEarth's built-in functions with a custom !ShaderFactory

 
Injecting User Functions
~~~~~~~~~~~~~~~~~~~~~~~~

In the core shader code above, you see "user functions" (e.g., "pre_tex_func_1()" etc). These don't exist in the default shaders that osgEarth generates; rather, they represent code that you as the developer can "inject" into various locations in the built-in shaders.

For example, let's use User Functions to create a simple "haze" effect.
(NOTE: see this example in its entirety in osgearth_shadercomp.cpp)::

    static char s_hazeVertShader[] =
        "varying vec3 v_pos; \n"
        "void setup_haze() \n"
        "{ \n"
        "    v_pos = vec3(gl_ModelViewMatrix * gl_Vertex); \n"
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

        vp->setFunction( "setup_haze", s_hazeVertShader, osgEarth::ShaderComp::LOCATION_VERTEX_POST_LIGHTING);
        vp->setFunction( "apply_haze", s_hazeFragShader, osgEarth::ShaderComp::LOCATION_FRAGMENT_POST_LIGHTING);

        return vp;
    }

    ...
    sceneGraph->getOrCreateStateSet()->setAttributeAndModes( createHaze() );

In this example, the function ``setup_haze`` is called from the core vertex shader
after the built-in vertex functions. The ``apply_haze`` function gets called from
the core fragment shader after the built-in fragment functions.

There are SIX injection points, as follows:

+------------------------------------------------+-------------+-----------------------------+
| Location                                       | Shader Type | Signature                   |
+================================================+=============+=============================+
| ShaderComp::LOCATION_VERTEX_PRE_TEXTURING      | VERTEX      | void func(void)             |
+------------------------------------------------+-------------+-----------------------------+
| ShaderComp::LOCATION_VERTEX_PRE_LIGHTING       | VERTEX      | void func(void)             |
+------------------------------------------------+-------------+-----------------------------+
| ShaderComp::LOCATION_VERTEX_POST_LIGHTING      | VERTEX      | void func(void)             |
+------------------------------------------------+-------------+-----------------------------+
| ShaderComp::LOCATION_FRAGMENT_PRE_TEXTURING    | FRAGMENT    | void func(inout vec4 color) |
+------------------------------------------------+-------------+-----------------------------+
| ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING     | FRAGMENT    | void func(inout vec4 color) |
+------------------------------------------------+-------------+-----------------------------+
| ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING     | FRAGMENT    | void func(inout vec4 color) |
+------------------------------------------------+-------------+-----------------------------+

As you can see, user functions literally let you inject code into the main shaders seamlessly.


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
    osgEarth::TerrainEngine* engine = mapNode->getTerrainEngine();
    osgEarth::TextureCompositor* comp = engine->getTextureCompositor();

    // next, request a sampling shader for the layer in question.
    osg::Shader* sampler = comp->createSamplerFunction( layer, "sampleMyLayer", osg::Shader::FRAGMENT );

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

