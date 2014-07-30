Utilities SDK
=============

The osgEarth *Utils* namespace includes a variety of useful classes for interacting
with the map. None of these are strictly necessary for using osgEarth, but they do
make it easier to perform some common operations.


AutoScale
---------

*AutoScale* is a special *Render Bin* that will scale geometry from meters to pixels.
That is: if you have an object that is 10 meters across, *AutoScale* will draw it in
the space of 10 pixels (at scale 1.0) regardless of its distance from the camera.
The effect is similar to OSG's ``AutoTransform::setAutoScaleToScreen`` method but is
done in a shader and does not require any special nodes.

To activate auto-scaling on a node::

    node->getOrCreateStateSet()->setRenderBinDetails( 0, osgEarth::AUTO_SCALE_BIN );
    
And to deactivate it:

    node->getOrCreateStateSet()->setRenderBinToInherit();


DataScanner
-----------

The ``DataScanner`` will recursively search a directory tree on the local filesystem
for files that it can load as ``ImageLayer`` objects. It is a quick and easy way to 
load a full directory of images at layers.

**NOTE** that only the *MP Terrain Engine* supports an unlimited number of image layers,
so it is wise to use that engine in conjunction with the DataScanner.

Use DataScanner like this::

    DataScanner scanner;
    ImageLayerVector imageLayers;
    scanner.findImageLayers( rootFolder, extensions, imageLayers );
    
You can then add the image layes to your ``Map`` object.

The ``extensions`` parameter lets you filter files by extension. For example, pass in 
"tif,ecw" to only consider files with those extensions. Separate multiple extensions
with a comma.


DetailTexture
-------------

``DetailTexture`` is a terrain controller that will apply a non-geospatial texture
cross the terrain. This is an old trick that you can use to generate "noise" that makes
a low resolution terrain appear more detailed::

    DetailTexture* detail = new DetailTexture();
    detail->setImage( osgDB::readImageFile("mytexture.jpg") );
    detail->setIntensity( 0.5f );
    detail->setImageUnit( 4 );
    mapnode->getTerrainEngine()->addEffect( detail );

Try the example. Zoom in fairly close to the terrain to see the effect::

    osgearth_detailtex readymap.earth



LOD Blending
------------

``LODBlending`` is a terrain controller that will attempt to smoothly morph vertices
and image textures from one LOD to the next as you zoom in or out. Basic usage is::

    LODBlending* effect = new LODBlending();
    mapnode->getTerrainEngine()->addEffect( effect );

Caveats: It requires that the terrain elevation tile size dimensions be odd-numbered
(e.g., 17x17, which is the default.) You can use the ``MapOptions::elevationTileSize``
property to configure this, or set ``elevation_tile_size`` in your earth file if you
want to change it::

    <map>
        <options elevation_tile_size="15" ...

For a demo, run this example and zoom into a mountainous area::

    osgearth_viewer lod_blending.earth

LOD blending supports the following properties (earth file and API):

    :delay:            Time to wait before starting a blending transition (seconds)
    :duration:         Duration of the blending transition (seconds)
    :vertical_scale:   Factor by which to vertically scale the terrain (default = 1.0)
    :blend_imagery:    Whether to blend imagery LODs (true)
    :blend_elevation:  Whether to morph elevation LODs (true)


Logarithmic Depth Buffer
------------------------

In whole-earth applications it's common that you want to see something up close (like
an aircraft at altitude) while seeing the Earth and its horizon off in the distance.
This poses a problem for modern graphic hardware because the standard depth buffer
precision heavily favors objects closer to the camera, and viewing such a wide range
of objects leads to "z-fighting" artifacts.

The ``LogarithmicDepthBuffer`` is one way to solve this problem. It uses a shader to
re-map the GPU's depth buffer values so they can be put to better use in this type
of scenario.

It's easy to install::

    LogarithmicDepthBuffer logdepth;
    logdepth->install( view->getCamera() );
    
Or you can activate it from ``osgearth_viewer`` or other examples::

    osgearth_viewer --logdepth ...

Since it does alter the projection-space coordinates of your geometry at draw time,
you do need to be careful that you aren't doing anything ELSE in clip space in your
own custom shaders that would conflict with this.

(10-Jul-2014: Some osgEarth features are incompatible with the log depth buffer;
namely, GPU clamping and Shadowing. Depth Offset works correctly though.)


Formatters
----------

Use *Formatters* to format geospatial coordinates as a string. There are two stock formatters,
the ``LatLongFormatter`` and the ``MGRSFormatter``. A formatter takes a ``GeoPoint`` and
returns a ``std::string`` like so::

    LatLongFormatter formatter;
    GeoPoint point;
    ....
    std::string = formatter.format( point );

LatLongFormatter
~~~~~~~~~~~~~~~~

The ``LatLongFormatter`` takes coordinates and generates a string. It supports the following
formats:

    :FORMAT_DECIMAL_DEGREES:            34.04582
    :FORMAT_DEGREES_DECIMAL_MINUTES:    34.20:30
    :FORMAT_DEGREES_MINUTES_SECONDS:    34:14:30

You can also specify options for the output string:

    :USE_SYMBOLS:   Use the degrees, minutes and seconds symbology
    :USE_COLONS:    Use colons between the components
    :USE_SPACES:    Use spaces between the components


MGRSFormatter
~~~~~~~~~~~~~

The ``MGRSFormatter`` echos a string according to the `Military Grid Reference System`_. 
Technically, an MGRS coordinate represents a *region* rather than an exact point, so you
have to specifiy a *precision* qualifier to control the size of the represented region.
Example::

    MGRSFormatter mgrs( MGRFormatter::PRECISION_1000M );
    std::string str = mgrs.format( geopoint );

.. _Military Grid Reference System: http://en.wikipedia.org/wiki/Military_grid_reference_system


MouseCoordsTool
---------------

The ``MouseCoordsTool`` reports the map coordinates under the mouse (or other pointing device).
Install a callback to respond to the reports. ``MouseCoordsTool`` is an ``osgGA::GUIEventHandler``
that you can install on a ``Viewer`` or any ``Node``, like so::

    MouseCoordsTool* tool = new MouseCoordsTool();
    tool->addCallback( new MyCallback() );
    viewer.addEventHandler( tool );
    
Create your own callback to respond to reports. Here is an example that prints the X,Y under the
mouse to a *Qt* status bar::

    struct PrintCoordsToStatusBar : public MouseCoordsTool::Callback
    {
    public:
        PrintCoordsToStatusBar(QStatusBar* sb) : _sb(sb) { }

        void set(const GeoPoint& p, osg::View* view, MapNode* mapNode)
        {
            std::string str = osgEarth::Stringify() << p.y() << ", " << p.x();
            _sb->showMessage( QString(str.c_str()) );
        }

        void reset(osg::View* view, MapNode* mapNode)
        {
            _sb->showMessage( QString("out of range") );
        }

        QStatusBar* _sb;
    };

For your convenience, ``MouseCoordsTool`` also comes with a stock callback that will
print the coords to ``osgEarthUtil::Controls::LabelControl``. You can even pass a
``LabelControl`` to the contructor to make it even easier.


NormalMap
---------

The ``NormalMap`` effect will use an ``ImageLayer`` as a bump map texture, adding
apparent detail to the terrain. 

A *normal map* is a kind of *bump map* in which each texel represents an XYZ normal
vector instead of an RGB color value. The GPU can then use this information to apply
lighting to the terrain on a per-pixel basis instead of per-vertex, rendering a
more detailed-looking surface with the same number of triangles.

First you need to create a normal map layer. You can use the **noise** driver to do
this. The setup looks like this in the earth file::

    <image name="bump" driver="noise" shared="true" visible="false">
        <normal_map>true</normal_map>
    </image>
    
The **noise driver** generates Perlin noise; this will will the image with pseudo-
random normal vectors. (Setting ``normal_map`` to ``true`` is what tells the driver
to make normal vectors instead of RGB values. You should also set ``shared`` to 
``true``; this will make the normal map available to the shader pipeline so that it
can do the custom lighting calculations.)

Once you have the image layer set up, install the ``NormalMap`` terrain effect and 
point it at our normal map layer. From the earth file::

    <map>
        ...
        <external>
            <normal_map layer="bump"/>
        </external>

Or from code::

    NormalMap* normalMap = new NormalMap();
    normalMap->setNormalMapLayer( myBumpLayer );
    mapnode->getTerrainEngine()->addEffect( normalMap );
    
Please refer to the **normalmap.earth** example for a demo.


VerticalScale
-------------

``VerticalScale`` scales the height values of the terrain. Basic usage is::

    VerticalScale* scale = new VerticalScale();
    scale->setScale( 2.0 );
    mapnode->getTerrainEngine()->addEffect( scale );

For a demo, run this example::

    osgearth_verticalscale readymap.earth
