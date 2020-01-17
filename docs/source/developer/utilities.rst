Utilities SDK
=============

The osgEarth *Utils* namespace includes a variety of useful classes for interacting
with the map. None of these are strictly necessary for using osgEarth, but they do
make it easier to perform some common operations.


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

The ``MGRSFormatter`` constructs a string according to the `Military Grid Reference System`_. 
Technically, an MGRS coordinate represents a *region* rather than an exact point, so you
have to specify a *precision* qualifier to control the size of the represented region.
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
``LabelControl`` to the constructor to make it even easier.

