Working with Maps
=================

A **map** is the central data model in osgEarth.
It is a container for image, elevation, and feature layers.

Loading a Map from an Earth File
--------------------------------

The easiest way to render a Map is to load it from an *earth file*.
Since osgEarth uses OpenSceneGraph plugins, you can do this with a single line of code::

    osg::Node* globe = osgDB::readNodeFile("myglobe.earth");

You now have an ``osg::Node`` that you can add to your scene graph and display.
Seriously, it really is that simple!

This method of loading a Map is, more often than not, all that an application will
need to do. However if you want to create your Map using the API, read on.


Programmatic Map Creation
-------------------------

osgEarth provides an API for creating Maps at runtime.

The basic steps to creating a Map with the API are:

1. Create a Map object
2. Add imagery and elevation layers to the Map as you see fit
3. Create a ``MapNode`` that will render the Map object
4. Add your ``MapNode`` to your scene graph.

You can add layers to the map at any time::

    using namespace osgEarth;
    using namespace osgEarth::Drivers;
    
    #include <osgEarth/Map>
    #include <osgEarth/MapNode>
    #include <osgEarthDrivers/tms/TMSOptions>
    #include <osgEarthDrivers/gdal/GDALOptions>

    using namespace osgEarth;
    using namespace osgEarth::Drivers;
    ...

    // Create a Map and set it to Geocentric to display a globe
    Map* map = new Map();

    // Add an imagery layer (blue marble from a TMS source)
    {
        TMSOptions tms;
        tms.url() = "http://labs.metacarta.com/wms-c/Basic.py/1.0.0/satellite/";
        ImageLayer* layer = new ImageLayer( "NASA", tms );
        map->addImageLayer( layer );
    }

    // Add an elevationlayer (SRTM from a local GeoTiff file)
    {
        GDALOptions gdal;
        gdal.url() = "c:/data/srtm.tif";
        ElevationLayer* layer = new ElevationLayer( "SRTM", gdal );
        map->addElevationLayer( layer );
    }

    // Create a MapNode to render this map:
    MapNode* mapNode = new MapNode( map );
    ...
    
    viewer->setSceneData( mapNode );

    
Working with a MapNode at Runtime
----------------------------------

A ``MapNode`` is the scene graph node that renders a ``Map``. Whether you loaded your
map from an Earth File or created it using the API, you can access the Map and its
``MapNode`` at runtime to make changes. If you did not explicitly create a ``MapNode``
using the API, you will first need to get a reference to the ``MapNode`` to work with.
Use the static ``get`` function::

    // Load the map
    osg::Node* loadedModel = osgDB::readNodeFile("mymap.earth");

    // Find the MapNode
    osgEarth::MapNode* mapNode = MapNode::get( loadedModel );

    
Once you have a reference to the ``MapNode``, you can get to the map::

    // Add an OpenStreetMap image source
    TMSOptions driverOpt;
    driverOpt.url() = "http://tile.openstreetmap.org/";
    driverOpt.tmsType() = "google";

    ImageLayerOptions layerOpt( "OSM", driverOpt );
    layerOpt.profile() = ProfileOptions( "global-mercator" );

    ImageLayer* osmLayer = new ImageLayer( layerOpt );
    mapNode->getMap()->addImageLayer( osmLayer );

    
You can also remove or re-order layers::

    // Remove a layer from the map.  All other layers are repositioned accordingly
    mapNode->getMap()->removeImageLayer( layer );

    // Move a layer to position 1 in the image stack
    mapNode->getMap()->moveImageLayer( layer, 1 );


Working with Layers
-------------------

The ``Map`` contains ``ImageLayer`` and ``ElevationLayer`` objects.
These contain some properties that you can adjust at runtime.
For example, you can toggle a layer on or off or adjust an ``ImageLayer`` opacity using the API::

    ImageLayer* layer;
    ...
    layer->setOpacity( 0.5 );  // makes the layer partially transparent

