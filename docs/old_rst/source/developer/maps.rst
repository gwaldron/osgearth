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
    #include <osgEarth/TMS>
    #include <osgEarth/GDAL>

    using namespace osgEarth;
    ...

    // Create a Map and set it to Geocentric to display a globe
    Map* map = new Map();

    // Add an imagery layer (blue marble from a TMS source)
    {
        TMSImageLayer* layer = new TMSImageLayer();
        layer->setName("ReadyMap");
        layer->setURL("http://readymap.org/readymap/tiles/1.0.0/7/");
        map->addLayer(layer);
    }

    // Add an elevation layer (SRTM from a local GeoTiff file)
    {
        GDALElevationLayer* layer = new GDALElevationLayer();
        layer->setName("SRTM");
        layer->setURL("c:/data/srtm.tif");
        map->addLayer( layer );
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
    osgEarth::MapNode* mapNode = osgEarth::MapNode::get( loadedModel );

    
Once you have a reference to the ``MapNode``, you can get to the map::

    // Add an OpenStreetMap image source
    XYZImageLayer* osmLayer = new XYZImageLayer();
    osmLayer->setName("OSM");
    osmLayer->setURL("http://[abc].tile.openstreetmap.org/{z}/{x}/{y}.png");
    osmLayer->setProfile(Profile::create("spherical-mercator"));
    osmLayer->setAttribution("Copyright OpenStreetMap Contributors");

    mapNode->getMap()->addImageLayer( osmLayer );

    
You can also remove or re-order layers::

    // Remove a layer from the map.  All other layers are repositioned accordingly
    mapNode->getMap()->removeImageLayer( layer );

    // Move a layer to position 1 in the image stack
    mapNode->getMap()->moveImageLayer( layer, 1 );
