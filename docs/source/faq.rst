FAQ
===

**Sections:**

* `Common Usage`_
* `Other Terrain Formats`_
* `Community and Support`_
* `Licensing`_


----

Common Usage
------------

How do I place a 3D model on the map?
.....................................

    One way to position a 3D model is to use the ``ModelNode``. Here is the basic idea::

        using namespace osgEarth;
        using namespace osgEarth::Symbology;
        ...

        // load your model:
        osg::Node* myModel = osgDB::readNodeFile(...);
        
        // establish the coordinate system you wish to use:
        const SpatialReference* latLong = SpatialReference::get("wgs84");
        
        // construct your symbology:
        Style style;
        style.getOrCreate<ModelSymbol>()->setModel( myModel );
        
        // make a ModelNode:
        ModelNode* model = new ModelNode( mapNode, style );
        
        // Set its location.
        model->setPosition( GeoPoint(latLong, -121.0, 34.0, 1000.0, ALTMODE_ABSOLUTE) );

    If you just want to make a ``osg::Matrix`` so you can position a model using your own 
    ``osg::MatrixTransform``, you can use the ``GeoPoint`` class like so::
    
        GeoPoint point(latLong, -121.0, 34.0, 1000.0, ALTMODE_ABSOLUTE);
        osg::Matrix matrix;
        point.createLocalToWorld( matrix );
        myMatrixTransform->setMatrix( matrix );

    Look at the ``osgearth_annotation.cpp`` sample for more inspiration.
    

How do make the terrain transparent?
....................................

    By default, the globe will be opaque white when there are no image layers, or when all the image
    layers have their opacities set to zero. To make the underlying globe transparent, set the 
    base color of the terrain to a transparent color like so::

        <map>
            <options>
                <terrain color="#ffffff00" ...

    In code, this option is found in the ``MPTerrainEngineOptions`` class::
    
        #include <osgEarthDrivers/engine_mp/MPTerrainEngineOptions>
        using namespace osgEarth::Drivers;
        ...
        MPTerrainEngineOptions options;
        options.color() = osg::Vec4(1,1,1,0);


How do I set the resolution of terrain tiles?
.............................................

    Each tile is a grid of vertices. The number of vertices can vary depending on source data
    and settings. By default (when you have no elevation data) it is an 15x15 grid, tessellated
    into triangles.
    
    If you do have elevation data, osgEarth will use the tile size of the first elevation layer 
    to decide on the overall tile size for the terrain.

    You can cotnrol this in a couple ways. If you have elevation data, you can set the
    ``tile_size`` property on the elevation layer. For example::
    
        <elevation name="srtm" driver="gdal">
            <url>...</url>
            <tile_size>31</tile_size>
        </elevation>
        
    That will read data as a grid of 31x31 vertices. If this is your first elevation layer,
    osgEarth will render tiles at a resolution of 31x31.

    Or, you can expressly set the terrain's tile size overall by using the Map options.
    osgEarth will then resample all elevation data to the size you specify::

        <map>
            <options>
                <elevation_tile_size>31</elevation_tile_size>
                ...


----

Other Terrain Formats
---------------------

Does osgEarth work with VirtualPlanetBuilder?
.............................................

	VirtualPlanetBuilder_ (VPB) is a command-line terrain generation tool. Before osgEarth
	came along, VPB	was probably the most-used open source tool for building terrains for
	OSG appliations. We	mention is here because many people ask questions about loading 
	VPB models or transitioning from VPB to osgEarth.
	
	osgEarth differs from VPB in that:
	
	* VPB builds static terrain models and saves them to disk. osgEarth generates terrain on
	  demand as your application runs; you do not (and cannot) save a model to disk.
	* Changing a VPB terrain generally requires that you rebuild the model. osgEarth does not
	  require a preprocessing step since it builds the terrain at run time.
	* osgEarth and VPB both use *GDAL* to read many types of imagery and elevation data from
	  the local file system. osgEarth also supports network-based data sources through its
	  plug-in framework.

	osgEarth has a *VPB driver* for "scraping" elevation and imagery tiles from a VPB model.
	See the ``vpb_earth_bayarea.earth`` example in the repo for usage.
	
	**Please Note** that this driver only exists as a **last resort** for people that have a VPB
	model but no longer have access to the source data from which it was built. If at all
	possible you should feed your source data directly into osgEarth instead of using the VPB
	driver.


Can osgEarth load TerraPage or MetaFlight?
..........................................

	osgEarth cannot natively load TerraPage (TXP) or MetaFlight. However, osgEarth does have a
	"bring your own terrain" plugin that allows you to load an external model and use it as your
	terrain. The caveat is that since osgEarth doesn't know anything about your terrain model, you
	will not be able to use some of the features of osgEarth (like being able to add or remove layers).
	
	For usage formation, please refer to the ``byo.earth`` example in the repo.

.. _VirtualPlanetBuilder:	http://www.openscenegraph.com/index.php/documentation/tools/virtual-planet-builder


----

Community and Support
---------------------

What is the "best practice" for using GitHub?
.............................................

	The best way to work with the osgEarth repository is to make your own clone on GitHub
	and to work from that clone. Why not work directly against the main repository? You
	can, but if you need to make changes, bug fixes, etc., you will need your own clone
	in order to issue Pull Requests.
	
	1. Create your own GitHub account and log in.
	2. Clone the osgEarth repo.
	3. Work from your clone. Sync it to the main repository peridocially to get the
	   latest changes.


How do I submit changes to osgEarth?
....................................

	We accept contributions and bug fixes through GitHub's `Pull Request`_ mechanism.

	First you need your own GitHub account and a fork of the repo (see above). Next,
	follow these guidelines:
	
	1. Create a *branch* in which to make your changes.
	2. Make the change.
	3. Issue a *pull request* against the main osgEarth repository.
	4. We will review the *PR* for inclusion.

	If we decide NOT to include your submission, you can still keep it in your cloned
	repository and use it yourself. Doing so maintains compliance with the osgEarth
	license since your changes are still available to the public - even if they are
	not merged into the master repository.
	
.. _Pull Request:   https://help.github.com/articles/using-pull-requests


Can I hire someone to help me with osgEarth?
............................................

    Of course! We at Pelican Mapping are in the business of supporting users of
    the osgEarth SDK and are available for contracting, training, and integration
    services. The easiest way to get in touch with us is through our web site
    `contact form`_.
    
.. _contact form:   http://pelicanmapping.com/?page_id=2


----

Licensing
---------

Can I use osgEarth in a commercial product?
...........................................

	Yes. The license permits use in a commercial product. The only requirement is that
	any changes you make to the actual osgEarth library *itself* be made available
	under the same license as osgEarth. You do *not* need to make other parts of your
	application public.


Can I use osgEarth in an iOS app?
.................................

	Yes. Apple's policy requires only statically linked libraries. Technically, the
	LGPL does not support static linking, but we grant an exception in this case.
