#include <osgEarth/GeographicTileBuilder>
#include <osgEarth/Mercator>
#include <osgEarth/TerrainTileEdgeNormalizerUpdateCallback>
#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/CoordinateSystemNode>
#include <osgDB/ReadFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

GeographicTileBuilder::GeographicTileBuilder( 
    MapConfig* _map,
    const std::string& _url_template,
    const osgDB::ReaderWriter::Options* _global_options ) :
TileBuilder( _map, _url_template, _global_options )
{
    //NOP
}

bool
GeographicTileBuilder::addChildren( osg::Group* tile_parent, const TileKey* key )
{
    osg::ref_ptr<osg::Node> q0 = createQuadrant(key->getSubkey(0));
    osg::ref_ptr<osg::Node> q1 = createQuadrant(key->getSubkey(1));
    osg::ref_ptr<osg::Node> q2;
    osg::ref_ptr<osg::Node> q3;

    bool allQuadrantsCreated = (q0.valid() && q1.valid());

    if ( key->getLevelOfDetail() > 0 || dynamic_cast<const MercatorTileKey*>( key ) )
    {
        q2 = createQuadrant( key->getSubkey( 2 ) );
        q3 = createQuadrant( key->getSubkey( 3 ) );
        allQuadrantsCreated = (allQuadrantsCreated && q2.valid() && q3.valid());
    }

    if (allQuadrantsCreated)
    {
        if (q0.valid()) tile_parent->addChild(q0.get());
        if (q1.valid()) tile_parent->addChild(q1.get());
        if (q2.valid()) tile_parent->addChild(q2.get());
        if (q3.valid()) tile_parent->addChild(q3.get());
    }
    else
    {
        osg::notify(osg::INFO) << "Couldn't create all 4 quadrants for " << key->str() << " time to stop subdividing!" << std::endl;
    }
    return allQuadrantsCreated;
}


osg::Node*
GeographicTileBuilder::createQuadrant( const TileKey* key )
{
    double min_lon, min_lat, max_lon, max_lat;
    if ( !key->getGeoExtents( min_lon, min_lat, max_lon, max_lat ) )
    {
        osg::notify( osg::WARN ) << "GET EXTENTS FAILED!" << std::endl;
        return NULL;
    }

    int tile_size = key->getProfile().pixelsPerTile();

    //Create the images
    std::vector<osg::ref_ptr<osg::Image>> images;
    //TODO: select/composite:
    if ( image_sources.size() > 0 )
    {
        //Add an image from each image source
        for (unsigned int i = 0; i < image_sources.size(); ++i)
        {
            osg::ref_ptr<osg::Image> image = image_sources[i]->createImage(key);
            if (!image.valid())
            {
                osg::notify(osg::INFO) << "createQuadrant: Could not create image for " << key->str() << std::endl;
                return NULL;
            }
            images.push_back(image);
        }
    }

    osgTerrain::Locator* geo_locator = new osgTerrain::Locator();
    geo_locator->setCoordinateSystemType( osgTerrain::Locator::GEOGRAPHIC ); // sort of.
    geo_locator->setTransformAsExtents( min_lon, min_lat, max_lon, max_lat );

    osg::HeightField* hf = NULL;

    //TODO: select/composite.
    if ( heightfield_sources.size() > 0 )
    {
        hf = heightfield_sources[0]->createHeightField( key );
    }

    if ( !hf )
    {
        // make an empty one if we couldn't fetch it
        hf = new osg::HeightField();
        hf->allocate( 8, 8 );
        for(unsigned int i=0; i<hf->getHeightList().size(); i++ )
        {
            hf->getHeightList()[i] = 0.0; 
        }
    }

    //Scale the heightfield elevations to degrees
    scaleHeightFieldToDegrees(hf);

    hf->setOrigin( osg::Vec3d( min_lon, min_lat, 0.0 ) );
    hf->setXInterval( (max_lon - min_lon)/(double)(hf->getNumColumns()-1) );
    hf->setYInterval( (max_lat - min_lat)/(double)(hf->getNumRows()-1) );
    hf->setBorderWidth( 0 );
    hf->setSkirtHeight( 0 );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( geo_locator );
    hf_layer->setHeightField( hf );

    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();
    tile->setLocator( geo_locator );
    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );
    tile->setTileID(key->getTileId());

    //Attach an updatecallback to normalize the edges of TerrainTiles.
    tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
    tile->setDataVariance(osg::Object::DYNAMIC);


    if ( images.size() > 0 )
    {
        for (unsigned int i = 0; i < images.size(); ++i)
        {
            if (images[i].valid())
            {
                osgTerrain::Locator* img_locator = geo_locator;
                if ( dynamic_cast<const MercatorTileKey*>( key ) )
                    img_locator = new MercatorLocator( *geo_locator, tile_size, key->getLevelOfDetail() );

                osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( images[i].get() );
                img_layer->setLocator( img_locator );
                tile->setColorLayer( i, img_layer );
            }
        }
    }
    
    osg::Vec3d centroid( (max_lon+min_lon)/2.0, (max_lat+min_lat)/2.0, 0 );

    double max_range = 1e10;
    double radius = (centroid-osg::Vec3d(min_lon,min_lat,0)).length();
    double min_range = radius * map->getMinTileRangeFactor();

    //Set the skirt height of the heightfield
    hf->setSkirtHeight(radius * map->getSkirtRatio());
   
    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setCenter( centroid );
    plod->addChild( tile, min_range, max_range );
    plod->setFileName( 1, createURI( key ) );
    plod->setRange( 1, 0.0, min_range );

    return plod;
}

osg::CoordinateSystemNode*
GeographicTileBuilder::createCoordinateSystemNode() const
{
    osg::CoordinateSystemNode* csn = new osg::CoordinateSystemNode();
    csn->setEllipsoidModel( NULL );
    csn->setCoordinateSystem( "+proj=eqc +lat_ts=0 +lon_0=0 +x_0=0 +y_0=0" );
    csn->setFormat( "PROJ4" );
    return csn;
}

void
GeographicTileBuilder::scaleHeightFieldToDegrees(osg::HeightField *hf)
{
    //The number of degrees in a meter at the equator
    float scale = 1.0f/111319.0f;
    if (hf)
    {
        for (unsigned int i = 0; i < hf->getHeightList().size(); ++i)
        {
            hf->getHeightList()[i] *= scale;
        }
    }
    else
    {
        osg::notify(osg::WARN) << "scaleHeightFieldToDegrees heightfield is NULL" << std::endl;
    }
}
