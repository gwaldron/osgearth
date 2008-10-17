#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/PlateCarre>
#include <osgEarth/Mercator>
#include <osgEarth/TerrainTileEdgeNormalizerUpdateCallback>
#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osg/Version>
#include <osgDB/ReadFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

//#define WGS84_WKT "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],TOWGS84[0,0,0,0,0,0,0],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9108\"]],AXIS["Lat",NORTH],AXIS["Long",EAST],AUTHORITY["EPSG","4326"]]

GeocentricTileBuilder::GeocentricTileBuilder( 
    MapConfig* map, 
    const std::string& url_template,
    const osgDB::ReaderWriter::Options* global_options ) :
TileBuilder( map, url_template, global_options )
{
    //NOP   
}


osg::Node*
GeocentricTileBuilder::createCap(const double &min_lat, const double &max_lat, const osg::Vec4ub &color)
{
    double min_lon = -180.0;
    double max_lon = 180.0;

    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();

    osgTerrain::Locator* locator = new osgTerrain::Locator();
    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
    locator->setTransformAsExtents(
        osg::DegreesToRadians( min_lon ),
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( max_lon ),
        osg::DegreesToRadians( max_lat ) );

    osg::HeightField *hf = new osg::HeightField();
    hf->allocate(32,32);
    for(unsigned int i=0; i<hf->getHeightList().size(); i++ ) hf->getHeightList()[i] = 0.0;

    hf->setOrigin( osg::Vec3d( min_lon, min_lat, 0.0 ) );
    hf->setXInterval( (max_lon - min_lon)/(double)(hf->getNumColumns()-1) );
    hf->setYInterval( (max_lat - min_lat)/(double)(hf->getNumRows()-1) );
    hf->setBorderWidth( 0 );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( locator );
    hf_layer->setHeightField( hf );

    osg::Image *image = new osg::Image();
    image->allocateImage(1,1,1, GL_RGBA, GL_UNSIGNED_BYTE);
    unsigned char *data = image->data(0,0);
    memcpy(data, color.ptr(), 4);

    osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image );
    img_layer->setLocator( locator );
    img_layer->setFilter( osgTerrain::Layer::LINEAR );
    tile->setColorLayer( 0, img_layer );

    tile->setLocator( locator );
    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );


    osg::EllipsoidModel* ellipsoid = locator->getEllipsoidModel();
    double x, y, z;
    ellipsoid->convertLatLongHeightToXYZ(
        osg::DegreesToRadians( (max_lat+min_lat)/2.0 ),
        osg::DegreesToRadians( (max_lon+min_lon)/2.0 ),
        0.0,
        x, y, z );

    osg::Vec3d centroid( x, y, z );

    double sw_x, sw_y, sw_z;
    ellipsoid->convertLatLongHeightToXYZ(
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( min_lon ),
        0.0,
        sw_x, sw_y, sw_z );

    double max_range = 1e10;
    double radius = (centroid-osg::Vec3d(sw_x,sw_y,sw_z)).length();
    double min_range = radius * map->getMinTileRangeFactor();

    //Set the skirt height of the heightfield
    hf->setSkirtHeight(radius * map->getSkirtRatio());

    osg::Vec3d normal = centroid;
    normal.normalize();
    // dot product: 0 = orthogonal to normal, -1 = equal to normal
    float deviation = -radius/locator->getEllipsoidModel()->getRadiusPolar();

    osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
    ccc->set( centroid, normal, deviation, radius );
    tile->setCullCallback( ccc );


    return tile;

}

bool
GeocentricTileBuilder::addChildren( osg::Group* tile_parent, const TileKey* key )
{
    if (key->getLevelOfDetail() == 0)
    {
        const osgEarth::TileGridProfile& profile = key->getProfile();
        
        //Extend the caps out so they slightly overlap neighboring tiles to hide seams
        double cap_offset = 0.1;
        
        //Draw a "cap" on the bottom of the earth to account for missing tiles
        if (profile.yMin() > -90)
        {
            tile_parent->addChild(createCap(-90, profile.yMin()+cap_offset,  map->getSouthCapColor()));
        }

        //Draw a "cap" on the top of the earth to account for missing tiles
        if (profile.yMax() < 90)
        {   
            tile_parent->addChild(createCap(profile.yMax()-cap_offset, 90, map->getNorthCapColor()));
        }
    }

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
GeocentricTileBuilder::createQuadrant( const TileKey* key )
{
    double min_lon, min_lat, max_lon, max_lat;
    if ( !key->getGeoExtents( min_lon, min_lat, max_lon, max_lat ) )
    {
        osg::notify( osg::WARN ) << "GET EXTENTS FAILED!" << std::endl;
        return NULL;
    }

    int tile_size = key->getProfile().pixelsPerTile();

    //Create the images
    std::vector<osg::ref_ptr<osg::Image> > images;
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
                //return NULL;
            }
            else
            {
                images.push_back(image);
            }
        }
    }

    // bail out if we couldn't load any images.
    if ( images.size() == 0 )
    {
        return NULL;
    }


    osgTerrain::Locator* locator = new osgTerrain::Locator();
    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
    locator->setTransformAsExtents(
        osg::DegreesToRadians( min_lon ),
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( max_lon ),
        osg::DegreesToRadians( max_lat ) );

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
            hf->getHeightList()[i] = 0.0; //(double)((::rand() % 10000) - 5000);
    }
    hf->setOrigin( osg::Vec3d( min_lon, min_lat, 0.0 ) );
    hf->setXInterval( (max_lon - min_lon)/(double)(hf->getNumColumns()-1) );
    hf->setYInterval( (max_lat - min_lat)/(double)(hf->getNumRows()-1) );
    hf->setBorderWidth( 0 );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( locator );
    hf_layer->setHeightField( hf );

    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();
    tile->setTileID(key->getTileId());

    //Attach an updatecallback to normalize the edges of TerrainTiles.
    tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
    tile->setDataVariance(osg::Object::DYNAMIC);

    tile->setLocator( locator );
    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );

    for (unsigned int i = 0; i < images.size(); ++i)
    {
        if (images[i].valid())
        {
            osgTerrain::Locator* img_locator = locator;

            // use a special image locator to warp the texture coords for mercator tiles :)
            // WARNING: TODO: this will not persist upn export....we need a nodekit.
            if ( dynamic_cast<const MercatorTileKey*>( key ) )
                img_locator = new MercatorLocator( *locator, tile_size, key->getLevelOfDetail() );
            osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( images[i].get() );
            img_layer->setLocator( img_locator );
            img_layer->setFilter( osgTerrain::Layer::LINEAR );
            tile->setColorLayer( i, img_layer );
        }
    }
    
    osg::EllipsoidModel* ellipsoid = locator->getEllipsoidModel();
    double x, y, z;
    ellipsoid->convertLatLongHeightToXYZ(
        osg::DegreesToRadians( (max_lat+min_lat)/2.0 ),
        osg::DegreesToRadians( (max_lon+min_lon)/2.0 ),
        0.0,
        x, y, z );
    
    osg::Vec3d centroid( x, y, z );

    double sw_x, sw_y, sw_z;
    ellipsoid->convertLatLongHeightToXYZ(
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( min_lon ),
        0.0,
        sw_x, sw_y, sw_z );

    double max_range = 1e10;
    double radius = (centroid-osg::Vec3d(sw_x,sw_y,sw_z)).length();
    double min_range = radius * map->getMinTileRangeFactor();

    //Set the skirt height of the heightfield
    hf->setSkirtHeight(radius * map->getSkirtRatio());

    osg::Vec3d normal = centroid;
    normal.normalize();
    // dot product: 0 = orthogonal to normal, -1 = equal to normal
    float deviation = -radius/locator->getEllipsoidModel()->getRadiusPolar();
            
    osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
    ccc->set( centroid, normal, deviation, radius );
    tile->setCullCallback( ccc );

    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setCenter( centroid );
    plod->addChild( tile, min_range, max_range );
    plod->setFileName( 1, createURI( key ) );
    plod->setRange( 1, 0.0, min_range );

    return plod;
}

osg::CoordinateSystemNode*
GeocentricTileBuilder::createCoordinateSystemNode() const
{
    osg::CoordinateSystemNode* csn = new osg::CoordinateSystemNode();
    csn->setEllipsoidModel( new osg::EllipsoidModel() );
    csn->setCoordinateSystem( "+init=epsg:4326" );
    csn->setFormat( "PROJ4" );
    return csn;
}

