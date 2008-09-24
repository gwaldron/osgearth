#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/PlateCarre>
#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osgDB/ReadFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

GeocentricTileBuilder::GeocentricTileBuilder(PlateCarreTileSource* _image_source,
                                             PlateCarreTileSource* _field_source )
: image_source( _image_source ),
  field_source( _field_source )
{
    //NOP
    if ( getenv("OSGEARTH_NO_DEM") )
        make_dem = false;
    else
        make_dem = true;
}

osg::Node*
GeocentricTileBuilder::createQuadrant( const PlateCarreCellKey& pc_key )
{
    double min_lon, min_lat, max_lon, max_lat;
    if ( !pc_key.getGeoExtents( min_lon, min_lat, max_lon, max_lat ) )
    {
        osg::notify( osg::WARN ) << "GET EXTENTS FAILED!" << std::endl;
        return NULL;
    }

    osgTerrain::Locator* locator = new osgTerrain::Locator();
    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
    locator->setTransformAsExtents(
        osg::DegreesToRadians( min_lon ),
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( max_lon ),
        osg::DegreesToRadians( max_lat ) );
    locator->setTransformScaledByResolution( false );

    osg::HeightField* hf = NULL;
    if ( make_dem )
        hf = field_source->createHeightField( pc_key );

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
    hf->setSkirtHeight( 0 );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( locator );
    hf_layer->setHeightField( hf );

    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();
    tile->setLocator( locator );
    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );

    osg::Image* image = image_source->createImage( pc_key );
    if ( image )
    {
        osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image );
        img_layer->setLocator( locator );
        tile->setColorLayer( 0, img_layer );
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
    double min_range = radius*5.0;

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
    plod->setFileName( 1, image_source->createURI( pc_key ) );
    plod->setRange( 1, 0.0, min_range );

    return plod;
}

osg::Node*
GeocentricTileBuilder::create( const PlateCarreCellKey& key )
{
    osg::Group* top;
    if ( key.getLevelOfDetail() == 0 )
    {
        osg::CoordinateSystemNode* csn = new osg::CoordinateSystemNode();
        csn->setEllipsoidModel( new osg::EllipsoidModel() );

        osgTerrain::Terrain* terrain = new osgTerrain::Terrain();
        terrain->setVerticalScale( 3.0f );
        csn->addChild( terrain );

        top = csn;
    }
    else
    {
        top = new osg::Group();
        top->setName( key.str() );
    }

    top->addChild( createQuadrant( key.getSubkey( 0 ) ) );
    top->addChild( createQuadrant( key.getSubkey( 1 ) ) );

    if ( key.getLevelOfDetail() > 0 )
    {
        top->addChild( createQuadrant( key.getSubkey( 2 ) ) );
        top->addChild( createQuadrant( key.getSubkey( 3 ) ) );
    }
    return top;
}