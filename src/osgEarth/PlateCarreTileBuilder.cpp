#include <osgEarth/PlateCarreTileBuilder>
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

PlateCarreTileBuilder::PlateCarreTileBuilder( MapConfig* _map, const std::string& _url_template ) :
TileBuilder( _map, _url_template )
{
    //NOP
}

osg::Node*
PlateCarreTileBuilder::createQuadrant( const PlateCarreCellKey& pc_key )
{
    double min_lon, min_lat, max_lon, max_lat;
    if ( !pc_key.getGeoExtents( min_lon, min_lat, max_lon, max_lat ) )
    {
        osg::notify( osg::WARN ) << "GET EXTENTS FAILED!" << std::endl;
        return NULL;
    }

    osgTerrain::Locator* locator = new osgTerrain::Locator();
    locator->setCoordinateSystemType( osgTerrain::Locator::GEOGRAPHIC ); // sort of.
    locator->setTransformAsExtents( min_lon, min_lat, max_lon, max_lat );
    locator->setTransformScaledByResolution( false );

    osg::HeightField* hf = NULL;

    //TODO: select/composite.
    if ( heightfield_sources.size() > 0 )
        hf = heightfield_sources[0]->createHeightField( pc_key );

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

    osg::Image* image = NULL;

    //TODO: select/composite:
    if ( image_sources.size() > 0 )
        image = image_sources[0]->createImage( pc_key );

    if ( image )
    {
        osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image );
        img_layer->setLocator( locator );
        tile->setColorLayer( 0, img_layer );
    }
    
    osg::Vec3d centroid( (max_lon+min_lon)/2.0, (max_lat+min_lat)/2.0, 0 );

    double max_range = 1e10;
    double radius = (centroid-osg::Vec3d(min_lon,min_lat,0)).length();
    double min_range = radius*5.0;
   
    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setCenter( centroid );
    plod->addChild( tile, min_range, max_range );
    plod->setFileName( 1, createURI( pc_key ) );
    plod->setRange( 1, 0.0, min_range );

    return plod;
}

osg::Node*
PlateCarreTileBuilder::createNode( const PlateCarreCellKey& key )
{
    osg::Group* top;

    if ( key.getLevelOfDetail() == 0 )
    {
        osgTerrain::Terrain* terrain = new osgTerrain::Terrain();
        terrain->setVerticalScale( map->getVerticalScale() );
        top = terrain;
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