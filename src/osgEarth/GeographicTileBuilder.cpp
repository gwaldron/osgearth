#include <osgEarth/GeographicTileBuilder>
#include <osgEarth/Mercator>
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

void
GeographicTileBuilder::addChildren( osg::Group* tile_parent, const TileKey* key )
{
    tile_parent->addChild( createQuadrant( key->getSubkey( 0 ) ) );
    tile_parent->addChild( createQuadrant( key->getSubkey( 1 ) ) );

    if ( key->getLevelOfDetail() > 0 || dynamic_cast<const MercatorTileKey*>( key ) )
    {
        tile_parent->addChild( createQuadrant( key->getSubkey( 2 ) ) );
        tile_parent->addChild( createQuadrant( key->getSubkey( 3 ) ) );
    }
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

    osg::Image* image = NULL;

    //TODO: select/composite:
    if ( image_sources.size() > 0 )
    {
        image = image_sources[0]->createImage( key );
    }

    if ( image )
    {
        osgTerrain::Locator* img_locator = geo_locator;
        if ( dynamic_cast<const MercatorTileKey*>( key ) )
            img_locator = new MercatorLocator( *geo_locator, tile_size, key->getLevelOfDetail() );

        osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image );
        img_layer->setLocator( img_locator );
        tile->setColorLayer( 0, img_layer );
    }
    
    osg::Vec3d centroid( (max_lon+min_lon)/2.0, (max_lat+min_lat)/2.0, 0 );

    double max_range = 1e10;
    double radius = (centroid-osg::Vec3d(min_lon,min_lat,0)).length();
    double min_range = radius * map->getMinTileRangeFactor();
   
    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setCenter( centroid );
    plod->addChild( tile, min_range, max_range );
    plod->setFileName( 1, createURI( key ) );
    plod->setRange( 1, 0.0, min_range );

    return plod;
}

std::string
GeographicTileBuilder::getProj4String() const
{
    return "+proj=eqc +lat_ts=0 +lon_0=0 +x_0=0 +y_0=0";
}
