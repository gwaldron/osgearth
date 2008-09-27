#include <osgEarth/TileBuilder>
#include <osgEarth/PlateCarreTileBuilder>
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

TileBuilder*
TileBuilder::create( MapConfig* map, const std::string& url_template )
{
    TileBuilder* result = NULL;
    if ( map )
    {
        if ( map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC )
        {
            result = new GeocentricTileBuilder( map, url_template );
        }
        else
        {
            result = new PlateCarreTileBuilder( map, url_template );
        }
    }
    return result;
}

static void
addSources(const SourceConfigList& from, 
           std::vector< osg::ref_ptr<PlateCarreTileSource> >& to,
           const std::string& url_template)
{        
    for( SourceConfigList::const_iterator i = from.begin(); i != from.end(); i++ )
    {
        SourceConfig* source = i->get();
        osgDB::ReaderWriter::Options* options = new osgDB::ReaderWriter::Options();
        for( SourceProperties::const_iterator p = source->getProperties().begin(); p != source->getProperties().end(); p++ )
        {
            options->setPluginData( p->first, (void*)p->second.c_str() );
            PlateCarreTileSource* tile_source = new ReaderWriterPlateCarreTileSource( source->getDriver(), options );
            to.push_back( tile_source );
        }
    }
}

TileBuilder::TileBuilder( MapConfig* _map, const std::string& _url_template ) :
map( _map ),
url_template( _url_template )
{
    if ( map.valid() )
    {
        addSources( map->getImageSources(), image_sources, url_template );
        addSources( map->getHeightFieldSources(), heightfield_sources, url_template );
    }
}

std::string
TileBuilder::createURI( const PlateCarreCellKey& key )
{
    std::stringstream buf;
    buf << key.str() << "." << url_template;
    return buf.str();
}

//osg::Node*
//TileBuilder::createGeocentricQuadrant( const PlateCarreCellKey& pc_key )
//{
//    double min_lon, min_lat, max_lon, max_lat;
//    if ( !pc_key.getGeoExtents( min_lon, min_lat, max_lon, max_lat ) )
//    {
//        osg::notify( osg::WARN ) << "GET EXTENTS FAILED!" << std::endl;
//        return NULL;
//    }
//
//    osgTerrain::Locator* locator = new osgTerrain::Locator();
//    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
//    locator->setTransformAsExtents(
//        osg::DegreesToRadians( min_lon ),
//        osg::DegreesToRadians( min_lat ),
//        osg::DegreesToRadians( max_lon ),
//        osg::DegreesToRadians( max_lat ) );
//    locator->setTransformScaledByResolution( false );
//
//    osg::HeightField* hf = NULL;
//
//    //TODO: select and composite
//    if ( heightfield_sources.size() > 0 )
//        hf = heightfield_sources[0]->createHeightField( pc_key );
//
//    if ( !hf )
//    {
//        // make an empty one if we couldn't fetch it
//        hf = new osg::HeightField();
//        hf->allocate( 8, 8 );
//        for(unsigned int i=0; i<hf->getHeightList().size(); i++ )
//            hf->getHeightList()[i] = 0.0; //(double)((::rand() % 10000) - 5000);
//    }
//    hf->setOrigin( osg::Vec3d( min_lon, min_lat, 0.0 ) );
//    hf->setXInterval( (max_lon - min_lon)/(double)(hf->getNumColumns()-1) );
//    hf->setYInterval( (max_lat - min_lat)/(double)(hf->getNumRows()-1) );
//    hf->setBorderWidth( 0 );
//    hf->setSkirtHeight( 0 );
//
//    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
//    hf_layer->setLocator( locator );
//    hf_layer->setHeightField( hf );
//
//    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();
//    tile->setLocator( locator );
//    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
//    tile->setElevationLayer( hf_layer );
//    tile->setRequiresNormals( true );
//
//    osg::Image* image = NULL;
//    
//    //TODO: select, composite
//    if ( image_sources.size() > 0 )
//        image = image_sources[0]->createImage( pc_key );
//
//    if ( image )
//    {
//        osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image );
//        img_layer->setLocator( locator );
//        tile->setColorLayer( 0, img_layer );
//    }
//    
//    osg::EllipsoidModel* ellipsoid = locator->getEllipsoidModel();
//    double x, y, z;
//    ellipsoid->convertLatLongHeightToXYZ(
//        osg::DegreesToRadians( (max_lat+min_lat)/2.0 ),
//        osg::DegreesToRadians( (max_lon+min_lon)/2.0 ),
//        0.0,
//        x, y, z );
//    
//    osg::Vec3d centroid( x, y, z );
//
//    double sw_x, sw_y, sw_z;
//    ellipsoid->convertLatLongHeightToXYZ(
//        osg::DegreesToRadians( min_lat ),
//        osg::DegreesToRadians( min_lon ),
//        0.0,
//        sw_x, sw_y, sw_z );
//
//    double max_range = 1e10;
//    double radius = (centroid-osg::Vec3d(sw_x,sw_y,sw_z)).length();
//    double min_range = radius*5.0;
//
//    osg::Vec3d normal = centroid;
//    normal.normalize();
//    // dot product: 0 = orthogonal to normal, -1 = equal to normal
//    float deviation = -radius/locator->getEllipsoidModel()->getRadiusPolar();
//            
//    osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
//    ccc->set( centroid, normal, deviation, radius );
//    tile->setCullCallback( ccc );
//
//    osg::PagedLOD* plod = new osg::PagedLOD();
//    plod->setCenter( centroid );
//    plod->addChild( tile, min_range, max_range );
//    plod->setFileName( 1, image_source->createURI( pc_key ) );
//    plod->setRange( 1, 0.0, min_range );
//
//    return plod;
//}
//
//osg::Node*
//TileBuilder::createQuadrant( const PlateCarreCellKey& pc_key )
//{
//    double min_lon, min_lat, max_lon, max_lat;
//    if ( !pc_key.getGeoExtents( min_lon, min_lat, max_lon, max_lat ) )
//    {
//        osg::notify( osg::WARN ) << "GET EXTENTS FAILED!" << std::endl;
//        return NULL;
//    }
//
//    osgTerrain::Locator* locator = new osgTerrain::Locator();
//    locator->setCoordinateSystemType( osgTerrain::Locator::GEOGRAPHIC ); // sort of.
//    locator->setTransformAsExtents( min_lon, min_lat, max_lon, max_lat );
//    locator->setTransformScaledByResolution( false );
//
//    osg::HeightField* hf = NULL;
//    if ( make_dem )
//        hf = field_source->createHeightField( pc_key );
//
//    if ( !hf )
//    {
//        // make an empty one if we couldn't fetch it
//        hf = new osg::HeightField();
//        hf->allocate( 8, 8 );
//        for(unsigned int i=0; i<hf->getHeightList().size(); i++ )
//        {
//            hf->getHeightList()[i] = 0.0; 
//        }
//    }
//    hf->setOrigin( osg::Vec3d( min_lon, min_lat, 0.0 ) );
//    hf->setXInterval( (max_lon - min_lon)/(double)(hf->getNumColumns()-1) );
//    hf->setYInterval( (max_lat - min_lat)/(double)(hf->getNumRows()-1) );
//    hf->setBorderWidth( 0 );
//    hf->setSkirtHeight( 0 );
//
//    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
//    hf_layer->setLocator( locator );
//    hf_layer->setHeightField( hf );
//
//    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();
//    tile->setLocator( locator );
//    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
//    tile->setElevationLayer( hf_layer );
//    tile->setRequiresNormals( true );
//
//    osg::Image* image = image_source->createImage( pc_key );
//    if ( image )
//    {
//        osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image );
//        img_layer->setLocator( locator );
//        tile->setColorLayer( 0, img_layer );
//    }
//    
//    osg::Vec3d centroid( (max_lon+min_lon)/2.0, (max_lat+min_lat)/2.0, 0 );
//
//    double max_range = 1e10;
//    double radius = (centroid-osg::Vec3d(min_lon,min_lat,0)).length();
//    double min_range = radius*5.0;
//   
//    osg::PagedLOD* plod = new osg::PagedLOD();
//    plod->setCenter( centroid );
//    plod->addChild( tile, min_range, max_range );
//    plod->setFileName( 1, image_source->createURI( pc_key ) );
//    plod->setRange( 1, 0.0, min_range );
//
//    return plod;
//}
//
//osg::Node*
//TileBuilder::createQuadrant( const PlateCarreCellKey& qk )
//{
//    if ( map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC )
//    {
//        return createGeocentricQuadrant( key );
//    }
//    else // MapConfig::CSTYPE_PLATECARRE 
//    {
//        return createPlateCarreQuadrant( key );
//    }
//}
//
//osg::Node*
//TileBuilder::create( const PlateCarreCellKey& key )
//{
//    osg::Group* top;
//    osg::Group* tile_parent;
//
//    if ( key.getLevelOfDetail() == 0 )
//    {
//        osgTerrain::Terrain* terrain = new osgTerrain::Terrain();
//        //TODO: remove; testing only
//        terrain->setVerticalScale( 3.0f );
//
//        if ( map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC )
//        {
//            osg::CoordinateSystemNode* csn = new osg::CoordinateSystemNode();
//            csn->setEllipsoidModel( new osg::EllipsoidModel() );
//            csn->addChild( terrain );
//            top = csn;
//        }
//        else // MapConfig::CSTYPE_PLATE_CARRE
//        {
//            top = terrain;
//        }
//
//        tile_parent = terrain;
//    }
//    else
//    {
//        top = new osg::Group();
//        top->setName( key.str() );
//        tile_parent = top;
//    }
//
//    tile_parent->addChild( createQuadrant( key.getSubkey( 0 ) ) );
//    tile_parent->addChild( createQuadrant( key.getSubkey( 1 ) ) );
//
//    if ( key.getLevelOfDetail() > 0 )
//    {
//        tile_parent->addChild( createQuadrant( key.getSubkey( 2 ) ) );
//        tile_parent->addChild( createQuadrant( key.getSubkey( 3 ) ) );
//    }
//    return top;
//}