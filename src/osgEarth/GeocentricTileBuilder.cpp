#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/PlateCarre>
#include <osgEarth/Mercator>
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

//#define WGS84_WKT "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],TOWGS84[0,0,0,0,0,0,0],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9108\"]],AXIS["Lat",NORTH],AXIS["Long",EAST],AUTHORITY["EPSG","4326"]]

class HeightFieldRandomizerUpdateCallback : public osg::NodeCallback
{
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        { 
            double updateTime = 2;
            double currFrameTime = nv->getFrameStamp()->getReferenceTime();
            if (currFrameTime - lastUpdateTime >= updateTime)
            {
                osgTerrain::TerrainTile* tt = dynamic_cast<osgTerrain::TerrainTile*>(node);
                if (tt)
                {
                    osgTerrain::HeightFieldLayer *hfl = dynamic_cast<osgTerrain::HeightFieldLayer*>(tt->getElevationLayer());
                    if (hfl)
                    {
                        osg::HeightField* hf = hfl->getHeightField();
                        if (hf)
                        {
                            //Randomly either add 10 or subtract 10 from the elevation
                            for (int i = 0; i < hf->getHeightList().size(); ++i)
                            {
                                double offset = 50;
                                double e = hf->getHeightList()[i];
                                if (rand() %2 == 0)
                                {
                                    e += offset;
                                }
                                else
                                {
                                    e -= offset;
                                }
                                hf->getHeightList()[i] = e;
                            }
                            hfl->dirty();
                            tt->setDirty(true);
                        }
                    }
                }
                lastUpdateTime = currFrameTime;
            }
            traverse(node, nv);
        }

        double lastUpdateTime;
};

GeocentricTileBuilder::GeocentricTileBuilder( 
    MapConfig* map, 
    const std::string& url_template,
    const osgDB::ReaderWriter::Options* global_options ) :
TileBuilder( map, url_template, global_options )
{
    //NOP
}

void
GeocentricTileBuilder::addChildren( osg::Group* tile_parent, const TileKey* key )
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
GeocentricTileBuilder::createQuadrant( const TileKey* key )
{
    double min_lon, min_lat, max_lon, max_lat;
    if ( !key->getGeoExtents( min_lon, min_lat, max_lon, max_lat ) )
    {
        osg::notify( osg::WARN ) << "GET EXTENTS FAILED!" << std::endl;
        return NULL;
    }

    int tile_size = key->getProfile().pixelsPerTile();

    osgTerrain::Locator* locator = new osgTerrain::Locator();
    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
    locator->setTransformAsExtents(
        osg::DegreesToRadians( min_lon ),
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( max_lon ),
        osg::DegreesToRadians( max_lat ) );
    //locator->setTransformScaledByResolution( false );

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

#if 0
    tile->setUpdateCallback(new HeightFieldRandomizerUpdateCallback());
#endif
    tile->setLocator( locator );
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
        osgTerrain::Locator* img_locator = locator;

        // use a special image locator to warp the texture coords for mercator tiles :)
        // WARNING: TODO: this will not persist upn export....we need a nodekit.
        if ( dynamic_cast<const MercatorTileKey*>( key ) )
            img_locator = new MercatorLocator( *locator, tile_size, key->getLevelOfDetail() );

        osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image );
        img_layer->setLocator( img_locator );
        img_layer->setFilter( osgTerrain::Layer::LINEAR );
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

std::string
GeocentricTileBuilder::getProj4String() const
{
    return "+init=epsg:4326";
}
