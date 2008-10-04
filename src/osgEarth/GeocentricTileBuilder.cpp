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


enum CardinalDirection
{
    NORTH,
    EAST,
    SOUTH,
    WEST
};

bool normalizeHeightFields(osg::HeightField* hf1, osg::HeightField *hf2, CardinalDirection direction)
{
    //osg::notify(osg::NOTICE) << "normalizeHeightFields " << direction << std::endl;
    //Check for NULL
    if (hf1 && hf2)
    {
        //Make sure dimensions are equal
        if (hf1->getNumColumns() == hf2->getNumColumns() && hf1->getNumRows() == hf2->getNumRows())
        {
            int width = hf1->getNumColumns();
            int height = hf1->getNumRows();

            switch (direction)
            {
            case NORTH:
                //Assume hf2 is to the north of hf1
                for (int c = 0; c < width; ++c)
                {
                    float val1 = hf1->getHeight(c, height-1);
                    float val2 = hf2->getHeight(c, 0);
                    float val = (val1 + val2) /2.0f;
                    hf1->setHeight(c, height-1, val);
                    hf2->setHeight(c, 0, val);
                }
                break;
            case EAST:
                //Assume hf2 is to the east of hf1
                for (int r = 0; r < height; ++r)
                {
                    float val1 = hf1->getHeight(width-1, r);
                    float val2 = hf2->getHeight(0, r);
                    float val = (val1 + val2)/2.0f;
                    hf1->setHeight(width-1, r, val);
                    hf2->setHeight(0, r, val);
                }
                break;
            case SOUTH:
                //Assume hf2 is to the south of hf1
                for (int c = 0; c < width; ++c)
                {
                    float val1 = hf1->getHeight(c, 0);
                    float val2 = hf2->getHeight(c, height-1);
                    float val = (val1 + val2) /2.0f;
                    hf1->setHeight(c, 0, val);
                    hf2->setHeight(c, height-1, val);
                }
                break;
            case WEST:
                //Assume hf2 is to the west of hf1
                for (int r = 0; r < height; ++r)
                {
                    float val1 = hf1->getHeight(0, r);
                    float val2 = hf2->getHeight(width-1, r);
                    float val = (val1 + val2)/2.0f;
                    hf1->setHeight(0, r, val);
                    hf2->setHeight(width-1, r, val);
                }
                break;
            }

            return true;
        }
    }
    return false;
}


class TerrainTileEdgeNormalizerUpdateCallback : public osg::NodeCallback
{
public:

    TerrainTileEdgeNormalizerUpdateCallback():
      _normalizedEast(false),
          _normalizedNorth(false),
          _normalizedSouth(false),
          _normalizedWest(false)
      {
      }


      bool normalizeEdge(osgTerrain::TerrainTile *tile, CardinalDirection direction)
      {
          if (tile && tile->getTerrain())
          {
              //TODO:  Remove the use of EarthTerrain once getTile fix is included in OpenSceneGraph proper
              osgEarth::EarthTerrain* et = dynamic_cast<osgEarth::EarthTerrain*>(tile->getTerrain());
              if (et)
              {
                  osgTerrain::TileID id1 = tile->getTileID();

                  int totalTiles = sqrt(pow(4.0, (id1.level)));

                  //Determine the edge TileID
                  osgTerrain::TileID id2(id1.level, id1.x, id1.y);
                  if (direction == WEST)
                  {
                      id2.x = (id2.x == 0 ? totalTiles-1 : id2.x-1);
                  }
                  else if (direction == EAST)
                  {
                      id2.x = (id2.x == totalTiles-1 ? 0 : id2.x+1);
                  }
                  else if (direction == NORTH)
                  {
                      id2.y = (id2.y == 0 ? totalTiles-1 : id2.y-1);
                  }
                  else if (direction == SOUTH)
                  {
                      id2.y = (id2.y == totalTiles-1 ? 0 : id2.y+1);
                  }

                  //osg::notify(osg::NOTICE) << "Tile is " << id1.level << " " << id1.x << " " << id1.y << std::endl;
                  //osg::notify(osg::NOTICE) << "Neighbor tile is " << id2.level << " " << id2.x << " " << id2.y << std::endl;

                  if (tile->getTerrain())
                  {
                      //TODO:  Remove the use of EarthTerrain once getTile fix is included in OpenSceneGraph proper
                      osgTerrain::TerrainTile* tile2 = et->getTileOverride(id2);//tile->getTerrain()->getTile(id2);

                      if (tile2)
                      {
                          //osg::notify(osg::NOTICE) << "Found neighbor tile " << std::endl;
                          //This callback will only work if we have a HeightFieldLayer
                          osgTerrain::HeightFieldLayer *hfl1 = dynamic_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
                          osgTerrain::HeightFieldLayer *hfl2 = dynamic_cast<osgTerrain::HeightFieldLayer*>(tile2->getElevationLayer());

                          if (hfl1 && hfl2)
                          {
                              bool normalized = normalizeHeightFields(hfl1->getHeightField(), hfl2->getHeightField(), direction);
                              if (normalized)
                              {
                                  hfl1->dirty();
                                  hfl2->dirty();
                                  tile->setDirty(true);
                                  tile2->setDirty(true);
                              }
                              return normalized;
                          }
                      }
                  }
              }
          }
          return false;
      }

      virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
      { 

          //TODO:  Normalized corners
          //TODO:  Look at heightDelta's to assist with normal generation.
          osgTerrain::TerrainTile* tt = dynamic_cast<osgTerrain::TerrainTile*>(node);
          if (!_normalizedNorth) _normalizedNorth = normalizeEdge(tt, NORTH);
          if (!_normalizedSouth) _normalizedSouth = normalizeEdge(tt, SOUTH);
          if (!_normalizedEast) _normalizedEast = normalizeEdge(tt, EAST);
          if (!_normalizedWest) _normalizedWest = normalizeEdge(tt, WEST);
          traverse(node, nv);
      }

      bool _normalizedWest;
      bool _normalizedEast;
      bool _normalizedNorth;
      bool _normalizedSouth;
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
    tile->setTileID(key->getTileId());

    //Attach an updatecallback to normalize the edges of TerrainTiles.
    tile->setUpdateCallback(new TerrainTileEdgeNormalizerUpdateCallback());
    tile->setDataVariance(osg::Object::DYNAMIC);

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
