#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/PlateCarre>
#include <osgEarth/Mercator>
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


enum CardinalDirection
{
    NORTH,
    EAST,
    SOUTH,
    WEST,
    NORTH_EAST,
    NORTH_WEST,
    SOUTH_EAST,
    SOUTH_WEST
};

bool normalizeCorner(osg::HeightField* ll, osg::HeightField* lr, osg::HeightField *ul, osg::HeightField *ur)
{
    //Check for NULL
    if (ll && lr && ul && ur)
    {
        float val1 = ll->getHeight(ll->getNumColumns()-1, ll->getNumRows()-1);
        float val2 = lr->getHeight(0, lr->getNumRows()-1);
        float val3 = ul->getHeight(ul->getNumColumns()-1, 0);
        float val4 = ur->getHeight(0, 0);

        //Average the 4 corners
        float corner_val = (val1 + val2 + val3 + val4) / 4.0f;

        ll->setHeight(ll->getNumColumns()-1, ll->getNumRows()-1, corner_val);
        lr->setHeight(0, lr->getNumRows()-1, corner_val);
        ul->setHeight(ul->getNumColumns()-1, 0, corner_val);
        ur->setHeight(0, 0, corner_val);

        return true;
    }
    return false;
}


bool normalizeEdge(osg::HeightField* hf1, osg::HeightField *hf2, CardinalDirection direction)
{
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
                for (int c = 1; c < width-1; ++c)
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
                for (int r = 1; r < height-1; ++r)
                {
                    float val1 = hf1->getHeight(width-1, r);
                    float val2 = hf2->getHeight(0, r);
                    float val = (val1 + val2)/2.0f;
                    hf1->setHeight(width-2, r, val);
                    hf2->setHeight(0, r, val);
                }
                break;
            case SOUTH:
                //Assume hf2 is to the south of hf1
                for (int c = 1; c < width-1; ++c)
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
                for (int r = 1; r < height-1; ++r)
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
      _normalizedWest(false),
      _normalizedNorthWest(false),
      _normalizedNorthEast(false),
      _normalizedSouthWest(false),
      _normalizedSouthEast(false)
      {
      }

      bool normalizeCorner(osgTerrain::TerrainTile *tile, CardinalDirection direction)
      {
          if (tile && tile->getTerrain())
          {
              //TODO:  Remove the use of EarthTerrain once getTile fix is included in OpenSceneGraph proper
              osgEarth::EarthTerrain* et = dynamic_cast<osgEarth::EarthTerrain*>(tile->getTerrain());
              if (et)
              {
                  //Determine the TileID's of the 4 tiles that need to take place in this normalization
                  osgTerrain::TileID ll_id;
                  osgTerrain::TileID lr_id;
                  osgTerrain::TileID ul_id;
                  osgTerrain::TileID ur_id;

                  if (direction == NORTH_EAST)
                  {
                      ll_id = tile->getTileID();
                      ul_id = getNeighborTile(ll_id, NORTH);
                      lr_id = getNeighborTile(ll_id, EAST);
                      ur_id = getNeighborTile(ll_id, NORTH_EAST);
                  }
                  else if (direction == NORTH_WEST)
                  {
                      lr_id = tile->getTileID();
                      ll_id = getNeighborTile(lr_id, WEST);
                      ul_id = getNeighborTile(lr_id, NORTH_WEST);
                      ur_id = getNeighborTile(lr_id, NORTH);
                  }
                  else if (direction = SOUTH_EAST)
                  {
                      ul_id = tile->getTileID();
                      ll_id = getNeighborTile(ul_id, SOUTH);
                      lr_id = getNeighborTile(ul_id, SOUTH_EAST);
                      ur_id = getNeighborTile(ul_id, EAST);
                  }
                  else if (direction == SOUTH_WEST)
                  {
                      ur_id = tile->getTileID();
                      ul_id = getNeighborTile(ur_id, WEST);
                      ll_id = getNeighborTile(ur_id, SOUTH_WEST);
                      lr_id = getNeighborTile(ur_id, SOUTH);
                  }
                  else
                  {
                      osg::notify(osg::WARN) << "Invalid CardinalDirection passed to normalizeCorner " << direction << std::endl;
                      return false;
                  }

                  //Get the terrain tiles
                  osgTerrain::TerrainTile *ll_tile = et->getTileOverride(ll_id);
                  osgTerrain::TerrainTile *lr_tile = et->getTileOverride(lr_id);
                  osgTerrain::TerrainTile *ul_tile = et->getTileOverride(ul_id);
                  osgTerrain::TerrainTile *ur_tile = et->getTileOverride(ur_id);

                  if (ll_tile && lr_tile && ul_tile && ur_tile)
                  {
                      osgTerrain::HeightFieldLayer *ll_hfl = dynamic_cast<osgTerrain::HeightFieldLayer*>(ll_tile->getElevationLayer());
                      osgTerrain::HeightFieldLayer *lr_hfl = dynamic_cast<osgTerrain::HeightFieldLayer*>(lr_tile->getElevationLayer());
                      osgTerrain::HeightFieldLayer *ul_hfl = dynamic_cast<osgTerrain::HeightFieldLayer*>(ul_tile->getElevationLayer());
                      osgTerrain::HeightFieldLayer *ur_hfl = dynamic_cast<osgTerrain::HeightFieldLayer*>(ur_tile->getElevationLayer());

                      if (ll_hfl && lr_hfl && ul_hfl && ur_hfl)
                      {
                          bool normalized = ::normalizeCorner(ll_hfl->getHeightField(), lr_hfl->getHeightField(), ul_hfl->getHeightField(), ur_hfl->getHeightField());
                          if (normalized)
                          {
                              //osg::notify(osg::NOTICE) << "Normalized corner" << std::endl;
                              ll_hfl->dirty();
                              lr_hfl->dirty();
                              ul_hfl->dirty();
                              ur_hfl->dirty();

                              ll_tile->setDirty(true);
                              lr_tile->setDirty(true);
                              ul_tile->setDirty(true);
                              ur_tile->setDirty(true);
                          }
                          return normalized;
                      }
                  }
              }
          }
          return false;
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

                  osgTerrain::TileID id2 = getNeighborTile(id1, direction);

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
                              bool normalized = ::normalizeEdge(hfl1->getHeightField(), hfl2->getHeightField(), direction);
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
          //TODO:  Look at heightDelta's to assist with normal generation.
          osgTerrain::TerrainTile* tt = dynamic_cast<osgTerrain::TerrainTile*>(node);
          if (!_normalizedNorth) _normalizedNorth = normalizeEdge(tt, NORTH);
          if (!_normalizedSouth) _normalizedSouth = normalizeEdge(tt, SOUTH);
          if (!_normalizedEast) _normalizedEast = normalizeEdge(tt, EAST);
          if (!_normalizedWest) _normalizedWest = normalizeEdge(tt, WEST);
          if (!_normalizedNorthWest) _normalizedNorthWest = normalizeCorner(tt, NORTH_WEST);
          if (!_normalizedNorthEast) _normalizedNorthEast = normalizeCorner(tt, NORTH_EAST);
          if (!_normalizedSouthWest) _normalizedSouthWest = normalizeCorner(tt, SOUTH_WEST);
          if (!_normalizedSouthEast) _normalizedSouthEast = normalizeCorner(tt, SOUTH_EAST);

          traverse(node, nv);
      }

      osgTerrain::TileID getNeighborTile(const osgTerrain::TileID &id, CardinalDirection direction) const
      {
          //The name of the lod changed after OSG 2.6 from layer to level
#if (OPENSCENEGRAPH_MAJOR_VERSION == 2 && OPENSCENEGRAPH_MINOR_VERSION < 7)
          int level = id.layer;
#else
          int level = id.level;
#endif

          //Determine the edge TileID
          osgTerrain::TileID id2(level, id.x, id.y);
          int totalTiles = sqrt(pow(4.0, (level)));
          if (direction == WEST || direction == SOUTH_WEST || direction == NORTH_WEST)
          {
              id2.x = (id2.x == 0 ? totalTiles-1 : id2.x-1);
          }

          if (direction == EAST || direction == SOUTH_EAST || direction == NORTH_EAST)
          {
              id2.x = (id2.x == totalTiles-1 ? 0 : id2.x+1);
          }

          if (direction == NORTH || direction == NORTH_EAST || direction == NORTH_WEST)
          {
              id2.y = (id2.y == 0 ? totalTiles-1 : id2.y-1);
          }

          if (direction == SOUTH || direction == SOUTH_WEST || direction == SOUTH_EAST)
          {
              id2.y = (id2.y == totalTiles-1 ? 0 : id2.y+1);
          }

          return id2;
      }

      bool _normalizedWest;
      bool _normalizedEast;
      bool _normalizedNorth;
      bool _normalizedSouth;
      bool _normalizedNorthWest;
      bool _normalizedNorthEast;
      bool _normalizedSouthWest;
      bool _normalizedSouthEast;
};

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

    return tile;

}

void
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

    //osg::Image* image = NULL;

    std::vector<osg::Image*> images;
    //TODO: select/composite:
    if ( image_sources.size() > 0 )
    {
        //Add an image from each image source
        for (unsigned int i = 0; i < image_sources.size(); ++i)
        {
            images.push_back(image_sources[i]->createImage(key));
        }
    }

    if ( images.size() > 0 )
    {
        for (unsigned int i = 0; i < images.size(); ++i)
        {
            osgTerrain::Locator* img_locator = locator;

            // use a special image locator to warp the texture coords for mercator tiles :)
            // WARNING: TODO: this will not persist upn export....we need a nodekit.
            if ( dynamic_cast<const MercatorTileKey*>( key ) )
                img_locator = new MercatorLocator( *locator, tile_size, key->getLevelOfDetail() );
            osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( images[i] );
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

std::string
GeocentricTileBuilder::getProj4String() const
{
    return "+init=epsg:4326";
}
