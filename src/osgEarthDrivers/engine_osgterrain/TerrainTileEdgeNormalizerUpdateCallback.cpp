/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#if 0
#include "TerrainTileEdgeNormalizerUpdateCallback"
#include "CustomTile"
#include "CustomTerrain"
#include <osg/Version>
#include <osg/Timer>

using namespace osgEarth;


struct NormalizerStats : public osg::Referenced
{
public:
    NormalizerStats():
      _maxTimePerFrame(4.0),
      _currentFrame(-1)
    {
    }

    void updateTime(const int &frame, const double &time)
    {
        checkFrame(frame);
        _totalTime += time;
    }

    bool isTimeRemaining(const int &frame)
    {
        checkFrame(frame);
        return (_totalTime < _maxTimePerFrame);
    }

    inline void checkFrame(const int &frame)
    {
        if (frame != _currentFrame)
        {            
            _currentFrame = frame;
            _totalTime = 0.0;
        }
    }

    int _currentFrame;
    double _totalTime;
    double _maxTimePerFrame;
};

NormalizerStats* getNormalizerStats()
{
    static osg::ref_ptr<NormalizerStats> s_stats = new  NormalizerStats;
    return s_stats.get();
}



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
                    hf1->setHeight(width-1, r, val);
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


TerrainTileEdgeNormalizerUpdateCallback::TerrainTileEdgeNormalizerUpdateCallback():
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

      bool TerrainTileEdgeNormalizerUpdateCallback::normalizeCorner(CustomTile *tile, CardinalDirection direction)
      {
          if (tile && tile->getTerrain())
          {
              //TODO:  Remove the use of EarthTerrain once getTile fix is included in OpenSceneGraph proper
              CustomTerrain* et = static_cast<CustomTerrain*>(tile->getTerrain());
              if (et)
              {
                  //Determine the TileID's of the 4 tiles that need to take place in this normalization
                  osgTerrain::TileID ll_id;
                  osgTerrain::TileID lr_id;
                  osgTerrain::TileID ul_id;
                  osgTerrain::TileID ur_id;

                  TileKey ll_key, lu_key, ul_key, ur_key;
                  
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
                  else if (direction == SOUTH_EAST)
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
                      OE_WARN << "Invalid CardinalDirection passed to normalizeCorner " << direction << std::endl;
                      return false;
                  }

                  //Get the terrain tiles
                  osg::ref_ptr<CustomTile> ll_tile, lr_tile, ul_tile, ur_tile;
                  et->getCustomTile(ll_id, ll_tile);
                  et->getCustomTile(lr_id, lr_tile);
                  et->getCustomTile(ul_id, ul_tile);
                  et->getCustomTile(ur_id, ur_tile);

                  if (ll_tile.valid() && lr_tile.valid() && ul_tile.valid() && ur_tile.valid())
                  {
                      osgTerrain::HeightFieldLayer *ll_hfl = static_cast<osgTerrain::HeightFieldLayer*>(ll_tile->getElevationLayer());
                      osgTerrain::HeightFieldLayer *lr_hfl = static_cast<osgTerrain::HeightFieldLayer*>(lr_tile->getElevationLayer());
                      osgTerrain::HeightFieldLayer *ul_hfl = static_cast<osgTerrain::HeightFieldLayer*>(ul_tile->getElevationLayer());
                      osgTerrain::HeightFieldLayer *ur_hfl = static_cast<osgTerrain::HeightFieldLayer*>(ur_tile->getElevationLayer());

                      if (ll_hfl && lr_hfl && ul_hfl && ur_hfl)
                      {
                          bool normalized = ::normalizeCorner(ll_hfl->getHeightField(), lr_hfl->getHeightField(), ul_hfl->getHeightField(), ur_hfl->getHeightField());
                          if (normalized)
                          {
                              if (ll_tile->getUseLayerRequests())
                                  ll_tile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                              else
                                  ll_tile->setDirty(true);

                              if (lr_tile->getUseLayerRequests())
                                  lr_tile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                              else
                                  lr_tile->setDirty(true);

                              if (ul_tile->getUseLayerRequests())
                                  ul_tile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                              else
                                  ul_tile->setDirty(true);

                              if (ur_tile->getUseLayerRequests())
                                  ur_tile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                              else
                                  ur_tile->setDirty(true);
                          }
                          return normalized;
                      }
                  }
              }
          }
          return false;
      }


      bool TerrainTileEdgeNormalizerUpdateCallback::normalizeEdge(CustomTile *tile, CardinalDirection direction)
      {
          if (tile && tile->getTerrain())
          {
              //TODO:  Remove the use of EarthTerrain once getTile fix is included in OpenSceneGraph proper
              CustomTerrain* et = static_cast<CustomTerrain*>(tile->getTerrain());
              if (et)
              {

                  osgTerrain::TileID id1 = tile->getTileID();

                  osgTerrain::TileID id2 = getNeighborTile(id1, direction);

                  //OE_NOTICE << "Tile is " << id1.level << " " << id1.x << " " << id1.y << std::endl;
                  //OE_NOTICE << "Neighbor tile is " << id2.level << " " << id2.x << " " << id2.y << std::endl;

                  if (tile->getTerrain())
                  {
                      osg::ref_ptr<CustomTile> tile2;
                      et->getCustomTile(id2, tile2);

                      if (tile2)
                      {
                          //OE_NOTICE << "Found neighbor tile " << std::endl;
                          //This callback will only work if we have a HeightFieldLayer
                          osg::ref_ptr<osgTerrain::HeightFieldLayer> hfl1 = static_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
                          osg::ref_ptr<osgTerrain::HeightFieldLayer> hfl2 = static_cast<osgTerrain::HeightFieldLayer*>(tile2->getElevationLayer());

                          if (hfl1.valid() && hfl2.valid())
                          {
                              bool normalized = ::normalizeEdge(hfl1->getHeightField(), hfl2->getHeightField(), direction);
                              if (normalized)
                              {
                                  if (tile->getUseLayerRequests())
                                      tile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                                  else
                                      tile->setDirty(true);

                                  if (tile2->getUseLayerRequests())
                                      tile2->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                                  else
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

      void TerrainTileEdgeNormalizerUpdateCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
      {         
          NormalizerStats* stats = getNormalizerStats();
          if (!stats->isTimeRemaining(nv->getFrameStamp()->getFrameNumber()))
          {
              //OE_NOTICE << "No time left, returning..." << std::endl;
              return;
          }

          osg::Timer_t start = osg::Timer::instance()->tick();
          //TODO:  Look at heightDelta's to assist with normal generation.
          CustomTile* tt = static_cast<CustomTile*>(node);

          if (!_normalizedNorth) _normalizedNorth = normalizeEdge(tt, NORTH);
          if (!_normalizedSouth) _normalizedSouth = normalizeEdge(tt, SOUTH);
          if (!_normalizedEast) _normalizedEast = normalizeEdge(tt, EAST);
          if (!_normalizedWest) _normalizedWest = normalizeEdge(tt, WEST);
          if (!_normalizedNorthWest) _normalizedNorthWest = normalizeCorner(tt, NORTH_WEST);
          if (!_normalizedNorthEast) _normalizedNorthEast = normalizeCorner(tt, NORTH_EAST);
          if (!_normalizedSouthWest) _normalizedSouthWest = normalizeCorner(tt, SOUTH_WEST);
          if (!_normalizedSouthEast) _normalizedSouthEast = normalizeCorner(tt, SOUTH_EAST);

          traverse(node, nv);

          osg::Timer_t end = osg::Timer::instance()->tick();

          double frameTime = osg::Timer::instance()->delta_m(start, end);
          //OE_NOTICE << "TileTime: " << frameTime << std::endl;
          stats->updateTime(nv->getFrameStamp()->getFrameNumber(), frameTime);

          if (_normalizedNorth && _normalizedSouth && _normalizedEast && _normalizedWest && 
              _normalizedNorthWest && _normalizedNorthEast && _normalizedSouthWest && _normalizedSouthEast)
          {
              node->setUpdateCallback(0);
          }
      }     


      osgTerrain::TileID TerrainTileEdgeNormalizerUpdateCallback::getNeighborTile(const osgTerrain::TileID &id, CardinalDirection direction) const
      {
          //The name of the lod changed after OSG 2.6 from layer to level
#if (OPENSCENEGRAPH_MAJOR_VERSION == 2 && OPENSCENEGRAPH_MINOR_VERSION < 7)
          int level = id.layer;
#else
          int level = id.level;
#endif

          //Determine the edge TileID
          osgTerrain::TileID id2(level, id.x, id.y);
          int totalTiles = 2 << (level-1); //TileKey::getMapSizeTiles(level);

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

#endif