/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/ElevationQuery>
#include <osgEarth/Locators>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/DPLineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

#define LC "[ElevationQuery] "

using namespace osgEarth;
using namespace OpenThreads;

namespace
{
    int nextPowerOf2(int x) {
        --x;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        return x+1;
    }
}

ElevationQueryCacheReadCallback::ElevationQueryCacheReadCallback()
{
    _maxNumFilesToCache = 2000;
}

void ElevationQueryCacheReadCallback::clearDatabaseCache()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    _filenameSceneMap.clear();
}

void ElevationQueryCacheReadCallback::pruneUnusedDatabaseCache()
{
}

#if OSG_VERSION_GREATER_OR_EQUAL(3,5,0)
osg::ref_ptr<osg::Node> ElevationQueryCacheReadCallback::readNodeFile(const std::string& filename)
#else
osg::Node* ElevationQueryCacheReadCallback::readNodeFile(const std::string& filename)
#endif
{
    // first check to see if file is already loaded.
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);

        FileNameSceneMap::iterator itr = _filenameSceneMap.find(filename);
        if (itr != _filenameSceneMap.end())
        {
            OSG_INFO<<"Getting from cache "<<filename<<std::endl;

            return itr->second.get();
        }
    }

    // now load the file.
    osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile(filename);

    // insert into the cache.
    if (node.valid())
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);

        if (_filenameSceneMap.size() < _maxNumFilesToCache)
        {
            OSG_INFO<<"Inserting into cache "<<filename<<std::endl;

            _filenameSceneMap[filename] = node;
        }
        else
        {
            // for time being implement a crude search for a candidate to chuck out from the cache.
            for(FileNameSceneMap::iterator itr = _filenameSceneMap.begin();
                itr != _filenameSceneMap.end();
                ++itr)
            {
                if (itr->second->referenceCount()==1)
                {
                    OSG_NOTICE<<"Erasing "<<itr->first<<std::endl;
                    // found a node which is only referenced in the cache so we can discard it
                    // and know that the actual memory will be released.
                    _filenameSceneMap.erase(itr);
                    break;
                }
            }
            OSG_INFO<<"And the replacing with "<<filename<<std::endl;
            _filenameSceneMap[filename] = node;
        }
    }

#if OSG_VERSION_GREATER_OR_EQUAL(3,5,0)
    return node;
#else
    return node.release();
#endif
}

ElevationQuery::ElevationQuery()
{
    postCTOR();
}

ElevationQuery::ElevationQuery(const Map* map) :
_mapf( map, (Map::ModelParts)(Map::TERRAIN_LAYERS | Map::MODEL_LAYERS) )
{
    postCTOR();
}

ElevationQuery::ElevationQuery(const MapFrame& mapFrame) :
_mapf( mapFrame )
{
    postCTOR();
}

void
ElevationQuery::setMapFrame(const MapFrame& frame)
{
    _mapf = frame;
    postCTOR();
}

void
ElevationQuery::postCTOR()
{
    // defaults:
    _maxLevelOverride = -1;
    _queries          = 0.0;
    _totalTime        = 0.0;
    _fallBackOnNoData = false;
    _cache.clear();
    _cache.setMaxSize( 500 );

    // set read callback for IntersectionVisitor
    setElevationQueryCacheReadCallback(new ElevationQueryCacheReadCallback);

    // find terrain patch layers.
    gatherPatchLayers();
}

void
ElevationQuery::sync()
{
    if ( _mapf.needsSync() )
    {
        _mapf.sync();
        _cache.clear();
        gatherPatchLayers();
    }
}

void
ElevationQuery::gatherPatchLayers()
{
    // cache a vector of terrain patch models.
    _patchLayers.clear();
    for(ModelLayerVector::const_iterator i = _mapf.modelLayers().begin();
        i != _mapf.modelLayers().end();
        ++i)
    {
        if ( i->get()->isTerrainPatch() )
            _patchLayers.push_back( i->get() );
    }
}

int
ElevationQuery::getMaxLevel( double x, double y, const SpatialReference* srs, const Profile* profile, unsigned tileSize) const
{
    int targetTileSizePOT = nextPowerOf2((int)tileSize);

    int maxLevel = -1;

    for( ElevationLayerVector::const_iterator i = _mapf.elevationLayers().begin(); i != _mapf.elevationLayers().end(); ++i )
    {
        const ElevationLayer* layer = i->get();

        // skip disabled layers
        if ( !layer->getEnabled() || !layer->getVisible() )
            continue;

        optional<int> layerMaxLevel;

        osgEarth::TileSource* ts = layer->getTileSource();
        if ( ts )
        {
            // TileSource is good; check for optional data extents:
            if ( ts->getDataExtents().size() > 0 )
            {
                osg::Vec3d tsCoord(x, y, 0);

                const SpatialReference* tsSRS = ts->getProfile() ? ts->getProfile()->getSRS() : 0L;
                if ( srs && tsSRS )
                    srs->transform(tsCoord, tsSRS, tsCoord);
                else
                    tsSRS = srs;

                for (osgEarth::DataExtentList::iterator j = ts->getDataExtents().begin(); j != ts->getDataExtents().end(); j++)
                {
                    if (j->maxLevel().isSet() &&
                        (!layerMaxLevel.isSet() || j->maxLevel() > layerMaxLevel.get() )
                        && j->contains( tsCoord.x(), tsCoord.y(), tsSRS ))
                    {
                        layerMaxLevel = j->maxLevel().value();
                    }
                }
            }
            else
            {
                // Just use the default max level.  Without any data extents we don't know the actual max
                layerMaxLevel = (int)(*layer->getTerrainLayerRuntimeOptions().maxLevel());
            }

            // cap the max to the layer's express max level (if set).
            if ( layerMaxLevel.isSet() && layer->getTerrainLayerRuntimeOptions().maxLevel().isSet() )
            {
                layerMaxLevel = std::min( layerMaxLevel.get(), (int)(*layer->getTerrainLayerRuntimeOptions().maxLevel()) );
            }

            // Need to convert the layer max of this TileSource to that of the actual profile
            if ( layerMaxLevel.isSet() )
            {
                layerMaxLevel = profile->getEquivalentLOD( ts->getProfile(), layerMaxLevel.get() );
            }
        }
        else
        {
            // no TileSource? probably in cache-only mode. Use the layer max (or its default).
            layerMaxLevel = (int)(layer->getTerrainLayerRuntimeOptions().maxLevel().value());
        }

        // Adjust for the tile size resolution differential, if supported by the layer.
        if ( layerMaxLevel.isSet() )
        {
			// TODO:  This native max resolution of the layer has already been computed here.
			//        The following block attempts to compute a higher resolution to undo the resolution
			//        mapping that populateHeightField will eventually do.  So for example, you might compute a maximum level of
			//        10 here, and this will adjust it to 14 with the knowledge that populateHeightField will adjust the 14 back to 10.
			//        The use of populateHeightField needs to be replaced by code that just works with the native resolution of the
			//        layers instead.
#if 1
            int layerTileSize = layer->getTileSize();
            if (layerTileSize > targetTileSizePOT)
            {
                int temp = std::max(targetTileSizePOT, 2);
                while(temp < layerTileSize)
                {
                    temp *= 2;
                    layerMaxLevel = layerMaxLevel.get() + 1;
                }
            }
#endif

            if (layerMaxLevel > maxLevel)
            {
                maxLevel = layerMaxLevel.get();
            }
        }
    }

    return maxLevel;
}


void
ElevationQuery::setMaxTilesToCache( int value )
{
    _cache.setMaxSize( value );
}

int
ElevationQuery::getMaxTilesToCache() const
{
    return _cache.getMaxSize();
}

void
ElevationQuery::setMaxLevelOverride(int maxLevelOverride)
{
    _maxLevelOverride = maxLevelOverride;
}

int
ElevationQuery::getMaxLevelOverride() const
{
    return _maxLevelOverride;
}

bool
ElevationQuery::getElevation(const GeoPoint&         point,
                             double&                 out_elevation,
                             double                  desiredResolution,
                             double*                 out_actualResolution)
{
    sync();
    if ( point.altitudeMode() == ALTMODE_ABSOLUTE )
    {
        return getElevationImpl( point, out_elevation, desiredResolution, out_actualResolution );
    }
    else
    {
        GeoPoint point_abs( point.getSRS(), point.x(), point.y(), 0.0, ALTMODE_ABSOLUTE );
        return getElevationImpl( point_abs, out_elevation, desiredResolution, out_actualResolution );
    }
}


bool
ElevationQuery::getElevations(std::vector<osg::Vec3d>& points,
                              const SpatialReference*  pointsSRS,
                              bool                     ignoreZ,
                              double                   desiredResolution )
{
    sync();
    for( osg::Vec3dArray::iterator i = points.begin(); i != points.end(); ++i )
    {
        double elevation;
        double z = (*i).z();
        GeoPoint p(pointsSRS, *i, ALTMODE_ABSOLUTE);
        if ( getElevationImpl( p, elevation, desiredResolution ) )
        {
            (*i).z() = ignoreZ ? elevation : elevation + z;
        }
    }
    return true;
}

bool
ElevationQuery::getElevations(const std::vector<osg::Vec3d>& points,
                              const SpatialReference*        pointsSRS,
                              std::vector<double>&           out_elevations,
                              double                         desiredResolution )
{
    sync();
    for( osg::Vec3dArray::const_iterator i = points.begin(); i != points.end(); ++i )
    {
        double elevation;
        GeoPoint p(pointsSRS, *i, ALTMODE_ABSOLUTE);

        if ( getElevationImpl(p, elevation, desiredResolution) )
        {
            out_elevations.push_back( elevation );
        }
        else
        {
            out_elevations.push_back( 0.0 );
        }
    }
    return true;
}

bool
ElevationQuery::getElevationImpl(const GeoPoint& point, /* abs */
                                 double&         out_elevation,
                                 double          desiredResolution,
                                 double*         out_actualResolution)
{
    // assertion.
    if ( !point.isAbsolute() )
    {
        OE_WARN << LC << "Assertion failure; input must be absolute" << std::endl;
        return false;
    }

    osg::Timer_t begin = osg::Timer::instance()->tick();

    // first try the terrain patches.
    if ( _patchLayers.size() > 0 )
    {
        osgUtil::IntersectionVisitor iv;

        if ( _eqcrc.valid() )
            iv.setReadCallback(_eqcrc);

        for(std::vector<ModelLayer*>::iterator i = _patchLayers.begin(); i != _patchLayers.end(); ++i)
        {
            // find the scene graph for this layer:
            osg::Node* node = (*i)->getSceneGraph( _mapf.getUID() );
            if ( node )
            {
                // configure for intersection:
                osg::Vec3d surface;
                point.toWorld( surface );

                // trivial bounds check:
                if ( node->getBound().contains(surface) )
                {
                    osg::Vec3d nvector;
                    point.createWorldUpVector(nvector);

                    osg::Vec3d start( surface + nvector*5e5 );
                    osg::Vec3d end  ( surface - nvector*5e5 );

                    // first time through, set up the intersector on demand
                    if ( !_patchLayersLSI.valid() )
                    {
                        _patchLayersLSI = new DPLineSegmentIntersector(start, end);
                        _patchLayersLSI->setIntersectionLimit( _patchLayersLSI->LIMIT_NEAREST );
                    }
                    else
                    {
                        _patchLayersLSI->reset();
                        _patchLayersLSI->setStart( start );
                        _patchLayersLSI->setEnd  ( end );
                    }

                    // try it.
                    iv.setIntersector( _patchLayersLSI.get() );
                    node->accept( iv );

                    // check for a result!!
                    if ( _patchLayersLSI->containsIntersections() )
                    {
                        osg::Vec3d isect = _patchLayersLSI->getIntersections().begin()->getWorldIntersectPoint();

                        // transform back to input SRS:
                        GeoPoint output;
                        output.fromWorld( point.getSRS(), isect );
                        out_elevation = output.z();
                        if ( out_actualResolution )
                            *out_actualResolution = 0.0;

                        return true;
                    }
                }
                else
                {
                    //OE_INFO << LC << "Trivial rejection (bounds check)" << std::endl;
                }
            }
        }
    }

    if ( _mapf.elevationLayers().empty() )
    {
        // this means there are no heightfields.
        out_elevation = 0.0;
        return true;
    }

    // tile size (resolution of elevation tiles)
    unsigned tileSize = 33; // ???

    // This is the max resolution that we actually have data at this point
    int bestAvailLevel = getMaxLevel( point.x(), point.y(), point.getSRS(), _mapf.getProfile(), tileSize );

    // A negative value means that no data is avaialble at that point at any resolution.
    if ( bestAvailLevel < 0 )
    {
        return false;
    }

    if (desiredResolution > 0.0)
    {
        int desiredLevel = _mapf.getProfile()->getLevelOfDetailForHorizResolution( desiredResolution, tileSize );
        if (desiredLevel < bestAvailLevel)
        {
            bestAvailLevel = desiredLevel;
        }
    }

    //OE_NOTICE << LC << "Best available data level " << point.x() << ", " << point.y() << " = "  << bestAvailLevel << std::endl;

    // transform the input coords to map coords:
    GeoPoint mapPoint = point;
    if ( point.isValid() && !point.getSRS()->isHorizEquivalentTo( _mapf.getProfile()->getSRS() ) )
    {
        mapPoint = point.transform(_mapf.getProfile()->getSRS());
        if ( !mapPoint.isValid() )
        {
            OE_WARN << LC << "Fail: coord transform failed" << std::endl;
            return false;
        }
    }

    // get the tilekey corresponding to the tile we need:
    TileKey key = _mapf.getProfile()->createTileKey( mapPoint.x(), mapPoint.y(), bestAvailLevel );
    if ( !key.valid() )
    {
        OE_WARN << LC << "Fail: coords fall outside map" << std::endl;
        return false;
    }

    bool result = false;

    while ( !result && key.valid() )
    {
        GeoHeightField geoHF;

        // Try to get the hf from the cache
        TileCache::Record record;
        if ( _cache.get( key, record ) )
        {
            geoHF = record.value();
        }
        else
        {
            // Create it
            osg::ref_ptr<osg::HeightField> hf = new osg::HeightField();
            hf->allocate( tileSize, tileSize );

            // Initialize the heightfield to nodata
            hf->getFloatArray()->assign( hf->getFloatArray()->size(), NO_DATA_VALUE );

            if (_mapf.populateHeightField(hf, key, false /*heightsAsHAE*/, 0L))
            {
                geoHF = GeoHeightField( hf.get(), key.getExtent() );
                _cache.insert( key, geoHF );
            }
        }

        if (geoHF.valid())
        {
            float elevation = 0.0f;
            result = geoHF.getElevation( mapPoint.getSRS(), mapPoint.x(), mapPoint.y(), _mapf.getMapInfo().getElevationInterpolation(), mapPoint.getSRS(), elevation);
            if (result && elevation != NO_DATA_VALUE)
            {
                out_elevation = (double)elevation;

                // report the actual resolution of the heightfield.
                if ( out_actualResolution )
                    *out_actualResolution = geoHF.getXInterval();
            }
            else
            {
                result = false;
            }
        }

        if ( geoHF.valid() && !_fallBackOnNoData )
        {
            break;
        }
        else if ( result == false )
        {
            key = key.createParentKey();
        }
    }


    osg::Timer_t end = osg::Timer::instance()->tick();
    _queries++;
    _totalTime += osg::Timer::instance()->delta_s( begin, end );

    return result;
}

void ElevationQuery::setElevationQueryCacheReadCallback(ElevationQueryCacheReadCallback* eqcrc)
{
    _eqcrc = eqcrc;
}
