/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

#include <osgEarth/CacheSeed>
#include <OpenThreads/ScopedLock>
#include <limits.h>

#define LC "[CacheSeed] "

using namespace osgEarth;
using namespace OpenThreads;

CacheSeed::CacheSeed():
          _minLevel(0),
          _maxLevel(12),          
          _total(0),
          _completed(0)
          {
          }

void CacheSeed::seed( Map* map )
{
    if ( !map->getCache() )
    {
        OE_WARN << LC << "Warning: No cache defined; aborting." << std::endl;
        return;
    }

    std::vector<TileKey> keys;
    map->getProfile()->getRootKeys(keys);

    //Add the map's entire extent if we don't have one specified.
    if (_extents.empty())
    {
        addExtent( map->getProfile()->getExtent() );
    }

    bool hasCaches = false;
    int src_min_level = INT_MAX;
    unsigned int src_max_level = 0;

    MapFrame mapf( map, Map::TERRAIN_LAYERS, "CacheSeed::seed" );

    //Assumes the the TileSource will perform the caching for us when we call createImage
    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); i++ )
    {
		ImageLayer* layer = i->get();
        TileSource* src   = layer->getTileSource();

        const ImageLayerOptions& opt = layer->getImageLayerOptions();

        if ( layer->isCacheOnly() )
        {
            OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" is set to cache-only; skipping." << std::endl;
        }
        else if (!src)
        {
            OE_WARN << "Warning: Layer \"" << layer->getName() << "\" could not create TileSource; skipping." << std::endl;
        }
        else if ( !src->supportsPersistentCaching() )
        {
            OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" does not support seeding; skipping." << std::endl;
        }
        else if ( !layer->getCache() )
        {
            OE_WARN << LC << "Notice: Layer \"" << layer->getName() << "\" has no cache defined; skipping." << std::endl;
        }
        else
        {
            hasCaches = true;

			if (opt.minLevel().isSet() && (int)opt.minLevel().get() < src_min_level)
                src_min_level = opt.minLevel().get();
			if (opt.maxLevel().isSet() && opt.maxLevel().get() > src_max_level)
                src_max_level = opt.maxLevel().get();
        }
    }

    for( ElevationLayerVector::const_iterator i = mapf.elevationLayers().begin(); i != mapf.elevationLayers().end(); i++ )
    {
		ElevationLayer* layer = i->get();
        TileSource*     src   = layer->getTileSource();
        const ElevationLayerOptions& opt = layer->getElevationLayerOptions();

        if ( layer->isCacheOnly() )
        {
            OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" is set to cache-only; skipping." << std::endl;
        }
        else if (!src)
        {
            OE_WARN << "Warning: Layer \"" << layer->getName() << "\" could not create TileSource; skipping." << std::endl;
        }
        else if ( !src->supportsPersistentCaching() )
        {
            OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" does not support seeding; skipping." << std::endl;
        }
        else if ( !layer->getCache() )
        {
            OE_WARN << LC << "Notice: Layer \"" << layer->getName() << "\" has no cache defined; skipping." << std::endl;
        }
        else
        {
            hasCaches = true;

			if (opt.minLevel().isSet() && (int)opt.minLevel().get() < src_min_level)
                src_min_level = opt.minLevel().get();
			if (opt.maxLevel().isSet() && opt.maxLevel().get() > src_max_level)
                src_max_level = opt.maxLevel().get();
		}
    }

    if ( !hasCaches )
    {
        OE_WARN << LC << "There are either no caches defined in the map, or no sources to cache; aborting." << std::endl;
        return;
    }

    if ( src_max_level > 0 && src_max_level < _maxLevel )
    {
        _maxLevel = src_max_level;
    }

    OE_NOTICE << LC << "Maximum cache level will be " << _maxLevel << std::endl;

    osg::Timer_t startTime = osg::Timer::instance()->tick();
    //Estimate the number of tiles
    _total = 0;    

    for (unsigned int level = _minLevel; level <= _maxLevel; level++)
    {
        double coverageRatio = 0.0;

        if (_extents.empty())
        {
            unsigned int wide, high;
            map->getProfile()->getNumTiles( level, wide, high );
            _total += (wide * high);
        }
        else
        {
            for (std::vector< GeoExtent >::const_iterator itr = _extents.begin(); itr != _extents.end(); itr++)
            {
                const GeoExtent& extent = *itr;
                double boundsArea = extent.area();

                TileKey ll = map->getProfile()->createTileKey(extent.xMin(), extent.yMin(), level);
                TileKey ur = map->getProfile()->createTileKey(extent.xMax(), extent.yMax(), level);

                int tilesWide = ur.getTileX() - ll.getTileX() + 1;
                int tilesHigh = ll.getTileY() - ur.getTileY() + 1;
                int tilesAtLevel = tilesWide * tilesHigh;
                //OE_NOTICE << "Tiles at level " << level << "=" << tilesAtLevel << std::endl;

                bool hasData = false;

                for (ImageLayerVector::const_iterator itr = mapf.imageLayers().begin(); itr != mapf.imageLayers().end(); itr++)
                {
                    TileSource* src = itr->get()->getTileSource();
                    if (src)
                    {
                        if (src->hasDataAtLOD( level ))
                        {
                            //Compute the percent coverage of this dataset on the current extent
                            if (src->getDataExtents().size() > 0)
                            {
                                double cov = 0.0;
                                for (unsigned int j = 0; j < src->getDataExtents().size(); j++)
                                {
                                    GeoExtent b = src->getDataExtents()[j].transform( extent.getSRS());
                                    GeoExtent intersection = b.intersectionSameSRS( extent );
                                    if (intersection.isValid())
                                    {
                                        double coverage = intersection.area() / boundsArea;
                                        cov += coverage; //Assumes the extents aren't overlapping                            
                                    }
                                }
                                if (coverageRatio < cov) coverageRatio = cov;
                            }
                            else
                            {
                                //We have no way of knowing how much coverage we have
                                coverageRatio = 1.0;
                            }
                            hasData = true;
                            break;
                        }
                    }
                }

                for (ElevationLayerVector::const_iterator itr = mapf.elevationLayers().begin(); itr != mapf.elevationLayers().end(); itr++)
                {
                    TileSource* src = itr->get()->getTileSource();
                    if (src)
                    {
                        if (src->hasDataAtLOD( level ))
                        {
                            //Compute the percent coverage of this dataset on the current extent
                            if (src->getDataExtents().size() > 0)
                            {
                                double cov = 0.0;
                                for (unsigned int j = 0; j < src->getDataExtents().size(); j++)
                                {
                                    GeoExtent b = src->getDataExtents()[j].transform( extent.getSRS());
                                    GeoExtent intersection = b.intersectionSameSRS( extent );
                                    if (intersection.isValid())
                                    {
                                        double coverage = intersection.area() / boundsArea;
                                        cov += coverage; //Assumes the extents aren't overlapping                            
                                    }
                                }
                                if (coverageRatio < cov) coverageRatio = cov;
                            }
                            else
                            {
                                //We have no way of knowing how much coverage we have
                                coverageRatio = 1.0;
                            }
                            hasData = true;
                            break;
                        }
                    }
                }

                //Adjust the coverage ratio by a fudge factor to try to keep it from being too small,
                //tiles are either processed or not and the ratio is exact so will cover tiles partially
                //and potentially be too small
                double adjust = 4.0;
                coverageRatio = osg::clampBetween(coverageRatio * adjust, 0.0, 1.0);

                //OE_NOTICE << level <<  " CoverageRatio = " << coverageRatio << std::endl;

                if (hasData)
                {
                    _total += (int)ceil(coverageRatio * (double)tilesAtLevel );
                }
            }
        }
    }

    //Adjust the # of tiles again to be bigger than computed to avoid giving false hope
    _total *= 2;
    osg::Timer_t endTime = osg::Timer::instance()->tick();
    //OE_NOTICE << "Counted tiles in " << osg::Timer::instance()->delta_s(startTime, endTime) << " s" << std::endl;

    OE_INFO << "Processing ~" << _total << " tiles" << std::endl;

    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        processKey( mapf, keys[i] );
    }

    _total = _completed;

    if ( _progress.valid()) _progress->reportProgress(_completed, _total, "Finished");
}

void CacheSeed::incrementCompleted( unsigned int total ) const
{    
    CacheSeed* nonconst_this = const_cast<CacheSeed*>(this);
    nonconst_this->_completed += total;
}

void
CacheSeed::processKey(const MapFrame& mapf, const TileKey& key ) const
{
    unsigned int x, y, lod;
    key.getTileXY(x, y);
    lod = key.getLevelOfDetail();

    bool gotData = true;

    if ( _minLevel <= lod && _maxLevel >= lod )
    {
        gotData = cacheTile( mapf, key );
        if (gotData)
        {
        incrementCompleted( 1 );
        }

    	if ( _progress.valid() && _progress->isCanceled() )
	        return; // Task has been cancelled by user

        if ( _progress.valid() && gotData && _progress->reportProgress(_completed, _total, std::string("Cached tile: ") + key.str()) )
            return; // Canceled
    }

    if ( gotData && lod <= _maxLevel )
    {
        TileKey k0 = key.createChildKey(0);
        TileKey k1 = key.createChildKey(1);
        TileKey k2 = key.createChildKey(2);
        TileKey k3 = key.createChildKey(3); 

        bool intersectsKey = false;
        if (_extents.empty()) intersectsKey = true;
        else
        {
            for (unsigned int i = 0; i < _extents.size(); ++i)
            {
                if (_extents[i].intersects( k0.getExtent() ) ||
                    _extents[i].intersects( k1.getExtent() ) ||
                    _extents[i].intersects( k2.getExtent() ) ||
                    _extents[i].intersects( k3.getExtent() ))
                {
                    intersectsKey = true;
                }

            }
        }

        //Check to see if the bounds intersects ANY of the tile's children.  If it does, then process all of the children
        //for this level
        if (intersectsKey)
        {
            processKey(mapf, k0);
            processKey(mapf, k1);
            processKey(mapf, k2);
            processKey(mapf, k3);
        }
    }
}

bool
CacheSeed::cacheTile(const MapFrame& mapf, const TileKey& key ) const
{
    bool gotData = false;

    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); i++ )
    {
        ImageLayer* layer = i->get();
        if ( layer->isKeyValid( key ) )
        {
            GeoImage image = layer->createImage( key );
            if ( image.valid() )
                gotData = true;
        }
    }

    if ( mapf.elevationLayers().size() > 0 )
    {
        osg::ref_ptr<osg::HeightField> hf;
        mapf.getHeightField( key, false, hf );
        if ( hf.valid() )
            gotData = true;
    }

    return gotData;
}

void
CacheSeed::addExtent( const GeoExtent& value)
{
    _extents.push_back( value );
}
