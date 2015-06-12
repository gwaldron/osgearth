/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include "HeightFieldCache"

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;

#define LC "[MP.HeightFieldCache] "


bool
HeightFieldCache::getOrCreateHeightField(const MapFrame&                 frame,
                                         const TileKey&                  key,
                                         const osg::HeightField*         parent_hf,
                                         osg::ref_ptr<osg::HeightField>& out_hf,
                                         bool&                           out_isFallback,
                                         ElevationSamplePolicy           samplePolicy,
                                         ElevationInterpolation          interp,
                                         ProgressCallback*               progress )
{                
    // default
    out_isFallback = false;
    
    // check the quick cache.
    HFKey cachekey;
    cachekey._key          = key;
    cachekey._revision     = frame.getRevision();
    cachekey._samplePolicy = samplePolicy;

    if (progress)
        progress->stats()["hfcache_try_count"] += 1;

    bool hit = false;
    LRUCache<HFKey,HFValue>::Record rec;
    if ( _cache.get(cachekey, rec) )
    {
        out_hf = rec.value()._hf.get();
        out_isFallback = rec.value()._isFallback;

        if (progress)
        {
            progress->stats()["hfcache_hit_count"] += 1;
            progress->stats()["hfcache_hit_rate"] = progress->stats()["hfcache_hit_count"]/progress->stats()["hfcache_try_count"];
        }

        return true;
    }

    // Find the parent tile and start with its heightfield.
    if ( parent_hf )
    {
        TileKey parentKey = key.createParentKey();
        
        out_hf = HeightFieldUtils::createSubSample(
            parent_hf,
            parentKey.getExtent(),
            key.getExtent(),
            interp );

        if ( !out_hf.valid() && ((int)key.getLOD())-1 > _firstLOD )
        {
            // This most likely means that a parent tile expired while we were building the child.
            // No harm done in that case as this tile will soo be discarded as well.
            // OE_WARN << LC << "MP HFC: Unable to find tile " << key.str() << " in the live tile registry" << std::endl;
            return false;
        }
    }
    

    if ( !out_hf.valid() )
    {
        //TODO.
        // This sets the elevation tile size; query size for all tiles.
        out_hf = HeightFieldUtils::createReferenceHeightField(
            key.getExtent(), _tileSize, _tileSize, true );
    }

    bool populated = frame.populateHeightField(
        out_hf,
        key,
        true, // convertToHAE
        samplePolicy,
        progress );

    // Treat Plate Carre specially by scaling the height values. (There is no need
    // to do this with an empty heightfield)
    const MapInfo& mapInfo = frame.getMapInfo();
    if ( mapInfo.isPlateCarre() )
    {
        HeightFieldUtils::scaleHeightFieldToDegrees( out_hf.get() );
    }

    // ONLY cache the new heightfield if a parent HF existed. Otherwise the new HF
    // may contain invalid data. This can happen if this task runs to completion
    // while the tile's parent expires from the scene graph. In that case the result
    // of this task will be discarded. Therefore we should not cache the result here.
    // This was causing intermittent rare "flat tiles" to appear in the terrain.
    if ( parent_hf )
    {
        // cache it.
        HFValue cacheval;
        cacheval._hf = out_hf.get();
        cacheval._isFallback = !populated;
        _cache.insert( cachekey, cacheval );
    }

    out_isFallback = !populated;
    return true;
}
