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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "HeightFieldCache"
#include <cstdlib>

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;

#define LC "[MP.HeightFieldCache] "


HeightFieldCache::HeightFieldCache(const MPTerrainEngineOptions& options) :
_cache   ( true, 128 ),
_tileSize( options.tileSize().get() )
{
    _useParentAsReferenceHF = (options.elevationSmoothing() == true);
    _enabled = (::getenv("OSGEARTH_MEMORY_PROFILE") == 0L);
}


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

    LRUCache<HFKey,HFValue>::Record rec;
    if ( _enabled && _cache.get(cachekey, rec) )
    {
        // Found it in the cache.
        out_hf         = rec.value()._hf.get();
        out_isFallback = rec.value()._isFallback;

        if (progress)
        {
            progress->stats()["hfcache_hit_count"] += 1;
            progress->stats()["hfcache_hit_rate"] = progress->stats()["hfcache_hit_count"]/progress->stats()["hfcache_try_count"];
        }
    }

    else
    {
        // Not in the cache, so we need to create a HF.
        TileKey parentKey = key.createParentKey();

        // Elevation "smoothing" uses the parent HF as the starting point for building
        // a new tile. This will cause lower-resolution data to propagate down the tree
        // and fill in any gaps in higher-resolution data. The result will be an elevation
        // grid that is "smoother" but not neccessarily as accurate.
        if ( _useParentAsReferenceHF && parent_hf && parentKey.valid() )
        {
            out_hf = HeightFieldUtils::createSubSample(
                parent_hf,
                parentKey.getExtent(),
                key.getExtent(),
                interp );
        }

        // If we are not smoothing, or we have no parent data, start with a basic
        // MSL=0 reference heightfield instead.
        if ( !out_hf.valid() )
        {
            out_hf = HeightFieldUtils::createReferenceHeightField( key.getExtent(), _tileSize, _tileSize );
        }

        // Next, populate it with data from the Map. The map will overwrite our starting
        // data with real data from the elevation stack.
        bool populated = frame.populateHeightField(
            out_hf,
            key,
            true, // convertToHAE
            progress );

        // If the map failed to provide any suitable data sources at all, replace the
        // heightfield with data from its parent (if available). 
        if ( !populated )
        {
            if ( parentKey.valid() && parent_hf )
            {        
                out_hf = HeightFieldUtils::createSubSample(
                    parent_hf,
                    parentKey.getExtent(),
                    key.getExtent(),
                    interp );
            }

            if ( !out_hf.valid() )
            {
                // NOTE: This is probably no longer be possible, but check anyway for completeness.
                return false;
            }
        }

        // ONLY cache the new heightfield if a parent HF existed. Otherwise the new HF
        // may contain invalid data. This can happen if this task runs to completion
        // while the tile's parent expires from the scene graph. In that case the result
        // of this task will be discarded. Therefore we should not cache the result here.
        // This was causing intermittent rare "flat tiles" to appear in the terrain.
        if ( _enabled && parent_hf )
        {
            // cache it.
            HFValue cacheval;
            cacheval._hf = out_hf.get();
            cacheval._isFallback = !populated;
            _cache.insert( cachekey, cacheval );
        }

        out_isFallback = !populated;
    }

    return true;
}
