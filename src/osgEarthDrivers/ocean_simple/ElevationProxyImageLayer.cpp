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
#include "ElevationProxyImageLayer"

#include <osgEarth/HeightFieldUtils>

using namespace osgEarth;
using namespace osgEarth::SimpleOcean;

#define LC "[ElevationProxyImageLayer] "


ElevationProxyImageLayer::ElevationProxyImageLayer(const Map* sourceMap,
                                                   const ImageLayerOptions& options ) :
ImageLayer( options ),
_mapf     ( sourceMap )
{
    _runtimeOptions.cachePolicy() = CachePolicy::NO_CACHE;
}

TileSource*
ElevationProxyImageLayer::createTileSource()
{
    return 0L;
}

bool
ElevationProxyImageLayer::isKeyInRange( const TileKey& key ) const
{
    return key.getLevelOfDetail() <= *_runtimeOptions.maxLevel();
}

bool
ElevationProxyImageLayer::isCached( const TileKey& key ) const
{
    return true;
}

GeoImage
ElevationProxyImageLayer::createImage(const TileKey& key, ProgressCallback* progress)
{
    if ( _mapf.needsSync() )
    {
        Threading::ScopedMutexLock lock(_mapfMutex);
        if ( _mapf.needsSync() )
        {
            _mapf.sync();
        }
    }

    osg::ref_ptr<osg::HeightField> hf = HeightFieldUtils::createReferenceHeightField(key.getExtent(), 257,257, true );

    if ( _mapf.populateHeightField(hf, key, true, 0L) )
    {
        // encode the heightfield as a 16-bit normalized LUNIMANCE image
        osg::Image* image = new osg::Image();
        image->allocateImage(hf->getNumColumns(), hf->getNumRows(), 1, GL_LUMINANCE, GL_UNSIGNED_SHORT);
        image->setInternalTextureFormat( GL_LUMINANCE16 );
        const osg::FloatArray* floats = hf->getFloatArray();
        for( unsigned int i = 0; i < floats->size(); ++i  )
        {
            int col = i % hf->getNumColumns();
            int row = i / hf->getNumColumns();
            *(unsigned short*)image->data( col, row ) = (unsigned short)(32768 + (short)floats->at(i));
        }

        return GeoImage( image, key.getExtent() );
    }
    else
    {
        return GeoImage::INVALID;
    }
}
