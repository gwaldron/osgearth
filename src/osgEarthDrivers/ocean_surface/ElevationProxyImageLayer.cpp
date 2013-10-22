/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

using namespace osgEarth_ocean_surface;
using namespace osgEarth;

ElevationProxyImageLayer::ElevationProxyImageLayer( Map* sourceMap, const ImageLayerOptions& options ) :
ImageLayer( options ),
_sourceMap( sourceMap ),
_mapf     ( sourceMap )
{
    _runtimeOptions.cachePolicy() = CachePolicy::NO_CACHE;
}

void
ElevationProxyImageLayer::initTileSource()
{
    _tileSourceInitAttempted = true;
    _tileSourceInitFailed    = true;
}

bool
ElevationProxyImageLayer::isKeyValid( const TileKey& key ) const
{
    return key.getLevelOfDetail() <= *_runtimeOptions.maxLevel();
}

bool
ElevationProxyImageLayer::isCached( const TileKey& key ) const
{
    return true;
}

GeoImage
ElevationProxyImageLayer::createImage(const TileKey& key, ProgressCallback* progress, bool forceFallback)
{
    osg::ref_ptr<Map> map = _sourceMap.get();
    if ( map.valid() )
    {
        osg::ref_ptr<osg::HeightField> hf;
        if ( map->getHeightField( key, true, hf ) )
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
    }
    return GeoImage::INVALID;
}
