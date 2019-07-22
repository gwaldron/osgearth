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

#include <osgEarth/Map>
#include <osgEarth/ElevationLayer>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/ImageUtils>

using namespace osgEarth;
using namespace osgEarth::SimpleOcean;

#define LC "[ElevationProxyImageLayer] "


ElevationProxyImageLayer::ElevationProxyImageLayer(const Map* sourceMap,
                                                   const ImageLayerOptions& inoptions ) :
ImageLayer( inoptions ),
_map(sourceMap)
{
    options().cachePolicy() = CachePolicy::NO_CACHE;
}

TileSource*
ElevationProxyImageLayer::createTileSource()
{
    return 0L;
}

bool
ElevationProxyImageLayer::isKeyInLegalRange( const TileKey& key ) const
{
    return key.getLevelOfDetail() <= options().maxLevel().get();
}

bool
ElevationProxyImageLayer::isCached( const TileKey& key ) const
{
    return true;
}

GeoImage
ElevationProxyImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress)
{
    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return GeoImage::INVALID;

    osg::ref_ptr<osg::HeightField> hf = HeightFieldUtils::createReferenceHeightField(key.getExtent(), 64, 64, 0, true );

    ElevationLayerVector elevation;
    map->getLayers(elevation);

    if (elevation.populateHeightFieldAndNormalMap(hf.get(), NULL, key, map->getProfileNoVDatum(), INTERP_BILINEAR, progress))
    {
        // encode the heightfield as a 16-bit normalized LUNIMANCE image
        osg::Image* image = new osg::Image();
        image->allocateImage(hf->getNumColumns(), hf->getNumRows(), 1, GL_RED, GL_FLOAT);
        image->setInternalTextureFormat( GL_R32F );
        const osg::FloatArray* floats = hf->getFloatArray();
        ImageUtils::PixelWriter write(image);
        for (unsigned t = 0; t < image->t(); ++t) {
            for (unsigned s = 0; s < image->s(); ++s) {
                float v = floats->at(t*image->s()+s);
                write(osg::Vec4(v,v,v,v), s, t);
            }
        }

        //for( unsigned int i = 0; i < floats->size(); ++i  )
        //{
        //    int col = i % hf->getNumColumns();
        //    int row = i / hf->getNumColumns();
        //    *(float*)image->data(col, row) = floats->at(i);
        //    //*(unsigned short*)image->data( col, row ) = (unsigned short)(32768 + (short)floats->at(i));
        //}

        return GeoImage( image, key.getExtent() );
    }
    else
    {
        return GeoImage::INVALID;
    }
}
