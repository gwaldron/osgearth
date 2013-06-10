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
#include "TileModel"
#include <osgEarth/MapInfo>
#include <osgEarth/HeightFieldUtils>
#include <osgTerrain/Locator>

using namespace osgEarth_engine_mp;
using namespace osgEarth;

#define LC "[TileModel] "

//------------------------------------------------------------------


TileModel::ElevationData::ElevationData(osg::HeightField* hf,
                                        GeoLocator*       locator,
                                        bool              fallbackData) :
_hf          ( hf ),
_locator     ( locator ),
_fallbackData( fallbackData )
{
    //nop
}

TileModel::ElevationData::ElevationData(const TileModel::ElevationData& rhs) :
_hf          ( rhs._hf.get() ),
_locator     ( rhs._locator.get() ),
_fallbackData( rhs._fallbackData ),
_parent      ( rhs._parent )
{
    for(unsigned i=0; i<8; ++i)
        _neighbors[i] = rhs._neighbors[i];
}

bool
TileModel::ElevationData::getHeight(const osg::Vec3d&      ndc,
                                    const GeoLocator*      ndcLocator,
                                    float&                 output,
                                    ElevationInterpolation interp ) const
{
    if ( !_locator.valid() || !ndcLocator )
        return false;

    osg::Vec3d hf_ndc;
    GeoLocator::convertLocalCoordBetween( *ndcLocator, ndc, *_locator.get(), hf_ndc );
    output = HeightFieldUtils::getHeightAtNormalizedLocation( _hf.get(), hf_ndc.x(), hf_ndc.y(), interp );
    return true;
}

bool
TileModel::ElevationData::getNormal(const osg::Vec3d& ndc,
                                    const GeoLocator* ndcLocator,
                                    osg::Vec3&        output ) const
{
    if ( !_locator.valid() || !ndcLocator )
    {
        output.set(0,0,1);
        return false;
    }

    osg::Vec3d hf_ndc;
    GeoLocator::convertLocalCoordBetween( *ndcLocator, ndc, *_locator.get(), hf_ndc );
    return HeightFieldUtils::getNormalAtNormalizedLocation( _hf.get(), hf_ndc.x(), hf_ndc.y(), output );
}

//------------------------------------------------------------------

TileModel::ColorData::ColorData(const osgEarth::ImageLayer* layer,
                                unsigned                    order,
                                osg::Image*                 image,
                                GeoLocator*                 locator,
                                const osgEarth::TileKey&    tileKey,
                                bool                        fallbackData) :
_layer       ( layer ),
_order       ( order ),
//_image       ( image ),
_locator     ( locator ),
_tileKey     ( tileKey ),
_fallbackData( fallbackData )
{
    _texture = new osg::Texture2D( image );
    _texture->setUnRefImageDataAfterApply( true );
    _texture->setMaxAnisotropy( 16.0f );
    _texture->setResizeNonPowerOfTwoHint(false);
    _texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    _texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    _texture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    _texture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    _texture->setWrap( osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE );
    //_image = 0L;
}

TileModel::ColorData::ColorData(const TileModel::ColorData& rhs) :
_layer       ( rhs._layer.get() ),
_locator     ( rhs._locator.get() ),
//_image       ( rhs._image.get() ),
_texture     ( rhs._texture.get() ),
_tileKey     ( rhs._tileKey ),
_fallbackData( rhs._fallbackData ),
_order       ( rhs._order )
{
    //nop
}

void
TileModel::ColorData::releaseGLObjects(osg::State* state) const
{
    if ( _texture.valid() && _texture->referenceCount() == 1 )
    {
        _texture->releaseGLObjects( state );
    }
}

//------------------------------------------------------------------

TileModel::TileModel(const TileModel& rhs) :
_map           ( rhs._map.get() ),
_tileKey       ( rhs._tileKey ),
_tileLocator   ( rhs._tileLocator.get() ),
_colorData     ( rhs._colorData ),
_elevationData ( rhs._elevationData ),
_sampleRatio   ( rhs._sampleRatio ),
_parentStateSet( rhs._parentStateSet )
{
    //nop
}


TileModel*
TileModel::createQuadrant(unsigned q) const
{
    // copy this object:
    TileModel* model = new TileModel( *this );

    // then modify it for the quadrant.
    TileKey childKey = _tileKey.createChildKey( q );
    model->_tileKey = childKey;
    model->_tileLocator = _tileLocator->createSameTypeForKey( childKey, MapInfo(_map.get()) );

    return model;
}


void
TileModel::setParentTileModel(const TileModel* parent)
{
    _parentModel = parent;
}
//    osg::Matrixf scaleBias;
//    scaleBias(0,0) = 0.5f;
//    scaleBias(1,1) = 0.0f;
//    scaleBias(3,0) = (float)(    _key.getTileX() & 0x1) * 0.5f;
//    scaleBias(3,1) = (float)(1 - _key.getTileY() & 0x1) * 0.5f;
//
//    for(ColorDataByUID::const_iterator i = _colorData.begin(); i != _colorData.end(); ++i )
//    {
//        ColorData& c = i->second;
//        c._scaleBiasMatrix = scaleBias;
//    }
//}


void
TileModel::releaseGLObjects(osg::State* state) const
{
    for(ColorDataByUID::const_iterator i = _colorData.begin(); i != _colorData.end(); ++i )
        i->second.releaseGLObjects( state );
}
