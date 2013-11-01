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
    _neighbors._center = hf;
}

TileModel::ElevationData::ElevationData(const TileModel::ElevationData& rhs) :
_hf          ( rhs._hf.get() ),
_locator     ( rhs._locator.get() ),
_fallbackData( rhs._fallbackData ),
_parent      ( rhs._parent )
{
    _neighbors._center = rhs._neighbors._center.get();
    for(unsigned i=0; i<8; ++i)
        _neighbors._neighbors[i] = rhs._neighbors._neighbors[i];
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
TileModel::ElevationData::getNormal(const osg::Vec3d&      ndc,
                                    const GeoLocator*      ndcLocator,
                                    osg::Vec3&             output,
                                    ElevationInterpolation interp ) const
{
    if ( !_locator.valid() || !ndcLocator )
    {
        output.set(0,0,1);
        return false;
    }

    double xcells = (double)(_hf->getNumColumns()-1);
    double ycells = (double)(_hf->getNumRows()-1);
    double xres = 1.0/xcells;
    double yres = 1.0/ycells;

    osg::Vec3d hf_ndc;
    GeoLocator::convertLocalCoordBetween( *ndcLocator, ndc, *_locator.get(), hf_ndc );

    osg::Vec3d west ( hf_ndc.x()-xres, hf_ndc.y(), 0.0 );
    osg::Vec3d east ( hf_ndc.x()+xres, hf_ndc.y(), 0.0 );
    osg::Vec3d south( hf_ndc.x(), hf_ndc.y()-yres, 0.0 );
    osg::Vec3d north( hf_ndc.x(), hf_ndc.y()+yres, 0.0 );

    west.z()  = HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, west.x(),  west.y(),  interp);
    east.z()  = HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, east.x(),  east.y(),  interp);
    south.z() = HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, south.x(), south.y(), interp);
    north.z() = HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, north.x(), north.y(), interp);

    osg::Vec3d westWorld, eastWorld, southWorld, northWorld;
    _locator->unitToModel(west,  westWorld);
    _locator->unitToModel(east,  eastWorld);
    _locator->unitToModel(south, southWorld);
    _locator->unitToModel(north, northWorld);

    output = (eastWorld-westWorld) ^ (northWorld-southWorld);
    output.normalize();

    return true;
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
_texture     ( rhs._texture.get() ),
_tileKey     ( rhs._tileKey ),
_fallbackData( rhs._fallbackData ),
_order       ( rhs._order )
{
    //nop
}

void
TileModel::ColorData::resizeGLObjectBuffers(unsigned maxSize)
{
    if ( _texture.valid() )
    {
        _texture->resizeGLObjectBuffers( maxSize );
    }
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
_mapInfo       ( rhs._mapInfo ),
_revision      ( rhs._revision ),
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
    model->_tileLocator = _tileLocator->createSameTypeForKey( childKey, _mapInfo );

    return model;
}

bool
TileModel::hasRealData() const
{
    for(ColorDataByUID::const_iterator i = _colorData.begin(); i != _colorData.end(); ++i )
        if ( !i->second.isFallbackData() )
            return true;

    if ( hasElevation() && !_elevationData.isFallbackData() )
        return true;

    return false;
}

void
TileModel::setParentTileModel(const TileModel* parent)
{
    _parentModel = parent;
}

void
TileModel::resizeGLObjectBuffers(unsigned maxSize)
{
    for(ColorDataByUID::iterator i = _colorData.begin(); i != _colorData.end(); ++i )
        i->second.resizeGLObjectBuffers( maxSize );
}

void
TileModel::releaseGLObjects(osg::State* state) const
{
    for(ColorDataByUID::const_iterator i = _colorData.begin(); i != _colorData.end(); ++i )
        i->second.releaseGLObjects( state );
}
