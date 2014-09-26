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
#include "TileModel"
#include <osgEarth/MapInfo>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osg/Texture2D>
#include <osg/Texture2DArray>
#include <osgTerrain/Locator>

using namespace osgEarth::Drivers::MPTerrainEngine;
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

    float centerHeight = HeightFieldUtils::getHeightAtNormalizedLocation(_hf.get(), hf_ndc.x(), hf_ndc.y(), interp);

    osg::Vec3d west ( hf_ndc.x()-xres, hf_ndc.y(), 0.0 );
    osg::Vec3d east ( hf_ndc.x()+xres, hf_ndc.y(), 0.0 );
    osg::Vec3d south( hf_ndc.x(), hf_ndc.y()-yres, 0.0 );
    osg::Vec3d north( hf_ndc.x(), hf_ndc.y()+yres, 0.0 );

    if (!HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, west.x(),  west.y(),  west.z(), interp))
        west.z() = centerHeight;
    if (!HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, east.x(),  east.y(),  east.z(), interp))
        east.z() = centerHeight;
    if (!HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, south.x(), south.y(), south.z(), interp))
        south.z() = centerHeight;
    if (!HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, north.x(), north.y(), north.z(), interp))
        north.z() = centerHeight;

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
                                bool                        fallbackData) :
_layer       ( layer ),
_order       ( order ),
_locator     ( locator ),
_fallbackData( fallbackData )
{
    osg::Texture::FilterMode minFilter = layer->getImageLayerOptions().minFilter().get();
    osg::Texture::FilterMode magFilter = layer->getImageLayerOptions().magFilter().get();

    if (image->r() <= 1)
    {
        _texture = new osg::Texture2D( image );
    }
    else // image->r() > 1
    {
        // If the image has a third dimension, split it into separate images
        // and stick them into a texture array.
        std::vector< osg::ref_ptr<osg::Image> > images;
        ImageUtils::flattenImage(image, images);

        osg::Texture2DArray* tex = new osg::Texture2DArray();
        tex->setTextureDepth(images.size());
        tex->setInternalFormat(images[0]->getInternalTextureFormat());
        tex->setSourceFormat(images[0]->getPixelFormat());
        for (int i = 0; i < (int) images.size(); ++i)
            tex->setImage( i, images[i].get() );

        _texture = tex;
    }


    const optional<bool>& unRefPolicy = Registry::instance()->unRefImageDataAfterApply();
    if ( unRefPolicy.isSet() )
        _texture->setUnRefImageDataAfterApply( unRefPolicy.get() );

    _texture->setMaxAnisotropy( 4.0f );
    _texture->setResizeNonPowerOfTwoHint(false);
    _texture->setFilter( osg::Texture::MAG_FILTER, magFilter );
    _texture->setFilter( osg::Texture::MIN_FILTER, minFilter  );
    _texture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    _texture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );

    layer->applyTextureCompressionMode( _texture.get() );

    // Disable mip mapping for npot tiles
    if (!ImageUtils::isPowerOfTwo( image ) || (!image->isMipmap() && ImageUtils::isCompressed(image)))
    {
        OE_DEBUG<<"Disabling mipmapping for non power of two tile size("<<image->s()<<", "<<image->t()<<")"<<std::endl;
        _texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    }    

    _hasAlpha = image && ImageUtils::hasTransparency(image);
}

TileModel::ColorData::ColorData(const TileModel::ColorData& rhs) :
_layer       ( rhs._layer.get() ),
_locator     ( rhs._locator.get() ),
_texture     ( rhs._texture.get() ),
_fallbackData( rhs._fallbackData ),
_order       ( rhs._order ),
_hasAlpha    ( rhs._hasAlpha )
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
_mapInfo         ( rhs._mapInfo ),
_revision        ( rhs._revision ),
_tileKey         ( rhs._tileKey ),
_tileLocator     ( rhs._tileLocator.get() ),
_colorData       ( rhs._colorData ),
_elevationData   ( rhs._elevationData ),
_sampleRatio     ( rhs._sampleRatio ),
_parentStateSet  ( rhs._parentStateSet )
{
    //nop
}

bool
TileModel::requiresUpdateTraverse() const
{
    for(ColorDataByUID::const_iterator i = _colorData.begin(); i != _colorData.end(); ++i )
    {
        if ( i->second.getMapLayer()->isDynamic() )
            return true;
    }
    return false;
}

void
TileModel::updateTraverse(osg::NodeVisitor& nv) const
{
    // Supports updatable images (ImageStream, etc.), since the built-in
    // mechanism for doing so requires the Texture/Image to be in a StateSet
    // in the scene graph, and we don't keep it there.
    for(ColorDataByUID::const_iterator i = _colorData.begin(); i != _colorData.end(); ++i )
    {
        if ( i->second.getMapLayer()->isDynamic() )
        {
            osg::Texture* tex = i->second.getTexture();
            if ( tex )
            {
                for(int r=0; r<(int)tex->getNumImages(); ++r )
                {
                    osg::Image* image = tex->getImage(r);
                    if ( image && image->requiresUpdateCall() )
                    {
                        image->update(&nv);
                    }
                }
            }
        }
    }
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
