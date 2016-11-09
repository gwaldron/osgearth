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
#include "TileModel"
#include <osgEarth/MapInfo>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/ImageToHeightFieldConverter>
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

    if (!HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, west.x(),  west.y(),  (float&)west.z(), interp))
        west.z() = centerHeight;
    if (!HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, east.x(),  east.y(),  (float&)east.z(), interp))
        east.z() = centerHeight;
    if (!HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, south.x(), south.y(), (float&)south.z(), interp))
        south.z() = centerHeight;
    if (!HeightFieldUtils::getHeightAtNormalizedLocation(_neighbors, north.x(), north.y(), (float&)north.z(), interp))
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


TileModel::NormalData::NormalData(osg::HeightField* hf,
                                  GeoLocator*       locator,
                                  bool              fallbackData) :
_hf          ( hf ),
_locator     ( locator ),
_fallbackData( fallbackData ),
_unit        ( -1 )
{
    _neighbors._center = hf;
}

TileModel::NormalData::NormalData(const TileModel::NormalData& rhs) :
_hf          ( rhs._hf.get() ),
_locator     ( rhs._locator.get() ),
_fallbackData( rhs._fallbackData ),
_parent      ( rhs._parent ),
_unit        ( rhs._unit )
{
    _neighbors._center = rhs._neighbors._center.get();
    for(unsigned i=0; i<8; ++i)
        _neighbors._neighbors[i] = rhs._neighbors._neighbors[i];
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
        // won't work, needs to be pre-compositing
        //if ( layer->isCoverage() )
        //{
        //    // for coverage data, quantize the alpha values to some arbitrary level for now
        //    ImageUtils::PixelReader read(image);
        //    ImageUtils::PixelWriter write(image);
        //    for(int s=0; s<image->s(); ++s) {
        //        for(int t=0; t<image->t(); ++t) {
        //            osg::Vec4f rgba = read(s,t);
        //            rgba.a() = rgba.a() < 0.2 ? 0.0 : 1.0;
        //            write(rgba, s, t);
        //        }
        //    }
        //}

        //if ( layer->isCoverage() )
        //{
        //    osg::ref_ptr<osg::Image> imageWithMM = ImageUtils::buildNearestNeighborMipmaps(image);
        //    _texture = new osg::Texture2D( imageWithMM.get() );
        //}
        //else
        {
            _texture = new osg::Texture2D( image );
        }
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

    // First check the unref globel policy:
    const optional<bool>& unRefPolicy = Registry::instance()->unRefImageDataAfterApply();
    if ( unRefPolicy.isSet() )
        _texture->setUnRefImageDataAfterApply( unRefPolicy.get() );

    // dynamic layer? Need to keep it around
    if ( layer->isDynamic() )
        _texture->setUnRefImageDataAfterApply( false );

    _texture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    _texture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    _texture->setResizeNonPowerOfTwoHint(false);

    if ( layer->isCoverage() )
    {
        // coverages: no filtering or compression allowed.
        _texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
        _texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
        _texture->setMaxAnisotropy( 1.0f );
    }
    else
    {
        _texture->setMaxAnisotropy( 4.0f );
        _texture->setFilter( osg::Texture::MAG_FILTER, magFilter );
        _texture->setFilter( osg::Texture::MIN_FILTER, minFilter  );

        // Disable mip mapping for npot tiles
        if (!ImageUtils::isPowerOfTwo( image ) || (!image->isMipmap() && ImageUtils::isCompressed(image)))
        {
            OE_DEBUG<<"Disabling mipmapping for non power of two tile size("<<image->s()<<", "<<image->t()<<")"<<std::endl;
            _texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
        }    
    }

    _hasAlpha = ImageUtils::hasTransparency(image);

    layer->applyTextureCompressionMode( _texture.get() );    
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

//------------------------------------------------------------------

TileModel::TileModel(const TileModel& rhs) :
_mapInfo         ( rhs._mapInfo ),
_revision        ( rhs._revision ),
_tileKey         ( rhs._tileKey ),
_tileLocator     ( rhs._tileLocator.get() ),
_colorData       ( rhs._colorData ),
_elevationData   ( rhs._elevationData ),
_parentStateSet  ( rhs._parentStateSet ),
_useParentData   ( rhs._useParentData )
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
TileModel::generateElevationTexture()
{
    osg::Image* image = 0L;
    osg::HeightField* hf = _elevationData.getHeightField();
    if ( hf )
    {
        ImageToHeightFieldConverter conv;
        image = conv.convert( hf, 32 );
    }
    else
    {
        // no heightfield; create one and initialize it to zero.
        image = new osg::Image();
        image->allocateImage(32, 32, 1, GL_LUMINANCE, GL_FLOAT);
        ImageUtils::PixelWriter write(image);
        for(int s=0; s<image->s(); ++s)
            for(int t=0; t<image->t(); ++t)
                write(osg::Vec4(0,0,0,0), s, t);        
    }

    _elevationTexture = new osg::Texture2D( image );

    _elevationTexture->setInternalFormat(GL_LUMINANCE32F_ARB);
    _elevationTexture->setSourceFormat(GL_LUMINANCE);
    _elevationTexture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    _elevationTexture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    _elevationTexture->setWrap  ( osg::Texture::WRAP_S,     osg::Texture::CLAMP_TO_EDGE );
    _elevationTexture->setWrap  ( osg::Texture::WRAP_T,     osg::Texture::CLAMP_TO_EDGE );
    _elevationTexture->setResizeNonPowerOfTwoHint( false );
    _elevationTexture->setMaxAnisotropy( 1.0f );
}

void
TileModel::generateNormalTexture()
{
    osg::Image* image = HeightFieldUtils::convertToNormalMap(
        _normalData.getNeighborhood(),
        _tileKey.getProfile()->getSRS() );

    _normalTexture = new osg::Texture2D( image );

    _normalTexture->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
    _normalTexture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    //_normalTexture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    _normalTexture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    _normalTexture->setWrap  ( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    _normalTexture->setWrap  ( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    _normalTexture->setResizeNonPowerOfTwoHint( false );
    _normalTexture->setMaxAnisotropy( 1.0f );

    // So the engine can automatically normalize across tiles.
    _normalTexture->setUnRefImageDataAfterApply( false );
}


void
TileModel::resizeGLObjectBuffers(unsigned maxSize)
{
    for(ColorDataByUID::iterator i = _colorData.begin(); i != _colorData.end(); ++i )
    {
        if (i->second.getTexture())
            i->second.getTexture()->resizeGLObjectBuffers(maxSize);
    }    
}

void
TileModel::releaseGLObjects(osg::State* state) const
{
    unsigned count=0;

    for (ColorDataByUID::const_iterator i = _colorData.begin(); i != _colorData.end(); ++i)
    {
        if (i->second.getTexture() && i->second.getTexture()->referenceCount() == 1)
            i->second.getTexture()->releaseGLObjects(state), ++count;
    }

    if (_normalTexture.valid() && _normalTexture->referenceCount() == 1)
        _normalTexture->releaseGLObjects(state), ++count;

    if (_elevationTexture.valid() && _elevationTexture->referenceCount() == 1)
        _elevationTexture->releaseGLObjects(state), ++count;

}
