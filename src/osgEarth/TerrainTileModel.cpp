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
#include <osgEarth/TerrainTileModel>
#include <osgEarth/ImageLayer>
#include <osgEarth/ImageUtils>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/Registry>
#include <osg/Texture2D>

using namespace osgEarth;

//...........................................................................

#undef  LC
#define LC "[TerrainTileLayerModel] "

TerrainTileLayerModel::TerrainTileLayerModel(TerrainTileModel::LayerType type) :
_type( type )
{
    //NOP
}
TerrainTileLayerModel::TerrainTileLayerModel(const std::string&          name,
                                             TerrainTileModel::LayerType type) :
_name( name ),
_type( type )
{
    //NOP
}

#if 0
void
TerrainTileLayerModel::generateTextureAndMatrix(TerrainTileModelStore* modelStore)
{
    osg::Image* image = getImage();

    // if image exists, generate a texture for it and leave the matrix empty
    // since it's identity.
    if ( image )
    {
        switch(_type)
        {
        case TerrainTileModel::LAYER_IMAGE:
            {
                osg::Texture2D* tex = new osg::Texture2D( _image.get() );
                tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
                tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
                tex->setResizeNonPowerOfTwoHint(false);

                const ImageLayer* imageLayer = dynamic_cast<const ImageLayer*>(_layer.get());

                osg::Texture::FilterMode magFilter = 
                    imageLayer ? imageLayer->getImageLayerOptions().minFilter().get() : osg::Texture::LINEAR;
                osg::Texture::FilterMode minFilter =
                    imageLayer ? imageLayer->getImageLayerOptions().minFilter().get() : osg::Texture::LINEAR;

                tex->setFilter( osg::Texture::MAG_FILTER, magFilter );
                tex->setFilter( osg::Texture::MIN_FILTER, minFilter );
                tex->setMaxAnisotropy( 4.0f );

                // Disable mip mapping for npot tiles
                if (!ImageUtils::isPowerOfTwo( image ) || (!image->isMipmap() && ImageUtils::isCompressed(image)))
                {
                    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
                }    

                const optional<bool>& unRefPolicy = Registry::instance()->unRefImageDataAfterApply();
                if ( unRefPolicy.isSet() )
                    tex->setUnRefImageDataAfterApply( unRefPolicy.get() );

                setTexture( tex );
            }
            break;

        case TerrainTileModel::LAYER_ELEVATION:
            {
                osg::Texture2D* tex = new osg::Texture2D( _image.get() );
                tex->setInternalFormat(GL_LUMINANCE32F_ARB);
                tex->setSourceFormat(GL_LUMINANCE);
                tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
                tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
                tex->setWrap  ( osg::Texture::WRAP_S,     osg::Texture::CLAMP_TO_EDGE );
                tex->setWrap  ( osg::Texture::WRAP_T,     osg::Texture::CLAMP_TO_EDGE );
                tex->setResizeNonPowerOfTwoHint( false );
                tex->setMaxAnisotropy( 1.0f );

                setTexture( tex );
            }
            break;

        case TerrainTileModel::LAYER_NORMAL:
            {
                osg::Texture2D* tex = new osg::Texture2D( image );
                tex->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
                tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
                tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
                tex->setWrap  ( osg::Texture::WRAP_S,     osg::Texture::CLAMP_TO_EDGE );
                tex->setWrap  ( osg::Texture::WRAP_T,     osg::Texture::CLAMP_TO_EDGE );
                tex->setResizeNonPowerOfTwoHint( false );
                tex->setMaxAnisotropy( 1.0f );

                setTexture( tex );
            }
            break;

        default:
            {
                //NOP
            }
            break;
        }
    }

    // no image? try to find the parent model, point to its texture, 
    // and create a scale/bias matrix for it.
    else if ( modelStore )
    {
        //TODO
        OE_WARN << LC << "nyi\n";
    }
}
#endif

//...........................................................................

#undef  LC
#define LC "[TerrainTileModel] "

TerrainTileModel::TerrainTileModel(const TileKey&  key,
                                   const Revision& revision) :
_key     ( key ),
_revision( revision )
{
    //NOP
}


TerrainTileLayerModel*
TerrainTileModel::getLayer(TerrainTileModel::LayerType type)
{
    for(LayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        if ( i->get()->getType() == type )
        {
            return i->get();
        }
    }
    return 0L;
}

const TerrainTileLayerModel*
TerrainTileModel::getLayer(TerrainTileModel::LayerType type) const
{
    for(LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        if ( i->get()->getType() == type )
        {
            return i->get();
        }
    }
    return 0L;
}

TerrainTileLayerModel*
TerrainTileModel::getLayer(const std::string& name)
{
    for(LayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        if ( i->get()->getName() == name )
        {
            return i->get();
        }
    }
    return 0L;
}

const TerrainTileLayerModel*
TerrainTileModel::getLayer(const std::string& name) const
{
    for(LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        if ( i->get()->getName() == name )
        {
            return i->get();
        }
    }
    return 0L;
}

TerrainTileLayerModel*
TerrainTileModel::getLayer(const UID& uid)
{
    for(LayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        osg::ref_ptr<Layer> layer;
        if ( i->get()->lockLayer(layer) && layer->getUID() == uid )
        {
            return i->get();
        }
    }
    return 0L;
}

const TerrainTileLayerModel*
TerrainTileModel::getLayer(const UID& uid) const
{
    for(LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        osg::ref_ptr<Layer> layer;
        if ( i->get()->lockLayer(layer) && layer->getUID() == uid )
        {
            return i->get();
        }
    }
    return 0L;
}