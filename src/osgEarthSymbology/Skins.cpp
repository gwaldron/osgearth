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
#include <osgEarthSymbology/Skins>
#include <osgEarthSymbology/Style>
#include <osgEarth/StringUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>

#include <osg/BlendFunc>
#include <osg/Texture2D>

#define LC "[SkinResource] "

using namespace osgEarth;
using namespace osgEarth::Symbology;

//---------------------------------------------------------------------------

SkinResource::SkinResource( const Config& conf ) :
Resource          ( conf ),
_imageWidth       ( 10.0f ),
_imageHeight      ( 3.0f ),
_minObjHeight     ( 0.0f ),
_maxObjHeight     ( FLT_MAX ),
_isTiled          ( false ),
_texEnvMode       ( osg::TexEnv::MODULATE ),
_maxTexSpan       ( 1024 ),
_imageBiasS       ( 0.0f ),
_imageBiasT       ( 0.0f ),
_imageLayer       ( 0 ),
_imageScaleS      ( 1.0f ),
_imageScaleT      ( 1.0f )
{
    mergeConfig( conf );
}

void
SkinResource::mergeConfig( const Config& conf )
{
    conf.getIfSet( "url",                 _imageURI );
    conf.getIfSet( "image_width",         _imageWidth );
    conf.getIfSet( "image_height",        _imageHeight );
    conf.getIfSet( "min_object_height",   _minObjHeight );
    conf.getIfSet( "max_object_height",   _maxObjHeight );
    conf.getIfSet( "tiled",               _isTiled );
    conf.getIfSet( "max_texture_span",    _maxTexSpan );

    conf.getIfSet( "texture_mode", "decal",    _texEnvMode, osg::TexEnv::DECAL );
    conf.getIfSet( "texture_mode", "modulate", _texEnvMode, osg::TexEnv::MODULATE );
    conf.getIfSet( "texture_mode", "replace",  _texEnvMode, osg::TexEnv::REPLACE );
    conf.getIfSet( "texture_mode", "blend",    _texEnvMode, osg::TexEnv::BLEND );

    // texture atlas support
    conf.getIfSet( "image_bias_s",        _imageBiasS );
    conf.getIfSet( "image_bias_t",        _imageBiasT );
    conf.getIfSet( "image_layer",         _imageLayer );
    conf.getIfSet( "image_scale_s",       _imageScaleS );
    conf.getIfSet( "image_scale_t",       _imageScaleT );
}

Config
SkinResource::getConfig() const
{
    Config conf = Resource::getConfig();
    conf.key() = "skin";

    conf.updateIfSet( "url",                 _imageURI );
    conf.updateIfSet( "image_width",         _imageWidth );
    conf.updateIfSet( "image_height",        _imageHeight );
    conf.updateIfSet( "min_object_height",   _minObjHeight );
    conf.updateIfSet( "max_object_height",   _maxObjHeight );
    conf.updateIfSet( "tiled",               _isTiled );
    conf.updateIfSet( "max_texture_span",    _maxTexSpan );
    
    conf.updateIfSet( "texture_mode", "decal",    _texEnvMode, osg::TexEnv::DECAL );
    conf.updateIfSet( "texture_mode", "modulate", _texEnvMode, osg::TexEnv::MODULATE );
    conf.updateIfSet( "texture_mode", "replace",  _texEnvMode, osg::TexEnv::REPLACE );
    conf.updateIfSet( "texture_mode", "blend",    _texEnvMode, osg::TexEnv::BLEND );

    // texture atlas support
    conf.updateIfSet( "image_bias_s",        _imageBiasS );
    conf.updateIfSet( "image_bias_t",        _imageBiasT );
    conf.updateIfSet( "image_layer",         _imageLayer );
    conf.updateIfSet( "image_scale_s",       _imageScaleS );
    conf.updateIfSet( "image_scale_t",       _imageScaleT );

    return conf;
}

std::string
SkinResource::getUniqueID() const
{
    return imageURI()->full();
}

osg::StateSet*
SkinResource::createStateSet( const osgDB::Options* dbOptions ) const
{
    OE_DEBUG << LC << "Creating skin state set for " << imageURI()->full() << std::endl;
    return createStateSet( createImage(dbOptions) );
}

osg::StateSet*
SkinResource::createStateSet( osg::Image* image ) const
{
    osg::StateSet* stateSet = 0L;
    if ( image )
    {
        stateSet = new osg::StateSet();
        
        osg::Texture* tex;

        if ( image->r() > 1 )
        {
            osg::Texture2DArray* ta = new osg::Texture2DArray();
            
            ta->setTextureDepth( image->r() );
            ta->setTextureWidth( image->s() );
            ta->setTextureHeight( image->t() );
            ta->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
            tex = ta;

            std::vector<osg::ref_ptr<osg::Image> > layers;
            ImageUtils::flattenImage(image, layers);
            for(unsigned i=0; i<layers.size(); ++i)
            {
                tex->setImage(i, layers[i].get());
            }
            tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
            tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
            stateSet->setTextureAttribute( 0, tex, osg::StateAttribute::ON );
        }
        else
        {
            tex = new osg::Texture2D( image );
            tex->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
            tex->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );     
            stateSet->setTextureAttributeAndModes( 0, tex, osg::StateAttribute::ON );
        }
        
        tex->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR_MIPMAP_LINEAR);
        tex->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);

        tex->setUnRefImageDataAfterApply(true);
        tex->setResizeNonPowerOfTwoHint(false);

        if ( _texEnvMode.isSet() )
        {
            osg::TexEnv* texenv = new osg::TexEnv();
            texenv = new osg::TexEnv();
            texenv->setMode( *_texEnvMode );
            stateSet->setTextureAttributeAndModes( 0, texenv, osg::StateAttribute::ON );
        }

        if ( ImageUtils::hasAlphaChannel( image ) )
        {
            osg::BlendFunc* blendFunc = new osg::BlendFunc();
            blendFunc->setFunction( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
            stateSet->setAttributeAndModes( blendFunc, osg::StateAttribute::ON );
            stateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
        }
    }

    return stateSet;
}

osg::Image*
SkinResource::createImage( const osgDB::Options* dbOptions ) const
{
    return _imageURI->readImage(dbOptions).releaseImage();
}

//---------------------------------------------------------------------------

OSGEARTH_REGISTER_SIMPLE_SYMBOL(skin, SkinSymbol);

SkinSymbol::SkinSymbol( const Config& conf ) :
_objHeight    ( 0.0f ),
_minObjHeight ( 0.0f ),
_maxObjHeight ( FLT_MAX ),
_isTiled      ( false ),
_randomSeed   ( 0 )
{
    if ( !conf.empty() )
        mergeConfig( conf );
}

void 
SkinSymbol::mergeConfig( const Config& conf )
{
    conf.getIfSet( "library",             _libraryName );
    conf.getIfSet( "object_height",       _objHeight );
    conf.getIfSet( "min_object_height",   _minObjHeight );
    conf.getIfSet( "max_object_height",   _maxObjHeight );
    conf.getIfSet( "tiled",               _isTiled );
    conf.getIfSet( "random_seed",         _randomSeed );
    conf.getObjIfSet( "name",                _name );

    addTags( conf.value("tags" ) );
}

Config 
SkinSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "skin";

    conf.addIfSet( "library",             _libraryName );
    conf.addIfSet( "object_height",       _objHeight );
    conf.addIfSet( "min_object_height",   _minObjHeight );
    conf.addIfSet( "max_object_height",   _maxObjHeight );
    conf.addIfSet( "tiled",               _isTiled );
    conf.addIfSet( "random_seed",         _randomSeed );
    conf.addObjIfSet( "name",                _name );

    std::string tagstring = this->tagString();
    if ( !tagstring.empty() )
        conf.set("tags", tagstring);

    return conf;
}


void
SkinSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "skin-library") ) {
        if ( !c.value().empty() ) 
            style.getOrCreate<SkinSymbol>()->libraryName() = c.value();
    }
    else if ( match(c.key(), "skin-tags") ) {
        style.getOrCreate<SkinSymbol>()->addTags( c.value() );
    }
    else if ( match(c.key(), "skin-tiled") ) {
        style.getOrCreate<SkinSymbol>()->isTiled() = as<bool>( c.value(), false );
    }
    else if ( match(c.key(), "skin-object-height") ) {
        style.getOrCreate<SkinSymbol>()->objectHeight() = as<float>( c.value(), 0.0f );
    }
    else if (match(c.key(), "skin-min-object-height") ) {
        style.getOrCreate<SkinSymbol>()->minObjectHeight() = as<float>( c.value(), 0.0f );
    }
    else if (match(c.key(), "skin-max-object-height") ) {
        style.getOrCreate<SkinSymbol>()->maxObjectHeight() = as<float>( c.value(), 0.0f );
    }
    else if (match(c.key(), "skin-random-seed") ) {
        style.getOrCreate<SkinSymbol>()->randomSeed() = as<unsigned>( c.value(), 0u );
    }
    else if (match(c.key(), "skin-name")) {
        style.getOrCreate<SkinSymbol>()->name() = StringExpression(c.value());
    }
}

#if 0
//---------------------------------------------------------------------------
SkinTextureArray::SkinTextureArray()
{        
}

osg::Texture2DArray* SkinTextureArray::getTexture()
{
    return _texture.get();
}

int SkinTextureArray::getSkinIndex( const SkinResource* skin )
{
    LayerIndex::iterator itr = _layerIndex.find( skin->name());
    if (itr != _layerIndex.end())
    {
        return itr->second;
    }        
    return -1;
}

void SkinTextureArray::build(SkinResourceVector& skins, const osgDB::Options* dbOptions)
{        
    _texture = 0;
    _layerIndex.clear();

    unsigned int maxWidth = 0;
    unsigned int maxHeight = 0;

    std::vector< osg::ref_ptr< osg::Image > > images;

    for (unsigned int i = 0; i < skins.size(); i++)
    {
        osg::ref_ptr< osg::Image > image = skins[i]->createImage( dbOptions );
        if (image.valid())
        {
            if (maxWidth < image->s()) maxWidth = image->s();
            if (maxHeight < image->t()) maxHeight = image->t();            
            _layerIndex[ skins[i]->name() ] = i;
            images.push_back( image.get() );
        }
    }

    // Now resize all the images to the largest size.
    for (unsigned int i = 0; i < images.size(); i++)
    {
        osg::ref_ptr< osg::Image> resized;
        if (images[i]->s() != maxWidth || images[i]->t() != maxHeight)
        {            
            OE_DEBUG << "resizing image to " << maxWidth << "x" << maxHeight << std::endl;
            ImageUtils::resizeImage( images[i].get(), maxWidth, maxHeight, resized, 0, true);
        }        
        else
        {
             resized = images[i].get();
        }
        resized = ImageUtils::convertToRGBA8( resized.get() );
        images[i] = resized.get();
    }

    osg::Texture2DArray* texture = new osg::Texture2DArray();
    texture->setTextureDepth( images.size() );
    texture->setTextureWidth( maxWidth );
    texture->setTextureHeight( maxHeight );
    texture->setSourceFormat( GL_RGBA );
    texture->setInternalFormat( GL_RGBA8 );

    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::REPEAT);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::REPEAT);
    texture->setResizeNonPowerOfTwoHint(false);
    for (unsigned int i = 0; i < images.size(); i++)
    {
        texture->setImage(i, images[i].get() );
    }
    _texture = texture;
}
#endif
