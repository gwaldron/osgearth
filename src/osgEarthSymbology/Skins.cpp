/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osg/Texture2DArray>

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
_imageScaleT      ( 1.0f ),
_atlasHint        ( true )
{
    mergeConfig( conf );
}

void
SkinResource::mergeConfig( const Config& conf )
{
    conf.get( "url",                 _imageURI );
    conf.get( "image_width",         _imageWidth );
    conf.get( "image_height",        _imageHeight );
    conf.get( "min_object_height",   _minObjHeight );
    conf.get( "max_object_height",   _maxObjHeight );
    conf.get( "tiled",               _isTiled );
    conf.get( "max_texture_span",    _maxTexSpan );

    conf.get( "texture_mode", "decal",    _texEnvMode, osg::TexEnv::DECAL );
    conf.get( "texture_mode", "modulate", _texEnvMode, osg::TexEnv::MODULATE );
    conf.get( "texture_mode", "replace",  _texEnvMode, osg::TexEnv::REPLACE );
    conf.get( "texture_mode", "blend",    _texEnvMode, osg::TexEnv::BLEND );

    // texture atlas support
    conf.get( "image_bias_s",        _imageBiasS );
    conf.get( "image_bias_t",        _imageBiasT );
    conf.get( "image_layer",         _imageLayer );
    conf.get( "image_scale_s",       _imageScaleS );
    conf.get( "image_scale_t",       _imageScaleT );

    conf.get( "atlas", _atlasHint );
    conf.get( "read_options", _readOptions );
}

Config
SkinResource::getConfig() const
{
    Config conf = Resource::getConfig();
    conf.key() = "skin";

    conf.set( "url",                 _imageURI );
    conf.set( "image_width",         _imageWidth );
    conf.set( "image_height",        _imageHeight );
    conf.set( "min_object_height",   _minObjHeight );
    conf.set( "max_object_height",   _maxObjHeight );
    conf.set( "tiled",               _isTiled );
    conf.set( "max_texture_span",    _maxTexSpan );
    
    conf.set( "texture_mode", "decal",    _texEnvMode, osg::TexEnv::DECAL );
    conf.set( "texture_mode", "modulate", _texEnvMode, osg::TexEnv::MODULATE );
    conf.set( "texture_mode", "replace",  _texEnvMode, osg::TexEnv::REPLACE );
    conf.set( "texture_mode", "blend",    _texEnvMode, osg::TexEnv::BLEND );

    // texture atlas support
    conf.set( "image_bias_s",        _imageBiasS );
    conf.set( "image_bias_t",        _imageBiasT );
    conf.set( "image_layer",         _imageLayer );
    conf.set( "image_scale_s",       _imageScaleS );
    conf.set( "image_scale_t",       _imageScaleT );
    
    conf.set( "atlas", _atlasHint );
    conf.set( "read_options", _readOptions );

    return conf;
}

std::string
SkinResource::getUniqueID() const
{
    return imageURI()->full();
}

osg::Texture*
SkinResource::createTexture(const osgDB::Options* readOptions) const
{
    OE_DEBUG << LC << "Creating skin texture for " << imageURI()->full() << std::endl;
    osg::ref_ptr<osg::Image> image = createImage(readOptions);
    return createTexture(image.get());
}

osg::Texture*
SkinResource::createTexture(osg::Image* image) const
{
    if ( !image ) return 0L;

    osg::Texture* tex;

    if (image->r() > 1)
    {
        osg::Texture2DArray* ta = new osg::Texture2DArray();

        ta->setTextureDepth(image->r());
        ta->setTextureWidth(image->s());
        ta->setTextureHeight(image->t());
        ta->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
        tex = ta;

        std::vector<osg::ref_ptr<osg::Image> > layers;
        ImageUtils::flattenImage(image, layers);
        for (unsigned i = 0; i < layers.size(); ++i)
        {
            tex->setImage(i, layers[i].get());
        }
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    }
    else
    {
        tex = new osg::Texture2D(image);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    }

    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);

    // skin textures are likely to be shared, paged, etc. so keep them in memory.
    tex->setUnRefImageDataAfterApply(false);

    // don't resize them, let it be
    tex->setResizeNonPowerOfTwoHint(false);

    ImageUtils::activateMipMaps(tex);

    return tex;
}

osg::StateSet*
SkinResource::createStateSet(const osgDB::Options* readOptions) const
{
    OE_DEBUG << LC << "Creating skin state set for " << imageURI()->full() << std::endl;
    osg::ref_ptr<osg::Image> image = createImage(readOptions);
    return createStateSet(image.get());
}

osg::StateSet*
SkinResource::createStateSet( osg::Image* image ) const
{
    osg::StateSet* stateSet = 0L;
    if ( image )
    {
        stateSet = new osg::StateSet();
        
        osg::Texture* tex = createTexture(image);
        if ( tex )
        {
            stateSet->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);

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
    }

    return stateSet;
}

osg::ref_ptr<osg::Image>
SkinResource::createImage( const osgDB::Options* dbOptions ) const
{
    if (getStatus().isError())
        return 0L;

    ReadResult result;
    if (_readOptions.isSet())
    {
        osg::ref_ptr<osgDB::Options> ro = Registry::cloneOrCreateOptions(dbOptions);
        ro->setOptionString(Stringify() << _readOptions.get() << " " << ro->getOptionString());
        result = _imageURI->readImage(ro.get());
    }
    else
    {
        result = _imageURI->readImage(dbOptions);
    }

    if (result.failed())
    {
        Threading::ScopedMutexLock lock(_mutex);
        if (_status.isOK())
            _status = Status::Error(Status::ServiceUnavailable, "Failed to load resource image\n");
    }
    return result.releaseImage();
}

//---------------------------------------------------------------------------

OSGEARTH_REGISTER_SIMPLE_SYMBOL(skin, SkinSymbol);

SkinSymbol::SkinSymbol(const SkinSymbol& rhs,const osg::CopyOp& copyop):
Taggable<Symbol>(rhs, copyop),
_library(rhs._library),
_objHeight(rhs._objHeight),
_minObjHeight(rhs._minObjHeight),
_maxObjHeight(rhs._maxObjHeight),
_isTiled(rhs._isTiled),
_randomSeed(rhs._randomSeed),
_name(rhs._name)
{
}

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
    conf.get( "library",             _library );
    conf.get( "object_height",       _objHeight );
    conf.get( "min_object_height",   _minObjHeight );
    conf.get( "max_object_height",   _maxObjHeight );
    conf.get( "tiled",               _isTiled );
    conf.get( "random_seed",         _randomSeed );
    conf.get( "name",                _name );

    addTags( conf.value("tags" ) );
}

Config 
SkinSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "skin";

    conf.set( "library",             _library );
    conf.set( "object_height",       _objHeight );
    conf.set( "min_object_height",   _minObjHeight );
    conf.set( "max_object_height",   _maxObjHeight );
    conf.set( "tiled",               _isTiled );
    conf.set( "random_seed",         _randomSeed );
    conf.set( "name",                _name );

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
            style.getOrCreate<SkinSymbol>()->library() = c.value();
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
