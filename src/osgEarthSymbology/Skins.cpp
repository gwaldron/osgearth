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
#include <osgEarthSymbology/Skins>
#include <osgEarthSymbology/Style>
#include <osgEarth/StringUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>

#include <osg/BlendFunc>
#include <osg/Texture2D>

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
_maxTexSpan       ( 1024 )
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

    return conf;
}

osg::StateSet*
SkinResource::createStateSet( const osgDB::Options* dbOptions ) const
{
    return createStateSet( createImage(dbOptions) );
}

osg::StateSet*
SkinResource::createStateSet( osg::Image* image ) const
{
    osg::StateSet* stateSet = 0L;
    if ( image )
    {
        stateSet = new osg::StateSet();

        osg::Texture* tex = new osg::Texture2D( image );
        tex->setResizeNonPowerOfTwoHint(false);
        tex->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
        tex->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
        stateSet->setTextureAttributeAndModes( 0, tex, osg::StateAttribute::ON );

        if ( _texEnvMode.isSet() )
        {
            osg::TexEnv* texenv = new osg::TexEnv();
            texenv = new osg::TexEnv();
            texenv->setMode( *_texEnvMode );
            stateSet->setTextureAttribute( 0, texenv, osg::StateAttribute::ON );
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
}
