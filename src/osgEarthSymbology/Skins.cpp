/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/StringUtils>

using namespace osgEarth;
using namespace osgEarth::Symbology;

//---------------------------------------------------------------------------

SkinResource::SkinResource( const Config& conf ) :
Resource          ( conf ),
_imageWidth       ( 10.0f ),
_imageHeight      ( 3.0f ),
_minObjHeight     ( 0.0f ),
_maxObjHeight     ( FLT_MAX ),
_repeatsVertically( false ),
_texEnvMode       ( osg::TexEnv::MODULATE ),
_maxTexSpan       ( 1024 )
{
    mergeConfig( conf );
}

void
SkinResource::mergeConfig( const Config& conf )
{
    _imageURL = conf.value( "url" );

    conf.getIfSet( "image_width",         _imageWidth );
    conf.getIfSet( "image_height",        _imageHeight );
    conf.getIfSet( "min_object_height",   _minObjHeight );
    conf.getIfSet( "max_object_height",   _maxObjHeight );
    conf.getIfSet( "repeats_vertically",  _repeatsVertically );
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

    conf.add( "url", _imageURL );

    conf.updateIfSet( "image_width",         _imageWidth );
    conf.updateIfSet( "image_height",        _imageHeight );
    conf.updateIfSet( "min_object_height",   _minObjHeight );
    conf.updateIfSet( "max_object_height",   _maxObjHeight );
    conf.updateIfSet( "repeats_vertically",  _repeatsVertically );
    conf.updateIfSet( "max_texture_span",    _maxTexSpan );

    conf.updateIfSet( "texture_mode", "decal",    _texEnvMode, osg::TexEnv::DECAL );
    conf.updateIfSet( "texture_mode", "modulate", _texEnvMode, osg::TexEnv::MODULATE );
    conf.updateIfSet( "texture_mode", "replace",  _texEnvMode, osg::TexEnv::REPLACE );
    conf.updateIfSet( "texture_mode", "blend",    _texEnvMode, osg::TexEnv::BLEND );

    return conf;
}

//---------------------------------------------------------------------------

SkinSymbol::SkinSymbol( const Config& conf ) :
_objHeight         ( 0.0f ),
_minObjHeight      ( 0.0f ),
_maxObjHeight      ( FLT_MAX ),
_repeatsVertically ( false )
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
    conf.getIfSet( "repeats_vertically",  _repeatsVertically );

    TagVector tags;
    StringTokenizer( conf.value("tags"), tags, " ", "\'", false, true );
    addTags( tags );
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
    conf.addIfSet( "repeats_vertically",  _repeatsVertically );

    std::string tagString = Taggable::tagString(_tags);
    if ( !tagString.empty() )
        conf.attr("tags") = tagString;

    return conf;
}
