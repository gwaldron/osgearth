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
#include <osgEarthSymbology/StyleSheet>
#include <osgEarthSymbology/CssUtils>
#include <algorithm>

#define LC "[StyleSheet] "

using namespace osgEarth;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

StyleSheet::StyleSheet()
{
    //nop
}

StyleSheet::StyleSheet(const Config& conf)
{
    mergeConfig( conf );
}

void
StyleSheet::addStyle( const Style& style )
{
    _styles[ style.getName() ] = style;
}

void
StyleSheet::removeStyle( const std::string& name )
{
    _styles.erase( name );
}

Style*
StyleSheet::getStyle( const std::string& name, bool fallBackOnDefault )
{
    StyleMap::iterator i = _styles.find( name );
    if ( i != _styles.end() ) {
        return &i->second;
    }
    else if ( name.length() > 1 && name.at(0) == '#' ) {
        std::string nameWithoutHash = name.substr( 1 );
        return getStyle( nameWithoutHash, fallBackOnDefault );
    }
    else if ( fallBackOnDefault ) {
        return getDefaultStyle();
    }
    else {
        return 0L;
    }
}

const Style*
StyleSheet::getStyle( const std::string& name, bool fallBackOnDefault ) const
{
    StyleMap::const_iterator i = _styles.find( name );
    if ( i != _styles.end() ) {
        return &i->second;
    }
    else if ( name.length() > 1 && name.at(0) == '#' ) {
        std::string nameWithoutHash = name.substr( 1 );
        return getStyle( nameWithoutHash, fallBackOnDefault );
    }
    else if ( fallBackOnDefault ) {
        return getDefaultStyle();
    }
    else {
        return 0L;
    }
}

const StyleSelector*
StyleSheet::getSelector( const std::string& name ) const
{
    for(StyleSelectorList::const_iterator i = _selectors.begin(); i != _selectors.end(); ++i )
    {
        if ( i->name() == name )
        {
            return &(*i);
        }
    }
    return 0L;
}

Style*
StyleSheet::getDefaultStyle()
{
    if ( _styles.find( "default" ) != _styles.end() ) {
        return &_styles.find( "default" )->second;
    }
    else if ( _styles.find( "" ) != _styles.end() ) {
        return &_styles.find( "" )->second;
    }
    if ( _styles.size() > 0 ) {
        return &_styles.begin()->second;
    }
    else {
        // insert the empty style and return it.
        _styles["default"] = _emptyStyle;
        return &_styles.begin()->second;
    }
}

const Style*
StyleSheet::getDefaultStyle() const
{
    if ( _styles.size() == 1 ) {
        return &_styles.begin()->second;
    }
    else if ( _styles.find( "default" ) != _styles.end() ) {
        return &_styles.find( "default" )->second;
    }
    else if ( _styles.find( "" ) != _styles.end() ) {
        return &_styles.find( "" )->second;
    }
    else {
        return &_emptyStyle;
    }
}

void
StyleSheet::addResourceLibrary( ResourceLibrary* lib )
{
    Threading::ScopedWriteLock exclusive( _resLibsMutex );
    _resLibs[ lib->getName() ] = lib;
}

ResourceLibrary*
StyleSheet::getResourceLibrary( const std::string& name ) const
{
    Threading::ScopedReadLock shared( const_cast<StyleSheet*>(this)->_resLibsMutex );
    ResourceLibraries::const_iterator i = _resLibs.find( name );
    if ( i != _resLibs.end() )
        return i->second.get();
    else
        return 0L;
}

void StyleSheet::setScript( ScriptDef* script )
{
  _script = script;
}


Config
StyleSheet::getConfig() const
{
    Config conf;

    for( StyleSelectorList::const_iterator i = _selectors.begin(); i != _selectors.end(); ++i )
    {
        conf.add( "selector", i->getConfig() );
    }

    for( StyleMap::const_iterator i = _styles.begin(); i != _styles.end(); ++i )
    {
        conf.add( "style", i->second.getConfig() );
    }

    {
        Threading::ScopedReadLock shared( const_cast<StyleSheet*>(this)->_resLibsMutex );

        for( ResourceLibraries::const_iterator i = _resLibs.begin(); i != _resLibs.end(); ++i )
        {
            if ( i->second.valid() )
            {
                Config libConf = i->second->getConfig();
                conf.add( "library", libConf );
            }
        }
    }

    if ( _script.valid() )
    {
        Config scriptConf("script");

        if ( !_script->name.empty() )
            scriptConf.set( "name", _script->name );
        if ( !_script->language.empty() )
            scriptConf.set( "language", _script->language );
        if ( !_script->uri.isSet() )
            scriptConf.set( "url", _script->uri->base() );
        else if ( !_script->code.empty() )
            scriptConf.value() = _script->code;

        conf.add( scriptConf );
    }

    return conf;
}

void
StyleSheet::mergeConfig( const Config& conf )
{
    _uriContext = URIContext( conf.referrer() );

    // read in any resource library references
    ConfigSet libraries = conf.children( "library" );
    for( ConfigSet::iterator i = libraries.begin(); i != libraries.end(); ++i )
    {
        ResourceLibrary* resLib = new ResourceLibrary( *i );
        _resLibs[resLib->getName()] = resLib;
    }

    // read in any scripts
    ConfigSet scripts = conf.children( "script" );
    for( ConfigSet::iterator i = scripts.begin(); i != scripts.end(); ++i )
    {
        _script = new ScriptDef();

        // load the code from a URI if there is one:
        if ( i->hasValue("url") )
        {
            _script->uri = URI( i->value("url"), _uriContext );
            OE_INFO << LC << "Loading script from \"" << _script->uri->full() << std::endl;
            _script->code = _script->uri->getString();
        }
        else
        {
            _script->code = i->value();
        }

        // name is optional and unused at the moment
        _script->name = i->value("name");

        std::string lang = i->value("language");
        _script->language = lang.empty() ? "javascript" : lang;
    }

    // read any style class definitions. either "class" or "selector" is allowed
    ConfigSet selectors = conf.children( "selector" );
    if ( selectors.empty() ) selectors = conf.children( "class" );
    for( ConfigSet::iterator i = selectors.begin(); i != selectors.end(); ++i )
    {
        _selectors.push_back( StyleSelector( *i ) );
    }

    // read in the actual styles
    ConfigSet styles = conf.children( "style" );
    for( ConfigSet::iterator i = styles.begin(); i != styles.end(); ++i )
    {
        const Config& styleConf = *i;

        if ( styleConf.value("type") == "text/css" )
        {
            // for CSS data, there may be multiple styles in one CSS block. So
            // parse them all out and add them to the stylesheet.

            // read the inline data:
            std::string cssString = styleConf.value();

            // if there's a URL, read the CSS from the URL:
            if ( styleConf.hasValue("url") )
            {
                URI uri( styleConf.value("url"), styleConf.referrer() );
                cssString = uri.readString().getString();
            }

            // break up the CSS into multiple CSS blocks and parse each one individually.
            std::vector<std::string> blocks;
            CssUtils::split( cssString, blocks );

            for( std::vector<std::string>::iterator i = blocks.begin(); i != blocks.end(); ++i )
            {
                Config blockConf( styleConf );
                blockConf.value() = *i;
                //OE_INFO << LC << "Style block = " << blockConf.toJSON() << std::endl;
                Style style( blockConf );
                _styles[ style.getName() ] = style;
            }
        }
        else
        {
            Style style( styleConf );
            _styles[ style.getName() ] = style;
        }
    }
}
