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
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/CssUtils>
#include <osgEarthSymbology/SLD>
#include <osgEarth/HTTPClient>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Symbology;

Style::Style( const std::string& name ) :
_name( name )
{
    //NOP
}

Style::Style(const Style& rhs) :
_name    ( rhs._name ),
_symbols ( rhs._symbols ),
_origType( rhs._origType ),
_origData( rhs._origData ),
_url     ( rhs._url )
{
    //nop
}

void Style::addSymbol(Symbol* symbol)
{
    _symbols.push_back(symbol);
    //dirty();
}


Style::Style( const Config& conf )
{
    mergeConfig( conf );
}

Style
Style::combineWith( const Style& rhs ) const
{
    // start by deep-cloning this style.
    Config conf = getConfig( false );
    Style newStyle( conf );

    // next, merge in the symbology from the other style.
    newStyle.mergeConfig( rhs.getConfig(false) );
    //SLDReader::readStyleFromCSSParams( rhs.getConfig(false), newStyle );

    if ( !this->empty() && !rhs.empty() )
        newStyle.setName( _name + ":" + rhs.getName() );
    else if ( !this->empty() && rhs.empty() )
        newStyle.setName( _name );
    else if ( this->empty() && !rhs.empty() )
        newStyle.setName( rhs.getName() );
    else
        newStyle.setName( _name );

    return newStyle;
}

void
Style::mergeConfig( const Config& conf )
{
    if ( _name.empty() )
        _name = conf.value( "name" );

    // if there's no explicit name, use the KEY as the name.
    if ( _name.empty() )
        _name = conf.key();

    conf.getIfSet( "url", _url );
    _origType = conf.value( "type" );

    if ( conf.value( "type" ) == "text/css" )
    {
        _origData = conf.value();
    }
    else
    {
        Config symbolConf = conf.child( "symbols" );
        if ( !symbolConf.empty() )
        {
            for( ConfigSet::const_iterator i = symbolConf.children().begin(); i != symbolConf.children().end(); ++i )
            {
                const Config& c = *i;

                if ( c.key() == "text" )
                {
                    getOrCreateSymbol<TextSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "point" )
                {
                    getOrCreateSymbol<PointSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "line" )
                {
                    getOrCreateSymbol<LineSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "polygon" )
                {
                    getOrCreateSymbol<PolygonSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "extrusion" )
                {
                    getOrCreateSymbol<ExtrusionSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "altitude" )
                {
                    getOrCreateSymbol<AltitudeSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "marker" )
                {
                    getOrCreateSymbol<MarkerSymbol>()->mergeConfig( c );
                }
            }
        }
    }

//    dirty();
}

Config
Style::getConfig( bool keepOrigType ) const
{
    Config conf( "style" );
    conf.attr("name") = _name;
    conf.addIfSet( "url", _url );
    
    if ( _origType == "text/css" && keepOrigType )
    {
        conf.attr("type") = _origType;
        conf.value() = _origData;            
    }
    else
    {
        Config symbolsConf( "symbols" );
        for( SymbolList::const_iterator i = _symbols.begin(); i != _symbols.end(); ++i )
        {
            symbolsConf.addChild( i->get()->getConfig() );
        }
        conf.addChild( symbolsConf );
    }

    return conf;
}

/************************************************************************/

StyleSelector::StyleSelector( const Config& conf )
{
    mergeConfig( conf );
}

std::string
StyleSelector::getSelectedStyleName() const 
{
    return _styleName.isSet() ? *_styleName : _name;
}

void
StyleSelector::mergeConfig( const Config& conf )
{
    _name = conf.value( "name" );
    conf.getIfSet( "style", _styleName ); // backwards compatibility
    conf.getIfSet( "class", _styleName );
    conf.getObjIfSet( "query", _query );
}

Config
StyleSelector::getConfig() const
{
    Config conf( "selector" );
    conf.add( "name", _name );
    conf.addIfSet( "class", _styleName );
    conf.addObjIfSet( "query", _query );
    return conf;
}


/************************************************************************/

StyleSheet::StyleSheet( const Config& conf ) :
    Configurable()
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

bool
StyleSheet::getStyle( const std::string& name, Style& out_style, bool fallBackOnDefault ) const
{
    StyleMap::const_iterator i = _styles.find( name );
    if ( i != _styles.end() ) {
        out_style = i->second;
        return true;
    }
    else if ( fallBackOnDefault && name.empty() && _styles.size() == 1 )
    {
        out_style = _styles.begin()->second;
        return true;
    }
    else
    {
        return false;
    }
}

bool
StyleSheet::getDefaultStyle( Style& out_style ) const
{
    if ( _styles.size() == 1 ) {
        out_style = _styles.begin()->second;
        return true;
    }
    else if ( _styles.find( "default" ) != _styles.end() ) {
        out_style = _styles.find( "default" )->second;
        return true;
    }
    else if ( _styles.find( "" ) != _styles.end() ) {
        out_style = _styles.find( "" )->second;
        return true;
    }
    else {
        return false;
    }
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
    return conf;
}

void
StyleSheet::mergeConfig( const Config& conf )
{
    // first read any style class definitions. either "class" or "selector" is allowed
    ConfigSet selectors = conf.children( "selector" );
    if ( selectors.empty() ) selectors = conf.children( "class" );
    for( ConfigSet::iterator i = selectors.begin(); i != selectors.end(); ++i )
    {
        _selectors.push_back( StyleSelector( *i ) );
    }

    ConfigSet styles = conf.children( "style" );
    for( ConfigSet::iterator i = styles.begin(); i != styles.end(); ++i )
    {
        const Config& styleConf = *i;

        if ( styleConf.value("type") == "text/css" )
        {
            // read the inline data:
            std::string cssString = styleConf.value();

            // if there's a URL, read the CSS from the URL:
            if ( styleConf.hasValue("url") )
                HTTPClient::readString( styleConf.value("url"), cssString );

            // a CSS style definition can actually contain multiple styles. Read them
            // and create one style for each in the catalog.
            std::stringstream buf( cssString );
            Config css = CssUtils::readConfig( buf );
            //OE_NOTICE << css.toString() << std::endl;
            for(ConfigSet::const_iterator j = css.children().begin(); j != css.children().end(); ++j )
            {
                Style style( styleConf );
                //Style* style = new Style( styleConf );
                if ( SLDReader::readStyleFromCSSParams( *j, style ) )
                    _styles[ j->key() ] = style;
            }            
        }
        else
        {
            Style style( styleConf );
            _styles[ style.getName() ] = style;
        }
    }
}

