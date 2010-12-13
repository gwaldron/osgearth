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
#include <osgEarthSymbology/GeometrySymbol>
#include <osgEarthSymbology/Text>
#include <osgEarth/HTTPClient>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Symbology;

Style::Style()
{
    //NOP
}

Style::Style(const Style&, const osg::CopyOp&)
{
    // NOP
}

void Style::addSymbol(Symbol* symbol)
{
    _symbols.push_back(symbol);
    dirty();
}


Style::Style( const Config& conf )
{
    mergeConfig( conf );
}

void
Style::addSubStyle( Style* style )
{
    _subStyles[style->getName()] = style;
}

const Style*
Style::getSubStyle( const std::string& name ) const
{
    StylesByName::const_iterator i = _subStyles.find( name );
    return i != _subStyles.end() ? i->second.get() : 0L;
}

Style*
Style::getSubStyle( const std::string& name )
{
    StylesByName::iterator i = _subStyles.find( name );
    return i != _subStyles.end() ? i->second.get() : 0L;
}

void
Style::mergeConfig( const Config& conf )
{
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
                    _symbols.push_back( new TextSymbol(c) );
                else if ( c.key() == "point" )
                    _symbols.push_back( new PointSymbol(c) );
                else if ( c.key() == "line" )
                    _symbols.push_back( new LineSymbol(c) );
                else if ( c.key() == "polygon" )
                    _symbols.push_back( new PolygonSymbol(c) );
            }
        }
    }

    const ConfigSet& children = conf.children( "style" );
    for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
    {
        addSubStyle( new Style( *i ) );
    }

    dirty();
}

Config
Style::getConfig() const
{
    Config conf( "style" );
    conf.attr("name") = _name;
    conf.addIfSet( "url", _url );
    
    if ( _origType == "text/css" )
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
    conf.getIfSet( "style", _styleName );
    conf.getObjIfSet( "query", _query );
}

Config
StyleSelector::getConfig() const
{
    Config conf( "selector" );
    conf.add( "name", _name );
    conf.addIfSet( "style", _styleName );
    conf.addObjIfSet( "query", _query );
    return conf;
}


/************************************************************************/

StyleCatalog::StyleCatalog( const Config& conf ) :
    Configurable(),
    _emptyStyle(new Style)
{
    mergeConfig( conf );
}

void
StyleCatalog::addStyle( Style* style )
{
    _styles[ style->getName() ] = style;
}

void
StyleCatalog::removeStyle( const std::string& name )
{
    _styles.erase( name );
}

bool
StyleCatalog::getStyle( const std::string& name, Style*& output ) const
{
    StyleMap::const_iterator i = _styles.find( name );
    if ( i != _styles.end() ) {
        output = i->second;
        return true;
    }
    else
        return false;
}

const Style*
StyleCatalog::getDefaultStyle() const
{
    if ( _styles.size() == 1 )
        return _styles.begin()->second;
    else if ( _styles.find( "default" ) != _styles.end() )
        return _styles.find( "default" )->second;
    else if ( _styles.find( "" ) != _styles.end() )
        return _styles.find( "" )->second;
    else
        return _emptyStyle;
}

Config
StyleCatalog::getConfig() const
{
    Config conf;
    for( StyleSelectorList::const_iterator i = _selectors.begin(); i != _selectors.end(); ++i )
    {
        conf.add( "selector", i->getConfig() );
    }
    for( StyleMap::const_iterator i = _styles.begin(); i != _styles.end(); ++i )
    {
        conf.add( "style", i->second->getConfig() );
    }
    return conf;
}

void
StyleCatalog::mergeConfig( const Config& conf )
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
                Style* style = new Style( styleConf );
                if ( SLDReader::readStyleFromCSSParams( *j, *style ) )
                    _styles[ j->key() ] = style;
            }            
        }
        else
        {
            Style* style = new Style( styleConf );
            _styles[ style->getName() ] = style;
        }
    }
}

