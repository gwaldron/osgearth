/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

//------------------------------------------------------------------------

#undef  LC
#define LC "[Style] "

Style::Style( const std::string& name ) :
_name( name )
{
    //NOP
}

Style::Style(const Style& rhs, const osg::CopyOp& op) :
_name    ( rhs._name ),
_symbols ( rhs._symbols ),
_origType( rhs._origType ),
_origData( rhs._origData ),
_uri     ( rhs._uri )
{
    if ( op.getCopyFlags() == osg::CopyOp::SHALLOW_COPY )
    {
        _symbols = rhs._symbols;
    }
    else
    {
        _symbols.clear();
        mergeConfig( rhs.getConfig(false) );
    }
}

Style&
Style::operator = ( const Style& rhs )
{
    _name.clear();
    _origType.clear();
    _origData.clear();
    _uri.unset();
    _symbols.clear();
    mergeConfig( rhs.getConfig(false) );
    return *this;
}

void Style::addSymbol(Symbol* symbol)
{
    if ( symbol )
        _symbols.push_back(symbol);
}

bool Style::removeSymbol(Symbol* symbol)
{
    SymbolList::iterator it = std::find(_symbols.begin(), _symbols.end(), symbol);
	if (it == _symbols.end())
		return false;
		
	_symbols.erase(it);
	
	return true;
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

    if ( !this->empty() && !rhs.empty() )
        newStyle.setName( _name + std::string(":") + rhs.getName() );
    else if ( !this->empty() && rhs.empty() )
        newStyle.setName( _name );
    else if ( this->empty() && !rhs.empty() )
        newStyle.setName( rhs.getName() );
    else
        newStyle.setName( _name );

    return newStyle;
}

void
Style::fromCSS( const std::string& css )
{
    SLDReader::readStyleFromCSSParams( css, *this );
}

void
Style::mergeConfig( const Config& conf )
{
    if ( _name.empty() )
        _name = conf.value( "name" );

    // if there's no explicit name, use the KEY as the name.
    if ( _name.empty() )
        _name = conf.key();

    conf.getIfSet( "url", _uri ); // named "url" for back compat

    _origType = conf.value( "type" );

    if ( conf.value( "type" ) == "text/css" )
    {
        _origData = conf.value();
        
        // just take the first block.
        ConfigSet blocks;
        CssUtils::readConfig( _origData, conf.referrer(), blocks );
        if ( blocks.size() > 0 )
            SLDReader::readStyleFromCSSParams( blocks.front(), *this );
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
                    getOrCreate<TextSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "point" )
                {
                    getOrCreate<PointSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "line" )
                {
                    getOrCreate<LineSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "polygon" )
                {
                    getOrCreate<PolygonSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "extrusion" )
                {
                    getOrCreate<ExtrusionSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "altitude" )
                {
                    getOrCreate<AltitudeSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "marker" )
                {
                    getOrCreate<MarkerSymbol>()->mergeConfig( c );
                }
                else if ( c.key() == "skin" )
                {
                    getOrCreate<SkinSymbol>()->mergeConfig( c );
                }
            }
        }
    }
}

Config
Style::getConfig( bool keepOrigType ) const
{
    Config conf( "style" );
    conf.set("name", _name);

    conf.addIfSet( "url", _uri );
    
    if ( _origType == "text/css" && keepOrigType )
    {
        conf.set("type", _origType);
        conf.value() = _origData;            
    }
    else
    {
        Config symbolsConf( "symbols" );
        for( SymbolList::const_iterator i = _symbols.begin(); i != _symbols.end(); ++i )
        {
            symbolsConf.add( i->get()->getConfig() );
        }
        conf.add( symbolsConf );
    }

    return conf;
}
