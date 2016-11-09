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
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/CssUtils>
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
        copySymbols(rhs);
    }
}

Style&
Style::operator = ( const Style& rhs )
{
    _name = rhs._name;
    _origType = rhs._origType;
    _origData = rhs._origData;
    _uri = rhs._uri;
    _symbols.clear();
    copySymbols(rhs);
    return *this;
}

void Style::addSymbol(Symbol* symbol)
{
    if ( symbol )
    {
        for( SymbolList::iterator i = _symbols.begin(); i != _symbols.end(); ++i )
        {
            if ( i->get()->isSameKindAs(symbol) )
            {
                (*i) = symbol;
                return;
            }
        }

        _symbols.push_back(symbol);
    }
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
    Style newStyle(*this);
    newStyle.copySymbols(rhs);
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

void Style::copySymbols(const Style& style)
{
    for (SymbolList::const_iterator itr = style._symbols.begin(); itr != style._symbols.end(); ++itr)
    {
        addSymbol(static_cast<Symbol*>(itr->get()->clone(osg::CopyOp::SHALLOW_COPY)));
    }
}

void
Style::fromSLD( const Config& sld )
{
    setName( sld.key() );

    for( ConfigSet::const_iterator kid = sld.children().begin(); kid != sld.children().end(); ++kid )
    {
        const Config& p = *kid;
		SymbolRegistry::instance()->parseSLD( p, *this );		
    }
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
    std::string textData = trim(conf.value());

    bool useCSS =
        _origType.compare("text/css") == 0 ||
        !textData.empty();

    if ( useCSS )
    {
        _origData = textData;
        
        // just take the first block.
        ConfigSet blocks;
        CssUtils::readConfig( _origData, conf.referrer(), blocks );
        if ( blocks.size() > 0 )
        {
            fromSLD( blocks.front() );
        }
    }
    else
    {
        Config symbolConf = conf.child( "symbols" );
        if ( !symbolConf.empty() )
        {
            for( ConfigSet::const_iterator i = symbolConf.children().begin(); i != symbolConf.children().end(); ++i )
            {
                const Config& c = *i;
				Symbol* symbol = SymbolRegistry::instance()->create( c );

				add(symbol);
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
