/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include <osgEarth/StyleSelector>
#include <osgEarth/StringUtils>
#include <osgEarth/Registry>

#define LC "[StyleSelector] "

using namespace osgEarth;

//------------------------------------------------------------------------

StyleSelector::StyleSelector( const Config& conf )
{
    mergeConfig( conf );
}

StyleSelector::StyleSelector(const std::string& name, const StringExpression& expr)
{
    _name = name;
    _styleExpression = expr;
}

std::string
StyleSelector::getSelectedStyleName() const 
{
    return _styleName.isSet() ? _styleName.get() : _name.get();
}

void
StyleSelector::mergeConfig( const Config& conf )
{
    conf.get( "name",       _name);
    conf.get( "style",      _styleName );
    conf.get( "class",      _styleName ); // alias
    conf.get( "style_expr", _styleExpression ); 
    conf.get( "class_expr", _styleExpression ); // alias
    conf.get( "query",      _query );
}

Config
StyleSelector::getConfig() const
{
    Config conf( "selector" );
    conf.set( "name",       _name );
    conf.set( "style",      _styleName );
    conf.set( "style_expr", _styleExpression );
    conf.set( "query",      _query );
    return conf;
}



OSGEARTH_REGISTER_SIMPLE_SYMBOL(select, SelectorSymbol);

SelectorSymbol::SelectorSymbol(const Config& conf) :
    Symbol(conf)
{
    mergeConfig(conf);
}

SelectorSymbol::SelectorSymbol(const SelectorSymbol& rhs, const osg::CopyOp& copy) :
    Symbol(rhs, copy),
    _predicate(rhs._predicate)
{
    //nop
}

Config
SelectorSymbol::getConfig() const
{
    auto conf = Symbol::getConfig();
    conf.key() = "selector";
    conf.set("predicate", predicate());
    return conf;
}

void
SelectorSymbol::mergeConfig(const Config& conf)
{
    conf.get("predicate", predicate());
}

void
SelectorSymbol::parseSLD(const Config& c, Style& style)
{
    if (match(c.key(), "select") || match(c.key(), "select-if"))
    {
        style.getOrCreate<SelectorSymbol>()->predicate() = c.value();
    }
}
