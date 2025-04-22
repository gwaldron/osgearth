/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/StyleSelector>

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
    conf.key() = "select";
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
