/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/CoverageSymbol>
#include <osgEarth/Style>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(coverage, CoverageSymbol);

CoverageSymbol::CoverageSymbol(const CoverageSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_valueExpr( rhs._valueExpr )
{
    //nop
}

CoverageSymbol::CoverageSymbol( const Config& conf ) :
Symbol( conf )
{
    mergeConfig(conf);
}

Config 
CoverageSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "coverage";
    conf.set( "value", _valueExpr );
    return conf;
}

void 
CoverageSymbol::mergeConfig( const Config& conf )
{
    conf.get( "value", _valueExpr );
}


void
CoverageSymbol::parseSLD(const Config& c, Style& style)
{
    if (match(c.key(), "library")) {
        if (!c.value().empty())
            style.getOrCreate<SkinSymbol>()->library() = Strings::unquote(c.value());
    }
    else
    if ( match(c.key(), "coverage-value") ) {
        style.getOrCreate<CoverageSymbol>()->valueExpression() = NumericExpression(c.value());
    }
}

