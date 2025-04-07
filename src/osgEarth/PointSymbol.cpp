/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/PointSymbol>
#include <osgEarth/Style>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(point, PointSymbol);

PointSymbol::PointSymbol(const PointSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_fill(rhs._fill),
_size(rhs._size),
_smooth(rhs._smooth)
{
}

PointSymbol::PointSymbol( const Config& conf ) :
Symbol( conf ),
_fill ( Fill() ), 
_size ( 1.0 ),
_smooth( false )
{
    mergeConfig(conf);
}

Config 
PointSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "point";
    conf.set( "fill", _fill );
    conf.set( "size", _size );
    conf.set( "smooth", _smooth );
    return conf;
}

void 
PointSymbol::mergeConfig( const Config& conf )
{
    conf.get( "fill", _fill );
    conf.get( "size", _size );
    conf.get( "smooth", _smooth );
}


void
PointSymbol::parseSLD(const Config& c, Style& style)
{
    if (match(c.key(), "library")) {
        if (!c.value().empty())
            style.getOrCreate<SkinSymbol>()->library() = Strings::unquote(c.value());
    }
    else
    if ( match(c.key(), "point-fill") ) {
        style.getOrCreate<PointSymbol>()->fill().mutable_value().color() = Color(c.value());
    }
    else if ( match(c.key(), "point-fill-opacity") ) {
        style.getOrCreate<PointSymbol>()->fill().mutable_value().color().a() = as<float>( c.value(), 1.0f );
    }
    else if ( match(c.key(), "point-size") ) {
        style.getOrCreate<PointSymbol>()->size() = as<float>(c.value(), 1.0f);
    }
    else if ( match(c.key(), "point-script") ) {
        style.getOrCreate<PointSymbol>()->script() = StringExpression(c.value());
    }
    else if (match(c.key(), "point-smooth")) {
        style.getOrCreate<PointSymbol>()->smooth() = as<bool>(c.value(), false);
    }
}

