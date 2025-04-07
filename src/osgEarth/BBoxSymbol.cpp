/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/BBoxSymbol>
#include <osgEarth/Style>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(bbox, BBoxSymbol);

BBoxSymbol::BBoxSymbol( const Config& conf ) :
Symbol                ( conf ),
_fill                 ( Fill( 1, 1, 1, 1 ) ),
_border               ( Stroke( 0.3, 0.3, 0.3, 1) ),
_margin               ( 3 ),
_bboxGeom             ( GEOM_BOX )
{
    mergeConfig(conf);
}

BBoxSymbol::BBoxSymbol(const BBoxSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_fill                 ( rhs._fill ),
_border               ( rhs._border ),
_margin               ( rhs._margin ),
_bboxGeom             ( rhs._bboxGeom )
{

}

Config
BBoxSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "bbox";
    conf.set( "fill", _fill );
    conf.set( "border", _border );
    conf.set( "margin", _margin );

    conf.set( "geom", "box", _bboxGeom, GEOM_BOX );
    conf.set( "geom", "box_oriented", _bboxGeom, GEOM_BOX_ORIENTED );

    return conf;
}

void
BBoxSymbol::mergeConfig( const Config& conf )
{
    conf.get( "fill", _fill );
    conf.get( "border", _border );
    conf.get( "margin", _margin );

    conf.get( "geom", "box", _bboxGeom, GEOM_BOX );
    conf.get( "geom", "box_oriented", _bboxGeom, GEOM_BOX_ORIENTED );
}

void
BBoxSymbol::parseSLD(const Config& c, Style& style)
{
    if (match(c.key(), "library")) {
        if (!c.value().empty())
            style.getOrCreate<SkinSymbol>()->library() = Strings::unquote(c.value());
    }
    else
    if ( match(c.key(), "text-bbox-fill") ) {
       style.getOrCreate<BBoxSymbol>()->fill().mutable_value().color() = Color(c.value());
    }
    else if ( match(c.key(), "text-bbox-border") ) {
        style.getOrCreate<BBoxSymbol>()->border().mutable_value().color() = Color(c.value());
    }
    else if ( match(c.key(), "text-bbox-border-width") ) {
        style.getOrCreate<BBoxSymbol>()->border().mutable_value().width() = Distance(c.value(), Units::PIXELS);
    }
    else if ( match(c.key(), "text-bbox-margin") ) {
        style.getOrCreate<BBoxSymbol>()->margin() = as<float>(c.value(), 3.0f);
    }
    else if ( match(c.key(), "text-bbox-geom") ) {
        if      ( match(c.value(), "box") )
            style.getOrCreate<BBoxSymbol>()->geom() = GEOM_BOX;
        else if ( match(c.value(), "box_oriented") )
            style.getOrCreate<BBoxSymbol>()->geom() = GEOM_BOX_ORIENTED;
    }
}
