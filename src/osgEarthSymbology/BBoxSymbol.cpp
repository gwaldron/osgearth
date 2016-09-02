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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthSymbology/BBoxSymbol>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

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
    conf.key() = "text-bbox";
    conf.addObjIfSet( "fill", _fill );
    conf.addObjIfSet( "border", _border );
    conf.addIfSet( "margin", _margin );

    conf.addIfSet( "geom", "box", _bboxGeom, GEOM_BOX );
    conf.addIfSet( "geom", "box_oriented", _bboxGeom, GEOM_BOX_ORIENTED );

    return conf;
}

void
BBoxSymbol::mergeConfig( const Config& conf )
{
    conf.getObjIfSet( "fill", _fill );
    conf.getObjIfSet( "border", _border );
    conf.getIfSet( "margin", _margin );

    conf.getIfSet( "geom", "box", _bboxGeom, GEOM_BOX );
    conf.getIfSet( "geom", "box_oriented", _bboxGeom, GEOM_BOX_ORIENTED );
}

void
BBoxSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "text-bbox-fill") ) {
       style.getOrCreate<BBoxSymbol>()->fill()->color() = Color(c.value());
    }
    else if ( match(c.key(), "text-bbox-border") ) {
        style.getOrCreate<BBoxSymbol>()->border()->color() = Color(c.value());
    }
    else if ( match(c.key(), "text-bbox-border-width") ) {
        style.getOrCreate<BBoxSymbol>()->border()->width() = as<float>( c.value(), 1.0f );
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
