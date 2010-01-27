/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthFeatures/Styling>
#include <osgEarthFeatures/CssUtils>
#include <osgEarthFeatures/SLD>
#include <stack>

using namespace osgEarth;
using namespace osgEarth::Features;


//void
//StyleVisitor::apply( class Stroke& obj ) { 
//    //nop
//}
//
//void
//StyleVisitor::apply( class Fill& obj ) {
//    //nop
//}
//
//void
//StyleVisitor::apply( class LineSymbolizer& obj ) {
//    obj.stroke().accept( *this );
//}
//
//void
//StyleVisitor::apply( class PolygonSymbolizer& obj ) {
//    obj.fill().accept( *this );
//}
//
//void
//StyleVisitor::apply( class TextSymbolizer& obj ) {
//    obj.fill().accept( *this );
//    obj.halo().accept( *this );
//}
//
//void
//StyleVisitor::apply( class Style& obj ) {
//    //obj.query().accept( *this );
//    obj.lineSymbolizer().accept( *this );
//    obj.polygonSymbolizer().accept( *this );
//}
//
//void
//StyleVisitor::apply( class StyledLayer& obj ) {
//    for(StyleList::iterator i = obj.styles().begin(); i != obj.styles().end(); i++ )
//        i->accept( *this );
//}
//
//void
//StyleVisitor::apply( class StyleCatalog& obj ) {
//    for(StyledLayers::iterator i = obj.namedLayers().begin(); i != obj.namedLayers().end(); i++ )
//        i->accept( *this );
//}

/**************************************************************************/


Stroke::Stroke() :
_color( 1, 1, 1, 1 ),
_width( 1.0f ),
_lineJoin( LINEJOIN_DEFAULT ),
_lineCap( LINECAP_DEFAULT )
{
    //nop
}

Fill::Fill() :
_color( 1, 1, 1, 1 )
{
    //nop
}

Symbolizer::Symbolizer()
{
    //nop
}

LineSymbolizer::LineSymbolizer() :
_stroke( Stroke() )
{
    //nop
}

PolygonSymbolizer::PolygonSymbolizer() :
_fill( Fill() )
{
    //nop
}

TextSymbolizer::TextSymbolizer() :
_fill( Fill() ),
_halo( Stroke() ),
_size( 16.0f ),
_font( "fonts/arial.ttf" )
{
    //nop
}

StyledLayer::StyledLayer()
{
    //nop
}

StyleCatalog::StyleCatalog()
{
    //nop
}

bool 
StyleCatalog::getNamedLayer( const std::string& name, StyledLayer& out_layer ) const
{
    for( StyledLayers::const_iterator i = _namedLayers.begin(); i != _namedLayers.end(); i++ )
    {
        if ( i->name() == name ) {
            out_layer = *i;
            return true;
        }
    }
    return false;
}


/**************************************************************************/

Style::Style() :
_query( Query() ),
_lineSymbolizer( LineSymbolizer() ),
_polygonSymbolizer( PolygonSymbolizer() ),
_textSymbolizer( TextSymbolizer() )
{
    //nop
}

osg::Vec4f
Style::getColor( const Geometry::Type& geomType ) const
{
    return
        geomType == Geometry::TYPE_POLYGON ? polygonSymbolizer()->fill()->color() :
        lineSymbolizer()->stroke()->color();
}

/**************************************************************************/

// reads inline style information into a style class (if the names match)
bool
StyleReader::readStyleFromCSS( const Config& conf, Style& out_sc, bool matchesOnly )
{
    std::string css = conf.value();

    if ( !conf.attr("href").empty() )
    {
        //TODO: load css from href url
    }
    std::stringstream buf( conf.value() );
    Config root = CssUtils::readConfig( buf );

    for(ConfigSet::const_iterator i = root.children().begin(); i != root.children().end(); i++ )
    {
        const Config& subConf = *i;
        if ( subConf.name() == out_sc.name() || !matchesOnly )
        {
            SLDReader::readStyleFromCSSParams( subConf, out_sc );
        }
    }
    
    return true;    
}

bool
StyleReader::readStyleListFromCSS( const Config& conf, StyleList& out_classes, bool createIfNecessary )
{
    std::string css = conf.value();

    if ( !conf.attr("href").empty() )
    {
        //TODO: load css from href url
    }
    std::stringstream buf( css );
    Config root = CssUtils::readConfig( buf );

    for(ConfigSet::const_iterator i = root.children().begin(); i != root.children().end(); i++ )
    {
        const Config& subConf = *i;
        if ( !subConf.name().empty() )
        {
            bool found = false;
            for( StyleList::iterator k = out_classes.begin(); k != out_classes.end(); k++ )
            {
                if ( k->name() == subConf.name() )
                {
                    SLDReader::readStyleFromCSSParams( subConf, *k );
                    found = true;
                    break;
                }
            }

            if ( !found && createIfNecessary )
            {
                Style sc;
                sc.name() == subConf.name();
                SLDReader::readStyleFromCSSParams( subConf, sc );
                out_classes.push_back( sc );
            }
        }
    }
    
    return true;
}

//bool
//StyleReader::readStyleFromSLD( const ConfigSet& confSet, Style& out_class, bool matchesOnly )
//{
//    return false;
//}
//
//bool
//StyleReader::readStyleListFromSLD( const ConfigSet& confSet, StyleList& out_classes, bool createIfNecessary )
//{
//    return false;
//}

bool
StyleReader::readStyle( const Config& conf, Style& out_class, bool matchesOnly )
{
    if ( conf.attr("type") == "text/css" )
    {
        return readStyleFromCSS( conf, out_class, matchesOnly );
    }
    //else if ( conf.attr("type") == "application/vnd.ogc.sld+xml" ||
    //          conf.attr("type") == "application/vnd.ogc.sld" )
    //{
    //    return readStyleFromSLD( conf.children(), out_class, matchesOnly );
    //}
    else
        return false;
}

bool
StyleReader::readStyleList( const Config& conf, StyleList& out_styles )
{
    if ( conf.attr("type") == "text/css" )
    {
        return readStyleListFromCSS( conf, out_styles, true );
    }
    //else if ( conf.attr("type") == "application/vnd.ogc.sld+xml" ||
    //          conf.attr("type") == "application/vnd.ogc.sld" )
    //{
    //    return readStyleListFromSLD( conf.children(), out_styles, true );
    //}
    else
        return false;
}

bool 
StyleReader::readLayerStyles( const std::string& layerName, const Config& conf, StyleCatalog& out_cat )
{
    StyledLayer layer;
    layer.name() = layerName;    

    bool readAtLeastOne = false;

    // first read any style class definitions:
    ConfigSet classes = conf.children( "class" );
    for( ConfigSet::iterator i = classes.begin(); i != classes.end(); ++i )
    {
        const Config& classConf = *i;
        Style style;

        style.name() = classConf.value( "name" );
        style.query() = Query( classConf.child( "query" ) );
        if ( classConf.hasChild( "style" ) )
        {
            readStyle( classConf.child("style"), style, false );
            readAtLeastOne = true;
        }

        layer.styles().push_back( style );
    }

    // next, read any style data:
    ConfigSet styles = conf.children( "style" );
    for( ConfigSet::iterator i = styles.begin(); i != styles.end(); ++i )
    {
        const Config& styleConf = *i;
        readStyleList( styleConf, layer.styles() );  
        readAtLeastOne = true;
    }

    // finally, add the new style layer.
    if ( readAtLeastOne )
        out_cat.namedLayers().push_back( layer );

    return true;
}
