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
using namespace osgEarthFeatures;
using namespace osgEarthFeatures::Styling;


void
StyleVisitor::apply( class Stroke& obj ) { 
    //nop
}

void
StyleVisitor::apply( class Fill& obj ) {
    //nop
}

void
StyleVisitor::apply( class LineSymbolizer& obj ) {
    obj.stroke().accept( *this );
}

void
StyleVisitor::apply( class PolygonSymbolizer& obj ) {
    obj.fill().accept( *this );
}

void
StyleVisitor::apply( class StyleClass& obj ) {
    //obj.query().accept( *this );
    obj.lineSymbolizer().accept( *this );
    obj.polygonSymbolizer().accept( *this );
}

void
StyleVisitor::apply( class NamedLayer& obj ) {
    for(StyleClasses::iterator i = obj.styleClasses().begin(); i != obj.styleClasses().end(); i++ )
        i->accept( *this );
}

void
StyleVisitor::apply( class StyleCatalog& obj ) {
    for(NamedLayers::iterator i = obj.namedLayers().begin(); i != obj.namedLayers().end(); i++ )
        i->accept( *this );
}

/**************************************************************************/

osg::Vec4ub
StyleClass::getColor( const FeatureProfile::GeometryType& geomType ) const
{
    osg::Vec4ub color;
    if ( geomType == FeatureProfile::GEOM_POLYGON )
    {
        color = polygonSymbolizer().fill().color();
        color.a() = (int)(255.0 * polygonSymbolizer().fill().opacity());
    }
    else
    {
        color = lineSymbolizer().stroke().color();
        color.a() = (int)(255.0 * lineSymbolizer().stroke().opacity());
    }
    return color;
}

/**************************************************************************/

// reads inline style information into a style class (if the names match)
bool
StyleReader::readStyleClassFromCSS( const Config& conf, StyleClass& out_sc, bool matchesOnly )
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
            SLDReader::readStyleClassFromCSSParams( subConf, out_sc );
        }
    }
    
    return true;    
}

bool
StyleReader::readStyleClassesFromCSS( const Config& conf, StyleClasses& out_classes, bool createIfNecessary )
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
            for( StyleClasses::iterator k = out_classes.begin(); k != out_classes.end(); k++ )
            {
                if ( k->name() == subConf.name() )
                {
                    SLDReader::readStyleClassFromCSSParams( subConf, *k );
                    found = true;
                    break;
                }
            }

            if ( !found && createIfNecessary )
            {
                StyleClass sc;
                sc.name() == subConf.name();
                SLDReader::readStyleClassFromCSSParams( subConf, sc );
                out_classes.push_back( sc );
            }
        }
    }
    
    return true;
}

bool
StyleReader::readStyleClassFromSLD( const ConfigSet& confSet, StyleClass& out_class, bool matchesOnly )
{
    return false;
}

bool
StyleReader::readStyleClassesFromSLD( const ConfigSet& confSet, StyleClasses& out_classes, bool createIfNecessary )
{
    return false;
}

bool
StyleReader::readStyleClass( const Config& conf, StyleClass& out_class, bool matchesOnly )
{
    if ( conf.attr("type") == "text/css" )
    {
        return readStyleClassFromCSS( conf, out_class, matchesOnly );
    }
    else if ( conf.attr("type") == "application/vnd.ogc.sld+xml" ||
              conf.attr("type") == "application/vnd.ogc.sld" )
    {
        return readStyleClassFromSLD( conf.children(), out_class, matchesOnly );
    }
    else
        return false;
}

bool
StyleReader::readStyleClasses( const Config& conf, StyleClasses& out_styleClasses )
{
    if ( conf.attr("type") == "text/css" )
    {
        return readStyleClassesFromCSS( conf, out_styleClasses, true );
    }
    else if ( conf.attr("type") == "application/vnd.ogc.sld+xml" ||
              conf.attr("type") == "application/vnd.ogc.sld" )
    {
        return readStyleClassesFromSLD( conf.children(), out_styleClasses, true );
    }
    else
        return false;
}

bool 
StyleReader::readLayerStyles( const std::string& layerName, const Config& conf, StyleCatalog& out_cat )
{
    NamedLayer layer;
    layer.name() = layerName;    

    // first read any style class definitions:
    ConfigSet classes = conf.children( "class" );
    for( ConfigSet::iterator i = classes.begin(); i != classes.end(); ++i )
    {
        const Config& classConf = *i;
        StyleClass sc;

        sc.name() = classConf.value( "name" );
        sc.query() = Query( classConf.child( "query" ) );
        if ( classConf.hasChild( "style" ) )
        {
            readStyleClass( classConf.child("style"), sc, false );
        }

        layer.styleClasses().push_back( sc );
    }

    // next, read any style data:
    ConfigSet styles = conf.children( "style" );
    for( ConfigSet::iterator i = styles.begin(); i != styles.end(); ++i )
    {
        const Config& styleConf = *i;
        readStyleClasses( styleConf, layer.styleClasses() );        
    }

    // finally, add the new style layer.
    out_cat.namedLayers().push_back( layer );
    return true;
}
