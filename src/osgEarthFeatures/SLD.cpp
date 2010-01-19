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
#include <osgEarthFeatures/SLD>
#include <osgEarthFeatures/CssUtils>
#include <osgEarth/XmlUtils>
#include <stack>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;

static osg::Vec4f
htmlColorToVec4f( const std::string& html )
{
    std::string t = html;
    std::transform( t.begin(), t.end(), t.begin(), ::tolower );
    osg::Vec4ub c(0,0,0,255);
    if ( t.length() == 7 ) {
        c.r() |= t[1]<='9' ? (t[1]-'0')<<4 : (10+(t[1]-'a'))<<4;
        c.r() |= t[2]<='9' ? (t[2]-'0')    : (10+(t[2]-'a'));
        c.g() |= t[3]<='9' ? (t[3]-'0')<<4 : (10+(t[3]-'a'))<<4;
        c.g() |= t[4]<='9' ? (t[4]-'0')    : (10+(t[4]-'a'));
        c.b() |= t[5]<='9' ? (t[5]-'0')<<4 : (10+(t[5]-'a'))<<4;
        c.b() |= t[6]<='9' ? (t[6]-'0')    : (10+(t[6]-'a'));
    }
    return osg::Vec4f( ((float)c.r())/255.0f, ((float)c.g())/255.0f, ((float)c.b())/255.0f, 1.0f );
}

#define CSS_STROKE          "stroke"
#define CSS_STROKE_WIDTH    "stroke-width"
#define CSS_STROKE_OPACITY  "stroke-opacity"
#define CSS_STROKE_LINECAP  "stroke-linecap"

#define CSS_FILL           "fill"
#define CSS_FILL_OPACITY   "fill-opacity"


//struct SLDFromConfigVisitor : public StyleVisitor
//{
//    void apply( class Stroke& obj ) {
//        ConfigSet set = _conf.top().children( "sld:CssParameter" );
//        for(ConfigSet::iterator i = set.begin(); i != set.end(); i++) {
//            if ( i->attr("name") == CSS_STROKE )
//                obj.color() = htmlColorToVec4ub( i->value( "ogc:Literal" ) );
//            else if ( i->attr("name") == CSS_STROKE_OPACITY )
//                obj.opacity() = i->value<float>( "ogc:Literal", obj.opacity() );
//            else if ( i->attr("name") == CSS_STROKE_WIDTH )
//                obj.width() = i->value<float>( "ogc:Literal", obj.width() );
//            //todo
//        }
//    }
//
//    void apply( class Fill& obj ) {
//        ConfigSet set = _conf.top().children( "sld:CssParameter" );
//        for(ConfigSet::iterator i = set.begin(); i != set.end(); i++) {
//            if ( i->attr("name") == CSS_FILL )
//                obj.color() = htmlColorToVec4ub( i->value( "ogc:Literal" ) );
//            else if ( i->attr("name") == CSS_FILL_OPACITY )
//                obj.opacity() = i->value<float>( "ogc:Literal", obj.opacity() );
//            //todo
//        }
//    }
//
//    void apply( class LineSymbolizer& obj ) {
//        traverse( _conf.top().child( "sld:Stroke" ), obj.stroke() );
//    }
//
//    void apply( class PolygonSymbolizer& obj ) {
//        traverse( _conf.top().child( "sld:Fill" ), obj.fill() );
//    }
//
//    //void apply( class FeatureQuery& obj ) {
//    //    //TODO
//    //}
//
//    void apply( class Style& obj ) {
//        //traverse( _conf.top().child( "sld:Filter" ), obj.filter() );
//        traverse( _conf.top().child( "sld:LineSymbolizer" ), obj.lineSymbolizer() );
//        traverse( _conf.top().child( "sld:PolygonSymbolizer" ), obj.polygonSymbolizer() );
//    }
//
//    void apply( class StyledLayer& obj ) {
//        traverse( _conf.top().child( "sld:UserStyle" ), obj );
//        traverse( _conf.top().child( "sld:FeatureTypeStyle" ), obj );
//        ConfigSet rules = _conf.top().children( "sld:Rule" );
//        for( ConfigSet::iterator i = rules.begin(); i != rules.end(); ++i ) {
//            Style style;
//            traverse( *i, style );
//            obj.styles().push_back( style );
//        }            
//    }
//
//    void apply( class StyleCatalog& obj ) {
//        ConfigSet c = _conf.top().children( "sld:NamedLayer" );
//        for(ConfigSet::iterator i = c.begin(); i != c.end(); i++ ) {
//            StyledLayer namedLayer;
//            traverse( *i, namedLayer );
//            obj.namedLayers().push_back( namedLayer );
//        }
//    }
//
//    void pushConfig( const Config& conf ) { _conf.push( conf ); }
//    void popConfig() { _conf.pop(); }
//    void traverse( const Config& conf, StyleComponent& sc ) {
//        if ( !conf.empty() ) {
//            pushConfig( conf );
//            sc.accept( *this );
//            popConfig();
//        }
//    }
//
//    std::stack<Config> _conf;
//};
//
//bool
//SLDReader::readConfig( const Config& conf, StyleCatalog& out_sld )
//{
//    SLDFromConfigVisitor visitor;
//    visitor.pushConfig( conf );
//    out_sld.accept( visitor );
//    return true;
//}
//
//bool
//SLDReader::readXML( std::istream& in, StyleCatalog& out_sld )
//{
//    osg::ref_ptr<XmlDocument> xml = XmlDocument::load( in );
//    Config conf = xml->toConfig();
//    return readConfig( conf, out_sld );
//}

static void
parseLineCap( const std::string& value, optional<Stroke::LineCapStyle>& cap )
{
    if ( value == "butt" ) cap = Stroke::LINECAP_BUTT;
    if ( value == "round" ) cap = Stroke::LINECAP_ROUND;
    if ( value == "square" ) cap = Stroke::LINECAP_SQUARE;
}

bool
SLDReader::readStyleFromCSSParams( const Config& conf, Style& sc )
{
    for(Properties::const_iterator p = conf.attrs().begin(); p != conf.attrs().end(); p++ )
    {
        if ( p->first == CSS_STROKE ) {
            sc.lineSymbolizer()->stroke()->color() = htmlColorToVec4f( p->second );
        }
        else if ( p->first == CSS_STROKE_OPACITY ) {
            sc.lineSymbolizer()->stroke()->color().a() = as<float>( p->second, 1.0f );
        }
        else if ( p->first == CSS_STROKE_WIDTH ) {
            sc.lineSymbolizer()->stroke()->width() = as<float>( p->second, 1.0f );
        }
        else if ( p->first == CSS_STROKE_LINECAP ) {
            parseLineCap( p->second, sc.lineSymbolizer()->stroke()->lineCap() );
        }
        else if ( p->first == CSS_FILL ) {
            sc.polygonSymbolizer()->fill()->color() = htmlColorToVec4f( p->second );
        }
        else if ( p->first == CSS_FILL_OPACITY ) {
            sc.polygonSymbolizer()->fill()->color().a() = as<float>( p->second, 1.0f );
        }
    }
    return true;
}


