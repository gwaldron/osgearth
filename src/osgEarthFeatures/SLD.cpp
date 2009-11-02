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
using namespace osgEarthFeatures;
using namespace osgEarthFeatures::Styling;

static osg::Vec4ub
htmlColorToVec4ub( const std::string& html )
{
    std::string t;
    std::transform( html.begin(), html.end(), t.begin(), tolower );
    osg::Vec4ub c(0,0,0,255);
    if ( t.length() == 7 ) {
        c.r() |= t[1]<='9' ? (t[1]-'0')<<8 : (t[1]-'a');
        c.r() |= t[2]<='9' ? (t[2]-'0') : t[2]-'a';
        c.g() |= t[3]<='9' ? (t[3]-'0')<<8 : (t[3]-'a');
        c.g() |= t[4]<='9' ? (t[4]-'0') : t[4]-'a';
        c.b() |= t[5]<='9' ? (t[5]-'0')<<8 : (t[5]-'a');
        c.b() |= t[6]<='9' ? (t[6]-'0') : t[6]-'a';
    }
    return c;
}

#define CSS_STROKE       "stroke"
#define CSS_STROKE_WIDTH "stroke-width"
#define CSS_FILL         "fill"

struct SLDFromConfigVisitor : public StyleVisitor
{
    void apply( class Stroke& obj ) {
        ConfigSet set = _conf.top().children( "sld:CssParameter" );
        for(ConfigSet::iterator i = set.begin(); i != set.end(); i++) {
            if ( i->attr("name") == CSS_STROKE )
                obj.color() == htmlColorToVec4ub( i->value( "ogc:Literal" ) );
            else if ( i->attr("name") == CSS_STROKE_WIDTH)
                obj.width() = i->value<float>( "ogc:Literal", obj.width() );
            //todo
        }
    }
    void apply( class Fill& obj ) {
        ConfigSet set = _conf.top().children( "sld:CssParameter" );
        for(ConfigSet::iterator i = set.begin(); i != set.end(); i++) {
            if ( i->attr("name") == CSS_FILL )
                obj.color() == htmlColorToVec4ub( i->value( "ogc:Literal" ) );
            //todo
        }
    }
    void apply( class LineSymbolizer& obj ) {
        traverse( _conf.top().child( "sld:Stroke" ), obj.stroke() );
    }
    void apply( class PolygonSymbolizer& obj ) {
        traverse( _conf.top().child( "sld:Fill" ), obj.fill() );
    }
    void apply( class RuleFilter& obj ) {
        //todo
    }
    void apply( class Rule& obj ) {
        traverse( _conf.top().child( "sld:Filter" ), obj.filter() );
        traverse( _conf.top().child( "sld:LineSymbolizer" ), obj.lineSymbolizer() );
        traverse( _conf.top().child( "sld:PolygonSymbolizer" ), obj.polygonSymbolizer() );
    }
    void apply( class FeatureTypeStyle& obj ) {
        ConfigSet set = _conf.top().children( "sld:Rule" );
        for(ConfigSet::iterator i = set.begin(); i != set.end(); i++ ) {
            Rule rule;
            traverse( *i, rule );
            obj.rules().push_back( rule );
        }
    }
    void apply( class UserLayer& obj ) {
        traverse( _conf.top().child( "sld:FeatureTypeStyle" ), obj.featureTypeStyle() );
    }
    void apply( class StyledLayerDescriptor& obj ) {
        ConfigSet c = _conf.top().children("sld:UserLayer");
        for(ConfigSet::iterator i = c.begin(); i != c.end(); i++ ) {
            UserLayer userLayer;
            traverse( *i, userLayer );
            obj.userLayers().push_back( userLayer );
        }
    }

    void pushConfig( const Config& conf ) { _conf.push( conf ); }
    void popConfig() { _conf.pop(); }
    void traverse( const Config& conf, StyleComponent& sc ) {
        if ( !conf.empty() ) {
            pushConfig( conf );
            sc.accept( *this );
            popConfig();
        }
    }

    std::stack<Config> _conf;
};

bool
SLDReader::readConfig( const Config& conf, StyledLayerDescriptor& out_sld )
{
    SLDFromConfigVisitor visitor;
    visitor.pushConfig( conf );
    out_sld.accept( visitor );
    return true;
}

bool
SLDReader::readXML( std::istream& in, StyledLayerDescriptor& out_sld )
{
    osg::ref_ptr<XmlDocument> xml = XmlDocument::load( in );
    Config conf = xml->toConfig();
    return readConfig( conf, out_sld );
}

bool
SLDReader::readRulesCSS( std::istream& css, RuleList& out_rules )
{
    Config root = CssUtils::readConfig( css );
    for(ConfigSet::const_iterator i = root.children().begin(); i != root.children().end(); i++ )
    {
        const Config& conf = *i;
        Rule rule;
        for(Properties::const_iterator p = conf.attrs().begin(); p != conf.attrs().end(); p++ )
        {
            if ( p->first == CSS_STROKE )
                rule.lineSymbolizer().stroke().color() = htmlColorToVec4ub( p->second );
            else if ( p->first == CSS_STROKE_WIDTH )
                rule.lineSymbolizer().stroke().width() = as<float>( p->second, 1.0f );
            else if ( p->first == CSS_FILL )
                rule.polygonSymbolizer().fill().color() == htmlColorToVec4ub( p->second );
            //TODO more..
        }
        out_rules.push_back( rule );
    }
    return true;
}