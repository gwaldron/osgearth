/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include "KMLReader"
#include <osgEarthSymbology/Color>
#include <osgEarthUtil/Annotation>
#include <osgEarth/XMLUtils>
#include <stack>

using namespace osgEarth;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Annotation;

//------------------------------------------------------------------------


#define for_one( NAME, FUNC, CONF, CX ) \
{ \
    Config c = conf.child( toLower( #NAME ) ); \
    if ( !c.empty() ) { \
        KML##NAME instance; \
        instance. FUNC (c, CX); \
    } \
}

#define for_many( NAME, FUNC, CONF, CX ) \
{ \
   ConfigSet c = conf.children( toLower( #NAME ) ); \
   for( ConfigSet::const_iterator i = c.begin(); i != c.end(); ++i ) { \
        KML##NAME instance; \
        instance. FUNC (*i, CX); \
   } \
}

#define for_features( FUNC, CONF, CX ) \
    for_many( Document,      FUNC, CONF, CX ); \
    for_many( Folder,        FUNC, CONF, CX ); \
    for_many( PhotoOverlay,  FUNC, CONF, CX ); \
    for_many( ScreenOverlay, FUNC, CONF, CX ); \
    for_many( GroundOverlay, FUNC, CONF, CX ); \
    for_many( NetworkLink,   FUNC, CONF, CX ); \
    for_many( Placemark,     FUNC, CONF, CX );



namespace // KML PROTOTYPES
{
    struct KMLContext
    {
        MapNode*                              _mapNode;
        osg::ref_ptr<StyleSheet>              _sheet;
        std::stack<osg::ref_ptr<osg::Group> > _groupStack;
    };

    struct KMLObject {
        virtual void scan( const Config& conf, KMLContext& cx ) { }
        virtual void build( const Config& conf, KMLContext& cx );
    };

    struct KMLIconStyle {
        virtual void scan( const Config& conf, Style& style );
    };

    struct KMLLabelStyle {
        virtual void scan( const Config& conf, Style& style );
    };

    struct KMLLineStyle {
        virtual void scan( const Config& conf, Style& style );
    };

    struct KMLPolyStyle {
        virtual void scan( const Config& conf, Style& style );
    };

    struct KMLStyleSelector : public KMLObject {
    };

    struct KMLStyle : public KMLStyleSelector {
        virtual void scan( const Config& conf, KMLContext& cx );
    };

    struct KMLStyleMap : public KMLStyleSelector {
        virtual void scan( const Config& conf, KMLContext& cx );
    };

    struct KMLGeometry : public KMLObject {
        virtual void parseCoords( const Config& conf, KMLContext& cx );
        virtual void build( const Config& conf, KMLContext& cx );
        osg::ref_ptr<Geometry> _geom;
    };

    struct KMLPoint : public KMLGeometry {
        //virtual void scan( const Config& conf, KMLContext& cx );
        virtual void parseCoords( const Config& conf, KMLContext& cx );
    };

    struct KMLLineString : public KMLGeometry {
        //virtual void scan( const Config& conf, KMLContext& cx );
        virtual void parseCoords( const Config& conf, KMLContext& cx );
    };

    struct KMLLinearRing : public KMLGeometry {
        //virtual void scan( const Config& conf, KMLContext& cx );
        virtual void parseCoords( const Config& conf, KMLContext& cx );
    };

    struct KMLPolygon : public KMLGeometry {
        //virtual void scan( const Config& conf, KMLContext& cx );
        virtual void parseCoords( const Config& conf, KMLContext& cx );
    };

    struct KMLMultiGeometry : public KMLGeometry {
        //virtual void scan( const Config& conf, KMLContext& cx );
        virtual void parseCoords( const Config& conf, KMLContext& cx );
    };

    struct KMLModel : public KMLGeometry {
        //virtual void scan( const Config& conf, KMLContext& cx );
    };

    struct KMLFeature : public KMLObject {
        virtual void scan( const Config& conf, KMLContext& cx );
        virtual void build( const Config& conf, KMLContext& cx );
    };

    struct KMLContainer : public KMLFeature {
        virtual void scan( const Config& conf, KMLContext& cx );
        virtual void build( const Config& conf, KMLContext& cx );
    };

    struct KMLFolder : public KMLContainer {
        virtual void scan( const Config& conf, KMLContext& cx );
        virtual void build( const Config& conf, KMLContext& cx );
    };

    struct KMLDocument: public KMLContainer {
        virtual void scan( const Config& conf, KMLContext& cx );  
        virtual void build( const Config& conf, KMLContext& cx );
    };

    struct KMLOverlay : public KMLFeature {
    };

    struct KMLPhotoOverlay : public KMLOverlay {
    };

    struct KMLScreenOverlay : public KMLOverlay {
    };

    struct KMLGroundOverlay : public KMLOverlay {
    };

    struct KMLNetworkLinkControl : public KMLObject {
    };

    struct KMLPlacemark : public KMLFeature {
        virtual void build( const Config& conf, KMLContext& cx );
    };

    struct KMLNetworkLink : public KMLFeature {
    };

    struct KMLSchema : public KMLObject {
        virtual void scan( const Config& conf, KMLContext& cx );
    };

}

namespace // KML IMPLEMENTATIONS
{
    void KMLObject::build( const Config& conf, KMLContext& cx )
    {
    }

    void KMLIconStyle::scan( const Config& conf, Style& style )
    {
        if ( !conf.empty() )
        {
            MarkerSymbol* marker = style.getOrCreate<MarkerSymbol>();
            std::string iconHref = conf.child("Icon").value("href");
            if ( !iconHref.empty() )
                marker->url() = iconHref;

            optional<float> scale;
            conf.getIfSet( "scale", scale );
            if ( scale.isSet() )
                marker->scale() = osg::Vec3f(*scale, *scale, *scale);
        }
    }

    void KMLLabelStyle::scan( const Config& conf, Style& style )
    {
    }

    void KMLLineStyle::scan( const Config& conf, Style& style )
    {
    }

    void KMLPolyStyle::scan( const Config& conf, Style& style )
    {
    }

    void KMLStyle::scan( const Config& conf, KMLContext& cx )
    {
        Style style( conf.value("id") );

        KMLIconStyle icon;
        icon.scan( conf.child("IconStyle"), style );

        KMLLabelStyle label;
        label.scan( conf.child("LabelStyle"), style );

        KMLLineStyle line;
        line.scan( conf.child("LineStyle"), style );

        KMLPolyStyle poly;
        poly.scan( conf.child("PolyStyle"), style );

        cx._sheet->addStyle( style ); 
    }

    void KMLContainer::scan( const Config& conf, KMLContext& cx )
    {
        KMLFeature::scan(conf, cx);
    }
    void KMLContainer::build( const Config& conf, KMLContext& cx )
    {
        KMLFeature::build(conf, cx);
    }

    void KMLFolder::scan( const Config& conf, KMLContext& cx )
    {
        KMLContainer::scan(conf, cx);
        for_features( scan, conf, cx );
    }
    void KMLFolder::build( const Config& conf, KMLContext& cx )
    {
        osg::Group* group = new osg::Group();
        cx._groupStack.top()->addChild( group );
        cx._groupStack.push( group );

        KMLContainer::build(conf, cx);
        for_features(build, conf, cx);

        cx._groupStack.pop();
    }

    void KMLGeometry::build( const Config& conf, KMLContext& cx )
    {
        if ( conf.hasChild("point") ) {
            KMLPoint g;
            g.parseCoords(conf.child("point"), cx);
            _geom = g._geom.get();
        }
        else if ( conf.hasChild("linestring") ) {
            KMLLineString g;
            g.parseCoords(conf.child("linestring"), cx);
            _geom = g._geom.get();
        }
        else if ( conf.hasChild("linearring") ) {
            KMLLinearRing g;
            g.parseCoords(conf.child("linearring"), cx);
            _geom = g._geom.get();
        }
        else if ( conf.hasChild("polygon") ) {
            KMLPolygon g;
            g.parseCoords(conf.child("polygon"), cx);
            _geom = g._geom.get();
        }
        else if ( conf.hasChild("multigeometry") ) {
            KMLMultiGeometry g;
            g.parseCoords(conf.child("multigeometry"), cx);
            _geom = g._geom.get();
        }
        else if ( conf.hasChild("model") ) {
            KMLModel g;
            g.parseCoords(conf.child("model"), cx);
            _geom = g._geom.get();
        }
    }

    void KMLGeometry::parseCoords( const Config& conf, KMLContext& cx )
    {
        const Config& coords = conf.child("coordinates");
        StringVector tuples;
        StringTokenizer( coords.value(), tuples, " ", "", false, true );
        for( StringVector::const_iterator s=tuples.begin(); s != tuples.end(); ++s ) {
            StringVector parts;
            StringTokenizer( *s, parts, ",", "", false, true );
            if ( parts.size() >= 2 ) {
                osg::Vec3d point;
                point.x() = as<double>( parts[0], 0.0 );
                point.y() = as<double>( parts[1], 0.0 );
                if ( parts.size() >= 3 ) {
                    point.z() = as<double>( parts[2], 0.0 );
                }
                _geom->push_back(point);
            }
        }
    }

    void KMLPoint::parseCoords( const Config& conf, KMLContext& cx ) {
        _geom = new PointSet();
        KMLGeometry::parseCoords( conf, cx );
    }

    void KMLLineString::parseCoords( const Config& conf, KMLContext& cx ) {
        _geom = new LineString();
        KMLGeometry::parseCoords( conf, cx );
    }

    void KMLLinearRing::parseCoords( const Config& conf, KMLContext& cx ) {
        _geom = new Ring();
        KMLGeometry::parseCoords( conf, cx );
    }

    void KMLPolygon::parseCoords( const Config& conf, KMLContext& cx ) {
        _geom = new Polygon();
        KMLGeometry::parseCoords( conf, cx );
    }

    void KMLMultiGeometry::parseCoords( const Config& conf, KMLContext& cx ) {
        _geom = new MultiGeometry();
        KMLGeometry::parseCoords( conf, cx );
    }

    void KMLFeature::scan( const Config& conf, KMLContext& cx )
    {
        KMLObject::scan(conf, cx);
        for_many( Style, scan, conf, cx );
    }
    void KMLFeature::build( const Config& conf, KMLContext& cx )
    {
        KMLObject::build(conf, cx);
    }

    void KMLPlacemark::build( const Config& conf, KMLContext& cx )
    {
        PlacemarkNode* node = new PlacemarkNode( cx._mapNode );
        
        node->setText(
            conf.hasValue("name") ? conf.value("name") :
            conf.hasValue("description") ? conf.value("description" ) :
            "Unnamed" );

        KMLGeometry geometry;
        geometry.build(conf, cx);
        if ( geometry._geom.valid() && geometry._geom->size() > 0 )
        {
            node->setPosition( (*geometry._geom)[0] );
        }

        cx._groupStack.top()->addChild( node );
    }

    void KMLSchema::scan( const Config& conf, KMLContext& cx ) {
    }

    void KMLDocument::scan( const Config& conf, KMLContext& cx ) {
        KMLContainer::scan(conf, cx);
        for_many    ( Schema, scan, conf, cx );
        for_features( scan, conf, cx );
    }
    void KMLDocument::build( const Config& conf, KMLContext& cx ) {
        KMLContainer::build(conf, cx);
        for_features( build, conf, cx );
    }

    struct KMLRoot : public KMLObject
    {
        virtual void scan( const Config& conf, KMLContext& cx ) {
            KMLObject::scan(conf, cx);
            for_one( Document, scan, conf, cx );
            for_one( NetworkLinkControl, scan, conf, cx );
        }
        virtual void build( const Config& conf, KMLContext& cx ) {
            KMLObject::build(conf, cx);
            for_one( Document, build, conf, cx );
        }
    };
}

//------------------------------------------------------------------------

KMLReader::KMLReader( MapNode* mapNode ) :
_mapNode( mapNode )
{
    //nop
}

osg::Node*
KMLReader::read( std::istream& in, const URIContext& context )
{
    // read the KML from an XML stream:
    osg::ref_ptr<XmlDocument> xml = XmlDocument::load( in, context );
    if ( !xml.valid() )
        return 0L;

    // convert to a config:
    Config config = xml->getConfig();

    osg::Node* node = read( config );
    return node;
}

osg::Node*
KMLReader::read( const Config& conf )
{
    osg::Group* root = new osg::Group();
    root->ref();

    root->setName( conf.uriContext() );

    KMLContext cx;
    cx._mapNode = _mapNode;
    cx._sheet = new StyleSheet();
    cx._groupStack.push( root );

    const Config& kml = conf.child("kml");
    if ( !kml.empty() )
    {
        KMLRoot kmlRoot;
        kmlRoot.scan( kml, cx );    // first pass
        kmlRoot.build( kml, cx );   // second pass.
    }

    return root;
}
