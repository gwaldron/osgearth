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
#include <osgEarthFeatures/MarkerFactory>
#include <osgEarthFeatures/FeatureNode>
#include <osgEarthUtil/Annotation>
#include <osgEarth/XMLUtils>
#include <osg/PagedLOD>
#include <stack>
#include <iterator>

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
        virtual void scan2( const Config& conf, KMLContext& cx ) { }
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
        virtual void scan2( const Config& conf, KMLContext& cx );
    };

    struct KMLGeometry : public KMLObject {
        KMLGeometry() : _extrude(false), _tessellate(false) { }
        virtual void parseCoords( const Config& conf, KMLContext& cx );
        virtual void parseStyle( const Config& conf, KMLContext& cs, Style& style );
        virtual void build( const Config& conf, KMLContext& cx, Style& style );
        osg::ref_ptr<Geometry> _geom;
        bool _extrude, _tessellate;
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
        virtual void parseCoords( const Config& conf, KMLContext& cx );
    };

    struct KMLPolygon : public KMLGeometry {
        //virtual void scan( const Config& conf, KMLContext& cx );
        virtual void parseStyle( const Config& conf, KMLContext& cx, Style& style);
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
        virtual void build( const Config& conf, KMLContext& cx );
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
            std::string iconHref = conf.child("icon").value("href");
            if ( !iconHref.empty() )
            {
                marker->url() = URI( Stringify() << "image(" << iconHref << ")", conf.uriContext() );
            }

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
        if ( !conf.empty() )
        {
            LineSymbol* line = style.getOrCreate<LineSymbol>();
            if ( conf.hasValue("color") ) {
                line->stroke()->color() = Color( Stringify() << "#" << conf.value("color"), Color::ABGR );
            }
            if ( conf.hasValue("width") ) {
                line->stroke()->width() = as<float>( conf.value("width"), 1.0f );
            }
        }
    }

    void KMLPolyStyle::scan( const Config& conf, Style& style )
    {
        if ( !conf.empty() )
        {
            bool fill = true;
            if ( conf.hasValue("fill") ) {
                fill = as<int>(conf.value("fill"), 1) == 1;
            }

            bool outline = false;
            if ( conf.hasValue("outline") ) {
                outline = as<int>(conf.value("outline"), 0) == 1;
            }

            Color color(Color::White);
            if ( conf.hasValue("color") ) {
                color = Color( Stringify() << "#" << conf.value("color"), Color::ABGR );
            }

            if ( fill ) {
                PolygonSymbol* poly = style.getOrCreate<PolygonSymbol>();
                poly->fill()->color() = color;
            }
            else {
                LineSymbol* line = style.getOrCreate<LineSymbol>();
                line->stroke()->color() = color;
            }
        }
    }

    void KMLStyle::scan( const Config& conf, KMLContext& cx )
    {
        Style style( conf.value("id") );

        KMLIconStyle icon;
        icon.scan( conf.child("iconstyle"), style );

        KMLLabelStyle label;
        label.scan( conf.child("labelstyle"), style );

        KMLLineStyle line;
        line.scan( conf.child("linestyle"), style );

        KMLPolyStyle poly;
        poly.scan( conf.child("polystyle"), style );

        cx._sheet->addStyle( style ); 
    }

    void KMLStyleMap::scan2( const Config& conf, KMLContext& cx )
    {
        const Config& pair = conf.child("pair");
        if ( !pair.empty() )
        {
            const std::string& url = pair.value("styleurl" );
            if ( !url.empty() ) {
                const Style* style = cx._sheet->getStyle( url );
                if ( style ) {
                    Style aliasStyle = *style;
                    aliasStyle.setName( conf.value("id") );
                    cx._sheet->addStyle( aliasStyle );
                }
            }
        }
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
        group->setName( conf.value("name") );
        cx._groupStack.top()->addChild( group );
        cx._groupStack.push( group );

        KMLContainer::build(conf, cx);
        for_features(build, conf, cx);

        cx._groupStack.pop();
    }

    void KMLGeometry::build( const Config& conf, KMLContext& cx, Style& style)
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
            g.parseStyle(conf.child("polygon"), cx, style);
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

    void KMLGeometry::parseStyle( const Config& conf, KMLContext& cs, Style& style )
    {
        _extrude = conf.value("extrude") == "1";
        _tessellate = conf.value("tessellate") == "1";

        std::string am = conf.value("altitudemode");
        if ( am.empty() || am == "clampToGround" )
        {
            AltitudeSymbol* af = style.getOrCreate<AltitudeSymbol>();
            af->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
            _extrude = false;
        }
        else if ( am == "relativeToGround" )
        {
            AltitudeSymbol* af = style.getOrCreate<AltitudeSymbol>();
            af->clamping() = AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
        }
        else if ( am == "absolute" )
        {
            AltitudeSymbol* af = style.getOrCreate<AltitudeSymbol>();
            af->clamping() = AltitudeSymbol::CLAMP_ABSOLUTE;
        }

        if ( _extrude )
        {
            ExtrusionSymbol* es = style.getOrCreate<ExtrusionSymbol>();
            es->flatten() = false;
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

    void KMLPolygon::parseStyle(const Config& conf, KMLContext& cx, Style& style)
    {
        KMLGeometry::parseStyle(conf, cx, style);
    }

    void KMLPolygon::parseCoords( const Config& conf, KMLContext& cx )
    {
        Polygon* poly = new Polygon();
        
        Config outerConf = conf.child("outerboundaryis");
        if ( !outerConf.empty() )
        {
            Config outerRingConf = outerConf.child("linearring");
            if ( !outerRingConf.empty() )
            {
                KMLLinearRing outer;
                outer.parseCoords( outerRingConf, cx );
                if ( outer._geom.valid() ) {
                    dynamic_cast<Ring*>(outer._geom.get())->rewind( Ring::ORIENTATION_CCW );
                    poly->reserve( outer._geom->size() );
                    std::copy( outer._geom->begin(), outer._geom->end(), std::back_inserter(*poly) );
                }
            }

            ConfigSet innerConfs = conf.children("innerboundaryis");
            for( ConfigSet::const_iterator i = innerConfs.begin(); i != innerConfs.end(); ++i )
            {
                Config innerRingConf = i->child("linearring");
                if ( !innerRingConf.empty() )
                {
                    KMLLinearRing inner;
                    inner.parseCoords( innerRingConf, cx );
                    if ( inner._geom.valid() ) {
                        Geometry* innerGeom = inner._geom.get();
                        dynamic_cast<Ring*>(innerGeom)->rewind( Ring::ORIENTATION_CW );
                        poly->getHoles().push_back( dynamic_cast<Ring*>(innerGeom) );
                    }
                }
            }
        }

        _geom = poly;
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
        Style style;
        if ( conf.hasValue("styleurl") )
        {
            const Style* ref_style = cx._sheet->getStyle( conf.value("styleurl"), false );
            if ( ref_style )
                style = *ref_style;
        }

        URI iconURI;
        MarkerSymbol* marker = style.get<MarkerSymbol>();
        if ( marker && marker->url().isSet() )
        {
            MarkerFactory mf;
            iconURI = mf.getRawURI( marker );
        }

        std::string text = 
            conf.hasValue("name") ? conf.value("name") :
            conf.hasValue("description") ? conf.value("description") :
            "Unnamed";

        // read in the geometry:
        osg::Vec3d position;
        KMLGeometry geometry;
        geometry.build(conf, cx, style);
        if ( geometry._geom.valid() && geometry._geom->size() > 0 )
        {
            Geometry* geom = geometry._geom.get();
            position = geom->getBounds().center();
        }

        //PlacemarkNode* pmNode = new PlacemarkNode( cx._mapNode, position, iconURI, text, style );
        //cx._groupStack.top()->addChild( pmNode );

        // if we have a non-single-point geometry, render it.
        if ( geometry._geom.valid() && geometry._geom->size() != 1 )
        {   
            const ExtrusionSymbol* ex = style.get<ExtrusionSymbol>();
            const AltitudeSymbol* alt = style.get<AltitudeSymbol>();

            bool draped =
                ex == 0L &&
                (alt && alt->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN);


            FeatureNode* fNode = new FeatureNode( cx._mapNode, new Feature(geometry._geom.get()), draped );
            fNode->setStyle( style );
            // drape if we're not extruding.
            fNode->setDraped( draped );
            cx._groupStack.top()->addChild( fNode );
        }
    }

    void KMLSchema::scan( const Config& conf, KMLContext& cx ) {
    }

    void KMLNetworkLink::build( const Config& conf, KMLContext& cx )
    {
        std::string name = conf.value("name");
        
        // parse the bounds:
        const Config& regionConf = conf.child("region");
        if ( regionConf.empty() )
            return;
        const Config& llaBoxConf = regionConf.child("latlonaltbox");
        if ( llaBoxConf.empty() )
            return;
        GeoExtent llaExtent(
            cx._mapNode->getMap()->getProfile()->getSRS()->getGeographicSRS(),
            conf.value<double>(regionConf.value("west"),  0.0),
            conf.value<double>(regionConf.value("south"), 0.0),
            conf.value<double>(regionConf.value("east"),  0.0),
            conf.value<double>(regionConf.value("north"), 0.0) );
        GeoExtent mapExtent = llaExtent.transform( cx._mapNode->getMap()->getProfile()->getSRS() );
        double x, y;
        mapExtent.getCentroid( x, y );
        osg::Vec3d lodCenter;
        cx._mapNode->getMap()->mapPointToWorldPoint( osg::Vec3d(x,y,0), lodCenter );

        // parse the LOD ranges:
        float minRange = 0, maxRange = 1e6;
        const Config& lodConf = conf.child("lod");
        if ( !lodConf.empty() ) 
        {
            // swapped
            maxRange = conf.value<float>( "minlodpixels", maxRange );
            minRange = conf.value<float>( "maxlodpixels", minRange );
        }
        
        // parse the link:
        const Config& linkConf = conf.child("link");
        if ( linkConf.empty() )
            return;
        std::string href = linkConf.value("href");
        if ( href.empty() )
            return;

        // build the node
        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setRangeMode( osg::LOD::PIXEL_SIZE_ON_SCREEN );
        if ( !endsWith(href, ".kml") )
            href += "&.kml";
        plod->setFileName( 0, href );
        plod->setRange( 0, minRange, maxRange );
        plod->setCenter( lodCenter );
        osgDB::Options* options = new osgDB::Options();
        options->setPluginData( "osgEarth::MapNode", cx._mapNode );
        plod->setDatabaseOptions( options );
        cx._groupStack.top()->addChild( plod );
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
        kmlRoot.scan2( kml, cx );   // second pass
        kmlRoot.build( kml, cx );   // third pass.
    }

    return root;
}
