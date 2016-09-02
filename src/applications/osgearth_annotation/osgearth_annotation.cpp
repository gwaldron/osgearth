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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarth/MapNode>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#include <osgEarthAnnotation/ImageOverlay>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/RectangleNode>
#include <osgEarthAnnotation/EllipseNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/LocalGeometryNode>
#include <osgEarthAnnotation/FeatureNode>

#include <osgEarthAnnotation/AnnotationEditing>
#include <osgEarthAnnotation/ImageOverlayEditor>

#include <osgEarthSymbology/GeometryFactory>

#include <osgViewer/Viewer>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Util;

//------------------------------------------------------------------

int
usage( char** argv )
{
    OE_WARN << "Usage: " << argv[0] << " <earthfile>" << std::endl;
    return -1;
}

//------------------------------------------------------------------

int
main(int argc, char** argv)
{
    osg::Group* root = new osg::Group();

    // try to load an earth file.
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file and parse demo arguments
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( !node )
        return usage(argv);

    root->addChild( node );

    // find the map node that we loaded.
    MapNode* mapNode = MapNode::findMapNode(node);
    if ( !mapNode )
        return usage(argv);

    // Group to hold all our annotation elements.
    osg::Group* annoGroup = new osg::Group();
    root->addChild( annoGroup );

    // Make a group for labels
    osg::Group* labelGroup = new osg::Group();
    annoGroup->addChild( labelGroup );

    osg::Group* editGroup = new osg::Group();
    root->addChild( editGroup );

    // Style our labels:
    Style labelStyle;
    labelStyle.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    labelStyle.getOrCreate<TextSymbol>()->fill()->color() = Color::Yellow;

    // A lat/long SRS for specifying points.
    const SpatialReference* geoSRS = mapNode->getMapSRS()->getGeographicSRS();

    //--------------------------------------------------------------------

    // A series of place nodes (an icon with a text label)
    {
        Style pm;
        pm.getOrCreate<IconSymbol>()->url()->setLiteral( "../data/placemark32.png" );
        pm.getOrCreate<IconSymbol>()->declutter() = true;
        pm.getOrCreate<TextSymbol>()->halo() = Color("#5f5f5f");

        // bunch of pins:
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -74.00, 40.71), "New York"      , pm));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -77.04, 38.85), "Washington, DC", pm));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS,-118.40, 33.93), "Los Angeles"   , pm));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -71.03, 42.37), "Boston"        , pm));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS,-157.93, 21.35), "Honolulu"      , pm));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, 139.75, 35.68), "Tokyo"         , pm));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -90.25, 29.98), "New Orleans"   , pm));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -80.28, 25.82), "Miami"         , pm));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS,-117.17, 32.72), "San Diego"     , pm));

        // test with an LOD:
        osg::LOD* lod = new osg::LOD();
        lod->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, 14.68, 50.0), "Prague", pm), 0.0, 2e6);
        labelGroup->addChild( lod );

        // absolute altitude:
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -87.65, 41.90, 1000, ALTMODE_ABSOLUTE), "Chicago", pm));
    }

    //--------------------------------------------------------------------

    // a box that follows lines of latitude (rhumb line interpolation, the default)
    {
        Geometry* geom = new Polygon();
        geom->push_back( osg::Vec3d(0,   40, 0) );
        geom->push_back( osg::Vec3d(-60, 40, 0) );
        geom->push_back( osg::Vec3d(-60, 60, 0) );
        geom->push_back( osg::Vec3d(0,   60, 0) );

        Feature* feature = new Feature(geom, geoSRS);
        feature->geoInterp() = GEOINTERP_RHUMB_LINE;

        Style geomStyle;
        geomStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::Cyan;
        geomStyle.getOrCreate<LineSymbol>()->stroke()->width() = 5.0f;
        geomStyle.getOrCreate<LineSymbol>()->tessellationSize() = 75000;
        geomStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        geomStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;
        
        FeatureNode* fnode = new FeatureNode(mapNode, feature, geomStyle);
        
        annoGroup->addChild( fnode );

        labelGroup->addChild( new LabelNode(mapNode, GeoPoint(geoSRS,-30, 50), "Rhumb line polygon", labelStyle) );
    }

    //--------------------------------------------------------------------

    // another rhumb box that crosses the antimeridian
    {
        Geometry* geom = new Polygon();
        geom->push_back( -160., -30. );
        geom->push_back(  150., -20. );
        geom->push_back(  160., -45. );
        geom->push_back( -150., -40. );
        Style geomStyle;

        Feature* feature = new Feature(geom, geoSRS);
        feature->geoInterp() = GEOINTERP_RHUMB_LINE;

        geomStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::Lime;
        geomStyle.getOrCreate<LineSymbol>()->stroke()->width() = 3.0f;
        geomStyle.getOrCreate<LineSymbol>()->tessellationSize() = 75000;
        geomStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        geomStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;

        FeatureNode* gnode = new FeatureNode(mapNode, feature, geomStyle);
        annoGroup->addChild( gnode );

        labelGroup->addChild( new LabelNode(mapNode, GeoPoint(geoSRS, -175, -35), "Antimeridian polygon", labelStyle) );
    }

    //--------------------------------------------------------------------



    // A path using great-circle interpolation.
    // Keep a pointer to it so we can modify it later on.
    FeatureNode* pathNode = 0;
    {
        Geometry* path = new LineString();
        path->push_back( osg::Vec3d(-74, 40.714, 0) );   // New York
        path->push_back( osg::Vec3d(139.75, 35.68, 0) ); // Tokyo

        Feature* pathFeature = new Feature(path, geoSRS);
        pathFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;

        Style pathStyle;
        pathStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::White;
        pathStyle.getOrCreate<LineSymbol>()->stroke()->width() = 1.0f;
        pathStyle.getOrCreate<LineSymbol>()->tessellationSize() = 75000;
        pathStyle.getOrCreate<PointSymbol>()->size() = 5;
        pathStyle.getOrCreate<PointSymbol>()->fill()->color() = Color::Red;
        pathStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        pathStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;

        //OE_INFO << "Path extent = " << pathFeature->getExtent().toString() << std::endl;

        pathNode = new FeatureNode(mapNode, pathFeature, pathStyle);
        annoGroup->addChild( pathNode );

        labelGroup->addChild( new LabelNode(mapNode, GeoPoint(geoSRS,-170, 61.2), "Great circle path", labelStyle) );
    }

    //--------------------------------------------------------------------

    // Two circle segments around New Orleans.
    {
        Style circleStyle;
        circleStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Cyan, 0.5);
        circleStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        circleStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

        CircleNode* circle = new CircleNode(
            mapNode,
            GeoPoint(geoSRS, -90.25, 29.98, 1000., ALTMODE_RELATIVE),
            Distance(300, Units::KILOMETERS),
            circleStyle, Angle(-45.0, Units::DEGREES), Angle(45.0, Units::DEGREES), true);
        annoGroup->addChild( circle );

        editGroup->addChild( new CircleNodeEditor(circle) );
    }

	{
		Style circleStyle;
		circleStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Red, 0.5);
		circleStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
		circleStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

		CircleNode* circle = new CircleNode(
			mapNode,
			GeoPoint(geoSRS, -90.25, 29.98, 1000., ALTMODE_RELATIVE),
			Distance(300, Units::KILOMETERS),
			circleStyle, Angle(45.0, Units::DEGREES), Angle(360.0 - 45.0, Units::DEGREES), true);
		annoGroup->addChild( circle );

        editGroup->addChild( new CircleNodeEditor(circle) );
	}

    //--------------------------------------------------------------------

    // An extruded ellipse around Miami.
    {
        Style ellipseStyle;
        ellipseStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Orange, 0.75);
        ellipseStyle.getOrCreate<ExtrusionSymbol>()->height() = 250000.0; // meters MSL
        EllipseNode* ellipse = new EllipseNode(
            mapNode, 
            GeoPoint(geoSRS, -80.28, 25.82, 0.0, ALTMODE_RELATIVE),
            Distance(250, Units::MILES),
            Distance(100, Units::MILES),
            Angle   (0, Units::DEGREES),
            ellipseStyle,
            Angle(45.0, Units::DEGREES),
            Angle(360.0 - 45.0, Units::DEGREES), 
            true);
        annoGroup->addChild( ellipse );

        editGroup->addChild( new EllipseNodeEditor(ellipse) );
    }
	{
		Style ellipseStyle;
		ellipseStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Blue, 0.75);
		ellipseStyle.getOrCreate<ExtrusionSymbol>()->height() = 250000.0; // meters MSL
		EllipseNode* ellipse = new EllipseNode(
			mapNode, 
			GeoPoint(geoSRS, -80.28, 25.82, 0.0, ALTMODE_RELATIVE),
			Distance(250, Units::MILES),
			Distance(100, Units::MILES),
			Angle   (0, Units::DEGREES),
			ellipseStyle, 
            Angle(-40.0, Units::DEGREES), 
            Angle(40.0, Units::DEGREES), 
            true);
		annoGroup->addChild( ellipse );

        editGroup->addChild( new EllipseNodeEditor(ellipse) );
	}
    
    //--------------------------------------------------------------------

    {
        // A rectangle around San Diego
        Style rectStyle;
        rectStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Green, 0.5);
        rectStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        rectStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
        RectangleNode* rect = new RectangleNode(
            mapNode, 
            GeoPoint(geoSRS, -117.172, 32.721),
            Distance(300, Units::KILOMETERS ),
            Distance(600, Units::KILOMETERS ),
            rectStyle);
        annoGroup->addChild( rect );

        editGroup->addChild( new RectangleNodeEditor(rect) );
    }    

    //--------------------------------------------------------------------

    // An extruded polygon roughly the shape of Utah. Here we demonstrate the
    // FeatureNode, where you create a geographic geometry and use it as an
    // annotation.
    {
        Geometry* utah = new Polygon();
        utah->push_back( -114.052, 37.0   );
        utah->push_back( -109.054, 37.0   );
        utah->push_back( -109.054, 41.0   );
        utah->push_back( -111.040, 41.0   );
        utah->push_back( -111.080, 42.059 );
        utah->push_back( -114.080, 42.024 );

        Style utahStyle;
        utahStyle.getOrCreate<ExtrusionSymbol>()->height() = 250000.0; // meters MSL
        utahStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::White, 0.8);

        Feature*     utahFeature = new Feature(utah, geoSRS);
        FeatureNode* featureNode = new FeatureNode(mapNode, utahFeature, utahStyle);
        annoGroup->addChild( featureNode );
    }

    //--------------------------------------------------------------------

    // an image overlay.
    {
        ImageOverlay* imageOverlay = 0L;
        osg::Image* image = osgDB::readImageFile( "../data/USFLAG.TGA" );
        if ( image )
        {
            imageOverlay = new ImageOverlay(mapNode, image);
            imageOverlay->setBounds( Bounds( -100.0, 35.0, -90.0, 40.0) );
            annoGroup->addChild( imageOverlay );

            editGroup->addChild( new ImageOverlayEditor(imageOverlay) );
        }
    }

    //--------------------------------------------------------------------

    // initialize the viewer:    
    viewer.setSceneData( root );    
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    return viewer.run();
}
