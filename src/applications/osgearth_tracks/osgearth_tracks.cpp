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
#include <osgEarth/MapNode>
#include <osgEarth/Random>
#include <osgEarth/StringUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/GeoMath>
#include <osgEarth/Units>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Formatters>
#include <osgEarthAnnotation/TrackNode>
#include <osgEarthAnnotation/Decluttering>
#include <osgEarthSymbology/Color>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osg/CoordinateSystemNode>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;

#define LC "[osgearth_tracks] "

/**
 * Demonstrates use of the TrackNode to display entity track symbols.
 */

// field names
#define FIELD_NAME     "name"
#define FIELD_POSITION "position"
#define FIELD_QUANTITY "quantity"

#define ICON_URL       "../data/m2525_air.png"
#define ICON_SIZE      40

/** Prints an error message */
int
usage( const std::string& message )
{
    OE_WARN << LC << message << std::endl;
    return -1;
}


/** A little track simulator that goes a simple great circle interpolation */
struct TrackSim : public osg::Referenced
{
    TrackNode* _track;
    Angular _startLat, _startLon, _endLat, _endLon;

    void update( double t )
    {
        osg::Vec3d pos;
        GeoMath::interpolate(
            _startLat.as(Units::RADIANS), _startLon.as(Units::RADIANS),
            _endLat.as(Units::RADIANS), _endLon.as(Units::RADIANS),
            t,
            pos.y(), pos.x() );

        pos.x() = osg::RadiansToDegrees(pos.x());
        pos.y() = osg::RadiansToDegrees(pos.y());

        _track->setPosition(pos);
    }
};
typedef std::list< osg::ref_ptr<TrackSim> > TrackSims;



/**
 * Creates a field schema that we'll later use as a labeling template for
 * TrackNode instances.
 */
void
createFieldSchema( TrackNodeFieldSchema& schema )
{
    TextSymbol* nameSymbol = new TextSymbol();
    nameSymbol->pixelOffset()->set( 0, 2+ICON_SIZE/2 );  // above the icon
    nameSymbol->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;
    nameSymbol->halo()->color() = Color::Black;
    schema[FIELD_NAME] = nameSymbol;

    TextSymbol* posSymbol = new TextSymbol();
    posSymbol->pixelOffset()->set( 0, -2-ICON_SIZE/2 );  // below the icon
    posSymbol->alignment() = TextSymbol::ALIGN_CENTER_TOP;
    posSymbol->halo()->color() = Color::Black;
    schema[FIELD_POSITION] = posSymbol;

    TextSymbol* quantitySymbol = new TextSymbol();
    quantitySymbol->pixelOffset()->set( -2-ICON_SIZE/2, 0 ); // left side
    quantitySymbol->alignment() = TextSymbol::ALIGN_RIGHT_CENTER;
    quantitySymbol->halo()->color() = Color::Black;
    schema[FIELD_QUANTITY] = quantitySymbol;
}

void
createTrackNodes( MapNode* mapNode, osg::Group* parent, const TrackNodeFieldSchema& schema, TrackSims& sims )
{
    // load an icon to use:
    osg::ref_ptr<osg::Image> srcImage = osgDB::readImageFile( ICON_URL );
    osg::ref_ptr<osg::Image> image;
    ImageUtils::resizeImage( srcImage.get(), ICON_SIZE, ICON_SIZE, image );

    // make some tracks.
    Random prng;
    LatLongFormatter llf(LatLongFormatter::FORMAT_DECIMAL_DEGREES, 2);

    for( unsigned i=0; i<100; ++i )
    {
        double lon0 = -180.0 + prng.next() * 360.0;
        double lat0 = -80.0 + prng.next() * 160.0;

        TrackNode* track = new TrackNode( mapNode, osg::Vec3d(lon0, lat0, 0), image, schema );

        track->setFieldValue( FIELD_NAME,     Stringify() << "Track-" << i );
        track->setFieldValue( FIELD_POSITION, Stringify() << llf.format(lat0) << " " << llf.format(lon0) );
        track->setFieldValue( FIELD_QUANTITY, Stringify() << (1 + prng.next(9)) );

        track->getOrCreateStateSet()->setRenderBinDetails( INT_MAX, OSGEARTH_DECLUTTER_BIN );

        parent->addChild( track );

        // add a simulator for this guy
        double lon1 = -180.0 + prng.next() * 360.0;
        double lat1 = -80.0 + prng.next() * 160.0;
        TrackSim* sim = new TrackSim();
        sim->_track = track;
        sim->_startLat = lat0; sim->_startLon = lon0;
        sim->_endLat = lat1; sim->_endLon = lon1;
        sims.push_back( sim );
    }
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // load a map from an earth file.
    MapNode* mapNode = MapNode::load( arguments );
    if ( !mapNode )
        return usage( "Missing required .earth file" );
    
    osg::Group* root = new osg::Group();
    root->addChild( mapNode );

    // build a track field schema.
    TrackNodeFieldSchema schema;
    createFieldSchema( schema );

    // a list of simulators for our tracks.
    TrackSims trackSims;
    double simDuration = 30.0; // seconds

    // create some track nodes.
    createTrackNodes( mapNode, root, schema, trackSims );

    // initialize a viewer.
    osgViewer::Viewer viewer( arguments );
    viewer.setCameraManipulator( new EarthManipulator );
    viewer.setSceneData( root );

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    while( !viewer.done() )
    {
        viewer.frame();

        double t = fmod(viewer.getFrameStamp()->getSimulationTime(), simDuration) / simDuration;

        for( TrackSims::iterator i = trackSims.begin(); i != trackSims.end(); ++i )
        {
            i->get()->update( t );
        }
    }
}
