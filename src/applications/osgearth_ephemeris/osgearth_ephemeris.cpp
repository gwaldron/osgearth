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

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/NodeUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Ephemeris>
#include <osgEarthUtil/Sky>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/Controls>
#include <osgEarthAnnotation/PlaceNode>

#define LC "[osgearth_ephemeris] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

namespace ui = osgEarth::Util::Controls;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " <file.earth> --sky" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

struct App
{
    osg::ref_ptr<PlaceNode> sunPos;
    osg::ref_ptr<PlaceNode> moonPos;
    SkyNode* sky;
};


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    if ( arguments.find("--sky") < 0 )
        return usage(argv[0]);

    osgViewer::Viewer viewer(arguments);

    EarthManipulator* em = new EarthManipulator(arguments);
    em->getSettings()->setMinMaxPitch(-89, 89);
    viewer.setCameraManipulator( em );

    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    osg::ref_ptr<osg::Image> mark = osgDB::readRefImageFile("../data/placemark32.png");
    
    App app;

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        osg::Group* root = new osg::Group();
        root->addChild( node );

        MapNode* mapNode = MapNode::get(node);

        app.sunPos = new PlaceNode(mapNode, GeoPoint(), mark.get(), "Sun");
        app.sunPos->setDynamic(true);
        mapNode->addChild( app.sunPos.get() );

        app.moonPos = new PlaceNode(mapNode, GeoPoint(), mark.get(), "Moon");
        app.moonPos->setDynamic(true);

        mapNode->addChild( app.moonPos.get() );        


        app.sky = osgEarth::findTopMostNodeOfType<SkyNode>(node);        
        const Ephemeris* ephemeris = 0L;
        if ( app.sky )
        {
            ephemeris = app.sky->getEphemeris();
        }

        LatLongFormatter llf;
        llf.setOptions( LatLongFormatter::Options(llf.FORMAT_DEGREES_MINUTES_SECONDS) );
        llf.setPrecision( 8 );

        viewer.setSceneData( root );

        CelestialBody sun, moon;

        while(!viewer.done())
        {
            viewer.frame();

            sun._observer = GeoPoint(SpatialReference::get("wgs84"), 0.0, 0.0, 0.0);

            app.sky->setDateTime(DateTime(1990, 4, 19, 0));
            //app.sky->setDateTime(DateTime(2008, 4, 24, 10));

            if ( ephemeris )
            {
                const DateTime& dt = app.sky->getDateTime();

                ephemeris->getSunPosition(dt, sun);
                GeoPoint sunPos;
                sunPos.fromWorld(mapNode->getMapSRS(), sun._geocentric);
                sunPos.alt() = 0.0;
                app.sunPos->setPosition( sunPos );
                app.sunPos->setText( "Sun\n" + llf.format(sunPos) );

                ephemeris->getMoonPosition(dt, moon);
                GeoPoint moonPos;
                moonPos.fromWorld(mapNode->getMapSRS(), moon._geocentric);
                moonPos.alt() = 0.0;
                app.moonPos->setPosition( moonPos );
                app.moonPos->setText( "Moon\n" + llf.format(moonPos) );

                if (viewer.getFrameStamp()->getFrameNumber() == 60)
                {
                    OE_INFO 
                        << "Sun RA = " << sun._rightAscension.as(Units::DEGREES)
                        << ", DECL = " << sun._declination.as(Units::DEGREES)
                        << ", Earth Lat = " << sun._latitude.as(Units::DEGREES)
                        << ", Earth Lon = " << sun._longitude.as(Units::DEGREES)
                        << ", Azimuth = " << sun._topoAzimuth.as(Units::DEGREES)
                        << ", Elev = " << sun._topoElevation.as(Units::DEGREES)
                        << std::endl;
                }
            }
        }
    }
    else
    {
        return usage(argv[0]);
    }
}
