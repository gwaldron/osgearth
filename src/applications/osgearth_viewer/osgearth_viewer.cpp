/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#include <osgEarth/Units>
#include <osgEarth/Viewpoint>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

int
main(int argc, char** argv)
{
#if 0
    Distance range(100.0, Units::CENTIMETERS);
    Config c = range.getConfig();
    c.key() = "distance";
    OE_WARN << c.toJSON() << "\n";
    Distance rangeIn(c);
    std::string parseable = rangeIn.asParseableString();
    
    optional<Distance> range3;
    c.getIfSet("distance", range3);
    OE_WARN << range3->asParseableString() << "\n";

    Viewpoint vp;
    vp.range()->set( 12324.0, Units::KILOMETERS );

    c = vp.getConfig();

    optional<Distance> d;
    c.getIfSet("range", d);
    OE_WARN << "D.iset = " << d.isSet() << "\n";
    OE_WARN << "D.value = " << d->asParseableString() << "\n";

    Viewpoint vp2(c);

    OE_WARN << "VP COnfig = " << c.toJSON() << "\n";
    OE_WARN << "VP2 Config = " << vp2.getConfig().toJSON() << "\n";

    return 0;
#endif


    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    if ( arguments.read("--stencil") )
        osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator() );    

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        viewer.setSceneData( node );

        viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
