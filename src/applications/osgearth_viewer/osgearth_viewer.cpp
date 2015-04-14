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
#include <osgEarthFeatures/FeatureSourceIndexNode>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;


struct FeaturePickCallback : public FeaturePicker::Callback
{
    void onPick(const std::set<FeatureID>& results)
    {
        for(std::set<FeatureID>::const_iterator i = results.begin(); i != results.end(); ++i)
        {
            OE_NOTICE << "Picked feature " << *i << "\n";
        }
    }
};

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

        viewer.getCamera()->setNearFarRatio(0.00002);
        viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

        FeaturePicker* picker = new FeaturePicker();
        viewer.addEventHandler( picker );
        picker->setPickGraph( MapNode::get(node)->getModelLayerGroup() );
        picker->setCallback( new FeaturePickCallback() );

        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
