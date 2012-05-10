/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

#include <osg/Notify>
#include <osgViewer/Viewer>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

struct GW : public osgGA::GUIEventHandler {
    GW(Map* m):_m(m){}
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) {
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'r' ) {
            _m->removeModelLayer(_m->getModelLayerAt(0));
            return true;
        }
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == '[' ) {
            _m->getModelLayerAt(0)->setOverlay( !_m->getModelLayerAt(0)->getOverlay() );
            return true;
        }
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'w') {
            dynamic_cast<EarthManipulator*>(dynamic_cast<osgViewer::View*>(aa.asView())->getCameraManipulator())->getSettings()->setCameraFrustumOffsets( osg::Vec2s(0,0) );
            return true;
        }
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'e') {
            dynamic_cast<EarthManipulator*>(dynamic_cast<osgViewer::View*>(aa.asView())->getCameraManipulator())->getSettings()->setCameraFrustumOffsets( osg::Vec2s(250,0) );
            return true;
        }
        
        return false;
    }
    Map* _m;
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--stencil") )
        osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        viewer.setSceneData( node );

        // configure the near/far so we don't clip things that are up close
        viewer.getCamera()->setNearFarRatio(0.00002);

        // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
        // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
        viewer.getDatabasePager()->setDoPreCompile( true );

        viewer.addEventHandler( new GW(MapNode::findMapNode(node)->getMap()));

        dynamic_cast<EarthManipulator*>(viewer.getCameraManipulator())->getSettings()->setMinMaxPitch(-90,0);
        dynamic_cast<EarthManipulator*>(viewer.getCameraManipulator())->getSettings()->setCameraFrustumOffsets( osg::Vec2s(-250,0) );
        return viewer.run();
    }
    else
    {
        OE_NOTICE 
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
    }
}
