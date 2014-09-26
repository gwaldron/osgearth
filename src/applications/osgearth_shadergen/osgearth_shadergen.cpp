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

#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/Optimizer>

#include <osgEarth/ShaderGenerator>
#include <osgEarth/StringUtils>
#include <osgEarth/Registry>
#include <osgEarth/StateSetCache>

#define LC "[shadergen] "

using namespace osgEarth;

int
usage(const char* name)
{
    OE_NOTICE << "\nUsage: " << name << " file" << std::endl;
    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") || argc < 2 )
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    StateSetCache* cache = Registry::stateSetCache();
    cache->setMaxSize(INT_MAX);

    // load the file
    osg::Node* node = osgDB::readNodeFile(
        Stringify() << argv[1] << ".osgearth_shadergen" );

    if ( !node )
        return usage(argv[0]);

    if ( cache )
        cache->dumpStats();

#if 0
    osgUtil::Optimizer o;
    o.optimize( node,
        osgUtil::Optimizer::INDEX_MESH |
        osgUtil::Optimizer::VERTEX_PRETRANSFORM |
        osgUtil::Optimizer::VERTEX_POSTTRANSFORM );
#endif

    viewer.setSceneData( node );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.run();
    return 0;
}
