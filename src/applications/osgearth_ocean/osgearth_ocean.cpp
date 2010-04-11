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
#include <osgEarth/Notify>

#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Projection>
#include <osg/AutoTransform>
#include <osg/Geometry>
#include <osg/Image>
#include <osg/CullFace>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/FindNode>
#include <osgEarth/MapNode>

#include <osgEarthUtil/OceanSurfaceNode>



#include <osgTerrain/TerrainTile>
#include <osgTerrain/GeometryTechnique>

#include <osgDB/WriteFile>

#include <osgText/Text>

#include <iostream>
#include <sstream>

using namespace osg;
using namespace osgDB;
using namespace osgTerrain;
using namespace osgEarth;
using namespace osgEarthUtil;


// An event handler that will print out the elevation at the clicked point
struct MyEventHandler : public osgGA::GUIEventHandler 
{
    MyEventHandler( OceanSurfaceNode* ocean )
        :_ocean(ocean) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN )
        {
            switch (ea.getKey())
            {
            case 'h':
                {
                    _ocean->setWaveHeight( _ocean->getWaveHeight() * 1.1 );
                }
                break;
            case 'H':
                {
                    _ocean->setWaveHeight( _ocean->getWaveHeight() * 0.9 );
                }
                break;
            case 'p':
                {
                    _ocean->setPeriod( _ocean->getPeriod() * 1.1 );
                }
                break;
            case 'P':
                {
                    _ocean->setPeriod( _ocean->getPeriod() * 0.9 );
                }
                break;
            case 'e':
                {
                    _ocean->setEnabled( !_ocean->getEnabled() );
                }
            case 'i':
                {
                    _ocean->setInvertMask( !_ocean->getInvertMask() );
                }
                break;
            }
        }
        return false;
    }

    osg::ref_ptr< OceanSurfaceNode > _ocean;
};




int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    {
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
        keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
        keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );
        keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );
        keyswitchManipulator->addMatrixManipulator( '5', "Spherical", new osgGA::SphericalManipulator() );

        std::string pathfile;
        char keyForAnimationPath = '6';
        while (arguments.read("-p",pathfile))
        {
            osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
            if (apm || !apm->valid()) 
            {
                unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
                keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
                keyswitchManipulator->selectMatrixManipulator(num);
                ++keyForAnimationPath;
            }
        }

        viewer.setCameraManipulator( keyswitchManipulator.get() );
    }

    osg::Group* group = new osg::Group;

    osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);   

    OceanSurfaceNode* ocean = new OceanSurfaceNode();
    ocean->setOceanMaskTextureUnit( 1 );
    group->addChild( loadedModel );

    //Find the MapNode and add the ocean decorator to the terrain group
    osgEarth::MapNode* mapNode = findTopMostNodeOfType<MapNode>(loadedModel.get());
    mapNode->addTerrainDecorator( ocean );


    viewer.addEventHandler(new MyEventHandler(ocean));

    loadedModel->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);


    // set the scene to render
    viewer.setSceneData(group);

    // run the viewers frame loop
    return viewer.run();
}
