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


#include <osg/Notify>
#include <osg/Material>
#include <osg/PolygonOffset>
#include <osg/PolygonMode>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Viewpoint>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthDrivers/tms/TMSOptions>

using namespace osgEarth::Drivers;

// some preset viewpoints.
static osgEarthUtil::Viewpoint VPs[] = {
    osgEarthUtil::Viewpoint( "Africa",        osg::Vec3d(    0.0,   0.0, 0.0 ), 0.0, -90.0, 10e6 ),
    osgEarthUtil::Viewpoint( "California",    osg::Vec3d( -121.0,  34.0, 0.0 ), 0.0, -90.0, 6e6 ),
    osgEarthUtil::Viewpoint( "Europe",        osg::Vec3d(    0.0,  45.0, 0.0 ), 0.0, -90.0, 4e6 ),
    osgEarthUtil::Viewpoint( "Washington DC", osg::Vec3d(  -77.0,  38.0, 0.0 ), 0.0, -90.0, 1e6 ),
    osgEarthUtil::Viewpoint( "Australia",     osg::Vec3d(  135.0, -20.0, 0.0 ), 0.0, -90.0, 2e6 )
};

// a simple handler that demonstrates the "viewpoint" functionality in 
// osgEarthUtil::EarthManipulator. Press a number key to fly to a viewpoint.
struct FlyToViewpointHandler : public osgGA::GUIEventHandler 
{
    FlyToViewpointHandler( osgEarthUtil::EarthManipulator* manip ) : _manip(manip) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() >= '1' && ea.getKey() <= '5' )
        {
            _manip->setViewpoint( VPs[ea.getKey()-'1'], 4.0 );
        }
        return false;
    }

    osg::observer_ptr<osgEarthUtil::EarthManipulator> _manip;
};


class KeyboardEventHandler : public osgGA::GUIEventHandler
{
public:
    
    KeyboardEventHandler(osgEarth::MapNode* mapNode) : _mapNode(mapNode)
    {}
    
    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
    {
        switch(ea.getEventType())
        {
            case(osgGA::GUIEventAdapter::KEYDOWN):
            {
                if (ea.getKey()=='n')
                {
                  osg::StateSet* stateset = new osg::StateSet;
                  osg::PolygonOffset* polyoffset = new osg::PolygonOffset;
                  polyoffset->setFactor(-1.0f);
                  polyoffset->setUnits(-1.0f);
                  osg::PolygonMode* polymode = new osg::PolygonMode;
                  polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
                  stateset->setAttributeAndModes(polyoffset,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
                  stateset->setAttributeAndModes(polymode,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
                  osg::Material* material = new osg::Material;
                  stateset->setAttributeAndModes(material,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
                  stateset->setMode(GL_LIGHTING,osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);
                  _mapNode->setStateSet(stateset);
                    return true;
                }
                if (ea.getKey()=='p')
                {
                    _mapNode->setStateSet(new osg::StateSet());
                    return true;
                }
                break;
            }
            default:
                break;
        }
        return false;
    }

private:

    osgEarth::MapNode* _mapNode;
};

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    // install the programmable manipulator.
    osgEarthUtil::EarthManipulator* manip = new osgEarthUtil::EarthManipulator();

    osg::Node* sceneData = osgDB::readNodeFiles( arguments );

    if (!sceneData)
    {
        // Create a "Map" dynamically if no nodes were loaded
        // The "Map" is the data model object that we will be visualizing. It will be
        // geocentric by default, but you can specify a projected map in the constructor.
        osgEarth::Map* map = new osgEarth::Map();

       // create transfer function
        {
          osg::TransferFunction1D* tf(new osg::TransferFunction1D());
          
          tf->setColor(-10000,osg::Vec4(        0.0,        0.0,120.0/255.0,1.0));
          tf->setColor(     0,osg::Vec4(        0.0,        0.0,        1.0,1.0)); // everything till 0m is interpreted as sea
          tf->setColor(  1e-6,osg::Vec4(        0.0,150.0/255.0,        0.0,1.0)); // everything above 0m is interpreted as land
          tf->setColor(   500,osg::Vec4(        0.0,200.0/255.0,        0.0,1.0));
          tf->setColor(  1000,osg::Vec4(110.0/255.0,210.0/255.0,        0.0,1.0));
          tf->setColor(  1500,osg::Vec4(220.0/255.0,220.0/255.0,        0.0,1.0));
          tf->setColor(  2000,osg::Vec4(220.0/255.0,197.0/255.0,        0.0,1.0));
          tf->setColor(  2500,osg::Vec4(220.0/255.0,172.0/255.0,        0.0,1.0));
          tf->setColor(  3000,osg::Vec4(220.0/255.0,160.0/255.0,        0.0,1.0));
          tf->setColor(  3500,osg::Vec4(185.0/255.0,155.0/255.0, 75.0/255.0,1.0));
          tf->setColor(  4000,osg::Vec4(170.0/255.0,155.0/255.0,110.0/255.0,1.0));
          tf->setColor(  4500,osg::Vec4(150.0/255.0,150.0/255.0,150.0/255.0,1.0));
          tf->setColor(  5000,osg::Vec4(190.0/255.0,190.0/255.0,190.0/255.0,1.0));
          tf->setColor(  5500,osg::Vec4(230.0/255.0,230.0/255.0,230.0/255.0,1.0));
          tf->setColor(  6000,osg::Vec4(250.0/255.0,250.0/255.0,250.0/255.0,1.0));
          tf->setColor(  9000,osg::Vec4(        1.0,        1.0,        1.0,1.0));
          osg::notify(osg::NOTICE) << "TransferFunction1D - min: " << tf->getMinimum() << "; max: " << tf->getMaximum() << std::endl;
          osg::notify(osg::NOTICE) << "Image size: " << tf->getImage()->s() << std::endl;
          map->setContourTransferFunction(tf);
        }
        // Add a heightfield layer to the map. You can add any number of heightfields and
        // osgEarth will composite them automatically.
        {
            osg::ref_ptr<TMSOptions> tms = new TMSOptions();
            tms->tileSize() = 256;
            tms->url() = "http://demo.pelicanmapping.com/rmweb/data/srtm30_plus_tms/tms.xml";
            map->addMapLayer( new HeightFieldMapLayer( "SRTM", tms.get() ) );
        }

        // The MapNode will render the Map object in the scene graph.
        osgEarth::MapNode* mapNode = new osgEarth::MapNode( map );

        sceneData = mapNode;
    }
    
    // Set a home viewpoint
    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( sceneData );
    if ( mapNode )
    {
        manip->setNode( mapNode );
        if ( mapNode->getMap()->isGeocentric() )
        {
            manip->setHomeViewpoint( 
                osgEarthUtil::Viewpoint( osg::Vec3d( -90, 0, 0 ), 0.0, -90.0, 5e7 ) );

            // add a handler that will automatically calculate good clipping planes
            // for a geocentric map:
            //viewer.addEventHandler( new osgEarthUtil::AutoClipPlaneHandler( mapNode ) );
        }
    }

    viewer.setSceneData( sceneData );
    viewer.setCameraManipulator( manip );

    manip->getSettings()->bindMouseDoubleClick(
        osgEarthUtil::EarthManipulator::ACTION_GOTO,
        osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON );
    // add our fly-to handler
    viewer.addEventHandler(new FlyToViewpointHandler( manip ));

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new KeyboardEventHandler(mapNode));

    return viewer.run();
}
