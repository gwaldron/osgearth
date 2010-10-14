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


#include <osgEarth/Export>
#include <osg/Version>
#include <osg/Notify>

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
#include <osg/DeleteHandler>

#include <osgTerrain/TerrainTile>
#include <osgTerrain/GeometryTechnique>

#include <osgDB/WriteFile>

#include <osgText/Text>

#include <iostream>

#include <osgEarth/MapNode>
#include <osgEarth/FindNode>
#include <osgEarth/TileSource>
#include <osgEarth/Registry>

#include <osgEarthUtil/Common>
#include <osgEarthUtil/FadeLayerNode>
#include <osgEarthUtil/EarthManipulator>

#include <osgEarthDrivers/arcgis/ArcGISOptions>
#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/engine_osgterrain/OSGTerrainOptions>

#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <osgWidget/Util>
#include <osgWidget/WindowManager>
#include <osgWidget/Box>
#include <osgWidget/Label>
#include <osgWidget/ViewerEventHandlers>

#include <osgViewer/ViewerEventHandlers>

//#define EXTRA_REFERENCED_DATA

#define CHECK_MEMORY 0

using namespace osg;
using namespace osgDB;
using namespace osgEarth;
using namespace osgEarthUtil;
using namespace osgEarth::Drivers;
using namespace osgWidget;

Vec4 normalColor(1,1,1,1);
Vec4 hotColor(0.4,0.56,1,1);
float textSize = 25.0f;
bool hudDirty = false;



const unsigned int MASK_2D = 0xF0000000;

struct BlankTileSource : public osgEarth::TileSource 
{
    BlankTileSource(const TileSourceOptions& options =TileSourceOptions()) : osgEarth::TileSource( options )
	{
	}

	virtual void initialize( const std::string& referenceURI, const Profile* overrideProfile =0)
	{
		if (overrideProfile)
		{
			setProfile( overrideProfile );
		}
		else
		{
			setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );
		}
	}


    virtual osg::Image* createImage( const TileKey& key, ProgressCallback* progress ) {
        osg::Image* image = new osg::Image();
        image->setName("BlankTileSource");
        image->setAllocationMode( osg::Image::USE_NEW_DELETE );
        image->allocateImage( 4, 4, 1, GL_RGBA, GL_UNSIGNED_BYTE );
        for( int y=0; y<image->t(); y++ ) {
            for( int x=0; x<image->s(); x++ ) {
                *((unsigned int*)(image->data(x,y))) = 0xff00ff00;
            }
        }
        return image;
    }

};

    class LogUpdateCallback : public osg::NodeCallback
    {
    public:
        double _lastLog;
        int _nbLog;
        bool _layerActive;
        LogUpdateCallback(Map* map) : _lastLog(0), _nbLog(0), _layerActive(false), _map(map) 
        {
            osg::ref_ptr<BlankTileSource> tileSource = new BlankTileSource();
            tileSource->initialize( "" );
            ImageLayerOptions layerOpt;
            layerOpt.name() = "Green";
            _layer = new ImageLayer( layerOpt, tileSource );
        }

        /** Callback method called by the NodeVisitor when visiting a node.*/
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        { 
            if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR) {
                if (nv->getFrameStamp()->getSimulationTime()-_lastLog > 1.0 && nv->getFrameStamp()->getSimulationTime() > 8.0) {
#if CHECK_MEMORY
                    osg::Referenced::reportCurrentMemoryObject();
#endif
                    _lastLog = nv->getFrameStamp()->getSimulationTime();
#if OSG_MIN_VERSION_REQUIRED(2,9,6)
                    osg::Texture::getTextureObjectManager(0)->reportStats();
#endif
                    _nbLog++;

                }

                MapFrame mapf(_map);

                if (_nbLog % 15 == 7 && !_layerActive) {
                    _map->addImageLayer( _layer.get() );
                    _layerActive = true;
                }
                if (_nbLog % 15 == 14 && _layerActive) {
                    _map->removeImageLayer( mapf.imageLayers()[0] );
                    _layerActive = false;
                }
            }
            // note, callback is responsible for scenegraph traversal so
            // they must call traverse(node,nv) to ensure that the
            // scene graph subtree (and associated callbacks) are traversed.
            traverse(node,nv);
        }
        osg::ref_ptr<ImageLayer> _layer;
        osg::ref_ptr<Map> _map;

    };

class PrintObjectReference : public osgGA::GUIEventHandler
{
public:

    PrintObjectReference() {}
    
    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
    {
        switch(ea.getEventType())
        {
        case(osgGA::GUIEventAdapter::KEYUP):
        {
#ifdef EXTRA_REFERENCED_DATA
            if (ea.getKey() == 'u') {
                osg::Referenced::reportCurrentMemoryObject();
                return true;
            } else if (ea.getKey() == 'c') {
                osg::Referenced::dumpStats();
                return true;
            }
#endif
        }

        default:
            return false;

        }
    }
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    OSGTerrainOptions terrainOptions;

    terrainOptions.loadingPolicy()->mode() = LoadingPolicy::MODE_SEQUENTIAL;

    if ( arguments.read( "--preemptive" ) || arguments.read( "--preemptive=ON" ) )
    {
        terrainOptions.loadingPolicy()->mode() = LoadingPolicy::MODE_PREEMPTIVE;
    }
    else if ( arguments.read( "--standard" ) || arguments.read( "--standard=ON" ) )
    {
        terrainOptions.loadingPolicy()->mode() = LoadingPolicy::MODE_STANDARD;
    }
    else if ( arguments.read( "--sequential" ) || arguments.read( "--sequential=ON" ) )
    {
        terrainOptions.loadingPolicy()->mode() = LoadingPolicy::MODE_SEQUENTIAL;
    }

    if (arguments.read( "--multipass") )
    {
        terrainOptions.compositingTechnique() = TerrainOptions::COMPOSITING_MULTIPASS;
        //Multipass mode is currently only available in STANDARD mode.
        terrainOptions.loadingPolicy()->mode() = LoadingPolicy::MODE_STANDARD;
    }
    {

        // construct the viewer.
        osgViewer::Viewer viewer(arguments);
        {
            osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

            keyswitchManipulator->addMatrixManipulator( '1', "EarthManipulator", new osgEarthUtil::EarthManipulator() );
            keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
            keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );
            keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );

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

        osg::ref_ptr<osg::Group> group = new osg::Group;
        group->setName("root");

        MapNodeOptions mapOptions;
        mapOptions.setTerrainOptions( terrainOptions );

        osg::ref_ptr<MapNode> mapNode = new MapNode(mapOptions);
        osg::ref_ptr<osg::Node> loadedModel = mapNode;


        osg::ref_ptr<FadeLayerNode> fadeLayerNode = new FadeLayerNode( mapNode->getMap(), mapOptions ); //mapNode->getEngine()->getEngineProperties());
        fadeLayerNode->addChild(loadedModel.get());
        group->addChild(fadeLayerNode);

        // autolog every one second
        group->addUpdateCallback(new LogUpdateCallback(fadeLayerNode->getMap()));
        viewer.addEventHandler(new PrintObjectReference());

        MapFrame mapf(mapNode->getMap());

        for (unsigned int i = 0; i < mapf.imageLayers().size(); ++i)
        {
            mapf.imageLayers()[i]->setOpacity( 1.0f );
            mapf.imageLayers()[i]->setEnabled( true );
        }

        //Setup the osgWidget interface
        // osg::ref_ptr<osgWidget::WindowManager> wm = new osgWidget::WindowManager(
        //     &viewer,
        //     800.0f,
        //     800.0f,
        //     MASK_2D,
        //     0 );

        // TOC toc(wm, mapNode->getMap(), fadeLayerNode, &viewer);
        // createAddLayersMenu(wm, fadeLayerNode, mapNode->getMap(), &viewer);
        group->setDataVariance(osg::Object::DYNAMIC);
//        group->setUpdateCallback(new TOCUpdateCallback(&toc));


        viewer.setUpViewInWindow(10, 10, 800,800);

        // osg::Camera* camera = wm->createParentOrthoCamera();
        // group->addChild(camera);

        // viewer.addEventHandler(new osgWidget::MouseHandler(wm));
        // viewer.addEventHandler(new osgWidget::KeyboardHandler(wm));
        // viewer.addEventHandler(new osgWidget::ResizeHandler(wm, camera));
        // viewer.addEventHandler(new osgWidget::CameraSwitchHandler(wm, camera));

        viewer.addEventHandler(new osgViewer::StatsHandler());
        viewer.addEventHandler(new osgViewer::WindowSizeHandler());
        // add the state manipulator
        viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

        // set the scene to render
        viewer.setSceneData(group);
        viewer.run();
        if (osg::Referenced::getDeleteHandler()) {
            OE_NOTICE << "flush all" << std::endl;
            osg::Referenced::getDeleteHandler()->flushAll();
        }
    }
 
    return 0;
}
