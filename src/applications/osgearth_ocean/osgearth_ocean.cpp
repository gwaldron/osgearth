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
#include <osg/Texture3D>
#include <osg/Texture2D>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

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

class MyGraphicsContext {
    public:
        MyGraphicsContext()
        {
            osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
            traits->x = 0;
            traits->y = 0;
            traits->width = 1;
            traits->height = 1;
            traits->windowDecoration = false;
            traits->doubleBuffer = false;
            traits->sharedContext = 0;
            traits->pbuffer = true;

            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());

            if (!_gc)
            {
                traits->pbuffer = false;
                _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
            }

            if (_gc.valid()) 
            {
                _gc->realize();
                _gc->makeCurrent();
            }
        }
        
        bool valid() const { return _gc.valid() && _gc->isRealized(); }
        
    private:
        osg::ref_ptr<osg::GraphicsContext> _gc;
};



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
            case 'C':
                {
                    osg::Vec4f color = _ocean->getModulationColor();
                    color.a() = osg::clampBelow( color.a() + 0.1f, 1.0f );
                    _ocean->setModulationColor( color );                        
                }
                break;
            case 'c':
                {
                    osg::Vec4f color = _ocean->getModulationColor();
                    color.a() = osg::clampAbove( color.a() - 0.1f, 0.0f );
                    _ocean->setModulationColor( color );                        
                }
                break;
            case 'A':
                { 
                    _ocean->setOceanAnimationPeriod(_ocean->getOceanAnimationPeriod() + 0.25);
                }
                break;
            case 'a':
                { 
                    _ocean->setOceanAnimationPeriod(_ocean->getOceanAnimationPeriod() - 0.25);
                }
                break;
            case 'J':
                {
                    _ocean->setOceanSurfaceImageSizeRadians( _ocean->getOceanSurfaceImageSizeRadians() * 1.5f);
                }
                break;
            case 'j':
                {
                    _ocean->setOceanSurfaceImageSizeRadians( _ocean->getOceanSurfaceImageSizeRadians() * 0.5f);
                }
                break;
			case 'm':
				{
					_ocean->setAdjustToMSL( !_ocean->getAdjustToMSL());
				}
				break;
            }
        }
        return false;
    }

    osg::ref_ptr< OceanSurfaceNode > _ocean;
};

typedef std::vector< osg::ref_ptr< osg::Image > > ImageList;

osg::Image* make3DImage(const ImageList& images)
{
    MyGraphicsContext gc;
    osg::notify(osg::NOTICE) << "Made graphic context " << std::endl;
    /*for (unsigned int i = 0; i < images.size(); ++i)
    {
        images[i]->scaleImage(256, 256, 1);
        osg::notify(osg::NOTICE) << "Scaled image " << i << std::endl;
    }*/

    osg::Image* image3D = new osg::Image;
    image3D->allocateImage(images[0]->s(), images[0]->t(), images.size(),
                           images[0]->getPixelFormat(), images[0]->getDataType());
    
    for (unsigned int i = 0; i < images.size(); ++i)
    {
        image3D->copySubImage(0, 0, i, images[i].get());
    }
    image3D->setInternalTextureFormat(images[0]->getInternalTextureFormat());
    return image3D;
}




int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    int maskUnit = -1;
    while(arguments.read("--mask-unit",maskUnit));    


    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    {
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
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

    osg::Group* group = new osg::Group;

    osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);   

    OceanSurfaceNode* ocean = new OceanSurfaceNode();
    ocean->setOceanMaskTextureUnit( maskUnit );

    ImageList waterImages;
    waterImages.push_back( osgDB::readImageFile("../data/watersurface1.png") );
    waterImages.push_back( osgDB::readImageFile("../data/watersurface2.png") );
    waterImages.push_back( osgDB::readImageFile("../data/watersurface3.png") );
    waterImages.push_back( osgDB::readImageFile("../data/watersurface4.png") );

    osg::ref_ptr<osg::Image> waterImage = make3DImage(waterImages);
    
    ocean->setOceanSurfaceImage( waterImage.get() );

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
