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
#include <osgEarth/Notify>

#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/OceanSurfaceNode>
#include <osgEarthUtil/Controls>

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

// build an on-screen menu
static osg::Node* createMenu( osgViewer::View* view )
{
    using namespace osgEarth::Util::Controls;

    ControlCanvas* canvas = ControlCanvas::get( view );

    Grid* grid = new Grid();
    grid->setBackColor( 0, 0, 0, 0.5 );
    grid->setMargin( 5 );
    grid->setChildSpacing( 3 );
    grid->setVertAlign( Control::ALIGN_BOTTOM );
    int row = 0;

    grid->setControl( 1, row++, new LabelControl( "Ocean Demo", 18.0f, osg::Vec4(1,1,0,1) ) );
    grid->setControl( 1, row++, new LabelControl( "Zoom in to the coastline to see ocean effects.", 14.0f, osg::Vec4(.6,.6,.6,1) ) );

    grid->setControl( 0, row  , new LabelControl( "e" ) );
    grid->setControl( 1, row++, new LabelControl( "toggle ocean effects" ) );
    grid->setControl( 0, row  , new LabelControl( "m" ) );
    grid->setControl( 1, row++, new LabelControl( "toggle MSL adjustment" ) );
    grid->setControl( 0, row  , new LabelControl( "h/H" ) );
    grid->setControl( 1, row++, new LabelControl( "inc/dec wave height" ) );
    grid->setControl( 0, row  , new LabelControl( "p/P" ) );
    grid->setControl( 1, row++, new LabelControl( "inc/dec wave period" ) );
    grid->setControl( 0, row  , new LabelControl( "c/C" ) );
    grid->setControl( 1, row++, new LabelControl( "inc/dec ocean modulation color" ) );
    grid->setControl( 0, row  , new LabelControl( "a/A" ) );
    grid->setControl( 1, row++, new LabelControl( "inc/dec shimmer effect period" ) );
    grid->setControl( 0, row  , new LabelControl( "j/J" ) );
    grid->setControl( 1, row++, new LabelControl( "inc/dec surface image size" ) );
    grid->setControl( 0, row  , new LabelControl( "i" ) );
    grid->setControl( 1, row++, new LabelControl( "toggle ocean mask inversion" ) );
    grid->setControl( 0, row  , new LabelControl( "w" ) );
    grid->setControl( 1, row++, new LabelControl( "toggle wireframe mode" ) );

    canvas->addControl( grid );
    return canvas;
}

// An event handler that will print out the elevation at the clicked point
struct MyEventHandler : public osgGA::GUIEventHandler 
{
    MyEventHandler( osgEarth::Util::OceanSurfaceNode* ocean )
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

    osg::ref_ptr< osgEarth::Util::OceanSurfaceNode > _ocean;
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


int usage( const std::string& msg )
{
    OE_NOTICE
        << msg << std::endl
        << "USAGE: osgearth_ocean [--mask-layer <layername>] [--invert-mask] <earthfile>" << std::endl
        << "(note: <layername> defaults to \"ocean\")" << std::endl;
    return -1;
}


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    std::string maskLayerName = "ocean";
    while( arguments.read("--mask-layer", maskLayerName) );

    bool invertMask = arguments.isOption("--invert-mask");

    osg::Group* group = new osg::Group;

    osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);   
    if ( !loadedModel.valid() )
        return usage( "Failed to load an earth file." );

    osgEarth::Util::OceanSurfaceNode* ocean = new osgEarth::Util::OceanSurfaceNode();

    if ( !maskLayerName.empty() )
    {
        osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( loadedModel.get() );
        if ( mapNode )
        {
            osgEarth::MapFrame mapf( mapNode->getMap() );
            ocean->setOceanMaskImageLayer( mapf.getImageLayerByName( maskLayerName ) );
        }
    }

    ocean->setInvertMask( !invertMask );

    // install some water-surface images for an interesting shimmering effect:
    ImageList waterImages;
    waterImages.push_back( osgDB::readImageFile("../data/watersurface1.png") );
    waterImages.push_back( osgDB::readImageFile("../data/watersurface2.png") );
    waterImages.push_back( osgDB::readImageFile("../data/watersurface3.png") );
    waterImages.push_back( osgDB::readImageFile("../data/watersurface4.png") );

    osg::ref_ptr<osg::Image> waterImage = make3DImage(waterImages);    
    ocean->setOceanSurfaceImage( waterImage.get() );

    // Find the MapNode and add the ocean as a terrain decorator.
    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( loadedModel.get() );
    mapNode->addTerrainDecorator( ocean );

    // assemble the rest of the scene graph and go
    osgViewer::Viewer viewer(arguments);

    group->addChild( loadedModel.get() );
    group->addChild( createMenu(&viewer) );
    viewer.setSceneData(group);
    
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    viewer.addEventHandler(new MyEventHandler(ocean));
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
    viewer.addEventHandler(new osgViewer::StatsHandler);

    return viewer.run();
}
