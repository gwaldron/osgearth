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

#include <osg/Notify>
#include <osg/Depth>
#include <osg/LineWidth>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/OverlayDecorator>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>
#include <osgEarthSymbology/Color>

#define LC "[viewer] "

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

static CheckBoxControl* s_cameraCheck;
//static CheckBoxControl* s_overlayCheck;
static CheckBoxControl* s_intersectionCheck;
static CheckBoxControl* s_rttCheck;

namespace
{
    void toggle(osg::Group* p, const std::string& name, bool onoff)
    {
        if (p->getNumChildren() > 1)
        {
            osg::Group* g = p->getChild(1)->asGroup();
            for(unsigned i=0; i<g->getNumChildren(); ++i)
            {
                if ( g->getChild(i)->getName() == name )
                {
                    g->getChild(i)->setNodeMask( onoff ? ~0 : 0 );
                    break;
                }
            }
        }
        else
        {
            OE_WARN << "No overlays to display / toggle." << std::endl;
        }
    }


    struct Toggle : public ControlEventHandler
    {
        osg::Group* _g;
        std::string _name;
        Toggle(osg::Group* g, const std::string& name) : _g(g), _name(name) { }
        void onValueChanged( Control* control, bool value )
        {
            toggle(_g, _name, value);
        }
    };


    // it's not used by osgEarth, but you can copy this code into a viewer app and
    // use it to visualize the various polyhedra created by the overlay decorator.
    // see the end of OverlayDecorator::cull for the dump types.
    struct PHDumper : public osgGA::GUIEventHandler
    {
        MapNode*    _mapNode;
        osg::Group* _parent;
        PHDumper(MapNode* mapNode, osg::Group* parent) : _mapNode(mapNode), _parent(parent)
        {
        }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            if ( ea.getEventType() == ea.FRAME )
            {
                osg::Node* dump = _mapNode->getOverlayDecorator()->getDump();
                if ( !dump )
                {
                    _mapNode->getOverlayDecorator()->requestDump();
                    aa.requestRedraw();
                }
                else
                {
                    dump->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(
                        osg::Depth::LEQUAL, 0, 1, false), 1 | osg::StateAttribute::OVERRIDE);
                    dump->getOrCreateStateSet()->setMode(GL_BLEND,1);
                    dump->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(1.5f), 1);

                    _parent->removeChildren(1, _parent->getNumChildren()-1);
                    _parent->addChild( dump );

                    toggle(_parent, "camera", s_cameraCheck->getValue());
                    //toggle(_parent, "overlay", s_overlayCheck->getValue());
                    toggle(_parent, "intersection", s_intersectionCheck->getValue());
                    toggle(_parent, "rtt", s_rttCheck->getValue());

                    aa.requestRedraw();
                }
            }
            return false;
        }
    };
}


void
setupOverlayView( osgViewer::View* view, osg::Group* parent, MapNode* mapNode )
{
    ControlCanvas* canvas = ControlCanvas::getOrCreate(view);

    VBox* v = canvas->addControl(new VBox());
    v->setBackColor( Color(Color::Black,0.75) );
    {
        HBox* camBox = v->addControl(new HBox());
        {
            camBox->addControl(s_cameraCheck = new CheckBoxControl(true, new Toggle(parent,"camera")));
            camBox->addControl(new LabelControl("Camera", Color("#00ff00")));
        }

        //HBox* overlayBox = v->addControl(new HBox());
        //{
        //    overlayBox->addControl(s_overlayCheck = new CheckBoxControl(false, new Toggle(parent,"overlay")));
        //    overlayBox->addControl(new LabelControl("Overlay", Color("#00ffff")));
        //}

        HBox* isectBox = v->addControl(new HBox());
        {
            isectBox->addControl(s_intersectionCheck = new CheckBoxControl(true, new Toggle(parent,"intersection")));
            isectBox->addControl(new LabelControl("Intersection",Color("#ff7f00")));
        }

        HBox* rttBox = v->addControl(new HBox());
        {
            rttBox->addControl(s_rttCheck = new CheckBoxControl(true, new Toggle(parent,"rtt")));
            rttBox->addControl(new LabelControl("RTT", Color("#ffff00")));
        }
    }
    
    view->addEventHandler( new PHDumper(mapNode, parent) );
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::CompositeViewer viewer(arguments);
    viewer.setThreadingModel( osgViewer::CompositeViewer::SingleThreaded );

    // query the screen size.
    osg::GraphicsContext::ScreenIdentifier si;
    si.readDISPLAY();
    if ( si.displayNum < 0 ) si.displayNum = 0;
    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    unsigned width, height;
    wsi->getScreenResolution( si, width, height );
    unsigned b = 50;

    osgViewer::View* mainView = new osgViewer::View();
    mainView->getCamera()->setNearFarRatio(0.00002);
    EarthManipulator* em = new EarthManipulator();
    em->getSettings()->setMinMaxPitch(-90, 0);
    mainView->setCameraManipulator( em );
    //mainView->setUpViewInWindow( 50, 50, 600, 600 );
    mainView->setUpViewInWindow( b, b, (width/2)-b*2, (height-b*4) );
    viewer.addView( mainView );

    osgViewer::View* overlayView = new osgViewer::View();
    overlayView->getCamera()->setNearFarRatio(0.00002);
    EarthManipulator* overlayEM = new EarthManipulator();
    overlayEM->getSettings()->setCameraProjection(overlayEM->PROJ_ORTHOGRAPHIC);
    overlayView->setCameraManipulator( overlayEM );
    
    //overlayView->setUpViewInWindow( 700, 50, 600, 600 );
    overlayView->setUpViewInWindow( (width/2), b, (width/2)-b*2, (height-b*4) );
    overlayView->addEventHandler(new osgGA::StateSetManipulator(overlayView->getCamera()->getOrCreateStateSet()));
    viewer.addView( overlayView );

    std::string pathfile;
    double animationSpeed = 1.0;
    if (arguments.read("-p", pathfile))
    {
        mainView->setCameraManipulator( new osgGA::AnimationPathManipulator(pathfile) );
    }

    osg::Node* node = MapNodeHelper().load( arguments, mainView );
    if ( node )
    {
        mainView->setSceneData( node );

        osg::Group* group = new osg::Group();
        group->addChild( MapNode::get(node) );
        overlayView->setSceneData( group );

        setupOverlayView( overlayView, group, MapNode::get(node) );

        return viewer.run();
    }
    else return -1;
}
