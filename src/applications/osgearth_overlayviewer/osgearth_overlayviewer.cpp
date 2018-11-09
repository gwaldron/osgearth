/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osg/Notify>
#include <osg/Depth>
#include <osg/LineWidth>
#include <osg/LineStipple>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/OverlayDecorator>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#define LC "[viewer] "

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

namespace
{
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
                    // note: the dumped geometry has some state in it (from ConvexPolyhedron::dumpGeometry)
                    // so we need to override it.
                    osg::Group* g = new osg::Group();
                    osg::StateSet* gss = g->getOrCreateStateSet();
                    gss->setAttributeAndModes(new osg::LineWidth(1.5f), 1);
                    gss->setRenderBinDetails(90210, "DepthSortedBin");

                    osg::Group* g0 = new osg::Group();
                    g->addChild( g0 );
                    osg::StateSet* g0ss = g0->getOrCreateStateSet();
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
                    g0ss->setAttributeAndModes(new osg::LineStipple(1, 0x000F), 1);
#endif
                    g0->addChild( dump );

                    osg::Group* g1 = new osg::Group();
                    g->addChild( g1 );
                    osg::StateSet* g1ss = g1->getOrCreateStateSet();
                    g1ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | 1);
                    g1->addChild( dump );

                    _parent->removeChildren(1, _parent->getNumChildren()-1);
                    _parent->addChild( g );

                    aa.requestRedraw();
                }
            }
            return false;
        }
    };
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
    mainView->setUpViewInWindow( b, b, (width/2)-b*2, (height-b*4) );
    viewer.addView( mainView );

    osgViewer::View* overlayView = new osgViewer::View();
    overlayView->getCamera()->setNearFarRatio(0.00002);
    overlayView->setCameraManipulator( new EarthManipulator() );
    
    overlayView->setUpViewInWindow( (width/2), b, (width/2)-b*2, (height-b*4) );
    overlayView->addEventHandler(new osgGA::StateSetManipulator(overlayView->getCamera()->getOrCreateStateSet()));
    viewer.addView( overlayView );

    std::string pathfile;
    double animationSpeed = 1.0;
    if (arguments.read("-p", pathfile))
    {
        mainView->setCameraManipulator( new osgGA::AnimationPathManipulator(pathfile) );
    }

    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        mainView->setSceneData( node );

        osg::Group* group = new osg::Group();
        group->addChild( MapNode::get(node) );
        overlayView->setSceneData( group );       
        overlayView->addEventHandler( new PHDumper(MapNode::get(node), group) );

        return viewer.run();
    }
    else return -1;
}
