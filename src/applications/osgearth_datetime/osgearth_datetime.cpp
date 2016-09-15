/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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

#include <osgEarth/MapNode>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/ECEF>
#include <osgEarth/Registry>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AnnotationEvents>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/ExampleResources>

#include <osgEarthAnnotation/AnnotationData>

#include <osgEarthDrivers/kml/KML>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgGA/EventVisitor>
#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Drivers;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

struct CollectDateTimeRangeVisitor : public osg::NodeVisitor
{
    DateTimeRange range;

    CollectDateTimeRangeVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) { }

    virtual void apply(osg::Node& node)
    {
        AnnotationData* data = dynamic_cast<AnnotationData*>(node.getUserData());
        if (data &&
            (data->getDateTimeRange().begin().isSet() || data->getDateTimeRange().end().isSet()))
        {
            range.expandBy(data->getDateTimeRange());
        }

        traverse(node);
    }    
};

struct CullNodeByDateTimeRange : public osg::NodeCallback
{
    DateTimeRange range;

    void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {        
        AnnotationData* data = dynamic_cast<AnnotationData*>(node->getUserData());
        if (data && !data->getDateTimeRange().intersects(range))
        {
            return;
        }

        traverse(node, nv);
    }
};

struct AssignCullCallbakVisitor : public osg::NodeVisitor
{
    CullNodeByDateTimeRange* callback;

    AssignCullCallbakVisitor(CullNodeByDateTimeRange* cb) :
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
        callback(cb)
    { }

    virtual void apply(osg::Node& node)
    {
        AnnotationData* data = dynamic_cast<AnnotationData*>(node.getUserData());
        if (data &&
            (data->getDateTimeRange().begin().isSet() || data->getDateTimeRange().end().isSet()))
        {
            node.addCullCallback(callback);
        }

        traverse(node);
    }    
};

struct TimeSliderHandler : public ControlEventHandler
{
    osg::ref_ptr<CullNodeByDateTimeRange> callback;
    DateTimeRange fullRange;

    TimeSliderHandler(CullNodeByDateTimeRange* cb, DateTimeRange r) :
        callback(cb),
        fullRange(r)
    {
    }

    void onValueChanged(Control* control, float value)
    {
        TimeStamp beginTs = fullRange.begin()->asTimeStamp();
        TimeStamp endTs = fullRange.end()->asTimeStamp();
        TimeStamp delta = endTs - beginTs;

        callback->range.begin() = DateTime(beginTs + delta * (value / 100));
        callback->range.end() = DateTime(beginTs + delta * ((value + 1) / 100));
    }
};

int
usage( char** argv )
{
    OE_WARN << "Usage: " << argv[0] << " <earthfile> --kml <kmlfile>" << std::endl;
    return -1;
}

int
main(int argc, char** argv)
{
    osg::Group* root = new osg::Group();

    // try to load an earth file.
    osg::ArgumentParser arguments(&argc,argv);
    
    osgViewer::Viewer viewer(arguments);
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy(false, false);
    viewer.setCameraManipulator( new EarthManipulator() );
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    viewer.getCamera()->setNearFarRatio(0.00002);

    // load an earth file and parse demo arguments
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( !node )
        return usage(argv);

    root->addChild( node );

    // find the map node that we loaded.
    MapNode* mapNode = MapNode::findMapNode(node);
    if ( !mapNode )
        return usage(argv);

    //--------------------------------------------------------------------

    CollectDateTimeRangeVisitor collectVisitor;
    collectVisitor.setNodeMaskOverride(~0);
    node->accept(collectVisitor);
    
    osg::ref_ptr<CullNodeByDateTimeRange> callback = new CullNodeByDateTimeRange;
    
    AssignCullCallbakVisitor assignVisitor(callback);
    assignVisitor.setNodeMaskOverride(~0);
    node->accept(assignVisitor);

    ControlCanvas* cs = ControlCanvas::getOrCreate(&viewer);

    HSliderControl* slider = new HSliderControl(0, 100);
    slider->setHorizAlign(Control::ALIGN_LEFT);
    slider->setVertAlign(Control::ALIGN_TOP);
    slider->setBackColor(.6, 0, 0, 1);
    slider->setHeight(25);
    slider->setWidth(300);    
    slider->addEventHandler(new TimeSliderHandler(callback, collectVisitor.range));
    slider->setValue(0.0);
    cs->addControl(slider);

    //--------------------------------------------------------------------

    // initialize the viewer:    
    viewer.setSceneData( root );

    viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode) );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    
    viewer.run();

    return 0;
}
