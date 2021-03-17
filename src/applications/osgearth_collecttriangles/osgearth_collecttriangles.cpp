/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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

#include <osgEarth/ImGuiUtils>
#include <osgEarth/OsgImGuiHandler.hpp>
#include <imgui_internal.h>

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgDB/ReadFile>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/Threading>
#include <osgEarth/Geocoder>
#include <osgEarth/NodeUtils>
#include <osgEarth/PagedNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/AnnotationUtils>
#include <osg/TriangleFunctor>
#include <osg/Depth>
#include <osg/PolygonMode>
#include <osg/io_utils>

#include <iostream>

#include <osgEarth/Metrics>


#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;
using namespace osgEarth::ImGuiUtil;

float query_range = 100.0;
long long query_time_ns = 0;
long long query_triangle_count = 0;
static unsigned int observer_id = 1;
static bool loading_data = false;

static osg::Group* root;


class Observer : public osg::Object
{
public:
    Observer()
    {
    }

    Observer(const Observer& rhs, const osg::CopyOp& copy = osg::CopyOp::SHALLOW_COPY):
        _bounds(rhs._bounds)
    {
    }

    Observer(osg::BoundingSphered& bounds):
        _bounds(bounds)
    {
    }

    META_Object(osgEarth, Observer);

    const osg::BoundingSphered& getBounds() const
    {
        return _bounds;
    }

    void setBounds(const osg::BoundingSphered& bounds)
    {
        _bounds = bounds;
    }

private:
    osg::BoundingSphered _bounds;
};

typedef std::vector< osg::ref_ptr< Observer > > ObserverList;

static ObserverList observers;

osg::Node* installMultiPassRendering(osg::Node* node)
{
    osg::Group* geode = new osg::Group();
    geode->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0, 1, true));
    geode->setCullingActive(false);

    osg::Group* wireframeGroup = new osg::Group();
    wireframeGroup->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE), 1);
    wireframeGroup->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0, 1, true));
    wireframeGroup->getOrCreateStateSet()->setDefine("WIREFRAME");
    wireframeGroup->addChild(node);
    geode->addChild(wireframeGroup);

    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setRenderBinDetails(99, "RenderBin");

    const char* tri_vs = R"(
            #version 330
            #pragma import_defines(WIREFRAME)
            vec4 vp_Color;
            void colorize_vs(inout vec4 vertex)
            {
                vp_Color = vec4(1.0, 0.0, 0.0, 1.0);
            }
        )";

    VirtualProgram::getOrCreate(geode->getOrCreateStateSet())->setFunction(
        "colorize_vs", tri_vs, ShaderComp::LOCATION_VERTEX_VIEW);

    return geode;
}

struct CollectTriangles
{
    CollectTriangles()
    {
        verts = new osg::Vec3Array();
    }
#if OSG_VERSION_LESS_THAN(3,5,6)
    inline void operator () (const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3, bool treatVertexDataAsTemporary)
#else
    inline void operator () (const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3)
#endif
    {
        verts->push_back(v1);
        verts->push_back(v2);
        verts->push_back(v3);
    }

    osg::ref_ptr< osg::Vec3Array > verts;
};

struct CollectTrianglesVisitor : public osg::NodeVisitor
{
    CollectTrianglesVisitor() :
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN)
    {
        _vertices.reserve(1000000);
    }

    bool intersects(osg::Node& node)
    {
        static osg::Matrix identity;
        osg::Matrix& matrix = _matrixStack.empty() ? identity : _matrixStack.back();

        osg::BoundingSphere nodeBounds = node.getBound();
        nodeBounds.center() += matrix.getTrans();

        return nodeBounds.intersects(_queryBounds);
    }

    void apply(osg::Node& node)
    {
        if (intersects(node))
        {
            traverse(node);
        }
    }

    void apply(osg::Transform& transform)
    {
        if (intersects(transform))
        {
            osg::Matrix matrix;
            if (!_matrixStack.empty()) matrix = _matrixStack.back();
            transform.computeLocalToWorldMatrix(matrix, this);
            pushMatrix(matrix);
            traverse(transform);
            popMatrix();
        }
    }

    void apply(osg::Drawable& drawable) override
    {
        if (intersects(drawable))
        {
            osg::TriangleFunctor<CollectTriangles> triangleCollector;
            drawable.accept(triangleCollector);
            for (unsigned int j = 0; j < triangleCollector.verts->size(); j++)
            {
                static osg::Matrix identity;
                osg::Matrix& matrix = _matrixStack.empty() ? identity : _matrixStack.back();
                osg::Vec3d v = (*triangleCollector.verts)[j];
                _vertices.emplace_back(v * matrix);
            }
        }
    }

    float getDistanceToEyePoint(const osg::Vec3& pos, bool /*withLODScale*/) const
    {
        // Use highest level of detail
        return 0.0;
    }

    osg::Node* buildNode()
    {
        osg::Geometry* geom = new osg::Geometry;
        osg::Vec3Array* verts = new osg::Vec3Array;
        geom->setVertexArray(verts);

        bool first = true;
        osg::Vec3d anchor;

        for (unsigned int i = 0; i < _vertices.size(); i++)
        {
            if (first)
            {
                anchor = _vertices[i];
                first = false;
            }
            verts->push_back(_vertices[i] - anchor);
        }
        geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, verts->size()));

        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix(osg::Matrixd::translate(anchor));
        mt->addChild(installMultiPassRendering(geom));

        return mt;
    }

    inline void pushMatrix(osg::Matrix& matrix) { _matrixStack.push_back(matrix); }

    inline void popMatrix() { _matrixStack.pop_back(); }

    typedef std::vector<osg::Matrix> MatrixStack;
    std::vector<osg::Vec3d>  _vertices;
    MatrixStack _matrixStack;
    osg::BoundingSphere _queryBounds;
};

struct QueryTrianglesHandler : public osgGA::GUIEventHandler
{
    QueryTrianglesHandler(MapNode* mapNode)
        : _mapNode(mapNode)
    {
        _marker = new osg::MatrixTransform;
        _marker->addChild(AnnotationUtils::createSphere(1.0, Color::White));
        _marker->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE), 1);
        root->addChild(_marker);
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());

        if (ea.getEventType() == ea.MOVE)
        {
            osg::Vec3d world;
            osgUtil::LineSegmentIntersector::Intersections hits;
            osg::NodePath path;
            path.push_back(_mapNode);
            if (view->computeIntersections(ea.getX(), ea.getY(), path, hits))
            {
                _marker->setNodeMask(~0u);
                // Get the point under the mouse:
                world = hits.begin()->getWorldIntersectPoint();

                // convert to map coords:
                GeoPoint mapPoint;
                mapPoint.fromWorld(_mapNode->getMapSRS(), world);
                osg::Matrixd l2w;
                mapPoint.createLocalToWorld(l2w);
                l2w.preMult(osg::Matrixd::scale(query_range, query_range, query_range));
                _marker->setMatrix(l2w);

                if (_extractedTriangles.valid())
                {
                    root->removeChild(_extractedTriangles.get());
                    _extractedTriangles = nullptr;
                }

                auto startTime = std::chrono::high_resolution_clock::now();

                CollectTrianglesVisitor v;
                v._queryBounds = osg::BoundingSphere(world, query_range);
                _mapNode->accept(v);

                auto endTime = std::chrono::high_resolution_clock::now();
                query_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime).count();
                query_triangle_count = v._vertices.size() / 3;


                _extractedTriangles = v.buildNode();
                if (_extractedTriangles.valid())
                {
                    osgEarth::Registry::shaderGenerator().run(_extractedTriangles.get());
                    root->addChild(_extractedTriangles.get());
                }

            }
            else
            {
                _marker->setNodeMask(0u);
            }
        }
        return false;
    }

    osgEarth::MapNode* _mapNode;
    osg::ref_ptr< osg::Node > _extractedTriangles;
    osg::MatrixTransform* _marker;
};

struct AddObserverHandler : public osgGA::GUIEventHandler
{
    AddObserverHandler(MapNode* mapNode)
        : _mapNode(mapNode)
    {
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());

        if (ea.getEventType() == ea.PUSH && ea.getButton() == ea.LEFT_MOUSE_BUTTON && ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)
        {
            osg::Vec3d world;
            osgUtil::LineSegmentIntersector::Intersections hits;
            osg::NodePath path;
            path.push_back(_mapNode);
            if (view->computeIntersections(ea.getX(), ea.getY(), path, hits))
            {

                // Get the point under the mouse:
                world = hits.begin()->getWorldIntersectPoint();

                Observer* observer = new Observer(osg::BoundingSphered(world, query_range));
                observer->setName(Stringify() << "Observer " << observer_id++);
                observers.push_back(observer);
            }
        }
        return false;
    }

    osgEarth::MapNode* _mapNode;
};

class ObserversNode : public osg::Group
{
public:
    ObserversNode():
        osg::Group()
    {
        setNumChildrenRequiringUpdateTraversal(1);
        _marker = AnnotationUtils::createSphere(1.0, Color::Yellow);
        _marker->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE), 1);
    }

    void traverse(osg::NodeVisitor& nv)
    {
        if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
        {
            removeChildren(0, getNumChildren());

            for (auto& o : observers)
            {
                osg::MatrixTransform* mt = new osg::MatrixTransform;
                mt->addChild(_marker);
                double radius = o->getBounds().radius();
                mt->setMatrix(osg::Matrixd::scale(radius, radius, radius) * osg::Matrixd::translate(o->getBounds().center()));
                addChild(mt);
            }

        }
        osg::Group::traverse(nv);
    }

    osg::ref_ptr< osg::Node > _marker;
};

class ImGuiDemo : public OsgImGuiHandler
{
public:
    ImGuiDemo(osgViewer::View* view, MapNode* mapNode, EarthManipulator* earthManip) :
        _mapNode(mapNode),
        _earthManip(earthManip),
        _view(view)
    {
    }

protected:
    void drawUi(osg::RenderInfo& renderInfo) override
    {
        // ImGui code goes here...
        ImGui::ShowDemoWindow();
        _layers.draw(renderInfo, _mapNode.get(), _view->getCamera(), _earthManip.get());


        ImGui::Begin("Triangle Query");

        ImGui::SliderFloat("Query Range", &query_range, 1.0, 2000.0f);

        if (loading_data)
        {
            ImGui::Text("Loading data....");
        }
        else if (ImGui::Button("Load data"))
        {
            loading_data = true;
        }

        if (ImGui::Button("Enable auto unload"))
        {
            EnableAutoUnloadVisitor v;
            _mapNode->accept(v);
        }

        ImGui::Separator();
        ImGui::Text("Observers");

        if (ImGui::Button("Add san francisco"))
        {
            auto observer = new Observer(osg::BoundingSphered(osg::Vec3d(-2709463.117513461, -4278909.160321064, 3864024.782792999), 1000.0));
            osgEarth::GeoPoint pt;
            pt.fromWorld(SpatialReference::create("wgs84"), observer->getBounds().center());
            observer->setName("San Francisco");
            observers.push_back(observer);
        }
        ImGui::BeginGroup();

        for (auto itr = observers.begin(); itr != observers.end();)
        {
            osg::ref_ptr< Observer > observer = itr->get();
            ImGui::PushID(observer.get());
            ImGui::Text(observer->getName().c_str()); ImGui::SameLine();
            if (ImGui::Button("Delete"))
            {
                observers.erase(itr);
            }
            else
            {
                itr++;
            }
            ImGui::SameLine();
            if (ImGui::Button("Zoom to"))
            {
                GeoPoint pt;
                pt.fromWorld(SpatialReference::create("wgs84"), observer->getBounds().center());
                _earthManip->setViewpoint(Viewpoint("", pt.x(), pt.y(), pt.alt(), 0, -90, observer->getBounds().radius() * 1.5), 0.0);
            }
            ImGui::PopID();
        }
        ImGui::EndGroup();

        ImGui::Separator();


        ImGui::Text("Collected %d triangles in %lf ms", query_triangle_count, (double)query_time_ns / 1.0e6);
        ImGui::End();
    }

    osg::ref_ptr< MapNode > _mapNode;
    osg::ref_ptr<EarthManipulator> _earthManip;
    osgViewer::View* _view;
    LayersGUI _layers;
};

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}


int
main(int argc, char** argv)
{
    ImGuiNotifyHandler* notifyHandler = new ImGuiNotifyHandler();
    osg::setNotifyHandler(notifyHandler);
    osgEarth::setNotifyHandler(notifyHandler);

    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc, argv);

    // help?
    if (arguments.read("--help"))
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // install our default manipulator (do this before calling load)
    EarthManipulator* manip = new EarthManipulator(arguments);
    viewer.setCameraManipulator(manip);

    // Setup the viewer for imgui
    viewer.setRealizeOperation(new ImGuiDemo::RealizeOperation);

    viewer.realize();

    root = new osg::Group;

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (node)
    {
        MapNode* mapNode = MapNode::findMapNode(node);
        if (mapNode)
        {
            viewer.getEventHandlers().push_front(new ImGuiDemo(&viewer, mapNode, manip));
        }

        viewer.getEventHandlers().push_front(new QueryTrianglesHandler(mapNode));
        viewer.getEventHandlers().push_front(new AddObserverHandler(mapNode));

        root->addChild(new ObserversNode());
        root->addChild(node);

        viewer.setSceneData(root);

        while (!viewer.done())
        {
            if (loading_data)
            {
                LoadDataVisitor v;
                for (auto& o : observers)
                {
                    v.getAreasToLoad().push_back(o->getBounds());
                }
                root->accept(v);
                if (v.isFullyLoaded())
                {
                    loading_data = false;
                }
            }

            viewer.frame();
        }
        return 0;
    }
    else
    {
        return usage(argv[0]);
    }
}
