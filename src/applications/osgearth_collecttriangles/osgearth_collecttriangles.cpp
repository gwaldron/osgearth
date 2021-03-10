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
#include <osgEarth/StateTransition>
#include <osg/TriangleFunctor>
#include <osg/Depth>
#include <osg/PolygonMode>

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
                vp_Color = vec4(2.0, 0.0, 0.0, 1.0);
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
        //osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
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


static osg::Group* root;

struct QueryTrianglesHandler : public osgGA::GUIEventHandler
{
    QueryTrianglesHandler(MapNode* mapNode)
        : _mapNode(mapNode)
    {
        _marker = new osg::MatrixTransform;
        _marker->addChild(osgDB::readNodeFile("d:/dev/osgearth/tests/sphere.obj"));
        _marker->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE), 1);
        osgEarth::Registry::shaderGenerator().run(_marker);
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
                // TODO:  Bounding sphere
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

    osg::Node* _boundsDisplay = nullptr;
    osgEarth::MapNode* _mapNode;
    osg::ref_ptr< osg::Node > _extractedTriangles;
    osg::MatrixTransform* _marker;
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
        //ImGui::ShowDemoWindow();
        _layers.draw(renderInfo, _mapNode.get(), _view->getCamera(), _earthManip.get());


        ImGui::Begin("Triangle Stuff");
        ImGui::SliderFloat("Query Range", &query_range, 1.0, 2000.0f);
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

        root->addChild(node);

        viewer.setSceneData(root);
        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
