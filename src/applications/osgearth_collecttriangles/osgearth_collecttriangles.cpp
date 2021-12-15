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
#include <osgEarth/ImGui/ImGuiApp>

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
#include <osgEarth/AnnotationUtils>
#include <osgEarth/TDTiles>
#include <osg/TriangleFunctor>
#include <osg/Depth>
#include <osg/PolygonMode>
#include <osg/io_utils>
#include <osg/Point>

#include <iostream>


#include <osgEarth/TerrainTileNode>


#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;
using namespace osgEarth::GUI;

float query_range = 100.0;
long long query_time_ns = 0;
long long predictive_data_loading_time_ns = 0;
long long query_triangle_count = 0;
static unsigned int observer_id = 1;
static bool loading_data = false;
static osg::Group* root;
static float range_boost = 1.0;
static bool load_highres_only = false;
static bool predictive_loading = false;
long long intersection_time = 0;
long long num_intersections = 0;
int num_threads = 4;

static float minLat = -90.0;
static float maxLat = 90.0;
static float minLon = -180.0;
static float maxLon = 180.0;
static int latSegments = 25;
static int lonSegments = 50;
static float altAdjust = 5.0f;
static bool serial = false;

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

    Observer(const osg::BoundingSphered& bounds):
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

static osg::ref_ptr< Observer > cameraObserver;

typedef std::vector< osg::ref_ptr< Observer > > ObserverList;

static ObserverList observers;

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
        //osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
        _vertices.reserve(1000000);
        _colors.reserve(1000000);
    }

    bool intersects(osg::Node& node)
    {
        static osg::Matrix identity;
        osg::Matrix& l2w = _matrixStack.empty() ? identity : _matrixStack.back();

        // Dealing with scale, taken from Transform.cpp
        osg::BoundingSphere bsphere = node.getBound();

        osg::BoundingSphere::vec_type xdash = bsphere._center;
        xdash.x() += bsphere._radius;
        xdash = xdash * l2w;

        osg::BoundingSphere::vec_type ydash = bsphere._center;
        ydash.y() += bsphere._radius;
        ydash = ydash * l2w;

        osg::BoundingSphere::vec_type zdash = bsphere._center;
        zdash.z() += bsphere._radius;
        zdash = zdash * l2w;

        bsphere._center = bsphere._center*l2w;

        xdash -= bsphere._center;
        osg::BoundingSphere::value_type sqrlen_xdash = xdash.length2();

        ydash -= bsphere._center;
        osg::BoundingSphere::value_type sqrlen_ydash = ydash.length2();

        zdash -= bsphere._center;
        osg::BoundingSphere::value_type sqrlen_zdash = zdash.length2();

        bsphere._radius = sqrlen_xdash;
        if (bsphere._radius < sqrlen_ydash) bsphere._radius = sqrlen_ydash;
        if (bsphere._radius < sqrlen_zdash) bsphere._radius = sqrlen_zdash;
        bsphere._radius = (osg::BoundingSphere::value_type)sqrt(bsphere._radius);

        return bsphere.intersects(_queryBounds);
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
            Random prng(static_cast<unsigned int>(reinterpret_cast<uint64_t>(&drawable)));
            osg::Vec4 color(prng.next(), prng.next(), prng.next(), 1.0f);

            osg::TriangleFunctor<CollectTriangles> triangleCollector;
            drawable.accept(triangleCollector);
            for (unsigned int j = 0; j < triangleCollector.verts->size(); j++)
            {
                static osg::Matrix identity;
                osg::Matrix& matrix = _matrixStack.empty() ? identity : _matrixStack.back();
                osg::Vec3d v = (*triangleCollector.verts)[j];
                _vertices.emplace_back(v * matrix);
                _colors.emplace_back(color);
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

        osg::Vec4Array* colors = new osg::Vec4Array();
        geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);

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
            colors->push_back(_colors[i]);
        }
        geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, verts->size()));

        
        geom->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0, 1, true));
        geom->setCullingActive(false);

        geom->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE), 1);
        geom->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0, 1, true));

        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix(osg::Matrixd::translate(anchor));
        mt->addChild(geom);

        return mt;
    }

    inline void pushMatrix(osg::Matrix& matrix) { _matrixStack.push_back(matrix); }

    inline void popMatrix() { _matrixStack.pop_back(); }

    typedef std::vector<osg::Matrix> MatrixStack;
    std::vector<osg::Vec3d>  _vertices;
    std::vector<osg::Vec4> _colors;
    MatrixStack _matrixStack;
    osg::BoundingSphere _queryBounds;
};

struct QueryTrianglesHandler : public osgGA::GUIEventHandler
{
    QueryTrianglesHandler(MapNode* mapNode)
        : _mapNode(mapNode),
        _observer(nullptr),
        _enabled(false)
    {
        _marker = new osg::MatrixTransform;
        _marker->addChild(AnnotationUtils::createSphere(1.0, Color::White));
        _marker->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE), 1);
        root->addChild(_marker);
        _marker->setNodeMask(_enabled ? ~0u : 0);
    }

    bool getEnabled() const { return _enabled; }
    void setEnabled(bool enabled)
    {
        _enabled = enabled;
        _marker->setNodeMask(_enabled ? ~0u : 0);
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        if (!_enabled) return false;

        osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());

        if (ea.getEventType() == ea.MOVE)
        {
            osg::Vec3d world;

            if (_observer)
            {
                auto itr = std::find(observers.begin(), observers.end(), _observer);
                if (itr != observers.end())
                {
                    observers.erase(itr);
                }
                _observer = nullptr;
            }

            

            osgUtil::LineSegmentIntersector::Intersections hits;

            osg::NodePath path;
            path.push_back(_mapNode);

            if (view->computeIntersections(ea.getX(), ea.getY(), path, hits))
            //if (_mapNode->getTerrain()->getWorldCoordsUnderMouse(view, ea.getX(), ea.getY(), world))
            {
                world = hits.begin()->getWorldIntersectPoint();

                _marker->setNodeMask(~0u);

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

#if 0
                _observer = new Observer(osg::BoundingSphered(world, query_range));
                _observer->setName("Mouse cursor");
                observers.push_back(_observer);
#endif
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
    bool _enabled;

    Observer* _observer;
};


struct IntersectionQuery
{
    IntersectionQuery() :
        valid(false)
    {
    }

    IntersectionQuery(const osg::Vec3d& _start, const osg::Vec3d& _end) :
        valid(false),
        start(_start),
        end(_end)
    {
    }

    osg::Vec3d start;
    osg::Vec3d end;
    bool valid;
    osg::Vec3d hit;
};

typedef std::vector< IntersectionQuery > QueryList;

static std::vector < osg::ref_ptr< osgUtil::LineSegmentIntersector > > intersectorCache;

void computeIntersections(osg::Node* node, std::vector< IntersectionQuery >& queries, unsigned int start, unsigned int count)
{
    osg::ref_ptr<osgUtil::IntersectorGroup> ivGroup = new osgUtil::IntersectorGroup();
    for (unsigned int i = start; i < start + count; i++)
    {
        const IntersectionQuery& q = queries[i];
        //osg::ref_ptr<osgUtil::LineSegmentIntersector> lsi = new osgUtil::LineSegmentIntersector(q.start, q.end);
        //ivGroup->addIntersector(lsi.get());
        // Assumes the size of the intersectorCache matches the size of the query list
        osg::ref_ptr<osgUtil::LineSegmentIntersector> lsi = intersectorCache[i].get();
        lsi->reset();
        lsi->setStart(q.start);
        lsi->setEnd(q.end);
        ivGroup->addIntersector(lsi.get());
    }
    osgUtil::IntersectionVisitor iv;
    iv.setIntersector(ivGroup.get());
    node->accept(iv);

    //for (unsigned int i = 0; i < queries.size(); i++)
    for (unsigned int i = start; i < start + count; i++)
    {
        IntersectionQuery& q = queries[i];
        osgUtil::LineSegmentIntersector* los = static_cast<osgUtil::LineSegmentIntersector*>(ivGroup->getIntersectors()[i - start].get());
        if (!los)
            continue;

        osgUtil::LineSegmentIntersector::Intersections& hits = los->getIntersections();

        if (!hits.empty())
        {
            q.valid = true;
            q.hit = hits.begin()->getWorldIntersectPoint();
        }
    }
}

void computeIntersectionsSerial(osg::Node* node, std::vector< IntersectionQuery >& queries)
{
    osg::ref_ptr<osgUtil::LineSegmentIntersector> lsi = new osgUtil::LineSegmentIntersector(osg::Vec3d(0,0,0), osg::Vec3d(0,0,0));
    for (unsigned int i = 0; i < queries.size(); ++i)
    {
        IntersectionQuery& q = queries[i];
        osgUtil::IntersectionVisitor iv;
        lsi->reset();
        lsi->setStart(q.start);
        lsi->setEnd(q.end);
        iv.setIntersector(lsi.get());
        node->accept(iv);
        const osgUtil::LineSegmentIntersector::Intersections& hits = lsi->getIntersections();
        if (!hits.empty())
        {
            q.valid = true;
            q.hit = hits.begin()->getWorldIntersectPoint();
        }
    }
}

void computeIntersectionsThreaded(osg::Node* node, std::vector< IntersectionQuery >& queries)
{
    if (intersectorCache.size() < queries.size())
    {
        std::cout << "Resizing intersections to " << queries.size() << std::endl;
        intersectorCache.resize(queries.size());
        for (unsigned int i = 0; i < intersectorCache.size(); ++i)
        {
            if (!intersectorCache[i].valid())
            {
                intersectorCache[i] = new osgUtil::LineSegmentIntersector(osg::Vec3d(0,0,0), osg::Vec3d(0,0,0));
            }
        }
    }

    JobArena::get("oe.intersections")->setConcurrency(num_threads);

    // Poor man's parallel for
    JobGroup intersections;

    //unsigned int workSize = 500;
    // Try to split the jobs evenly among the threads
    unsigned int start = 0;
    unsigned int workSize = ceil((double)queries.size() / (double)num_threads);
    //std::cout << "Computed a work size of " << workSize << std::endl;

    unsigned int numJobs = 0;

    while (true)
    {
        unsigned int curStart = start;
        unsigned int curSize = curStart + workSize <= queries.size() ? workSize : queries.size() - curStart;
        if (curSize > 0)
        {
            Job job;
            job.setArena("oe.intersections");
            job.setGroup(&intersections);
            job.dispatch([node, curStart, curSize, &queries](Cancelable*) {
                computeIntersections(node, queries, curStart, curSize);
            });
            ++numJobs;
        }
        start += workSize;
        if (start >= queries.size())
        {
            break;
        }
    }
    //std::cout << "Dispatched " << numJobs << " jobs" << std::endl;
    intersections.join();
}


void generateQueries(const GeoPoint& mapPoint, QueryList& queries)
{
    osg::Matrixd l2w, w2l;
    mapPoint.createLocalToWorld(l2w);
    mapPoint.createWorldToLocal(w2l);

    float latSpan = maxLat - minLat;
    float lonSpan = maxLon - minLon;

    float latSize = latSpan / latSegments; // degrees
    float lonSize = lonSpan / lonSegments; // degrees

    double radius = query_range;
    for (int y = 0; y <= latSegments; ++y)
    {
        float lat = minLat + latSize * (float)y;
        for (int x = 0; x < lonSegments; ++x)
        {
            float lon = minLon + lonSize * (float)x;

            float u = osg::DegreesToRadians(lon);
            float v = osg::DegreesToRadians(lat);
            float cos_u = cosf(u);
            float sin_u = sinf(u);
            float cos_v = cosf(v);
            float sin_v = sinf(v);

            osg::Vec3d start(0.0, 0.0, 0.0);
            osg::Vec3d end(radius * cos_u * cos_v,
                radius * sin_u * cos_v,
                radius * sin_v);
            start = start * l2w;
            end = end * l2w;

            queries.emplace_back(start, end);
        }
    }
}

class QueryRenderer : public osg::MatrixTransform
{
public:
    QueryRenderer()
    {
        _geom = new osg::Geometry;
        _geom->setUseDisplayList(false);
        _geom->setUseVertexBufferObjects(true);
        _geom->setDataVariance(osg::Object::DYNAMIC);

        _verts = new osg::Vec3Array;
        _geom->setVertexArray(_verts);

        _colors = new osg::Vec4Array;
        _geom->setColorArray(_colors, osg::Array::BIND_PER_VERTEX);

        _verts->getVertexBufferObject()->setUsage(GL_DYNAMIC_DRAW_ARB);
        _colors->getVertexBufferObject()->setUsage(GL_DYNAMIC_DRAW_ARB);

        _drawArrays = new osg::DrawArrays(GL_LINES);
        _geom->addPrimitiveSet(_drawArrays);

        _geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        addChild(_geom);
    }

    void update(const QueryList& queries)
    {
        if (queries.empty())
        {
            setNodeMask(0);
            return;
        }

        osg::Vec3d anchor = queries[0].start;
        setMatrix(osg::Matrixd::translate(anchor));

        if (_verts->size() < queries.size() * 2)
        {
            _verts->resize(queries.size() * 2);
            _colors->resize(queries.size() * 2);
        }

        unsigned index = 0;
        for (unsigned int i = 0; i < queries.size(); ++i)
        {
            unsigned int index = i * 2;
            auto& q = queries[i];
            if (q.valid)
            {
                (*_verts)[index] = q.start - anchor;
                (*_verts)[index+1] = q.hit - anchor;
                (*_colors)[index] = osg::Vec4(1, 0, 0, 1);
                (*_colors)[index + 1] = osg::Vec4(1, 0, 0, 1);
            }
            else
            {
                (*_verts)[index] = q.start - anchor;
                (*_verts)[index + 1] = q.end - anchor;
                (*_colors)[index] = osg::Vec4(0, 1, 0, 1);
                (*_colors)[index + 1] = osg::Vec4(0, 1, 0, 1);
            }
        }

        _verts->dirty();
        _colors->dirty();

        _drawArrays->set(GL_LINES, 0, queries.size() * 2);
        _drawArrays->dirty();
    }

    osg::Geometry* _geom;
    osg::Vec3Array* _verts;
    osg::Vec4Array* _colors;
    osg::DrawArrays* _drawArrays;
};

struct IntersectorHandler : public osgGA::GUIEventHandler
{
    IntersectorHandler(MapNode* mapNode)
        : _mapNode(mapNode),
        _enabled(false)
    {
        _queryRenderer = new QueryRenderer;
        root->addChild(_queryRenderer);
        _queryRenderer->setNodeMask(_enabled ? ~0u : 0);
    }

    bool getEnabled() const { return _enabled; }
    void setEnabled(bool enabled)
    {
        _enabled = enabled;
        _queryRenderer->setNodeMask(_enabled ? ~0u : 0);
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        if (!_enabled) return false;

        osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());

        if (ea.getEventType() == ea.MOVE)
        {
            osg::Vec3d world;
            osgUtil::LineSegmentIntersector::Intersections hits;
            osg::NodePath path;
            path.push_back(_mapNode);

            if (view->computeIntersections(ea.getX(), ea.getY(), path, hits))
            {
                _queryRenderer->setNodeMask(~0u);
                // Get the point under the mouse:
                world = hits.begin()->getWorldIntersectPoint();

                // convert to map coords:
                GeoPoint mapPoint;
                mapPoint.fromWorld(_mapNode->getMapSRS(), world);
                // Raise it up a bit so it's not right on the ground.
                mapPoint.alt() += altAdjust;
                mapPoint.toWorld(world);

                const SpatialReference* geoSRS = SpatialReference::create("wgs84");
                QueryList queries;

                generateQueries(mapPoint, queries);

#if 0
                double delta = 0.02;
                unsigned int numEntitites = 50;
                Random prng;
                for (unsigned int i = 0; i < numEntitites; ++i)
                {
                    double lon = (mapPoint.x() - delta/2.0) + prng.next() * delta;
                    double lat = (mapPoint.y() - delta / 2.0) + prng.next() * delta;
                    GeoPoint pt(geoSRS, lon, lat, mapPoint.alt());
                    generateQueries(pt, queries);
                    //generateQueries(mapPoint, queries);
                }
#endif

                //generateQueries(mapPoint, queries);

                auto startTime = std::chrono::high_resolution_clock::now();
                if (serial)
                {
                    computeIntersectionsSerial(_mapNode, queries);
                }
                else
                {
                    computeIntersectionsThreaded(_mapNode, queries);
                }

                auto endTime = std::chrono::high_resolution_clock::now();
                intersection_time = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime).count();
                num_intersections = queries.size();

                _queryRenderer->update(queries);

#if 0
                if (_results)
                {
                    root->removeChild(_results);
                    _results = nullptr;
                }

                if (queries.size() > 0)
                {
                    _results = new osg::MatrixTransform;

                    osg::Geometry* lineGeom = new osg::Geometry;
                    lineGeom->setUseVertexBufferObjects(true);
                    lineGeom->setUseDisplayList(false);
                    osg::Vec3Array* verts = new osg::Vec3Array;
                    lineGeom->setVertexArray(verts);

                    osg::Vec4Array* colors = new osg::Vec4Array;
                    lineGeom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);

                    osg::Vec3d anchor = queries[0].start;
                    _results->setMatrix(osg::Matrixd::translate(anchor));

                    for (auto &q : queries)
                    {
                        if (q.valid)
                        {
                            verts->push_back(q.start - anchor);
                            verts->push_back(q.hit - anchor);
                            colors->push_back(osg::Vec4(1, 0, 0, 1));
                            colors->push_back(osg::Vec4(1, 0, 0, 1));
                        }
                        else
                        {
                            verts->push_back(q.start - anchor);
                            verts->push_back(q.end - anchor);
                            colors->push_back(osg::Vec4(0, 1, 0, 1));
                            colors->push_back(osg::Vec4(0, 1, 0, 1));
                        }
                    }

                    lineGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, verts->size()));
                    lineGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
                    _results->addChild(lineGeom);
                    root->addChild(_results);
                }
#endif
            }
            else
            {
                _queryRenderer->setNodeMask(0u);
            }
        }
        return false;
    }

    osgEarth::MapNode* _mapNode;
    QueryRenderer* _queryRenderer;
    osg::MatrixTransform* _results = nullptr;

    bool _enabled;
};


QueryTrianglesHandler* queryTrianglesHandler = nullptr;
IntersectorHandler* intersectorHandler = nullptr;


struct AddObserverHandler : public osgGA::GUIEventHandler
{
    AddObserverHandler(MapNode* mapNode)
        : _mapNode(mapNode)
    {
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());

        if (ea.getEventType() == ea.PUSH && ea.getButton() == ea.LEFT_MOUSE_BUTTON && ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT)
        {
            osg::Vec3d world;
            osgUtil::LineSegmentIntersector::Intersections hits;
            osg::NodePath path;
            path.push_back(_mapNode);
            if (view->computeIntersections(ea.getX(), ea.getY(), path, hits))
            {
                // Get the point under the mouse:
                world = hits.begin()->getWorldIntersectPoint();

                std::cout << std::setprecision(16) << "Adding observer at " << world << std::endl;

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

struct PredictiveDataLoader : public osg::NodeVisitor
{
    PredictiveDataLoader():
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
    }

    //! Whether all potential data in the areas are loaded
    bool isFullyLoaded() const
    {
        return _fullyLoaded;
    }

    //! Resets isFullyLoaded so you can reuse the same visitor multiple times.
    void reset()
    {
        _fullyLoaded = true;
    }

    bool intersects(osg::Node& node)
    {
        static osg::Matrix identity;
        osg::Matrix& matrix = _matrixStack.empty() ? identity : _matrixStack.back();

        osg::BoundingSphere nodeBounds = node.getBound();
        osg::BoundingSphered worldBounds(nodeBounds.center(), nodeBounds.radius());
        worldBounds.center() = worldBounds.center() * matrix;

        bool result = false;
        for (auto& bs : _areasToLoad)
        {
            if (bs.intersects(worldBounds))
            {
                result = true;
                break;
            }
        }


        return result;
    }

    float getMinRange(PagedNode2& node)
    {
        static osg::Matrix identity;
        osg::Matrix& matrix = _matrixStack.empty() ? identity : _matrixStack.back();

        osg::BoundingSphere nodeBounds = node.getBound();
        osg::BoundingSphered worldBounds(nodeBounds.center(), nodeBounds.radius());
        worldBounds.center() = worldBounds.center() * matrix;

        float minRange = FLT_MAX;

        for (auto& bs : _areasToLoad)
        {
            float range = (bs.center() - worldBounds.center()).length();
            range /= range_boost;
            if (range < minRange) minRange = range;
        }

        return minRange;
    }

    //! A list of bounding spheres in world coordinates to intersect the scene graph against.
    std::vector<osg::BoundingSphered>& getAreasToLoad() { return _areasToLoad; }

    void apply(osg::Node& node)
    {
        if (intersects(node))
        {
            PagedNode2* pagedNode = dynamic_cast<PagedNode2*>(&node);
            if (pagedNode)
            {
                apply(*pagedNode);
            }
            traverse(node);
        }
    }

    void apply(PagedNode2& pagedNode)
    {
        float range = getMinRange(pagedNode);
        if (range < pagedNode.getMaxRange())
        {
            if (!pagedNode.isLoaded())
            {
                float priority = -range;
                // TODO:  ICO
                pagedNode.load(priority, nullptr);
                _fullyLoaded = false;
            }
            pagedNode.touch();
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

    inline void pushMatrix(osg::Matrix& matrix) { _matrixStack.push_back(matrix); }
    inline void popMatrix() { _matrixStack.pop_back(); }

    typedef std::vector<osg::Matrix> MatrixStack;
    MatrixStack _matrixStack;
    bool _fullyLoaded = true;

    std::vector< osg::BoundingSphered > _areasToLoad;
};

class LoadableNodesGUI : public BaseGUI
{
public:
    LoadableNodesGUI() :
        BaseGUI("Loadable Nodes Inspector")
    {
    }

    void load(const Config& conf) override
    {
    }

    void save(Config& conf) override
    {
    }

protected:
    void draw(osg::RenderInfo& ri) override
    {
        if (!isVisible()) return;

        if (!_mapNode.valid())
            _mapNode = osgEarth::findTopMostNodeOfType<MapNode>(ri.getCurrentCamera());

        ImGui::Begin(name(), visible());

        bool dirty = (ri.getView()->getFrameStamp()->getFrameNumber() % 60 == 0);
        dirty |= ImGui::Checkbox("Loaded", &_loaded); ImGui::SameLine();
        ImGui::Text("%d", _numLoaded);
        dirty |= ImGui::Checkbox("Unloaded", &_unloaded); ImGui::SameLine();
        ImGui::Text("%d", _numUnloaded);

        dirty |= ImGui::Checkbox("PagedNode2", &_pagedNode2); ImGui::SameLine();
        ImGui::Text("Loaded=%d Unloaded=%d", _numLoadedPagedNode2, _numUnloadedPagedNode2);

        //dirty |= ImGui::Checkbox("Terrain Tiles", &_terrainTiles); ImGui::SameLine();
        //ImGui::Text("Loaded=%d Unloaded=%d", _numLoadedTerrainTiles, _numUnloadedTerrainTiles);

        dirty |= ImGui::Checkbox("3D Tiles", &_threedTiles); ImGui::SameLine();
        ImGui::Text("Loaded=%d Unloaded=%d", _numLoadedThreedTiles, _numUnloadedThreedTiles);

        //ImGui::SliderInt("Min Display Level", &_minDisplayLevel, 0, 19);
        //ImGui::SliderInt("Max Display Level", &_maxDisplayLevel, 0, 19);

        if (ImGui::Button("Refresh") || dirty)
        {
            _numLoaded = 0;
            _numUnloaded = 0;
            //_numLoadedTerrainTiles = 0;
            //_numUnloadedTerrainTiles = 0;
            _numLoadedPagedNode2 = 0;
            _numUnloadedPagedNode2 = 0;
            _numLoadedThreedTiles = 0;
            _numUnloadedThreedTiles = 0;


            auto group = new osg::Group;

            osgEarth::FindNodesVisitor<LoadableNode> findLoadableNodes;
            _mapNode->accept(findLoadableNodes);

            if (_boundsNode.valid())
            {
                _mapNode->removeChild(_boundsNode.get());
                _boundsNode = nullptr;
            }

            for (auto& node : findLoadableNodes._results)
            {
                auto n = dynamic_cast<osg::Node*>(node);
                //auto terrainTile = dynamic_cast<TerrainTileNode*>(n);
                auto pagedNode2 = dynamic_cast<PagedNode2*>(n);
                auto threedTiles = dynamic_cast<osgEarth::Contrib::ThreeDTiles::ThreeDTileNode*>(n);
                //if (terrainTile && node->isLoaded()) _numLoadedTerrainTiles++;
                //if (terrainTile && !node->isLoaded()) _numUnloadedTerrainTiles;

                if (pagedNode2 && node->isLoaded()) _numLoadedPagedNode2++;
                if (pagedNode2 && !node->isLoaded()) _numUnloadedPagedNode2++;

                if (threedTiles && node->isLoaded()) _numLoadedThreedTiles++;
                if (threedTiles && !node->isLoaded()) _numUnloadedThreedTiles++;

                if (node->isLoaded()) _numLoaded++;
                if (!node->isLoaded()) _numUnloaded++;

                //if (terrainTile && !_terrainTiles) continue;
                if (pagedNode2 && !_pagedNode2) continue;
                if (threedTiles && !_threedTiles) continue;

                if (node->isLoaded() && _loaded || !node->isLoaded() && _unloaded)
                {
                    osg::Vec4 color = node->isLoaded() ? osg::Vec4(0.0, 1.0, 0.0, 1.0) : osg::Vec4(1.0, 0.0, 0.0, 1.0);

                    /*
                    if (terrainTile)
                    {
                        color = Color::Purple;
                        if (terrainTile->getKey().getLevelOfDetail() < _minDisplayLevel || terrainTile->getKey().getLevelOfDetail() > _maxDisplayLevel)
                        {
                            continue;
                        }
                    }
                    */

                    if (threedTiles)
                    {
                        color = node->isLoaded() ? Color::Cyan : Color::White;
                    }

                    osg::NodePath nodePath = n->getParentalNodePaths()[0];
                    osg::Matrixd localToWorld = osg::computeLocalToWorld(nodePath);

                    osg::BoundingSphered bs = osg::BoundingSphered(n->getBound().center(), n->getBound().radius());
                    if (bs.radius() > 0)
                    {
                        auto mt = new osg::MatrixTransform;
                        localToWorld.preMultTranslate(n->getBound().center());
                        mt->setMatrix(localToWorld);
                        mt->addChild(AnnotationUtils::createSphere(n->getBound().radius(), color));
                        mt->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE), 1);
                        group->addChild(mt);
                    }
                }
            }

            _boundsNode = group;
            _mapNode->addChild(_boundsNode.get());
        }

        if (!*visible())
        {
            if (_boundsNode.valid())
            {
                _mapNode->removeChild(_boundsNode.get());
            }
        }
        ImGui::End();
    }

    osg::observer_ptr<osg::Node> _boundsNode;
    osg::observer_ptr<MapNode> _mapNode;

    bool _loaded = true;
    bool _unloaded = true;
    unsigned int _numLoaded = 0;
    unsigned int _numUnloaded = 0;
    int _minDisplayLevel = 10;
    int _maxDisplayLevel = 13;

    int _numLoadedPagedNode2 = 0;
    int _numUnloadedPagedNode2 = 0;

    int _numLoadedTerrainTiles = 0;
    int _numUnloadedTerrainTiles = 0;

    int _numUnloadedThreedTiles = 0;
    int _numLoadedThreedTiles = 0;

    bool _terrainTiles = true;
    bool _pagedNode2 = true;
    bool _threedTiles = true;
};


class TrianglesGUI : public BaseGUI
{
public:
    TrianglesGUI(osgViewer::View* view, MapNode* mapNode, EarthManipulator* earthManip) :
        BaseGUI("Triangles"),
        _mapNode(mapNode),
        _earthManip(earthManip),
        _view(view)
    {
    }

    void load(const Config& conf) override
    {
    }

    void save(Config& conf) override
    {
    }

protected:
    void draw(osg::RenderInfo& ri) override
    {
        if (!isVisible()) return;

        ImGui::Begin(name(), visible());

        bool queryTriangleEnabled = queryTrianglesHandler->getEnabled();
        ImGui::Checkbox("Query Triangles", &queryTriangleEnabled);
        queryTrianglesHandler->setEnabled(queryTriangleEnabled);

        bool intersectHandlerEnabled = intersectorHandler->getEnabled();
        ImGui::Checkbox("Collect intersections", &intersectHandlerEnabled);
        intersectorHandler->setEnabled(intersectHandlerEnabled);
        if (intersectHandlerEnabled)
        {
            ImGui::Checkbox("Serial", &serial);
            ImGui::InputInt("Num Threads", &num_threads);
            ImGui::InputFloat("Min Lat", &minLat);
            ImGui::InputFloat("Max Lat", &maxLat);
            ImGui::InputFloat("Min Lon", &minLon);
            ImGui::InputFloat("Max Lon", &maxLon);
            ImGui::InputInt("Num Lat", &latSegments);
            ImGui::InputInt("Num Lon", &lonSegments);
            ImGui::InputFloat("Alt Adjust", &altAdjust);

            ImGui::Text("%d intersections in %lf ms", num_intersections, intersection_time / 1e6);
            ImGui::Text("%lf intersections/s", num_intersections * (1e9 / intersection_time));
        }

        ImGui::SliderFloat("Range Boost", &range_boost, 1.0, 10.0);
        ImGui::SliderFloat("Query Range", &query_range, 1.0, 2000.0f);

        ImGui::Checkbox("Load Highest Resolution Only", &load_highres_only);

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

        ImGui::Checkbox("Predictive Loading", &predictive_loading);


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
        if (ImGui::Button("Add AGI"))
        {
            auto observer = new Observer(osg::BoundingSphered(osg::Vec3d(1216434.192393575, -4736488.256417232, 4081467.571742828), 1000.0));
            osgEarth::GeoPoint pt;
            pt.fromWorld(SpatialReference::create("wgs84"), observer->getBounds().center());
            observer->setName("AGI");
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
        ImGui::Text("Data load took %lf", (double)predictive_data_loading_time_ns / 1.0e6);
        ImGui::End();
    }

    osg::ref_ptr< MapNode > _mapNode;
    osg::ref_ptr<EarthManipulator> _earthManip;
    osgViewer::View* _view;
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
    viewer.setRealizeOperation(new GUI::ApplicationGUI::RealizeOperation);

    root = new osg::Group;

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Node* node = MapNodeHelper().loadWithoutControls(arguments, &viewer);
    if (node)
    {
        MapNode* mapNode = MapNode::findMapNode(node);
        if (mapNode)
        {
            //viewer.getEventHandlers().push_front(new MyGUI(&viewer, mapNode, manip));
            auto gui = new GUI::ApplicationGUI(true);
            gui->add(new TrianglesGUI(&viewer, mapNode, manip));
            gui->add(new LoadableNodesGUI());
            viewer.getEventHandlers().push_front(gui);
        }

        queryTrianglesHandler = new QueryTrianglesHandler(mapNode);
        intersectorHandler = new IntersectorHandler(mapNode);

        viewer.getEventHandlers().push_front(queryTrianglesHandler);
        viewer.getEventHandlers().push_front(intersectorHandler);
        viewer.getEventHandlers().push_front(new AddObserverHandler(mapNode));

        root->addChild(new ObserversNode());
        root->addChild(node);

        viewer.setSceneData(root);

        while (!viewer.done())
        {
            if (loading_data)
            {
                LoadDataVisitor v;
                v.setLoadHighestResolutionOnly(load_highres_only);
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


            if (predictive_loading)
            {
                PredictiveDataLoader loader;
                for (auto& o : observers)
                {
                    loader.getAreasToLoad().push_back(o->getBounds());
                }

                // Add an observer for the camera
                osg::Vec3d eye, center, up;
                viewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);

#if 0
                if (!cameraObserver.valid())
                {
                    cameraObserver = new Observer();
                    cameraObserver->setName("Camera");
                    observers.push_back(cameraObserver);
                }

                if (cameraObserver.valid())
                {
                    double fovy, ar, zNear, zFar;
                    viewer.getCamera()->getProjectionMatrixAsPerspective(fovy, ar, zNear, zFar);
                    osg::BoundingSphered bounds(eye, 100000.0);
                    cameraObserver->setBounds(bounds);
                    loader.getAreasToLoad().push_back(cameraObserver->getBounds());
                }
#endif

                auto startTime = std::chrono::high_resolution_clock::now();
                //root->accept(loader);
                mapNode->getLayerNodeGroup()->accept(loader);
                auto endTime = std::chrono::high_resolution_clock::now();
                predictive_data_loading_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime).count();

                loading_data = !loader.isFullyLoaded();
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
