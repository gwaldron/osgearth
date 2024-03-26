/* -*-c++-*- */
/* CMakeListst.txt
INCLUDE_DIRECTORIES(${ OSG_INCLUDE_DIRS } "H:/devel/clipper2/install/include")

SET(CLIPPER2_LIB "H:/devel/clipper2/install/lib/clipper2z.lib")

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY
    OPENTHREADS_LIBRARY  CLIPPER2_LIB)

SET(TARGET_SRC osgearth_viewer.cpp)

SETUP_IMGUI_APPLICATION(osgearth_viewer)
*/

#include <osgEarth/ImGui/ImGuiApp>
#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/Threading>
#include <osgEarth/ShaderGenerator>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/PolygonOffset>
#include <osgGA/TrackballManipulator>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/Optimizer>
#include <osgEarth/LineDrawable>
#include <osgEarth/PointDrawable>
#include <iostream>
#include <osgEarth/Math>
#include <osgEarth/GLUtils>
#include <osgEarth/CullingUtils>
#include <osgEarth/XYZFeatureSource>
#include <osgEarth/TileRasterizer>
#include <osgEarth/SimplePager>
#include <osgEarth/AltitudeFilter>
#include <osgEarth/DrapeableNode>
#include <osgEarth/rtree.h>
#include <osgEarth/TileMesher>
#include <osgEarth/TerrainMeshLayer>
#include <osgEarth/Containers>
#include <osgEarth/CropFilter>
#include <osgEarth/Utils>
#include <osgEarth/Chonk>

#include <clipper2/clipper.h>
#include <osgEarth/weemesh.h>
#include <osgEarth/earcut.hpp>

namespace mapbox {
    namespace util {
        template <>
        struct nth<0, osg::Vec2> {
            inline static float get(const osg::Vec2& t) {
                return t.x();
            };
        };

        template <>
        struct nth<1, osg::Vec2> {
            inline static float get(const osg::Vec2& t) {
                return t.y();
            };
        };

        template <>
        struct nth<0, osg::Vec3d> {
            inline static float get(const osg::Vec3d& t) {
                return t.x();
            };
        };

        template <>
        struct nth<1, osg::Vec3d> {
            inline static float get(const osg::Vec3d& t) {
                return t.y();
            };
        };
    }
}


using namespace osgEarth;


#if 0
struct ClampGeometry : public osg::NodeVisitor
{
    ElevationPool* _pool;
    ElevationPool::WorkingSet _ws;

    ClampGeometry() {
        setTraversalMode(TRAVERSE_ALL_CHILDREN);
        setNodeMaskOverride(~0);
    }

    void setMap(const Map* map) {
        _pool = map->getElevationPool();
    }

    void apply(osg::Drawable& d) override
    {
        auto geom = d.asGeometry();
        if (geom)
        {
            auto verts = static_cast<osg::Vec3Array*>(geom->getVertexArray());
            auto& vec = verts->asVector();
            _pool->sampleMapCoords(vec.begin(), vec.end(), &_ws, nullptr);
        }
    }
};
#endif



using index_t = uint32_t;
using rank_t = uint8_t;
using point_t = osg::Vec3d;
using vec_t = osg::Vec3d;
using FID = uint8_t;
std::atomic<uint32_t> s_uidgen;
const double EPSILON = 1.0; // meters

vec_t cross(const vec_t& a, const vec_t& b)
{
    return a ^ b;
}

double cross_2d(const vec_t& a, const vec_t& b)
{
    return a.x() * b.y() - b.x() * a.y();
}

vec_t normalize(const vec_t& v1)
{
    double L = v1.length();
    return L > 0.0 ? v1 / L : v1;
}

double dot(const vec_t& v1, const vec_t& v2)
{
    return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
}

std::string str(const vec_t& v)
{
    return std::to_string(v.x()) + "," + std::to_string(v.y()) + "," + std::to_string(v.z());
}



#define ROAD_THICKNESS 0.0f //0.1f //0.5f

struct edge_t;

struct art_t
{
    osg::ref_ptr<osg::Texture> texture;
    osg::ref_ptr<osg::StateSet> stateset;
    float width; // m
    float length; // m
    bool tiled_along_width = true;
    bool tiled_along_length = true;
};

struct properties_t
{
    std::string kind;
    art_t surface;
    art_t lines;
    art_t crossing;
    float default_width = 5.0f;
    rank_t rank = 0;
    uint8_t lanes = 2;
};

struct intersection_t
{
    enum Type { NONE, GENERIC, MERGE, T, Y, X } type;
    float backoff_length;
    float crossing_length;
    vec_t orientation;

    intersection_t() :
        type(NONE), backoff_length(0.0f), crossing_length(0.0f), orientation(1,0,0){ }
};

struct node_t
{
    const UID uid;
    const point_t p;
    mutable std::vector<edge_t*> edges;
    intersection_t intersection;
    bool has_crossing = false;
    int subgraph_id = -1;
    const properties_t* props = nullptr;

    node_t() : uid(s_uidgen++) {
    }
    node_t(double x, double y) : uid(s_uidgen++), p(x, y, 0) {
    }

    // equality function for unordered_set/map
    bool operator == (const node_t& rhs) const {
        return fabs(p.x() - rhs.p.x()) <= EPSILON && fabs(p.y() - rhs.p.y()) <= EPSILON;
    }
    bool operator != (const node_t& rhs) const {
        return !operator==(rhs);
    }
    // hash function for unordered_set/map
    std::size_t operator()(const node_t& me) const {
        return hash_value_unsigned((int)(1000 * me.p.x()), (unsigned)(1000 * me.p.y()), (unsigned)(1000 * me.p.z()));
    }
    // only used for geospatial comparison (not for a set/map)
    bool operator < (const node_t& rhs) const {
        if (p.x() < rhs.p.x()) return true;
        if (p.x() > rhs.p.x()) return false;
        return p.y() < rhs.p.y();
    }
};

struct edge_t
{
    const UID uid;
    const node_t& node1;      // first endpoint
    const node_t& node2;      // second endpoint
    FID fid;       // unique source id (feature id)
    float width;  // m
    const properties_t* props = nullptr;

    float angle = 0.0f; // relative to +x axis, used for sorting

    // end points, extruded to width
    point_t node1_left, node1_right;
    point_t node2_left, node2_right;

    // end points minus crossing area, extruded to width
    point_t node1_cr_left, node1_cr_right;
    point_t node2_cr_left, node2_cr_right;

    // constructor keeps the nodes sorted in geospatial order.
    edge_t(const node_t& a, const node_t& b, float w, const properties_t* d) :
        uid(s_uidgen++),
        node1(a < b ? a : b),
        node2(a < b ? b : a),
        width(w),
        props(d)
    {
        auto dir = node2.p - node1.p;
        angle = atan2(dir.y(), dir.x());
        if (w <= 0.0f && props != nullptr)
            width = props->default_width;
    }

    vec_t direction() const {
        return normalize(node2.p - node1.p);
    }

    const node_t& other_node(const node_t& n) const {
        return n == node1 ? node2 : node1;
    }

    // get the node that the two edges share, or null if they don't
    const node_t* node_shared_with(const edge_t& rhs) const {
        if (node1 == rhs.node1 || node1 == rhs.node2) return &node1;
        else if (node2 == rhs.node1 || node2 == rhs.node2) return &node2;
        else return nullptr;
    }
    // get the node that is NOT shared between the 2 edges
    const node_t* node_not_shared_with(const edge_t& rhs) const {
        if (node1 != rhs.node1 && node1 != rhs.node2) return &node1;
        else if (node2 != rhs.node1 && node2 != rhs.node2) return &node2;
        else return nullptr;
    }

    // dot product of the two vectors forming the 2 edges and
    // extending from their shared node
    double cos_of_angle_with(const edge_t& rhs) const {
        auto shared = node_shared_with(rhs);
        if (!shared) return 0;
        auto n1 = node_shared_with(rhs);
        if (!n1) return 0;
        auto n2 = rhs.node_not_shared_with(*this);
        if (!n2) return 0;
        auto n3 = node_not_shared_with(rhs);
        if (!n3) return 0;
        auto v2 = normalize(n2->p - n1->p);
        auto v3 = normalize(n3->p - n1->p);
        return dot(v2, v3);
    }

    // true if they intersect, and returns the point in i.
    bool intersects(const edge_t rhs, point_t& i) const {
        if (node_shared_with(rhs)) return false;
        auto d1 = node2.p - node1.p;
        auto d2 = rhs.node2.p - rhs.node1.p;
        auto determinant = cross_2d(d1, d2);
        if (equivalent(determinant, 0.0))
            return false;
        double u = cross_2d(rhs.node1.p - node1.p, d2) / determinant;
        double v = cross_2d(rhs.node1.p - node1.p, d1) / determinant;
        i = node1.p + d1 * u;
        return (u > 0.0 && u < 1.0 && v > 0.0 && v < 1.0);
            
    }

    bool operator == (const edge_t& rhs) const {
        return uid == rhs.uid;
        //return (node1 == rhs.node1 && node2 == rhs.node2);
    }
    bool operator != (const edge_t& rhs) const {
        return !operator==(rhs);
    }
    bool operator < (const edge_t& rhs) const {
        return uid < rhs.uid;
    }
    std::size_t operator()(const edge_t& me) const {
        return me.uid;
    }
};

struct graph_t
{
    int num_subgraphs = -1;
    std::unordered_set<node_t, node_t> nodes;
    std::set<edge_t> edges;

    // sets a node's id and then propagates it to all neighbors
    void set_subgraph_id(node_t& n, int id) {
        n.subgraph_id = id;
        for (auto& edge : n.edges) {
            auto& other_node = edge->other_node(n);
            if (other_node.subgraph_id != id)
                set_subgraph_id(const_cast<node_t&>(other_node), id);
        }
    }

     node_t* add_node(double x1, double y1, const properties_t* props) {
        auto& n = nodes.emplace(x1, y1).first;
        auto node = const_cast<node_t*>(&(*n));
        node->props = props;
        return node;
    }

    edge_t* add_edge(double x1, double y1, double x2, double y2, float width, const properties_t* props) {
        auto& node1 = nodes.emplace(x1, y1).first;
        auto& node2 = nodes.emplace(x2, y2).first;
        if (node1->uid == node2->uid) // safety catch; shouldn't happen
            return nullptr;
        auto& e = edges.emplace(*node1, *node2, width, props);
        auto edge = const_cast<edge_t*>(&(*e.first));
        if (!e.second)
            return edge; // dupe
        node1->edges.emplace_back(edge);
        node2->edges.emplace_back(edge);

        auto& n1 = const_cast<node_t&>(*node1);
        auto& n2 = const_cast<node_t&>(*node2);

        // propagate the subgraph ids.
        int new_id =
            n1.subgraph_id < 0 && n2.subgraph_id < 0 ? ++num_subgraphs :
            n1.subgraph_id < 0 ? n2.subgraph_id :
            n2.subgraph_id < 0 ? n1.subgraph_id :
            std::min(n1.subgraph_id, n2.subgraph_id);

        set_subgraph_id(n1, new_id); // will set n2 as well

        return edge;
    }
    // find crossing segments and break them up.
    void split_intersecting_edges()
    {
        bool restart = true;

        while(restart)
        {
            restart = false;
            restart = false;
            for(auto lhs_iter = edges.begin(); lhs_iter != edges.end() && !restart; ++lhs_iter)
            {
                auto& lhs = *lhs_iter;
                auto rhs_iter = lhs_iter;
                for (++rhs_iter; rhs_iter != edges.end() && !restart;)
                {
                    auto& rhs = *rhs_iter;
                    if (lhs != rhs)
                    {
                        point_t i;
                        if (lhs.intersects(rhs, i))
                        {
                            add_edge(lhs.node1.p.x(), lhs.node1.p.y(), i.x(), i.y(), lhs.width, lhs.props);
                            add_edge(lhs.node2.p.x(), lhs.node2.p.y(), i.x(), i.y(), lhs.width, lhs.props);
                            add_edge(rhs.node1.p.x(), rhs.node1.p.y(), i.x(), i.y(), rhs.width, rhs.props);
                            add_edge(rhs.node2.p.x(), rhs.node2.p.y(), i.x(), i.y(), rhs.width, rhs.props);
                            edges.erase(lhs_iter);
                            edges.erase(rhs_iter);
                            restart = true;
                        }
                    }
                    if (restart) break;
                    else  ++rhs_iter;
                }
                if (restart) break;
                //else  ++lhs_iter;
            }
        }
    }
};

struct edge_tree_t
{
    using index_t = RTree<edge_t*, double, 2>;
    index_t _index;

    edge_tree_t(graph_t& graph)
    {
        for (auto& edge : graph.edges)
        {
            double min[2] = { std::min(edge.node1.p.x(), edge.node2.p.x()), std::min(edge.node1.p.y(), edge.node2.p.y()) };
            double max[2] = { std::max(edge.node1.p.x(), edge.node2.p.x()), std::max(edge.node1.p.y(), edge.node2.p.y()) };
            _index.Insert(min, max, const_cast<edge_t*>(&edge));
        }
    }

    point_t closestPointOnEdgeTo(double x, double y) const
    {
        osg::Vec2d point(x, y);
        double closest2 = DBL_MAX;
        osg::Vec3d closest_point(0, 0, NO_DATA_VALUE);

        auto function = [&](const edge_t* edge) -> bool
        {
            osg::Vec2d e1(edge->node1.p.x(), edge->node1.p.y());
            osg::Vec2d e2(edge->node2.p.x(), edge->node2.p.y());
            osg::Vec2d result;

            osg::Vec2d qp = e2 - e1;
            osg::Vec2d xp = point - e1;
            double u = (xp * qp) / (qp * qp);
            if (u < 0.0) result = e1;
            else if (u > 1.0) result = e2;
            else result = e1 + qp * u;

            double dist2 = (result - point).length2();
            if (dist2 < closest2)
            {
                closest2 = dist2;
                closest_point.set(result.x(), result.y(),
                    edge->node1.p.z() + u * (edge->node2.p.z() - edge->node1.p.z()));
            }

            return true; // true to continue searching
        };

        const double radius = 1000.0;
        double min[2] = { x - radius, y - radius }, max[2] = { x + radius, y + radius };
        _index.Search(min, max, function);
        return closest_point;
    }
};


struct Art
{
    //edge_meta_t secondary;
    properties_t road_basic;
    properties_t road_one_lane;

    Art()
    {
        auto asphalt = new osg::Texture2D(osgDB::readRefImageFile("D:/data/textures/ambientcg/Asphalt026B_4K-JPG/Asphalt026B_4K-JPG_Color.jpg"));
        asphalt->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
        asphalt->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
        auto asphalt_ss = new osg::StateSet();
        asphalt_ss->setTextureAttribute(0, asphalt);

        // basic 2 lane road
        {
            auto lines = new osg::Texture2D(osgDB::readRefImageFile("D:/data/textures/ambientcg/RoadLines007_1K-PNG/RoadLines007_1K-PNG_Color.png"));
            //auto lines = new osg::Texture2D(osgDB::readRefImageFile("../data/reference_grid.jpg"));
            lines->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            lines->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
            lines->setMaxAnisotropy(4.0f);
            auto lines_ss = new osg::StateSet();
            lines_ss->setTextureAttribute(0, lines);

            auto crossing = new osg::Texture2D(URI("D:/data/textures/ambientcg/RoadLines004_1K-PNG/RoadLines004_1K-PNG_Color.png").getImage());
            //auto crossing = new osg::Texture2D(osgDB::readRefImageFile("D:/data/textures/road_crossing.png"));
            crossing->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
            crossing->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            crossing->setMaxAnisotropy(4.0f);
            auto crossing_ss = new osg::StateSet();
            crossing_ss->setTextureAttribute(0, crossing);

            road_basic = {
                "road",
                { asphalt, asphalt_ss, 8, 10 },     // surface art, width, length 
                { lines, lines_ss, 5, 5 },        // lines art, width, length
                { crossing, crossing_ss, 0.5, 1.5 }, // crossing art, width, length
                5.0f,                   // default road width(m)
                1,                      // rank
                2                       // lanes
            };
        }

        // skinny one land road
        {
            auto crossing = new osg::Texture2D(osgDB::readRefImageFile("D:/data/textures/road_crossing_simple.png"));
            crossing->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            crossing->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            crossing->setMaxAnisotropy(4.0f);

            road_one_lane = road_basic;
            road_one_lane.default_width = 5.0f;
            road_one_lane.lines.texture = nullptr; // no lines
            road_one_lane.crossing.texture = crossing;
            road_one_lane.crossing.length = 3.0f;
        }

        //secondary = { { texture, 10, 10 }, 20.0f, 4, 4 };
    }
};

void compile_3way_intersection(node_t& node)
{
    // 3 edges in play:
    auto& a = *node.edges[0];
    auto& b = *node.edges[1];
    auto& c = *node.edges[2];

    // direction vectors
    vec_t da = a.direction();
    vec_t db = b.direction();
    vec_t dc = c.direction();

    // dot products
    double ab = dot(da, db);
    double bc = dot(db, dc);
    double ac = dot(da, dc);

    vec_t ori;

    if (ab > ac && ab > bc) // c is the t-edge
    {
        ori = (a.direction() + b.direction()) * 0.5;
    }
    else if (bc > ab && bc > ac) // a is the t-edge
    {
        ori = (b.direction() + c.direction()) * 0.5;
    }
    else // b is the t-edge
    {
        ori = (a.direction() + c.direction()) * 0.5;
    }

    node.intersection.orientation = ori;
}

// figure out the settings for a node crossing, if applicable.
void compile_intersection(node_t& node)
{
    node.intersection.backoff_length = 0.0f;

    if (node.edges.size() < 3)
    {
        node.intersection.type = intersection_t::NONE;
    }
    else
    {
        // generic for starters.
        node.intersection.type = intersection_t::GENERIC;

        // compute backoff distance for connecting edges
        for (auto& edge : node.edges)
        {
            node.intersection.backoff_length = std::max(
                node.intersection.backoff_length, edge->width * 0.5f);
        }
        node.intersection.crossing_length = node.intersection.backoff_length * 0.5;

        // compute the orientation of the crossing geometry
        if (node.edges.size() == 3)
        {
            compile_3way_intersection(node);
        }
    }
}

// squares off an unconnected edge node
void compile_end_cap(edge_t& e, const node_t& n)
{
    vec_t dir = e.direction();
    vec_t left = cross(vec_t(0, 0, 1), dir);
    vec_t extrusion = (left * e.width * 0.5);
    vec_t backoff = (dir * n.intersection.backoff_length);
    vec_t crossing = (dir * n.intersection.crossing_length);

    if (e.node2 == n)
    {
        e.node2_left = e.node2.p + extrusion - backoff;
        e.node2_right = e.node2.p - extrusion - backoff;
        e.node2_cr_left = e.node2_left - crossing;
        e.node2_cr_right = e.node2_right - crossing;
    }
    else
    {
        e.node1_left = e.node1.p + extrusion + backoff;
        e.node1_right = e.node1.p - extrusion + backoff;
        e.node1_cr_left = e.node1_left + crossing;
        e.node1_cr_right = e.node1_right + crossing;
    }
}

// joins two edges together with a miter.
void compile_2way_join(edge_t& A, const node_t& n)
{
    edge_t& B = *n.edges[0] == A ? *n.edges[1] : *n.edges[0];

    // too tight a turn? cap it
    double cosangle = A.cos_of_angle_with(B);
    if (cosangle > 0.0)
    {
        compile_end_cap(A, n);
        return;
    }

    auto other_end_of_A = A.node_not_shared_with(B);
    if (!other_end_of_A) return;
    auto other_end_of_B = B.node_not_shared_with(A);
    if (!other_end_of_B) return;

    vec_t in_A = normalize(other_end_of_A->p - n.p);
    vec_t out_B = normalize(n.p - other_end_of_B->p);

    vec_t left_A = cross(vec_t(0, 0, 1), in_A);
    vec_t left_B = cross(vec_t(0, 0, 1), out_B);
    
    vec_t median = (left_A + left_B) * 0.5;
    double median_len = (A.width * 0.5) / dot(left_B, median);

    if (A.node1 == n)
    {
        A.node1_left = n.p + median * median_len;
        A.node1_right = n.p - median * median_len;
        A.node1_cr_left = A.node1_left;
        A.node1_cr_right = A.node1_right;
    }
    else
    {
        A.node2_left = n.p - median * median_len;
        A.node2_right = n.p + median * median_len;
        A.node2_cr_left = A.node2_left;
        A.node2_cr_right = A.node2_right;
    }
}

// joins a "third" edge to two edges that are joined with a simple miter.
void compile_t_edge(edge_t& t, const node_t& n)
{
    edge_t* a;
    edge_t* b;

    if (*n.edges[0] == t)
        a = (n.edges[1]), b = (n.edges[2]);
    else if (*n.edges[1] == t)
        a = (n.edges[0]), b = (n.edges[2]);
    else
        a = (n.edges[0]), b = (n.edges[1]);


}

// 3-ways can be T, Y, or MERGE.
void compile_3way_join(edge_t& e, const node_t& n)
{
    // 3 edges in play:
    auto& a = *n.edges[0];
    auto& b = *n.edges[1];
    auto& c = *n.edges[2];

    // direction vectors
    vec_t da = normalize(a.node2.p - a.node1.p);
    vec_t db = normalize(b.node2.p - b.node1.p);
    vec_t dc = normalize(c.node2.p - c.node1.p);

    // dot products
    double ab = dot(da, db);
    double bc = dot(db, dc);
    double ac = dot(da, dc);

    // whichever pair of edges are closest in direction
    // (i.e have the highest dot products), the 3rd edge
    // is the "T" edge.
    auto& t_edge =
        (ab > ac && ab > bc) ? c :
        (ac > ab && ac > bc) ? b :
        a;

    // back off for a crossing..
    // todo: detect and manage a small road T-ing a big road.
    compile_end_cap(e, n);
    return;


    if (t_edge == e)
    {
        //compile_t_edge(e, n);
        compile_end_cap(e, n);
    }
    else
    {
        compile_2way_join(e, n);
    }
}

// Calculates all the values needed to render the graph
void compile(graph_t& g)
{
    // sort the edges by angle
    const auto sort_by_angle = [](const edge_t* a, const edge_t* b) { return a->angle < b->angle; };
    for (auto& n : g.nodes)
    {
        auto& node = const_cast<node_t&>(n);
        std::sort(node.edges.begin(), node.edges.end(), sort_by_angle);
    }

    // calculate crossing types and backoff distances
    for (auto& n : g.nodes)
    {
        auto& node = const_cast<node_t&>(n);
        compile_intersection(node);
    }

    // calculate the join points
    for (auto& e : g.edges)
    {
        auto& edge = const_cast<edge_t&>(e);

        for (const node_t* node : { &edge.node1, &edge.node2 })
        {
            if (node->edges.size() == 1)
            {
                compile_end_cap(edge, *node);
            }
            else if (node->edges.size() == 2)
            {
                compile_2way_join(edge, *node);
            }
            else if (node->edges.size() >= 3)
            {
                compile_3way_join(edge, *node);
            }
        }
    }
}

osg::Node* tessellate_edge(const edge_t& edge, std::function<osg::Vec3(const osg::Vec3d&)>& transform)
{
    auto verts = new osg::Vec3Array();
    auto colors = new osg::Vec4Array(osg::Array::BIND_OVERALL);
    auto uvs = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX);

    verts->push_back(transform(edge.node1_right));
    verts->push_back(transform(edge.node2_right));
    verts->push_back(transform(edge.node2_left));
    verts->push_back(transform(edge.node1_left));

    colors->push_back(osg::Vec4(1, 1, 1, 1));

    // rotate the segment onto the x-axis to get the correct repeating
    osg::Quat q;
    q.makeRotate(normalize(edge.node2.p - edge.node1.p), vec_t(1, 0, 0));
    float s = edge.width / edge.props->surface.width;
    if (edge.props->surface.texture->getWrap(osg::Texture::WRAP_S) != osg::Texture::REPEAT)
        s = std::ceil(s);
    float t;
    t = (q * (*verts)[0]).x() / edge.props->surface.length;
    uvs->push_back(osg::Vec2f(0, t));
    t = (q * (*verts)[1]).x() / edge.props->surface.length;
    uvs->push_back(osg::Vec2f(0, t));
    t = (q * (*verts)[2]).x() / edge.props->surface.length;
    uvs->push_back(osg::Vec2f(s, t));
    t = (q * (*verts)[3]).x() / edge.props->surface.length;
    uvs->push_back(osg::Vec2f(s, t));

    const GLushort indices[] = { 0, 1, 2, 0, 2, 3 };
    auto prim = new osg::DrawElementsUShort(GL_TRIANGLES, 6, indices);

    auto geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);
    geom->setName("roads:tessellate_edge");
    geom->setVertexArray(verts);
    geom->setColorArray(colors);
    geom->setTexCoordArray(0, uvs);
    geom->addPrimitiveSet(prim);

    geom->setStateSet(edge.props->surface.stateset);

    return geom;
}

osg::Node* tessellate_edge_center_line(const edge_t& edge, std::function<osg::Vec3(const osg::Vec3d&)>& transform)
{
    auto verts = new osg::Vec3Array();
    auto colors = new osg::Vec4Array(osg::Array::BIND_OVERALL);

    verts->push_back(transform(edge.node1.p));
    verts->push_back(transform(edge.node2.p));

    colors->push_back(osg::Vec4(1, 1, 0, 1));

    const GLushort indices[] = { 0, 1 };
    auto prim = new osg::DrawElementsUShort(GL_LINES, 2, indices);

    auto geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);
    geom->setName("roads:tessellate_edge_center_line");
    geom->setVertexArray(verts);
    geom->setColorArray(colors);
    geom->addPrimitiveSet(prim);

    return geom;
}

osg::Node* tessellate_lane_lines(
    const edge_t& edge,
    std::function<osg::Vec3(const osg::Vec3d&)> transform,
    float z = 0)
{
    if (edge.props->lines.texture == nullptr)
        return new osg::Group();

    auto verts = new osg::Vec3Array();
    auto colors = new osg::Vec4Array(osg::Array::BIND_OVERALL);
    auto uvs = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX);

    osg::Vec3d zup(0, 0, z);

    verts->push_back(transform(edge.node1_cr_right + zup));
    verts->push_back(transform(edge.node2_cr_right + zup));
    verts->push_back(transform(edge.node2_cr_left + zup));
    verts->push_back(transform(edge.node1_cr_left + zup));

    //for (int i = 0; i < 4; ++i)
    //    (*verts)[i].z() = z;

    colors->push_back(osg::Vec4(1, 1, 1, 0.85));

    // rotate the segment onto the x-axis to get the correct repeating
    osg::Quat q;
    q.makeRotate(normalize(edge.node2.p - edge.node1.p), vec_t(1, 0, 0));

    float s = edge.width / edge.props->lines.width;

    float t;
    t = (q * (*verts)[0]).x() / edge.props->lines.length;
    uvs->push_back(osg::Vec2f(0, t));
    t = (q * (*verts)[1]).x() / edge.props->lines.length;
    uvs->push_back(osg::Vec2f(0, t));
    t = (q * (*verts)[2]).x() / edge.props->lines.length;
    uvs->push_back(osg::Vec2f(s, t));
    t = (q * (*verts)[3]).x() / edge.props->lines.length;
    uvs->push_back(osg::Vec2f(s, t));

    const GLushort indices[] = { 0, 1, 2, 0, 2, 3 };
    auto prim = new osg::DrawElementsUShort(GL_TRIANGLES, 6, indices);

    auto geom = new osg::Geometry();
    geom->setName("roads:tessellate_lane_lines");
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);
    geom->setUseVertexArrayObject(true);
    geom->setVertexArray(verts);
    geom->setColorArray(colors);
    geom->setTexCoordArray(0, uvs);
    geom->addPrimitiveSet(prim);

    geom->setStateSet(edge.props->lines.stateset);

    osgUtil::SmoothingVisitor sv;
    sv.setCreaseAngle(0.0f);
    geom->accept(sv);

    return geom;
}

osg::Node* tessellate_crossing(
    const node_t& node,
    std::function<osg::Vec3(const osg::Vec3d&)> transform,
    float z = 0.01)
{
    if (node.props == nullptr ||
        node.props->crossing.texture == nullptr ||
        node.edges.size() != 2)
    {
        return new osg::Group();
    }

    const node_t& node2 = node.edges[0]->other_node(node);
    const node_t& node3 = node.edges[1]->other_node(node);

    vec_t d2 = (node2.p - node.p);
    vec_t d3 = (node.p - node3.p); // (node3.p - node.p);
    vec_t orientation = normalize(d2 + d3);
    //orientation = normalize(cross(vec_t(0, 0, 1), orientation));

    osg::Quat q;
    q.makeRotate(osg::Vec3d(0,1,0), orientation);
    float road_width = node.edges.front()->width; // road width
    float crossing_width = node.props->crossing.width;
    float hw = road_width * 0.5;
    float hl = node.props->crossing.length * 0.5;
    float s = std::ceil(road_width / crossing_width);

    auto geom = new osg::Geometry();
    geom->setName("roads:tessellate_crossing");
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);
    geom->setUseVertexArrayObject(true);

    auto verts = new osg::Vec3Array();
    verts->reserve(4);
    verts->push_back(transform(node.p + q * osg::Vec3d(-hw, -hl, z)));
    verts->push_back(transform(node.p + q * osg::Vec3d(hw, -hl, z)));
    verts->push_back(transform(node.p + q * osg::Vec3d(hw, hl, z)));
    verts->push_back(transform(node.p + q * osg::Vec3d(-hw, hl, z)));
    geom->setVertexArray(verts);

    const GLushort index_data[] = { 0, 1, 2, 0, 2, 3 }; 
    geom->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, 6, index_data));

    const osg::Vec4 color_data[] = { { 1, 1, 1, 1 } };
    geom->setColorArray(new osg::Vec4Array(osg::Array::BIND_OVERALL, 1, color_data));

    const osg::Vec2 uv_data[] = { { 0,0 }, {s, 0}, {s, 1}, {0, 1} };
    geom->setTexCoordArray(0, new osg::Vec2Array(osg::Array::BIND_PER_VERTEX, 4, uv_data));

    geom->setStateSet(node.props->crossing.stateset);

    return geom;
}

osg::Node* tessellate_markings(
    const graph_t& g,
    std::function<osg::Vec3(const osg::Vec3d&)> transform,
    float z = 0)
{
    auto group = new osg::Group();

    for (auto& edge : g.edges)
    {
        group->addChild(tessellate_lane_lines(edge, transform, z)); // z + 0.01));
    }
    for (auto& node : g.nodes)
    {
        if (node.props && node.has_crossing)
        {
            group->addChild(tessellate_crossing(node, transform)); // , z + 0.01)); // z + 0.01));
        }
    }
    return group;
}


osg::Node* tessellate_intersection(const node_t& node)
{
    if (node.intersection.type == intersection_t::NONE)
        return nullptr;

    osg::Quat r;
    r.makeRotate(osg::Vec3d(1, 0, 0), node.intersection.orientation);
    float bh = node.intersection.backoff_length;

    auto geom = new osg::Geometry();

    auto verts = new osg::Vec3Array();
    verts->push_back(node.p + (r * osg::Vec3f(-bh, -bh, 0)));
    verts->push_back(node.p + (r * osg::Vec3f( bh, -bh, 0)));
    verts->push_back(node.p + (r * osg::Vec3f( bh,  bh, 0)));
    verts->push_back(node.p + (r * osg::Vec3f(-bh,  bh, 0)));
    geom->setVertexArray(verts);

    const osg::Vec4 color_data[] = { {.4, .4, .4, 1} };
    auto colors = new osg::Vec4Array(osg::Array::BIND_OVERALL, 1, color_data);
    geom->setColorArray(colors);

    const osg::Vec2 uv_data[] = { {0,0}, {1,0}, {1,1}, {0,1} };
    auto uvs = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX, 4, uv_data);
    geom->setTexCoordArray(0, uvs);

    const GLushort index_data[] = { 0, 1, 2, 0, 2, 3 };
    geom->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, 6, index_data));

    return geom;
}

osg::Node* tessellate(const graph_t& graph, std::function<osg::Vec3(const osg::Vec3d&)> transform)
{
    auto group = new osg::Group();

    for (auto& edge : graph.edges)
    {
        group->addChild(tessellate_edge(edge, transform));
        //group->addChild(tessellate_edge_center_line(edge, transform));
    }

#if 0
    for (auto& node : graph.nodes)
    {
        if (node.intersection.type != node.intersection.NONE)
        {
            group->addChild(tessellate_intersection(node));
        }
    }
#endif

    return group;
}

osg::Node* tessellate_surface(
    graph_t& graph,
    std::function<osg::Vec3(const osg::Vec3d&)> transform)
{
    // sort disconnected subgraphs and process them separately
    std::map<int, std::list<const edge_t*>> edges_by_subgraph;
    for (auto& edge : graph.edges)
    {
        edges_by_subgraph[edge.node1.subgraph_id].push_back(&edge);
    }

    auto geom = new osg::Geometry();
    const osg::Vec4 color_data[] = { { 1, 1, 1, 1 } };
    geom->setColorArray(new osg::Vec4Array(osg::Array::BIND_OVERALL, 1, color_data));

    auto verts = new osg::Vec3Array();
    verts->reserve(graph.nodes.size() * 2);
    geom->setVertexArray(verts);

    //auto normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    //normals->reserve(verts->size());

    for (auto& iter : edges_by_subgraph)
    {
        unsigned subgraph_offset = verts->size();

        // MAYBE layer, we simply have separate GRAPHS for each KIND.
        // (road, bikepath, walkingpath) I like that better.
        // they never interact?

        // Since roads have different widths, we have to inflate each one
        // separately and then union the result.
        Clipper2Lib::PathsD all_paths;

        for (auto& edge : iter.second)
        {
            Clipper2Lib::PathsD paths;
            Clipper2Lib::PathD path;
            path.push_back(Clipper2Lib::PointD(edge->node1.p.x(), edge->node1.p.y()));
            path.push_back(Clipper2Lib::PointD(edge->node2.p.x(), edge->node2.p.y()));
            paths.push_back(path);

            auto solution = Clipper2Lib::InflatePaths(
                paths,
                edge->width,
                Clipper2Lib::JoinType::Round,
                Clipper2Lib::EndType::Round);

            Clipper2Lib::RamerDouglasPeucker(solution, EPSILON);

            for (auto& p : solution)
                all_paths.push_back(p);
        }

        // now union them
        auto solution = Clipper2Lib::Union(
            all_paths,
            Clipper2Lib::FillRule::NonZero);

        // copy the solution into a place where we can tessellate it
        std::vector<std::vector<osg::Vec3d>> polygon;
        for (auto& path : solution)
        {
            std::vector<osg::Vec3d> ring;
            for (auto& point : path)
            {
                ring.push_back(osg::Vec3d(point.x, point.y, 0.0));
                verts->push_back(transform(osg::Vec3(point.x, point.y, 0.0)));
            }
            polygon.push_back(ring);
        }

        // tessellate it
        auto indices = mapbox::earcut<uint16_t>(polygon);

        if (ROAD_THICKNESS > 0.0f)
        {
            // extrude it
            // copy surface verts down by the thickness
            unsigned polygon_size = verts->size() - subgraph_offset;
            for (unsigned i = 0; i < polygon_size; ++i)
            {
                verts->push_back((*verts)[subgraph_offset + i]);
                verts->back().z() -= ROAD_THICKNESS;
            }

            // tessellate the sides
            unsigned ring_offset = 0;
            for (auto& ring : polygon)
            {
                for (unsigned k = 0; k < ring.size() - 1; ++k)
                {
                    indices.push_back(ring_offset + k);
                    indices.push_back(ring_offset + k + polygon_size);
                    indices.push_back(ring_offset + k + 1);

                    indices.push_back(ring_offset + k + 1);
                    indices.push_back(ring_offset + k + polygon_size);
                    indices.push_back(ring_offset + k + polygon_size + 1);
                }
                ring_offset += ring.size();
            }
        }

        // offset the indices for this subgraph
        for (auto& i : indices)
            i += subgraph_offset;

        // add the prim.
        geom->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, indices.size(), indices.data()));
    }

    osgUtil::SmoothingVisitor smoothie;
    geom->accept(smoothie);

    // apply the surface texture
    // TODO: different may have different surface textures .. probably will need to
    // assign a texture index in the vertex attribute.
    if (graph.edges.begin()->props)
    {
        auto& props = graph.edges.begin()->props;
        auto uvs = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX);
        uvs->reserve(verts->size());
        for (unsigned i = 0; i < verts->size(); ++i)
        {
            float s = (*verts)[i].y() / props->surface.width;
            float t = (*verts)[i].x() / props->surface.length;
            uvs->push_back(osg::Vec2f(s, t));
        }
        geom->setTexCoordArray(0, uvs);
        geom->getOrCreateStateSet()->setTextureAttribute(0, props->surface.texture);
    }

    auto ss = geom->getOrCreateStateSet();
    ss->setAttributeAndModes(new osg::PolygonOffset(-1, -1));

    return geom;
}

osg::Node* debug_surface_outline(graph_t& graph, float z = 0)
{
    Clipper2Lib::PathsD all_paths;

    for (auto& edge : graph.edges)
    {
        Clipper2Lib::PathsD paths;
        Clipper2Lib::PathD path;
        path.push_back(Clipper2Lib::PointD(edge.node1.p.x(), edge.node1.p.y()));
        path.push_back(Clipper2Lib::PointD(edge.node2.p.x(), edge.node2.p.y()));
        paths.push_back(path);

        auto solution = Clipper2Lib::InflatePaths(
            paths,
            edge.width,
            Clipper2Lib::JoinType::Round,
            Clipper2Lib::EndType::Round);

        Clipper2Lib::RamerDouglasPeucker(solution, EPSILON);

        for (auto& p : solution)
            all_paths.push_back(p);
    }

    // now union them
    auto solution = Clipper2Lib::Union(
        all_paths,
        Clipper2Lib::FillRule::NonZero);

    auto group = new osg::Group();

    for (auto& path : solution)
    {
        auto geom = new LineDrawable(GL_LINE_LOOP);
        geom->setColor(Color::Cyan);
        geom->setLineWidth(3);
        geom->setLineSmooth(true);

        for (auto& point : path)
        {
            geom->pushVertex(osg::Vec3(point.x, point.y, z));
        }
        geom->finish();

        group->addChild(geom);
    }

    return group;
}


osg::Drawable* debug_edges(const graph_t& g, float z = 0)
{
    auto line = new LineDrawable(GL_LINES);
    line->setColor(Color::Magenta);
    line->setLineWidth(2.0f);
    line->setLineSmooth(true);
    for (auto& edge : g.edges)
    {
        line->pushVertex(osg::Vec3(edge.node1.p.x(), edge.node1.p.y(), z));
        line->pushVertex(osg::Vec3(edge.node2.p.x(), edge.node2.p.y(), z));
    }
    line->finish();
    return line;
}

osg::Drawable* debug_nodes(const graph_t& g, float z = 0)
{
    auto points = new PointDrawable();
    points->setColor(Color::Yellow);
    points->setPointSize(13.0f);
    points->setPointSmooth(true);
    const Color colors[] = { Color::Red, Color::Red, Color::Magenta, Color::Yellow };
    for (auto& node : g.nodes)
    {
        points->pushVertex(osg::Vec3(node.p.x(), node.p.y(), z));
        points->setColor(points->size() - 1, colors[std::min(node.edges.size(), (size_t)3)]);
    }
    points->finish();
    return points;
}

osg::Node* debug_edge_outlines(const graph_t& g, float z = 0)
{
    auto group = new osg::Group();
    for (auto& edge : g.edges)
    {
        auto backoff = new LineDrawable(GL_LINE_LOOP);
        backoff->setColor(Color::Green);
        backoff->setLineWidth(2.0f);
        backoff->setLineSmooth(true);
        backoff->pushVertex(osg::Vec3(edge.node1_left.x(), edge.node1_left.y(), z));
        backoff->pushVertex(osg::Vec3(edge.node1_right.x(), edge.node1_right.y(), z));
        backoff->pushVertex(osg::Vec3(edge.node2_right.x(), edge.node2_right.y(), z));
        backoff->pushVertex(osg::Vec3(edge.node2_left.x(), edge.node2_left.y(), z));
        backoff->finish();
        group->addChild(backoff);

        if (edge.props->crossing.texture)
        {
            if (edge.node1.intersection.crossing_length > 0.0f)
            {
                auto crossing = new LineDrawable(GL_LINE_LOOP);
                crossing->setColor(Color::Lime);
                crossing->setLineWidth(3.0f);
                crossing->setLineSmooth(true);
                crossing->pushVertex(osg::Vec3(edge.node1_left.x(), edge.node1_left.y(), z + 0.1));
                crossing->pushVertex(osg::Vec3(edge.node1_right.x(), edge.node1_right.y(), z + 0.1));
                crossing->pushVertex(osg::Vec3(edge.node1_cr_right.x(), edge.node1_cr_right.y(), z + 0.1));
                crossing->pushVertex(osg::Vec3(edge.node1_cr_left.x(), edge.node1_cr_left.y(), z + 0.1));
                crossing->finish();
                group->addChild(crossing);
            }

            if (edge.node2.intersection.crossing_length > 0.0f)
            {
                auto crossing = new LineDrawable(GL_LINE_LOOP);
                crossing->setColor(Color::Lime);
                crossing->setLineWidth(3.0f);
                crossing->setLineSmooth(true);
                crossing->pushVertex(osg::Vec3(edge.node2_left.x(), edge.node2_left.y(), z + 0.1));
                crossing->pushVertex(osg::Vec3(edge.node2_right.x(), edge.node2_right.y(), z + 0.1));
                crossing->pushVertex(osg::Vec3(edge.node2_cr_right.x(), edge.node2_cr_right.y(), z + 0.1));
                crossing->pushVertex(osg::Vec3(edge.node2_cr_left.x(), edge.node2_cr_left.y(), z + 0.1));
                crossing->finish();
                group->addChild(crossing);
            }
        }
    }
    return group;
}

graph_t make_osm_graph(Art& art)
{
    graph_t graph;

    auto profile = Profile::create(Profile::SPHERICAL_MERCATOR);

    osg::ref_ptr<XYZFeatureSource> fs = new XYZFeatureSource();
    fs->options().profile() = ProfileOptions("spherical-mercator");
    fs->setURL("http://readymap.org/readymap/mbtiles/daylight-v1.2/{z}/{x}/{-y}.pbf");
    fs->setMinLevel(14);
    fs->setMaxLevel(14);
    fs->setFormat("pbf");
    fs->setFIDAttribute("@id");

    auto status = fs->open();
    if (status.isError())
    {
        std::cout << status.message() << std::endl;
        exit(0);

    }

    //GeoPoint p(SpatialReference::get("wgs84"), 7.311, 46.210, 0); // sion
    GeoPoint p(SpatialReference::get("wgs84"), 2.333, 48.86667, 0); // paris
    //GeoPoint p(SpatialReference::get("wgs84"), -71.09624, 42.35053, 0); // bu
    //GeoPoint p(SpatialReference::get("wgs84"), -71.0584421, 42.3623348); // boston
    //GeoPoint p(SpatialReference::get("wgs84"), 139.7742968, 35.6993466); // Tokyo
    p.transformInPlace(fs->getFeatureProfile()->getSRS());
    TileKey key = profile->createTileKey(p, 14);

    osg::Vec3d origin = key.getExtent().getCentroid().vec3d();

    auto cursor = fs->createFeatureCursor(key);

    while (cursor->hasMore())
    {
        auto feature = cursor->nextFeature();
        auto geom = feature->getGeometry();
        if (!geom)
            continue;

        // for now, skip features that are underground
        if (feature->getInt("layer") < 0)
            continue;

        auto highway = feature->getString("highway");

        if (feature && (
            highway == "trunk" ||
            highway == "motorway" ||
            highway == "primary" ||
            highway == "secondary" ||
            highway == "tertiary" ||
            highway == "unclassified" ||
            highway == "residential"))
        {
            auto width = feature->getDouble("width", art.road_basic.default_width);
            auto lanes = feature->getInt("lanes", art.road_basic.lanes);

            ConstGeometryIterator iter(geom);
            while (iter.hasMore())
            {
                auto part = iter.next();
                if (part)
                {
                    for (int i = 0; i < part->size() - 1; ++i)
                    {
                        auto p1 = (*part)[i] - origin;
                        auto p2 = (*part)[i + 1] - origin;

                        // skip segments that don't meet the minimum length requirement
                        while ((p1 - p2).length() <= EPSILON && i < part->size() - 1)
                        {
                            ++i;
                            p2 = (*part)[i + 1] - origin;
                        }
                        if (i < part->size() - 1)
                        {
                            graph.add_edge(p1.x(), p1.y(), p2.x(), p2.y(), width, &art.road_basic);
                        }
                    }
                }
            }
        }

        else if (highway == "crossing")
        {
            if (feature->getString("crossing") != "unmarked")
            {
                if (geom->size() > 0)
                {
                    auto pos = (*geom)[0] - origin;
                    auto node = graph.add_node(pos.x(), pos.y(), &art.road_basic);
                    node->has_crossing = true;
                }
            }
        }
    }

    // OSM data should be properly noded. Any intersecting edges
    // are likely layered data (e.g. bridges, overpasses, tunnels).
    //graph.split_intersecting_edges();

    return graph;
}

graph_t make_graph(Art& art)
{
    graph_t g;
    g.add_edge(-100,    0,   0,    0, 0, &art.road_basic);
    g.add_edge(   0,    0, 120,    0, 0, &art.road_basic);
    g.add_edge(   0, -100,   0,    0, 0, &art.road_basic);
    g.add_edge(-100, -120,   0, -100, 0, &art.road_basic);
    g.add_edge(   0, -100, 100,  -80, 0, &art.road_basic);
    g.add_edge(100,   -80, 100,   50, 0, &art.road_basic);

    g.add_edge(0, 0, 0, 100, 0, &art.road_one_lane);

    g.add_edge(-100, 0, 0, -100, 0, &art.road_one_lane);
    g.add_edge(-100, -120, -120, -100, 0, &art.road_basic);
    g.add_edge(-120, -50, -120, -100, 0, &art.road_basic);

    g.split_intersecting_edges();

    return g;
}




class RoadsLayer : public TiledModelLayer
{
public:
    class Options : public TiledModelLayer::Options {
        META_LayerOptions(osgEarth, Options, TiledModelLayer::Options);
        OE_OPTION_LAYER(FeatureSource, featureSource);
        OE_OPTION_VECTOR(ConfigOptions, filters);
        OE_OPTION_LAYER(StyleSheet, styleSheet);
        void fromConfig(const Config& c);
        Config getConfig() const override;
    };
public:
    META_Layer(osgEarth, RoadsLayer, Options, TiledModelLayer, roads);
    
    Status openImplementation() override;
    Status closeImplementation() override;
    void addedToMap(const Map* map) override;
    void removedFromMap(const Map* map) override;
    void init() override;

    osg::ref_ptr<osg::Node> createTileImplementation(const TileKey& key, ProgressCallback* p) const override;
    const Profile* getProfile() const override;

private:
    FeatureFilterChain _filterChain;
    osg::ref_ptr<Session> _session;
    Art _art;
};

void
RoadsLayer::Options::fromConfig(const Config& conf)
{
    featureSource().get(conf, "features");
    styleSheet().get(conf, "styles");
    for(auto& i : conf.child("filters").children())
        filters().push_back(ConfigOptions(i));
}

Config
RoadsLayer::Options::getConfig() const
{
    Config conf = super::Options::getConfig();
    featureSource().set(conf, "features");
    styleSheet().set(conf, "styles");
    conf.set_with_function("filters", [this](Config& c) {
        for (auto& filter : filters())
            c.add(filter.getConfig());
    });

    return conf;
}

void
RoadsLayer::init()
{
    super::init();
    setMinLevel(14);
    options().nvgl() = GLUtils::useNVGL();
    getNode()->getOrCreateStateSet()->setMode(GL_BLEND, 1);
}

Status
RoadsLayer::openImplementation()
{
    auto status = super::openImplementation();
    if (status.isError())
        return status;

    auto fs_status = options().featureSource().open(getReadOptions());
    if (fs_status.isError())
        return fs_status;

    auto ss_status = options().styleSheet().open(getReadOptions());
    // stylesheet is optional

    return Status::NoError;
}

Status
RoadsLayer::closeImplementation()
{
    return super::closeImplementation();
}

void
RoadsLayer::addedToMap(const Map* map)
{
    options().featureSource().addedToMap(map);
    
    auto fs = options().featureSource().getLayer();

    OE_SOFT_ASSERT_AND_RETURN(fs, void());
    OE_SOFT_ASSERT_AND_RETURN(fs->isOpen(), void());

    auto featureProfile = fs->getFeatureProfile();
    OE_SOFT_ASSERT_AND_RETURN(featureProfile, void());

    _filterChain = FeatureFilterChain::create(options().filters(), getReadOptions());
    _session = new Session(map);

    super::addedToMap(map);
}

void
RoadsLayer::removedFromMap(const Map* map)
{
    VisibleLayer::removedFromMap(map);
    options().featureSource().removedFromMap(map);
}

const Profile*
RoadsLayer::getProfile() const
{
    return Profile::create(Profile::GLOBAL_GEODETIC);

    // take local refs to isolate this method from the member objects
    osg::ref_ptr<FeatureSource> featureSource(options().featureSource().getLayer());
    if (featureSource.valid()) {
        auto featureProfile = featureSource->getFeatureProfile();
        if (featureProfile)
            return featureProfile->getTilingProfile();
    }
    return nullptr;
}

osg::ref_ptr<osg::Node> 
RoadsLayer::createTileImplementation(const TileKey& key, ProgressCallback* progress) const
{
    // take local refs to isolate this method from the member objects
    auto featureSource(options().featureSource().getLayer());

    OE_SOFT_ASSERT_AND_RETURN(featureSource, nullptr);
    OE_SOFT_ASSERT_AND_RETURN(featureSource->getStatus().isOK(), nullptr);

    auto featureProfile = featureSource->getFeatureProfile();
    OE_SOFT_ASSERT_AND_RETURN(featureProfile, nullptr);

    auto featureSRS = featureProfile->getSRS();
    OE_SOFT_ASSERT_AND_RETURN(featureSRS, nullptr);

    // Fetch the set of features to render
    FilterContext context(_session.get());
    context.extent() = key.getExtent().transform(featureSRS);

    auto cursor = featureSource->createFeatureCursor(key, _filterChain, &context, progress);
    if (!cursor.valid())
        return nullptr;

    FeatureList features;
    if (cursor->fill(features) == 0)
        return nullptr;

    CropFilter crop(CropFilter::METHOD_CROPPING);
    context.extent() = key.getExtent().transform(featureSRS);
    context = crop.push(features, context);

    osg::Group* root = new osg::Group();

    graph_t graph;
    osg::Vec3d origin(0, 0, 0);

    auto centroid = key.getExtent().getCentroid();
    auto local_srs = key.getExtent().getSRS()->createTangentPlaneSRS(centroid.vec3d());

    for (auto& feature : features)
    {
        if (!feature.valid())
            continue;

        auto geom = feature->getGeometry();
        if (!geom)
            continue;

        // for now, skip features that are underground
        if (feature->getInt("layer") < 0)
            continue;
        
        feature->transform(local_srs);

        auto highway = feature->getString("highway");
        auto width = feature->getDouble("width", 0.0);
        auto lanes = feature->getInt("lanes", 0);

        if (highway == "crossing")
        {
            if (feature->getString("crossing") != "unmarked")
            {
                if (geom->size() > 0)
                {
                    auto pos = (*geom)[0] - origin;
                    auto node = graph.add_node(pos.x(), pos.y(), &_art.road_basic);
                    node->has_crossing = true;
                }
            }
        }
        else
        {
            ConstGeometryIterator iter(geom);
            while (iter.hasMore())
            {
                auto part = iter.next();
                if (part)
                {
                    for (int i = 0; i < part->size() - 1; ++i)
                    {
                        auto p1 = (*part)[i] - origin;
                        auto p2 = (*part)[i + 1] - origin;

                        // skip segments that don't meet the minimum length requirement
                        while ((p1 - p2).length() <= EPSILON && i < part->size() - 2)
                        {
                            ++i;
                            p2 = (*part)[i + 1] - origin;
                        }
                        if (i < part->size() - 1)
                        {
                            graph.add_edge(p1.x(), p1.y(), p2.x(), p2.y(), width, &_art.road_basic);
                        }
                    }
                }
            }
        }
    }

    if (graph.edges.empty())
    {
        return {};
    }

    // OSM data should be properly noded. Any intersecting edges
    // are likely layered data (e.g. bridges, overpasses, tunnels).
    //graph.split_intersecting_edges();

    compile(graph);

    ElevationPool::WorkingSet ws;
    auto pool = _session->getMap()->getElevationPool();
    GeoPoint temp;
    auto xform = [local_srs, pool, &ws, &temp](const osg::Vec3d& p)
        {
            temp.set(local_srs, p, ALTMODE_ABSOLUTE);
            auto z = pool->getSample(temp, {}, &ws, nullptr);
            return osg::Vec3d(p.x(), p.y(), z.elevation().as(Units::METERS));
        };


    //root->addChild(tessellate(graph, xform));
    root->addChild(tessellate_markings(graph, xform));
    //root->addChild(debug_surface_outline(graph, xform));
    //root->addChild(tessellate_surface(graph, xform));

    auto sheet = options().styleSheet().getLayer();
    auto style = sheet ? sheet->getDefaultStyle() : nullptr;
    auto render = style ? style->get<RenderSymbol>() : nullptr;
    if (render) render->applyTo(root);

    osgUtil::Optimizer::MergeGeometryVisitor mgv;
    mgv.setTargetMaximumNumberOfVertices(~0);
    root->accept(mgv);

    GeometryValidator gv;
    root->accept(gv);

    auto mt = new osg::MatrixTransform();
    osg::Matrixd local2world;
    centroid.createLocalToWorld(local2world);
    mt->setMatrix(local2world);
    mt->addChild(root);

    return mt;
}

REGISTER_OSGEARTH_LAYER(roads, RoadsLayer);




#if 1
class RoadsImageLayer : public ImageLayer
{
public:
    class Options : public ImageLayer::Options {
    public:
        META_LayerOptions(osgEarth, Options, ImageLayer::Options);
        OE_OPTION_LAYER(FeatureSource, featureSource);
        OE_OPTION_VECTOR(ConfigOptions, filters);
        virtual Config getConfig() const;
    private:
        void fromConfig(const Config& conf);
    };

public:
    META_Layer(osgEarth, RoadsImageLayer, Options, ImageLayer, roadsimage);

    void setFeatureSource(FeatureSource* layer);
    FeatureSource* getFeatureSource() const;

    Status openImplementation() override;
    Status closeImplementation() override;
    void addedToMap(const Map*) override;
    void removedFromMap(const Map*) override;
    GeoImage createImageImplementation(const TileKey&, ProgressCallback*) const override;
    osg::Node* getNode() const override;

protected:
    void init() override;

private:
    osg::ref_ptr<TileRasterizer> _rasterizer;
    osg::ref_ptr<Session> _session;
    FeatureFilterChain _filterChain;
    Art _art;

    void getFeatures(
        FeatureSource* fs,
        const TileKey& key,
        FeatureList& output,
        ProgressCallback* progress) const;
};

REGISTER_OSGEARTH_LAYER(roadsimage, RoadsImageLayer);

Config
RoadsImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    featureSource().set(conf, "features");
    return conf;
}

void
RoadsImageLayer::Options::fromConfig(const Config& conf)
{
    featureSource().get(conf, "features");

    const Config& filtersConf = conf.child("filters");
    for (ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back(ConfigOptions(*i));
}

void
RoadsImageLayer::init()
{
    ImageLayer::init();

    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
}

void
RoadsImageLayer::setFeatureSource(FeatureSource* value)
{
    if (getFeatureSource() != value)
    {
        options().featureSource().setLayer(value);
        if (value && value->getStatus().isError())
        {
            setStatus(value->getStatus());
        }
    }
}

FeatureSource*
RoadsImageLayer::getFeatureSource() const
{
    return options().featureSource().getLayer();
}

osg::Node*
RoadsImageLayer::getNode() const
{
    return _rasterizer.get();
}

Status
RoadsImageLayer::openImplementation()
{
    auto status = ImageLayer::openImplementation();
    if (status.isError())
        return status;

    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    _filterChain = FeatureFilterChain::create(options().filters(), getReadOptions());

    if (!_rasterizer.valid())
    {
        _rasterizer = new TileRasterizer(getTileSize(), getTileSize());
    }

    return Status::NoError;
}

Status
RoadsImageLayer::closeImplementation()
{
    _rasterizer = nullptr;
    return ImageLayer::closeImplementation();
}

void
RoadsImageLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);
    _session = new Session(map, nullptr, nullptr, getReadOptions());
    _session->setResourceCache(new ResourceCache());
    options().featureSource().addedToMap(map);
}

void
RoadsImageLayer::removedFromMap(const Map* map)
{
    ImageLayer::removedFromMap(map);
    options().featureSource().removedFromMap(map);
    _session = nullptr;
}

GeoImage
RoadsImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    //if (key.getLOD() % 2 == 1)
    //    return GeoImage::INVALID;

    // take local refs to isolate this method from the member objects
    osg::ref_ptr<FeatureSource> featureSource(getFeatureSource());
    osg::ref_ptr<TileRasterizer> rasterizer(_rasterizer);
    osg::ref_ptr<Session> session(_session);

    if (!featureSource.valid())
    {
        setStatus(Status::ServiceUnavailable, "No feature source");
        return GeoImage::INVALID;
    }

    if (featureSource->getStatus().isError())
    {
        setStatus(featureSource->getStatus());
        return GeoImage::INVALID;
    }

    osg::ref_ptr<const FeatureProfile> featureProfile = featureSource->getFeatureProfile();
    if (!featureProfile.valid())
    {
        setStatus(Status::ConfigurationError, "Feature profile is missing");
        return GeoImage::INVALID;
    }

    if (!rasterizer.valid() || !session.valid())
    {
        return GeoImage::INVALID;
    }

    const SpatialReference* featureSRS = featureProfile->getSRS();
    if (!featureSRS)
    {
        setStatus(Status(Status::ConfigurationError, "Feature profile has no SRS"));
        return GeoImage::INVALID;
    }

    // Fetch the set of features to render
    FeatureList features;
    getFeatures(featureSource.get(), key, features, progress);

    graph_t graph;

    GeoExtent outputExtent = key.getExtent();
    auto keysrs = outputExtent.getSRS();
    osg::Vec3d pos(key.getExtent().west(), key.getExtent().south(), 0);
    osg::ref_ptr<const SpatialReference> outputSRS = keysrs->createTangentPlaneSRS(pos);
    outputExtent = outputExtent.transform(outputSRS);

    osg::Vec3d origin(0, 0, 0); // outputExtent.west(), outputExtent.south(), 0);

    for(auto& feature : features)
    {
        auto geom = feature->getGeometry();
        if (!geom)
            continue;

        // for now, skip features that are underground
        if (feature->getInt("layer") < 0)
            continue;

        auto highway = feature->getString("highway");

        if (feature /*&& (
            highway == "trunk" ||
            highway == "motorway" ||
            highway == "primary" ||
            highway == "secondary" ||
            highway == "tertiary" ||
            highway == "unclassified" ||
            highway == "residential")) */ )
        {
            feature->transform(outputSRS.get());

            auto width = feature->getDouble("width", 0.0);
            auto lanes = feature->getInt("lanes", 0);

            ConstGeometryIterator iter(geom);
            while (iter.hasMore())
            {
                auto part = iter.next();
                if (part)
                {
                    for (int i = 0; i < part->size() - 1; ++i)
                    {
                        auto p1 = (*part)[i] - origin;
                        auto p2 = (*part)[i + 1] - origin;

                        // skip segments that don't meet the minimum length requirement
                        while ((p1 - p2).length() <= EPSILON && i < part->size() - 2)
                        {
                            ++i;
                            p2 = (*part)[i + 1] - origin;
                        }
                        if (i < part->size() - 1)
                        {
                            graph.add_edge(p1.x(), p1.y(), p2.x(), p2.y(), width, &_art.road_basic);
                        }
                    }
                }
            }
        }

        else if (highway == "crossing")
        {
            if (feature->getString("crossing") != "unmarked")
            {
                if (geom->size() > 0)
                {
                    auto pos = (*geom)[0] - origin;
                    auto node = graph.add_node(pos.x(), pos.y(), &_art.road_basic);
                    node->has_crossing = true;
                }
            }
        }
    }

    if (graph.edges.empty())
    {
        return GeoImage::INVALID;
    }

    // OSM data should be properly noded. Any intersecting edges
    // are likely layered data (e.g. bridges, overpasses, tunnels).
    //graph.split_intersecting_edges();

    //return graph;

    compile(graph);

    osg::ref_ptr<osg::Group> root = new osg::Group();
    auto xform = [](const osg::Vec3d& p) { return p; };
    //root->addChild(tessellate_surface(graph, xform));
    root->addChild(tessellate_markings(graph, xform));
    //root->addChild(debug_surface_outline(graph, 0.1));

    //ShaderGenerator gen;
    //root->accept(gen);

    auto result = rasterizer->render(root.get(), outputExtent);

    osg::ref_ptr<ProgressCallback> local_progress = new ProgressCallback(
        progress,
        [&]() { return !isOpen(); }
    );

    osg::ref_ptr<osg::Image> image = result.join(local_progress.get());

    if (image.valid())
    {
        osgDB::writeImageFile(*image, "out/" + std::to_string(key.hash()) + ".png");

        if (!ImageUtils::isEmptyImage(image.get()))
        {
            return GeoImage(image.get(), key.getExtent());
        }
    }

    return GeoImage::INVALID;


    //root->addChild(tessellate_markings(graph, 0.05));
    //root->addChild(debug_edges(graph, 0.5));
    //root->addChild(debug_nodes(graph, 0.5));
    //root->addChild(debug_edge_outlines(graph, 0.1));
    //root->addChild(debug_surface_outline(graph, 0.1));
}

void
RoadsImageLayer::getFeatures(
    FeatureSource* fs,
    const TileKey& key,
    FeatureList& output,
    ProgressCallback* progress) const
{
    OE_SOFT_ASSERT_AND_RETURN(fs != nullptr, void());

    Distance buffer(0.0, Units::METERS); // TODO

    // Collect all the features, using a small LRU cache and a
    // Gate to optimize fetching and sharing with other threads
    auto cursor = fs->createFeatureCursor(key, _filterChain, nullptr, progress);
    if (cursor.valid())
        cursor->fill(output);
}
#endif

#define DEFAULT_WIDTH 5.0

class RoadSurfaceOutlineFilterSimple : public FeatureFilter
{
public:
    FilterContext push(FeatureList& input, FilterContext& context) override
    {
        if (input.empty())
            return context;

        // Notes:
        // You will notice that this code separately inflates each line string.
        // This sounds wierd but it is an order of magnitude faster than inflating
        // the entire collection of linestrings at once.

        FeatureList output;

        Clipper2Lib::PathsD solution;
        Clipper2Lib::PathsD temp = { Clipper2Lib::PathD(2) };

        for (auto& feature : input)
        {
            if (feature->getGeometry()->getComponentType() == Geometry::TYPE_POLYGON)
            {
                output.emplace_back(feature.get());
            }
            else
            {
                auto width = feature->getDouble("width", DEFAULT_WIDTH);

                ConstGeometryIterator iter(feature->getGeometry());
                while (iter.hasMore())
                {
                    auto* part = iter.next();
                    if (part->isLinear())
                    {
                        temp[0].clear();
                        temp[0].reserve(part->size());
                        for (auto& p : *part)
                        {
                            temp[0].emplace_back(p.x(), p.y());
                        }

                        // If we are using this for SDF, we need the inflation and the SDF-ing to
                        // total the road width.
                        const double sdf_max_distance = 2.0;
                        auto inflate_solution = Clipper2Lib::InflatePaths(
                            temp,
                            0.5,
                            //(0.5 * width) - sdf_max_distance,
                            Clipper2Lib::JoinType::Round,
                            Clipper2Lib::EndType::Round);

                        for (auto& s : inflate_solution)
                        {
                            solution.emplace_back(std::move(s));
                        }
                    }
                }
            }
        }

        // do we need this? nah
        //Clipper2Lib::RamerDouglasPeucker(solution, EPSILON);

        // now union all the inflated linestrings:
        solution = Clipper2Lib::Union(solution, Clipper2Lib::FillRule::NonZero);

        // Annoyingly, there is no guarantee as to the order in which
        // clipper will output polygons and holes, so we need to sort each
        // hole into the corresponding polygon.
        std::vector<osg::ref_ptr<osgEarth::Polygon>> polygons;
        std::vector<Clipper2Lib::PathD*> polygon_paths;

        // find the outer ring of each polygon:
        for (auto& path : solution)
        {
            if (Clipper2Lib::IsPositive(path))
            {
                polygon_paths.push_back(&path);
                auto poly = new osgEarth::Polygon();
                polygons.push_back(poly);
                poly->reserve(path.size());
                for (auto& point : path)
                {
                    poly->push_back(point.x, point.y);
                }
            }
        }

        // find the holes and sort them into the correct polygons
        // by testing the first point
        for (auto& path : solution)
        {
            if (!Clipper2Lib::IsPositive(path) && path.size() > 0) // holes
            {
                auto& p = path.front();
                for (int i = 0; i < polygon_paths.size(); ++i)
                {
                    auto& polygon_path = *polygon_paths[i];
                    if (Clipper2Lib::PointInPolygon(p, polygon_path) == Clipper2Lib::PointInPolygonResult::IsInside)
                    {
                        auto ring = new osgEarth::Ring();
                        polygons[i]->getHoles().push_back(ring);
                        ring->reserve(path.size());
                        for (auto& point : path)
                        {
                            ring->push_back(point.x, point.y);
                        }
                        break;
                    }
                }
            }
        }

        // Combine all the polygons into a single geometry.
        osg::ref_ptr<osgEarth::Geometry> geom;
        if (polygons.size() == 1)
        {
            geom = polygons[0];
        }
        else
        {
            auto multi = new osgEarth::MultiGeometry();
            multi->getComponents().reserve(polygons.size());
            for (auto& polygon : polygons)
            {
                multi->add(polygon.get());
            }
            geom = multi;
        }

        output.emplace_back(new Feature(geom.get(), input.front()->getSRS()));
        input.swap(output);

        return context;
    }
};

//#define INCLUDE_CENTERLINE
//#include INCLUDE_CROSSINGS
//#define CLAMP_ALL_NODES

// inputs the line features, outputs buffered, unioned polygons for the road surfaces.
class RoadSurfaceOutlineFilter : public FeatureFilter
{
    Art _art;

public:
    FilterContext push(FeatureList& input, FilterContext& context) override
    {
        if (input.empty())
            return context;

        graph_t graph;

        FeatureList output;

        for (auto& feature : input)
        {
            auto geom = feature->getGeometry();
            if (!geom)
                continue;

            // for now, skip features that are underground
            if (feature->getInt("layer") < 0)
                continue;

            //auto highway = feature->getString("highway");

            if (feature)
            {
                auto width = feature->getDouble("width", 0.0);
                auto lanes = feature->getInt("lanes", 0);

                ConstGeometryIterator geom_iter(geom);
                while (geom_iter.hasMore())
                {
                    ConstSegmentIterator seg_iter(geom_iter.next());
                    while (seg_iter.hasMore())
                    {
                        auto& segment = seg_iter.next();
                        auto& p1 = segment.first;
                        auto& p2 = segment.second;
                        graph.add_edge(p1.x(), p1.y(), p2.x(), p2.y(), width, &_art.road_basic);
                    }
                }

#if 0
                ConstGeometryIterator iter(geom);
                while (iter.hasMore())
                {
                    auto part = iter.next();
                    if (part)
                    {
                        for (int i = 0; i < part->size() - 1; ++i)
                        {
                            auto p1 = (*part)[i];
                            auto p2 = (*part)[i + 1];

                            // skip segments that don't meet the minimum length requirement
                            //while ((p1 - p2).length() <= EPSILON && i < part->size() - 2)
                            //{
                            //    ++i;
                            //    p2 = (*part)[i + 1];
                            //}
                            //if (i < part->size() - 1)
                            {
                                graph.add_edge(p1.x(), p1.y(), p2.x(), p2.y(), width, &_art.road_basic);
                            }
                        }
                    }
                }
#endif

#ifdef INCLUDE_CENTERLINE
                // include the input feature as a centerline.
                // MAKE OPTIONAL
                output.push_back(feature);
#endif
            }

#ifdef INCLUDE_CROSSINGS
            else if (highway == "crossing")
            {
                if (feature->getString("crossing") != "unmarked")
                {
                    if (geom->size() > 0)
                    {
                        auto pos = (*geom)[0];
                        auto node = graph.add_node(pos.x(), pos.y(), &_art.road_basic);
                        node->has_crossing = true;
                    }
                }
            }
#endif
        }

        if (graph.edges.empty())
        {
            input.clear();
            return context;
        }

        // OSM data should be properly noded. Any intersecting edges
        // are likely layered data (e.g. bridges, overpasses, tunnels).
        //graph.split_intersecting_edges();

        //compile(graph);

#ifdef CLAMP_ALL_NODES
        // clamp all nodes.
        std::vector<osg::Vec3d> temp(1);
        for (auto& node : graph.nodes)
        {
            input.front()->getSRS()->transform2D(node.p.x(), node.p.y(), context.getSession()->getMapSRS(), temp.front().x(), temp.front().y());
            context.getSession()->getMap()->getElevationPool()->sampleMapCoords(temp.begin(), temp.end(), Distance(), nullptr, nullptr);
            const_cast<point_t*>(&node.p)->z() = temp.front().z();
        }
#endif

        Clipper2Lib::PathsD solution;

        for (auto& edge : graph.edges)
        {
            Clipper2Lib::PathsD paths;
            Clipper2Lib::PathD path;
            path.push_back(Clipper2Lib::PointD(edge.node1.p.x(), edge.node1.p.y()));
            path.push_back(Clipper2Lib::PointD(edge.node2.p.x(), edge.node2.p.y()));
            paths.push_back(path);

            auto inflate_solution = Clipper2Lib::InflatePaths(
                paths,
                0.5 * edge.width,
                Clipper2Lib::JoinType::Round,
                Clipper2Lib::EndType::Round); // , 2.0, 8);

            Clipper2Lib::RamerDouglasPeucker(inflate_solution, EPSILON);

            for (auto& p : inflate_solution)
                solution.push_back(p);
        }

#if 1
        // now union them
        solution = Clipper2Lib::Union(
            solution,
            Clipper2Lib::FillRule::NonZero);
#endif

#if 0 // testing
        auto srs = input.front()->getSRS();
        for (auto& path : solution)
        {
            auto line = new LineString();
            for (int i = 0; i < path.size() - 1; ++i)
            {
                line->push_back(path[i].x, path[i].y, 0);
            }
            output.push_back(new Feature(line, srs));
        }
#else

        // As it turns out, there is no guarantee as to the order in which
        // clipper will output polygons and holes, so we need to sort each
        // hole into the corresponding polygon.
        std::vector<osg::ref_ptr<osgEarth::Polygon>> polygons;
        std::vector<Clipper2Lib::PathD*> polygon_paths;

        // find the outer ring of each polygon:
        for (auto& path : solution)
        {
            if (Clipper2Lib::IsPositive(path))
            {
                polygon_paths.push_back(&path);
                auto poly = new osgEarth::Polygon();
                polygons.push_back(poly);
                for (auto& point : path)
                {
                    poly->push_back(point.x, point.y);
                }
                //poly->close();
            }
        }

        // find the holes and sort them into the correct polygons
        // by testing the first point
        for(auto& path : solution)
        {
            if (!Clipper2Lib::IsPositive(path) && path.size() > 0) // holes
            {
                auto& p = path.front();
                for(int i=0; i<polygon_paths.size(); ++i)
                {
                    auto& polygon_path = *polygon_paths[i];
                    if (Clipper2Lib::PointInPolygon(p, polygon_path) == Clipper2Lib::PointInPolygonResult::IsInside)
                    {
                        auto ring = new osgEarth::Ring();
                        polygons[i]->getHoles().push_back(ring);
                        for (auto& point : path)
                            ring->push_back(point.x, point.y);
                        //ring->close();
                        break;
                    }
                }
            }
        }

        osg::ref_ptr<osgEarth::Geometry> geom;
        if (polygons.size() == 1)
        {
            geom = polygons[0];
        }
        else
        {
            auto multi = new osgEarth::MultiGeometry();
            for (auto& polygon : polygons)
            {
                multi->add(polygon.get());
            }
            geom = multi;
        }

        // make sure everything is copacetic
        //geom->normalize();

#ifdef CLAMP_ALL_NODES
        // clamp new points
        edge_tree_t tree(graph);
        GeometryIterator gi(geom, true);
        while (gi.hasMore())
        {
            auto part = gi.next();
            for (auto& point : *part)
            {
                point_t closest = tree.closestPointOnEdgeTo(point.x(), point.y());
                if (closest.z() != NO_DATA_VALUE)
                    point.z() = closest.z();
            }
        }
#endif

        auto result = new Feature(geom.get(), input.front()->getSRS());

        output.push_back(result);
#endif
        input.swap(output);

        return context;
    }
};


class RoadSurfaceOutlineFilterPlugin : public FeatureFilterDriver
{
public:
    RoadSurfaceOutlineFilterPlugin() : FeatureFilterDriver()
    {
        this->supportsExtension("osgearth_featurefilter_clipper2", className());
    }

    const char* className() const
    {
        return "RoadSurfaceOutlineFilterPlugin";
    }

    ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if (!acceptsExtension(osgDB::getLowerCaseFileExtension(file_name)))
            return ReadResult::FILE_NOT_HANDLED;

        return new RoadSurfaceOutlineFilterSimple();
    }
};


REGISTER_OSGPLUGIN(osgearth_featurefilter_clipper2, RoadSurfaceOutlineFilterPlugin);



namespace
{
    template<class UNIT_TO_WORLD>
    void build_regular_gridded_mesh(weemesh::mesh_t& mesh, unsigned tileSize, UNIT_TO_WORLD&& unit_to_local)
    {
        mesh.set_boundary_marker(VERTEX_BOUNDARY);
        mesh.set_constraint_marker(VERTEX_CONSTRAINT);
        mesh.set_has_elevation_marker(VERTEX_HAS_ELEVATION);

        mesh.verts.reserve(tileSize * tileSize);
        mesh.triangles.reserve((tileSize - 1) * (tileSize - 1) * 2);

        for (unsigned row = 0; row < tileSize; ++row)
        {
            double ny = (double)row / (double)(tileSize - 1);
            for (unsigned col = 0; col < tileSize; ++col)
            {
                double nx = (double)col / (double)(tileSize - 1);
                osg::Vec3d unit(nx, ny, 0.0);
                osg::Vec3d model;
                osg::Vec3d local;

                local = unit_to_local(unit);

                int marker = VERTEX_VISIBLE;

                // mark the perimeter as a boundary (for skirt generation)
                if (row == 0 || row == tileSize - 1 || col == 0 || col == tileSize - 1)
                    marker |= VERTEX_BOUNDARY;

                int i = mesh.get_or_create_vertex(
                    weemesh::vert_t(local.x(), local.y(), local.z()),
                    marker);

                if (row > 0 && col > 0)
                {
                    mesh.add_triangle(i, i - 1, i - tileSize - 1);
                    mesh.add_triangle(i, i - tileSize - 1, i - tileSize);
                }
            }
        }
    }
}

#if 0
// inputs the polygons genreated by the RoadSurfaceOutlineFilter, and
// outputs a new multipolygon feature containing a triangle mesh.
class RoadSurfacePolygonsToTriangleMeshFeatures : public FeatureFilter
{
public:
    FilterContext push(FeatureList& input, FilterContext& context) override
    {
        if (input.empty()) return context;

        // calculate the bounds of the input.
        GeoExtent ex(input.front()->getSRS());
        for(auto& f : input)
            ex.expandToInclude(f->getExtent());

        // create a mesh based on the extent.
        weemesh::mesh_t mesh;
        build_regular_gridded_mesh(mesh, 129, [&ex](const osg::Vec3d& unit)
            {
                return osg::Vec3d(
                    ex.xMin() + unit.x() * ex.width(),
                    ex.yMin() + unit.y() * ex.height(),
                    unit.z());
            });

        // now we need to clip the mesh to the input polygons.
        int marker = VERTEX_VISIBLE | VERTEX_CONSTRAINT;
        for (auto& f : input)
        {
            GeometryIterator gi(f->getGeometry(), true);
            while (gi.hasMore())
            {
                auto geom = gi.next();
                ConstSegmentIterator si(geom);
                while (si.hasMore())
                {
                    auto& seg = si.next();
                    mesh.insert(weemesh::segment_t{
                        weemesh::vert_t{seg.first.x(), seg.first.y(), seg.first.z()},
                        weemesh::vert_t{seg.second.x(), seg.second.y(), seg.second.z()} },
                        marker);
                }
            }
        }

        // finally we need to remove all exterior triangles.
        bool remove_exterior = true;
        bool remove_interior = false;
        bool have_any_removal_requests = remove_interior || remove_exterior;

        if (have_any_removal_requests)
        {
            std::unordered_set<weemesh::triangle_t*> insiders;
            std::unordered_set<weemesh::triangle_t*> insiders_to_remove;
            std::unordered_set<weemesh::triangle_t*> outsiders_to_possibly_remove;
            weemesh::vert_t centroid;
            const double one_third = 1.0 / 3.0;
            std::vector<weemesh::triangle_t*> tris;

            if (remove_exterior || remove_interior)
            {
                for (auto& feature : input)
                {
                    // skip the polygon holes.
                    GeometryIterator geom_iter(feature->getGeometry(), false);
                    while (geom_iter.hasMore())
                    {
                        Geometry* part = geom_iter.next();

                        // Note: the part was already transformed in a previous step.

                        const auto& bb = part->getBounds();
                        if (part->isPolygon() && bb.intersects(ex.bounds()))
                        {
                            if (remove_exterior)
                            {
                                // expensive path, much check ALL triangles when removing exterior.
                                for (auto& tri_iter : mesh.triangles)
                                {
                                    weemesh::triangle_t* tri = &tri_iter.second;

                                    bool inside = part->contains2D(tri->centroid.x, tri->centroid.y);

                                    if (inside)
                                    {
                                        insiders.insert(tri);
                                        if (remove_interior)
                                        {
                                            insiders_to_remove.insert(tri);
                                        }
                                    }
                                    else if (remove_exterior)
                                    {
                                        outsiders_to_possibly_remove.insert(tri);
                                    }
                                }
                            }
                            else // removeInterior ONLY
                            {
                                // fast path when we are NOT removing exterior tris.
                                mesh.get_triangles(bb.xMin(), bb.yMin(), bb.xMax(), bb.yMax(), tris);

                                for (auto tri : tris)
                                {
                                    bool inside = part->contains2D(tri->centroid.x, tri->centroid.y);
                                    if (inside)
                                    {
                                        insiders_to_remove.insert(tri);
                                    }
                                }
                            }
                        }

                        //if (cancelable && cancelable->canceled())
                        //    return {};
                    }
                }
            }

            for (auto tri : insiders_to_remove)
            {
                mesh.remove_triangle(*tri);
            }

            for (auto tri : outsiders_to_possibly_remove)
            {
                if (insiders.count(tri) == 0)
                {
                    mesh.remove_triangle(*tri);
                }
            }
        }

        // TODO: this stores lots of unused verts and reserves space for degen indices.
        // need to "compress" the data while transfering it (or in the mesh_t object itself.)
        auto g = new TriMesh();
        g->reserve(mesh.verts.size());
        std::transform(mesh.verts.begin(), mesh.verts.end(), std::back_inserter(*g), [](const weemesh::vert_t& v)
            {
                return osg::Vec3d(v.x, v.y, v.z);
            });
        g->_indices.reserve(mesh.triangles.size() * 3);

        for(auto iter : mesh.triangles)
        {
            auto& tri = iter.second;
            if (!tri.is_2d_degenerate)
            {
                g->_indices.push_back(tri.i0);
                g->_indices.push_back(tri.i1);
                g->_indices.push_back(tri.i2);
            }
        }

        input.clear();
        input.emplace_back(new Feature(g, ex.getSRS()));

        return context;
    }
};

class RoadSurfacePolygonsToTriangleMeshFeaturesPlugin : public FeatureFilterDriver
{
public:
    RoadSurfacePolygonsToTriangleMeshFeaturesPlugin() : FeatureFilterDriver()
    {
        this->supportsExtension("osgearth_featurefilter_trimesh", className());
    }

    const char* className() const
    {
        return "RoadSurfacePolygonsToTriangleMeshFeaturesPlugin";
    }

    ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if (!acceptsExtension(osgDB::getLowerCaseFileExtension(file_name)))
            return ReadResult::FILE_NOT_HANDLED;

        return new RoadSurfacePolygonsToTriangleMeshFeatures();
    }
};

REGISTER_OSGPLUGIN(osgearth_featurefilter_trimesh, RoadSurfacePolygonsToTriangleMeshFeaturesPlugin);
#endif


struct App
{
    osg::Node* surface;
    osg::Node* markings;
    osg::Node* debug_nodes;
    osg::Node* debug_edges;
    osg::Node* debug_edge_outlines;
    osg::Node* debug_surface_outline;
};

struct MyGui : public GUI::BaseGUI
{
    App& app;
    MyGui(App& _app) : GUI::BaseGUI("Roads"), app(_app)
    {
        //nop
    }
    void draw(osg::RenderInfo& ri) override
    {
        ImGui::GetIO().FontGlobalScale = 2.0f;

        bool surface = (app.surface->getNodeMask() != 0);
        if (ImGui::Checkbox("Surface", &surface))
            app.surface->setNodeMask(~app.surface->getNodeMask());

        bool markings = (app.markings->getNodeMask() != 0);
        if (ImGui::Checkbox("Markings", &markings))
            app.markings->setNodeMask(~app.markings->getNodeMask());

        bool debug_nodes = (app.debug_nodes->getNodeMask() != 0);
        if (ImGui::Checkbox("Debug - nodes", &debug_nodes))
            app.debug_nodes->setNodeMask(~app.debug_nodes->getNodeMask());

        bool debug_edges = (app.debug_edges->getNodeMask() != 0);
        if (ImGui::Checkbox("Debug - edges", &debug_edges))
            app.debug_edges->setNodeMask(~app.debug_edges->getNodeMask());

        bool debug_edge_outlines = (app.debug_edge_outlines->getNodeMask() != 0);
        if (ImGui::Checkbox("Debug - edge outlines", &debug_edge_outlines))
            app.debug_edge_outlines->setNodeMask(~app.debug_edge_outlines->getNodeMask());

        bool debug_surface_outline = (app.debug_surface_outline->getNodeMask() != 0);
        if (ImGui::Checkbox("Debug - surface outline", &debug_surface_outline))
            app.debug_surface_outline->setNodeMask(~app.debug_surface_outline->getNodeMask());
    }
};

int
main(int argc, char** argv)
{
    osgEarth::initialize();
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer(arguments);
    viewer.setThreadingModel(viewer.SingleThreaded);
    viewer.setRealizeOperation(new GUI::ApplicationGUI::RealizeOperation);
    viewer.addEventHandler(new GUI::ApplicationGUI(arguments, true));
    viewer.getCamera()->setClearColor(osg::Vec4(0.1, 0.1, 0.1, 1));


#if 0
    Art art;
    //graph_t g = make_graph(art);
    graph_t g = make_osm_graph(art);

    if (g.nodes.size() == 0 || g.edges.size() == 0)
    {
        std::cout << "*************** NO DATA - EXITING ******************" << std::endl;
        exit(0);
    }
    
    // recompute everything when changes occur
    compile(g);
    
    auto root = new osg::Group();
    root->getOrCreateStateSet()->setMode(GL_BLEND, 1);

    App app;
    root->addChild(app.surface = tessellate_surface(g));
    root->addChild(app.markings = tessellate_markings(g, 0.05));    
    root->addChild(app.debug_edges = debug_edges(g, 0.5));
    root->addChild(app.debug_nodes = debug_nodes(g, 0.5));
    root->addChild(app.debug_edge_outlines = debug_edge_outlines(g, 0.1));
    root->addChild(app.debug_surface_outline = debug_surface_outline(g, 0.1));
    
    viewer.getCamera()->addCullCallback(new InstallCameraUniform());
    MapNodeHelper().configureView(&viewer);
    auto gui = new GUI::ApplicationGUI(arguments, false);
    gui->add(new MyGui(app));
    viewer.getEventHandlers().push_front(gui);
    viewer.setSceneData(root);
#endif

#if 0
    viewer.getCamera()->addCullCallback(new InstallCameraUniform());

    Map* map = new Map();

    auto imagery = new TMSImageLayer();
    imagery->setURL("http://readymap.org/readymap/tiles/1.0.0/7/");
    map->addLayer(imagery);

    auto elevation = new TMSElevationLayer();
    elevation->setURL("http://readymap.org/readymap/tiles/1.0.0/116/");
    elevation->setVerticalDatum("egm96");
    map->addLayer(elevation);

    osg::ref_ptr<XYZFeatureSource> fs = new XYZFeatureSource();
    fs->options().profile() = ProfileOptions("spherical-mercator");
    fs->setURL("http://readymap.org/readymap/mbtiles/daylight-v1.2/{z}/{x}/{-y}.pbf");
    fs->setMinLevel(14);
    fs->setMaxLevel(14);
    fs->setFormat("pbf");
    fs->setFIDAttribute("@id");

    //auto roads = new RoadLayer();
    //roads->setTileSize(1024);
    //roads->setFeatureSource(fs.get());
    //roads->setMinLevel(14);
    //roads->setMaxDataLevel(18);

    auto roads = new RoadLayer();
    roads->options().featureSource().setLayer(fs.get());

    map->addLayer(roads);

    auto mapnode = new MapNode(map);
    viewer.setSceneData(mapnode);

    MapNodeHelper().configureView(&viewer);

    //auto scene = MapNodeHelper().loadWithoutControls(arguments, &viewer);
    //if (!scene.valid())
    //    return 0;
    viewer.setCameraManipulator(new EarthManipulator(arguments));
//    viewer.setSceneData(scene);
#endif


    viewer.setCameraManipulator(new EarthManipulator(arguments));
    auto scene = MapNodeHelper().load(arguments, &viewer);
    if (scene.valid())
    {
        viewer.setSceneData(scene);
        return viewer.run();
    }
    return 0;
}