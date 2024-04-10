#include "RoadLayer"
#include <osgEarth/SimplePager>
#include <osgEarth/CropFilter>
#include <osgEarth/rtree.h>
#include <osgEarth/FeatureSDFLayer>
#include <osgEarth/TiledModelLayer>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainResources>
#include <osgEarth/ShaderLoader>
#include <osgEarth/ResampleFilter>
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>
#include <osg/Depth>
#include <osg/PolygonOffset>

using namespace osgEarth;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(roads, RoadLayer);

namespace
{
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
            type(NONE), backoff_length(0.0f), crossing_length(0.0f), orientation(1, 0, 0) { }
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

            while (restart)
            {
                restart = false;
                restart = false;
                for (auto lhs_iter = edges.begin(); lhs_iter != edges.end() && !restart; ++lhs_iter)
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
        properties_t road_basic;
        properties_t road_one_lane;

        Art()
        {
            //Config conf;
            //conf.fromXML(URIStream(URI("D:/data/textures/ambientcg/RoadLines007_1K-PNG/oe.xml")));
            //PBRMaterial lines_mat(conf.child("material"));
            auto asphalt = new osg::Texture2D(osgDB::readRefImageFile("D:/data/textures/ambientcg/Asphalt026B_4K-JPG/Asphalt026B_4K-JPG_Color.jpg"));
            asphalt->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
            asphalt->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
            asphalt->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
            asphalt->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
            auto asphalt_ss = new osg::StateSet();
            asphalt_ss->setTextureAttribute(0, asphalt);

            // basic 2 lane road
            {
                Config conf;
                conf.fromURI(URI("D:/data/textures/ambientcg/RoadLines007_1K-PNG/oe.xml"));
                PBRMaterial lines_mat(conf);
                //auto lines = new osg::Texture2D(osgDB::readRefImageFile("D:/data/textures/ambientcg/RoadLines007_1K-PNG/RoadLines007_1K-PNG_Color.png"));
                //auto lines = new osg::Texture2D(osgDB::readRefImageFile("D:/data/textures/reference_grid_Color.jpg"));
                auto lines = lines_mat.createTexture();
                lines->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
                lines->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
                //lines->setMaxAnisotropy(4.0f);
                auto lines_ss = new osg::StateSet();
                lines_ss->setTextureAttribute(0, lines);
                lines_ss->addUniform(new osg::Uniform("markings_tex_pbr", 0));

                auto crossing = new osg::Texture2D(URI("D:/data/textures/ambientcg/RoadLines004_1K-PNG/oe.xml").getImage());
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
        q.makeRotate(osg::Vec3d(0, 1, 0), orientation);
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
}

#define DEFAULT_WIDTH 5.0

#ifdef OSGEARTH_HAVE_CLIPPER2
#include <clipper2/clipper.h>
namespace
{
    class Clipper2OffsetFilter : public FeatureFilter
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
                    double width = DEFAULT_WIDTH;

                    if (feature->hasAttr("width"))
                        width = feature->getDouble("width", width);
                    else if (feature->hasAttr("lanes"))
                        width = feature->getDouble("lanes", 1.0) * DEFAULT_WIDTH;

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
                                //0.5*width,
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
}
#endif // OSGEARTH_HAVE_CLIPPER2

namespace
{
    const char* surface_shaders = R"(
        #pragma vp_function render_sdf_vs, vertex_view
        vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD);
        out vec4 oe_layer_tilec;
        out vec2 sdf_uv;
        void render_sdf_vs(inout vec4 vert)
        {
            sdf_uv = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, 19);
        }

        [break]

        #pragma vp_function render_sdf_fs, fragment
        uniform sampler2DArray road_tex_pbr;
        in vec2 sdf_uv;
        vec3 vp_Normal;
        mat3 oe_normalMapTBN;
        struct OE_PBR { float roughness, ao, metal, brightness, contrast; } oe_pbr;
        
        void texture_pbr(in sampler2DArray tex, in vec2 uv, inout vec3 rgb, inout vec3 N, out float height, inout OE_PBR pbr)
        {
            vec4 rgbh = texture(tex, vec3(uv.x, uv.y, 0.0));
            vec4 nnra = texture(tex, vec3(uv.x, uv.y, 1.0));
            rgb = rgbh.rgb;
            height = rgbh.a;
            //color.a = height_and_effect_mix(1, 0, max(rgbh.a, 0.5), color.a);
            pbr.roughness = nnra[2];
            pbr.ao = nnra[3];
            vec3 normal = nnra.xyz * 2.0 - 1.0;
            normal.z = 1.0 - abs(normal.x) - abs(normal.y);
            N = normalize(N + oe_normalMapTBN * normal);
            //vp_Normal = normalize(vp_Normal + oe_normalMapTBN * N);
        }

        float height_and_effect_mix(in float h1, in float a1, in float h2, in float a2)
        {
            // https://tinyurl.com/y5nkw2l9
            float ma = max(h1 + a1, h2 + a2) - 0.05; //displacement_depth;
            float b1 = max(h1 + a1 - ma, 0.0);
            float b2 = max(h2 + a2 - ma, 0.0);
            return b2 / (b1 + b2);
        }
                
        void render_sdf_fs(inout vec4 color)
        {
            const float min_height = 0.5;
            float dist = 1.0-color.r; // SDF distance [0..1]
            float height;

            texture_pbr(road_tex_pbr, sdf_uv, color.rgb, vp_Normal, height, oe_pbr);

            color.a = height_and_effect_mix(1, 0, max(height,min_height), dist);


            //const float min_h = 0.5;
            //float a = 1.0-color.r;

            //vec4 rgbh = texture(road_tex_pbr, vec3(sdf_uv.x, sdf_uv.y, 0.0));
            //vec4 nnra = texture(road_tex_pbr, vec3(sdf_uv.x, sdf_uv.y, 1.0));
            //        
            //const float min_h = 0.5;
            //float a = 1.0-color.r;
            //float h = rgbh[3];

            //color.rgb = rgbh.rgb;                    
            //color.a = height_and_effect_mix(1, 0, max(h,min_h), a);
            //        
            //oe_pbr.roughness = nnra[2];
            //oe_pbr.ao = nnra[3];
            //        
            //vec3 N = nnra.xyz * 2.0 - 1.0;
            //N.z = 1.0 - abs(N.x) - abs(N.y);
            //vp_Normal = normalize(vp_Normal + oe_normalMapTBN * N);
        }
    )";


    const char* markings_shaders = R"(
        #pragma vp_function pull_vs, vertex_clip, last
        uniform float pull_offset = 0.1;
        vec3 vp_Normal;
        out float ndc_depth;
        out vec2 oe_markings_uv;
        void pull_vs(inout vec4 vert)
        {
            ndc_depth = (vert.z/vert.w) - pull_offset;
            oe_markings_uv = gl_MultiTexCoord0.st;
        }

        [break]
        #pragma vp_function render_markings_fs, fragment
        uniform sampler2DArray markings_tex_pbr;
        in vec2 oe_markings_uv;
        vec3 vp_Normal;
        mat3 oe_normalMapTBN;
        struct OE_PBR { float roughness, ao, metal, brightness, contrast; } oe_pbr;

        void texture_pbr(in sampler2DArray tex, in vec2 uv, inout vec3 rgb, inout vec3 N, out float height, inout OE_PBR pbr)
        {
            vec4 rgbh = texture(tex, vec3(uv.x, uv.y, 0.0));
            vec4 nnra = texture(tex, vec3(uv.x, uv.y, 1.0));
            rgb = rgbh.rgb;
            height = rgbh.a;
            pbr.roughness = nnra[2];
            pbr.ao = nnra[3];
            vec3 normal = nnra.xyz * 2.0 - 1.0;
            normal.z = 1.0 - abs(normal.x) - abs(normal.y);
            N = normalize(N + oe_normalMapTBN * normal);
        }

        void render_markings_fs(inout vec4 color)
        {
            float height;
            texture_pbr(markings_tex_pbr, oe_markings_uv, color.rgb, vp_Normal, height, oe_pbr);
            color.a = height; // assume height is the opacity for markings
        }

        [break]
        #pragma vp_function pull_fs, fragment, last
        in float ndc_depth;
        void pull_fs(inout vec4 color)
        {
            if (color.a < 0.15) discard;
            gl_FragDepth = ((gl_DepthRange.diff*ndc_depth)+gl_DepthRange.near+gl_DepthRange.far)*0.5;
        }
    )";
}



namespace
{
    class SurfaceLayer : public FeatureSDFLayer
    {
    public:
        class Options : public FeatureSDFLayer::Options {
            META_LayerOptions(osgEarthProcedural, Options, FeatureSDFLayer::Options);
            void fromConfig(const Config&) { }
        };
        META_Layer(osgEarthProcedural, SurfaceLayer, Options, FeatureSDFLayer, _roads_surface);

    public:
        
        PBRMaterial _material;
        osg::ref_ptr<osg::Texture> _texture;
        TextureImageUnitReservation _unit;
        double _sdf_offset = DEFAULT_WIDTH;

        void init() override
        {
            super::init();
        }

        Status openImplementation() override
        {
            auto s = super::openImplementation();
            if (s.isError())
                return s;

            _texture = _material.createTexture(getReadOptions());
            if (!_texture.valid())
                return Status(Status::ResourceUnavailable, "RoadSurface: Failed to load surface material");

            Style style;
            style.getOrCreate<RenderSymbol>()->sdfMinDistance()->setLiteral(0.0);
            style.getOrCreate<RenderSymbol>()->sdfMaxDistance()->setLiteral(_sdf_offset);
            
            auto styles = new StyleSheet();
            styles->addStyle(style);
            options().styleSheet().setLayer(styles);

#ifdef OSGEARTH_HAVE_CLIPPER2
            // add the offseter if available
            _filterChain.push_back(new Clipper2OffsetFilter());
            _sdf_offset = 1.0;
#endif

            return s;
        }

        void prepareForRendering(TerrainEngine* engine) override
        {
            super::prepareForRendering(engine);

            if (!engine->getResources()->reserveTextureImageUnitForLayer(_unit, this))
            {
                setStatus(Status::ResourceUnavailable, "Failed to reserve texture image unit");
                return;
            }

            // the uniform for our custom shader:
            auto ss = getOrCreateStateSet();
            ss->getOrCreateUniform("road_tex_pbr", osg::Uniform::SAMPLER_2D_ARRAY)->set(_unit.unit());
            ss->setTextureAttribute(_unit.unit(), _texture);

            // and the shader itself:
            auto vp = VirtualProgram::getOrCreate(ss);
            vp->setName("Road Surface");
            ShaderLoader::load(vp, surface_shaders);
        }
    };


    class MarkingsLayer : public TiledModelLayer
    {
    public:
        class Options : public TiledModelLayer::Options {
            META_LayerOptions(osgEarthProcedural, Options, TiledModelLayer::Options);
        };
        META_Layer(osgEarthProcedural, MarkingsLayer, Options, TiledModelLayer, _roads_markings);

        LayerReference<FeatureSource> _features;
        osg::ref_ptr<StyleSheet> _stylesheet;
        osg::ref_ptr<Session> _session;
        FeatureFilterChain _filterChain; // not needed, use filteredfeaturesource
        Art _art;

        void init() override
        {
            super::init();
            setMinLevel(14);
            setMaxLevel(14);
            options().nvgl() = GLUtils::useNVGL();

            // blending on, depth writes off
            auto ss = getOrCreateStateSet();
            ss->setMode(GL_BLEND, 1);
            ss->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0, 1, false), 1);
            //ss->setAttributeAndModes(new osg::PolygonOffset(-1, -1), 1);
            
            _filterChain.push_back(new ResampleFilter(0.0, 25.0));
        }

        Status openImplementation() override
        {
            auto s = super::openImplementation();
            if (s.isError())
                return s;

            _features.open(getReadOptions());
            return s;
        }

        void addedToMap(const Map* map)
        {
            super::addedToMap(map);

            _features.addedToMap(map);

            auto fs = _features.getLayer();
            if (!fs)
            {
                setStatus(Status::ConfigurationError, "RoadMarkingsDecalLayer: Missing required feature source");
                return;
            }

            if (!fs->getFeatureProfile())
            {
                setStatus(Status::ConfigurationError, "RoadMarkingsDecalLayer: Feature source has no feature profile");
                return;
            }

            _session = new Session(map);
        }

        void removedFromMap(const Map* map)
        {
            super::removedFromMap(map);
            _features.removedFromMap(map);
        }

        void prepareForRendering(TerrainEngine* engine) override
        {
            super::prepareForRendering(engine);

            // and the shader itself:
            auto vp = VirtualProgram::getOrCreate(getOrCreateStateSet());
            ShaderLoader::load(vp, markings_shaders);
            vp->setName("Road Markings");
        }

        const Profile* getProfile() const override
        {
            return Profile::create(Profile::GLOBAL_GEODETIC);

            auto fs = _features.getLayer();

            if (fs && fs->getFeatureProfile())
                return fs->getFeatureProfile()->getTilingProfile();
            else
                return nullptr;
        }

        osg::ref_ptr<osg::Node> createTileImplementation(const TileKey& key, ProgressCallback* progress) const override
        {
            // take local refs to isolate this method from the member objects
            auto featureSource(_features.getLayer());

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

            auto sheet = _stylesheet.get();
            auto style = sheet ? sheet->getDefaultStyle() : nullptr;
            auto render = style ? style->get<RenderSymbol>() : nullptr;
            if (render) render->applyTo(root);

            // no need if we chonkin'
            //osgUtil::Optimizer::MergeGeometryVisitor mgv;
            //mgv.setTargetMaximumNumberOfVertices(~0);
            //root->accept(mgv);

            // debugging only
            //GeometryValidator gv;
            //root->accept(gv);

            auto mt = new osg::MatrixTransform();
            osg::Matrixd local2world;
            centroid.createLocalToWorld(local2world);
            mt->setMatrix(local2world);
            mt->addChild(root);

            return mt;
        }
    };
}



void
RoadLayer::Options::fromConfig(const Config& conf)
{
    features().get(conf, "features");
    conf.child("surface").get("material", surfaceMaterial());
}

Config
RoadLayer::Options::getConfig() const
{
    Config conf = super::getConfig();
    features().set(conf, "features");
    conf.add("surface", surfaceMaterial()->getConfig());
    return conf;
}

void
RoadLayer::init()
{
    super::init();
}

Status
RoadLayer::openImplementation()
{
    // Layer that splats the surface texture on the terrain
    auto surface = new SurfaceLayer();
    surface->setName("  Roads - surface");
    surface->setMinLevel(getMinLevel());
    surface->setMaxDataLevel(getMaxDataLevel());
    surface->options().features() = options().features();
    surface->_material = options().surfaceMaterial().value();
    surface->setUserProperty("show_in_ui", "false");
    Status s = surface->open(getReadOptions());
    if (s.isError())
        return s;
    _surfaceLayer = surface;

    // Layer that renders the road marking decal geometries
    auto* markings = new MarkingsLayer();
    markings->setName("  Roads - markings");
    markings->setMinLevel(getMinLevel());
    markings->_features = options().features();
    markings->setUserProperty("show_in_ui", "false");
    Status s2 = markings->open(getReadOptions());
    if (s2.isError())
        return s2;
    _markingsLayer = markings;

    _sublayers.emplace_back(surface);
    _sublayers.emplace_back(markings);

    return super::openImplementation();
}

void
RoadLayer::addedToMap(const Map* map)
{
    //options().features().addedToMap(map);
}

void
RoadLayer::removedFromMap(const Map* map)
{
    //options().features().removedFromMap(map);
}
