#include "MeshEditor"
#include "GeometryPool"

#include <osgEarth/Locators>
#include <osgEarth/Map>
#include <osgEarth/Math>
#include <osgEarth/TerrainConstraintLayer>
#include <osgEarth/rtree.h>
#include <algorithm>
#include <iostream>

#define LC "[MeshEditor] "

using namespace osgEarth;
using namespace osgEarth::REX;

MeshEditor::MeshEditor(const TileKey& key, unsigned tileSize, const Map* map, ProgressCallback* progress) :
    _key( key ), 
    _tileSize(tileSize),
    _tileEmpty(false)
{
    // Iterate over all constraint layers:
    std::vector<osg::ref_ptr<TerrainConstraintLayer>> layers;
    map->getOpenLayers(layers);

    const GeoExtent& keyExtent = key.getExtent();

    for(auto& layer : layers)
    {
        // not to the min LOD yet?
        if (layer->getMinLevel() > key.getLOD())
            continue;

        // extents don't intersect?
        if (!layer->getExtent().intersects(keyExtent))
            continue;

        // For each feature, check that it intersects the tile key,
        // and then xform it to the correct SRS and clone it for
        // editing.
        FeatureSource* fs = layer->getFeatureSource();
        if (fs)
        {
            osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(
                key,
                progress);

            Edit edit;
            while (cursor.valid() && cursor->hasMore())
            {
                Feature* f = cursor->nextFeature();
                if (f->getExtent().intersects(keyExtent))
                {
                    osg::ref_ptr<Feature> f_xform = osg::clone(f, osg::CopyOp::DEEP_COPY_ALL);
                    f_xform->transform(keyExtent.getSRS());
                    edit._features.push_back(f_xform);
                }
            }

            if (!edit._features.empty())
            {
                edit._layer = layer;
                _edits.emplace_back(edit);
            }
        }
    }
}

namespace
{
    // MESHING SDK

    // 2.5D vertex. Holds a Z, but most operations only use X/Y
    struct vert_t
    {
        typedef double value_type;
        double _x, _y, _z;
        double& x() { return _x; }
        const double& x() const { return _x; }
        double& y() { return _y; }
        const double& y() const { return _y; }
        double& z() { return _z; }
        const double& z() const { return _z; }
        vert_t() { }
        vert_t(value_type a, value_type b, value_type c) : _x(a), _y(b), _z(c){ }
        vert_t(const vert_t& rhs) : _x(rhs.x()), _y(rhs.y()), _z(rhs.z()) { }
        vert_t(const osg::Vec3d& rhs) : _x(rhs.x()), _y(rhs.y()), _z(rhs.z()) { }
        bool operator < (const vert_t& rhs) const {
            if (x() < rhs.x()) return true;
            if (x() > rhs.x()) return false;
            return y() < rhs.y();
        }
        vert_t operator - (const vert_t& rhs) const {
            return vert_t(x() - rhs.x(), y() - rhs.y(), z() - rhs.z());
        }
        vert_t operator + (const vert_t& rhs) const {
            return vert_t(x() + rhs.x(), y() + rhs.y(), z() + rhs.z());
        }
        vert_t operator * (value_type a) const {
            return vert_t(x()*a, y()*a, z()*a);
        }
        value_type dot2d(const vert_t& rhs) const {
            return x()*rhs.x() + y() * rhs.y();
        }
        value_type cross2d(const vert_t& rhs) const {
            return x()*rhs.y() - rhs.x()*y();
        }
        vert_t normalize2d() const {
            double len = length();
            return vert_t(x() / len, y() / len, z());
        }
        void set(value_type a, value_type b, value_type c) {
            _x = a, _y = b, _z = c;
        }
        value_type length() const {
            return sqrt((_x*_x) + (_y*_y));
        }
        value_type length2() const {
            return (_x*_x) + (_y*_y);
        }
    };

    const vert_t::value_type zero(0.0);

    inline bool equivalent(vert_t::value_type a, vert_t::value_type b)
    {
        //return osg::equivalent((float)a, (float)b);
        return osg::equivalent(a, b);
    }

    inline bool equivalent(const vert_t& a, const vert_t& b, vert_t::value_type epsilon)
    {
        return
            osg::equivalent(a.x(), b.x(), epsilon) &&
            osg::equivalent(a.y(), b.y(), epsilon);
    }


    // uniquely map vertices to indices
    typedef std::map<vert_t, int> vert_table_t;

    // array of vert_t's
    struct vert_array_t : public osg::MixinVector<vert_t> { };

    // line segment connecting two verts
    struct segment_t : std::pair<vert_t, vert_t>
    {
        segment_t(const vert_t& a, const vert_t& b) :
            std::pair<vert_t, vert_t>(a, b) { }

        // true if 2 segments intersect; intersection point in "out"
        bool intersect(const segment_t& rhs, vert_t& out) const
        {
            vert_t r = second - first;
            vert_t s = rhs.second - rhs.first;
            vert_t::value_type det = r.cross2d(s);

            if (equivalent(det, zero))
                return false;

            vert_t diff = rhs.first - first;
            vert_t::value_type u = diff.cross2d(s) / det;
            vert_t::value_type v = diff.cross2d(r) / det;

            out = first + (r * u);
            return (u > 0.0 && u < 1.0 && v > 0.0 && v < 1.0);
        }
    };

    struct triangle_t
    {
        UID uid; // unique id
        vert_t p0, p1, p2; // vertices
        unsigned i0, i1, i2; // indices
        vert_t::value_type e01, e12, e20; // edge lengths
        vert_t::value_type a_min[2]; // bbox min
        vert_t::value_type a_max[2]; // bbox max
        vert_t::value_type area;
        bool is_2d_degenerate;

        // true if the triangle contains point P (in xy)
        bool contains2d(const vert_t& P) const
        {
            vert_t c = p2 - p0;
            vert_t b = p1 - p0;
            vert_t p = P - p0;

            vert_t::value_type cc = c.dot2d(c), bc = b.dot2d(c), pc = c.dot2d(p), bb = b.dot2d(b), pb = b.dot2d(p);
            vert_t::value_type denom = cc * bb - bc * bc;
            if (equivalent(denom, zero))
                return false;

            float u = (bb*pc - bc * pb) / denom;
            float v = (cc*pb - bc * pc) / denom;

            return u >= 0 && v >= 0 && (u + v < 1.0);
        }

        vert_t::value_type dist_to_nearest_vertex(const vert_t& P) const
        {
            return std::min((P - p0).length(), std::min((P - p1).length(), (P - p2).length()));
        }

        // true if point P is one of the triangle's verts
        bool is_vertex(const vert_t& p) const
        {
            if (equivalent(p.x(), p0.x()) &&
                equivalent(p.y(), p0.y()))
                return true;
            if (equivalent(p.x(), p1.x()) &&
                equivalent(p.y(), p1.y()))
                return true;
            if (equivalent(p.x(), p2.x()) &&
                equivalent(p.y(), p2.y()))
                return true;
            return false;
        }
    };

#if 1
    typedef RTree<UID, vert_t::value_type, 2> spatial_index_t;
#else
    //! A dirt-simple (and slow) spatial index, just for testing.
    struct spatial_index_t {
        struct rec {
            UID uid;
            double a_min[2];
            double a_max[2];
            bool operator < (const rec& rhs) const {
                return uid < rhs.uid;
            }
        };
        std::map<UID, rec> _recs;

        void Insert(double* a_min, double* a_max, UID uid)
        {
            rec& r = _recs[uid];
            r.uid = uid;
            r.a_min[0] = a_min[0], r.a_min[1] = a_min[1];
            r.a_max[0] = a_max[0], r.a_max[1] = a_max[1];
        }

        void Remove(double* a_min, double* a_max, UID uid)
        {
            _recs.erase(uid);
        }

        void Search(double* a_min, double* a_max, std::unordered_set<UID>* hits, int maxHits) const
        {
            for (auto& e : _recs)
            {
                if (e.second.a_min[0] > a_max[0] ||
                    e.second.a_max[0] < a_min[0] ||
                    e.second.a_min[1] > a_max[1] ||
                    e.second.a_max[1] < a_min[1])
                {
                    continue;
                }
                else
                {
                    hits->emplace(e.first);
                }
            }
        }
    };
#endif

    // a mesh edge connecting to verts
    struct edge_t
    {
        int _i0, _i1; // vertex indicies
        edge_t() : _i0(-1), _i1(-1) { }
        edge_t(int i0, int i1) : _i0(i0), _i1(i1) { }

        // don't care about direction
        bool operator == (const edge_t& rhs) const {
            return
                (_i0 == rhs._i0 && _i1 == rhs._i1) ||
                (_i0 == rhs._i1 && _i1 == rhs._i0);
        }

        // hash table function. This needs to combine i0 and i1 in 
        // a commutative way, i.e., such that if _i0 and _i1 are 
        // interchanged, they will return the same hash code.
        std::size_t operator()(const edge_t& edge) const {
            return hash_value_unsigned(_i0 + _i1);
        }
    };

    // connected mesh of triangles, verts, and associated markers
    struct mesh_t
    {
        int uidgen;
        std::unordered_map<UID, triangle_t> _triangles;
        spatial_index_t _spatial_index;
        vert_table_t _vert_lut;
        vert_array_t _verts;
        std::vector<int> _markers;
        int _num_splits;

        mesh_t() : uidgen(0), _num_splits(0) {
            //nop
        }

        // delete triangle from the mesh
        void remove_triangle(triangle_t& tri)
        {
            UID uid = tri.uid;
            //_spatial_index.Remove(tri, uid);
            _spatial_index.Remove(tri.a_min, tri.a_max, uid);
            _triangles.erase(uid);
            _num_splits++;
        }

        // add new triangle to the mesh from 3 indices
        UID add_triangle(int i0, int i1, int i2)
        {
            if (i0 == i1 || i1 == i2 || i2 == i0)
                return -1;

            UID uid(uidgen++);
            triangle_t tri;
            tri.uid = uid;
            tri.i0 = i0;
            tri.i1 = i1;
            tri.i2 = i2;
            tri.p0 = get_vertex(i0);
            tri.p1 = get_vertex(i1);
            tri.p2 = get_vertex(i2);
            tri.e01 = (tri.p0 - tri.p1).length();
            tri.e12 = (tri.p1 - tri.p2).length();
            tri.e20 = (tri.p2 - tri.p0).length();
            tri.a_min[0] = std::min(tri.p0.x(), std::min(tri.p1.x(), tri.p2.x()));
            tri.a_min[1] = std::min(tri.p0.y(), std::min(tri.p1.y(), tri.p2.y()));
            tri.a_max[0] = std::max(tri.p0.x(), std::max(tri.p1.x(), tri.p2.x()));
            tri.a_max[1] = std::max(tri.p0.y(), std::max(tri.p1.y(), tri.p2.y()));
            tri.area = 0.5 * fabs(
                tri.p0.x()*(tri.p1.y() - tri.p2.y()) +
                tri.p1.x()*(tri.p2.y() - tri.p0.y()) +
                tri.p2.x()*(tri.p0.y() - tri.p1.y()));

            constexpr double E = 0.0005;
            tri.is_2d_degenerate =
                equivalent(tri.p0, tri.p1, E) ||
                equivalent(tri.p1, tri.p2, E) ||
                equivalent(tri.p2, tri.p0, E) ||
                equivalent((tri.p1 - tri.p0).normalize2d(), (tri.p2 - tri.p0).normalize2d(), E) ||
                equivalent((tri.p2 - tri.p1).normalize2d(), (tri.p0 - tri.p1).normalize2d(), E) ||
                equivalent((tri.p0 - tri.p2).normalize2d(), (tri.p1 - tri.p2).normalize2d(), E);

            if (equivalent(tri.area, 0.0))
                return -1;

            _triangles.emplace(uid, tri);
            //_spatial_index.Insert(tri, uid);
            _spatial_index.Insert(tri.a_min, tri.a_max, uid);
            return uid;
        }

        // find a vertex by its index
        const vert_t& get_vertex(unsigned i) const
        {
            return _verts[i];
        }

        // find a vertex by its index
        vert_t& get_vertex(unsigned i)
        {
            return _verts[i];
        }

        // find the marker for a vertex
        int& get_marker(const vert_t& vert)
        {
            return _markers[_vert_lut[vert]];
        }

        // find the marker for a vertex index
        int get_marker(int i)
        {
            return _markers[i];
        }

        // add a new vertex (or lookup a matching one) and return its index
        int get_or_create_vertex(const vert_t& input, int marker)
        {
            int index;
            vert_table_t::iterator i = _vert_lut.find(input);
            if (i != _vert_lut.end())
            {
                index = i->second;
                _markers[i->second] = marker;
            }
            else
            {
                _verts.push_back(input);
                _markers.push_back(marker);
                _vert_lut[input] = _verts.size() - 1;
                index = _verts.size() - 1;
            }
            if (index >= 0xFFFF)
            {
                OE_WARN << "Exceeded maximum allowable verts" << std::endl;
            }

            return index;
        }

        // insert a point into the mesh, cutting triangles as necessary
        void insert(const vert_t& vert, int marker)
        {
            // restrict edge length to a fraction of the original edge lengths
            // (hueristic value) - problem is tile is too big (curvature)
            //vert_t::value_type min_edge = (_triangles.begin()->second.e01) * 0.01;
            //vert_t::value_type min_area = 1.0;
            vert_t::value_type min_edge = 0.0;
            vert_t::value_type min_area = 0.0;

            // search for possible intersecting triangles (should only be one)
            vert_t::value_type a_min[2];
            vert_t::value_type a_max[2];
            a_min[0] = a_max[0] = vert.x();
            a_min[1] = a_max[1] = vert.y();

            std::unordered_set<UID> uids;
            _spatial_index.Search(a_min, a_max, &uids, ~0);

            for (auto uid : uids)
            {
                triangle_t& tri = _triangles[uid];

                if (tri.is_2d_degenerate)
                    continue;

                if (tri.area <= min_area) // probably redundant
                    continue;

                if (tri.contains2d(vert) &&
                    tri.dist_to_nearest_vertex(vert) > min_edge)
                {
                    inside_split(tri, vert, nullptr, marker);
                    break;
                }
            }
        }

        // insert a segment into the mesh, cutting triangles as necessary
        void insert(const segment_t& seg, int marker)
        {
            // restrict edge length to a fraction of the original edge lengths
            // (hueristic value) - problem is tile is too big (curvature)
            vert_t::value_type min_edge =
                (_triangles.begin()->second.e01) * 0.15;

            // search for possible intersecting triangles:
            vert_t::value_type a_min[2];
            vert_t::value_type a_max[2];
            a_min[0] = std::min(seg.first.x(), seg.second.x());
            a_min[1] = std::min(seg.first.y(), seg.second.y());
            a_max[0] = std::max(seg.first.x(), seg.second.x());
            a_max[1] = std::max(seg.first.y(), seg.second.y());
            std::unordered_set<UID> uids;
            _spatial_index.Search(a_min, a_max, &uids, ~0);

            // The working set of triangles which we will add to if we have
            // to split triangles. Any triangle only needs to be split once,
            // even if it gets intersected multiple times. That is because each
            // split generates new traigles, and discards the original, and further
            // splits will just happen on the new triangles later. (That's why
            // every split operation is followed by a "continue" to short-circuit
            // to loop)
            std::list<UID> uid_list;
            std::copy(uids.begin(), uids.end(), std::back_inserter(uid_list));
            for (auto uid : uid_list)
            {
                triangle_t& tri = _triangles[uid];

                if (tri.is_2d_degenerate)
                    continue;

                // is the first segment endpoint inside a triangle? if so
                // insert the point and split it into three new triangles
                if (tri.contains2d(seg.first) &&
                    tri.dist_to_nearest_vertex(seg.first) > min_edge)
                {
                    inside_split(tri, seg.first, &uid_list, marker);
                    continue;
                }

                // is the second segment endpoint inside a triangle? if so
                // insert the point and split it into three new triangles
                if (tri.contains2d(seg.second) &&
                    tri.dist_to_nearest_vertex(seg.second) > min_edge)
                {
                    inside_split(tri, seg.second, &uid_list, marker);
                    continue;
                }

                // next try to intersect the segment with each triangle edge.
                // in each case, make sure the intersection point isn't 
                // coincident with an existing vertex (in which case ignore it)
                vert_t out;
                UID new_uid;
                int new_i;

                // does the segment cross first triangle edge?
                if (tri.e01 > min_edge)
                {
                    segment_t edge0(tri.p0, tri.p1);
                    if (seg.intersect(edge0, out))
                    {
                        int new_marker = marker;
                        if ((_markers[tri.i0] & VERTEX_BOUNDARY) && (_markers[tri.i1] & VERTEX_BOUNDARY))
                            new_marker |= VERTEX_BOUNDARY;

                        new_i = get_or_create_vertex(out, new_marker);

                        if (!tri.is_vertex(out))
                        {
                            int new_tris = 0;

                            new_uid = add_triangle(new_i, tri.i2, tri.i0);
                            if (new_uid >= 0) {
                                uid_list.push_back(new_uid);
                                ++new_tris;
                            }

                            new_uid = add_triangle(new_i, tri.i1, tri.i2);
                            if (new_uid >= 0) {
                                uid_list.push_back(new_uid);
                                ++new_tris;
                            }

                            if (new_tris > 0)
                            {
                                remove_triangle(tri);
                                continue;
                            }
                        }
                    }
                }

                // does the segment cross second triangle edge?
                if (tri.e12 > min_edge)
                {
                    segment_t edge1(tri.p1, tri.p2);
                    if (seg.intersect(edge1, out))
                    {
                        int new_marker = marker;
                        if ((_markers[tri.i1] & VERTEX_BOUNDARY) &&(_markers[tri.i2] & VERTEX_BOUNDARY))
                            new_marker |= VERTEX_BOUNDARY;

                        new_i = get_or_create_vertex(out, new_marker);

                        if (!tri.is_vertex(out))
                        {
                            int new_tris = 0;

                            new_uid = add_triangle(new_i, tri.i0, tri.i1);
                            if (new_uid >= 0) {
                                uid_list.push_back(new_uid);
                                ++new_tris;
                            }

                            new_uid = add_triangle(new_i, tri.i2, tri.i0);
                            if (new_uid >= 0) {
                                uid_list.push_back(new_uid);
                                ++new_tris;
                            }

                            if (new_tris > 0)
                            {
                                remove_triangle(tri);
                                continue;
                            }
                        }
                    }
                }

                // does the segment cross third triangle edge?
                if (tri.e20 > min_edge)
                {
                    segment_t edge2(tri.p2, tri.p0);
                    if (seg.intersect(edge2, out))
                    {
                        int new_marker = marker;
                        if ((_markers[tri.i2] & VERTEX_BOUNDARY) && (_markers[tri.i0] & VERTEX_BOUNDARY))
                            new_marker |= VERTEX_BOUNDARY;

                        new_i = get_or_create_vertex(out, new_marker);

                        if (!tri.is_vertex(out))
                        {
                            int new_tris = 0;

                            new_uid = add_triangle(new_i, tri.i1, tri.i2);
                            if (new_uid >= 0) {
                                uid_list.push_back(new_uid);
                                ++new_tris;
                            }

                            new_uid = add_triangle(new_i, tri.i0, tri.i1);
                            if (new_uid >= 0) {
                                uid_list.push_back(new_uid);
                                ++new_tris;
                            }

                            if (new_tris > 0)
                            {
                                remove_triangle(tri);
                                continue;
                            }
                        }
                    }
                }
            }
        }

        // inserts point "p" into the interior of triangle "tri",
        // adds three new triangles, and removes the original triangle.
        void inside_split(triangle_t& tri, const vert_t& p, std::list<UID>* uid_list, int new_marker)
        {
            int new_i = get_or_create_vertex(p, new_marker);

            UID new_uid;
            int new_tris = 0;

            new_uid = add_triangle(tri.i0, tri.i1, new_i);
            if (new_uid >= 0 && uid_list) {
                uid_list->push_back(new_uid);
                ++new_tris;
            }

            new_uid = add_triangle(tri.i1, tri.i2, new_i);
            if (new_uid >= 0 && uid_list) {
                uid_list->push_back(new_uid);
                ++new_tris;
            }

            new_uid = add_triangle(tri.i2, tri.i0, new_i);
            if (new_uid >= 0 && uid_list) {
                uid_list->push_back(new_uid);
                ++new_tris;
            }

            if (new_tris > 0)
                remove_triangle(tri);
        }
    };

    // a graph node
    struct node_t
    {
        int _vertex_index; // vertex index
        int _graphid;
        std::set<node_t*> _edges;
        node_t(int vi=0) : _vertex_index(vi), _graphid(-1) { }
        bool operator == (const node_t& rhs) const {
            return _vertex_index == rhs._vertex_index;
        }
    };

    // collection of edges (optionally corresponding to marker data)
    struct edgeset_t
    {
        std::unordered_set<edge_t, edge_t> _edges;

        edgeset_t(const mesh_t& mesh, int marker_mask)
        {
            for (auto& tri_iter : mesh._triangles)
            {
                const triangle_t& tri = tri_iter.second;
                add_triangle(tri, mesh, marker_mask);
            }
        }

        void add_triangle(const triangle_t& tri, const mesh_t& mesh, int marker_mask)
        {
            bool m0 = (mesh._markers[tri.i0] & marker_mask) != 0;
            bool m1 = (mesh._markers[tri.i1] & marker_mask) != 0;
            bool m2 = (mesh._markers[tri.i2] & marker_mask) != 0;
            
            if (m0 && m1)
                _edges.emplace(edge_t(tri.i0, tri.i1));
            if (m1 && m2)
                _edges.emplace(edge_t(tri.i1, tri.i2));
            if (m2 && m0)
                _edges.emplace(edge_t(tri.i2, tri.i0));
        }
    };

    // undirected graph representing a mesh
    // (currently unused!)
    struct graph_t
    {
        std::unordered_map<int, node_t> _nodes;
        int _num_subgraphs;

        graph_t(const mesh_t& mesh)
        {
            for (auto& tri_iter : mesh._triangles)
            {
                const triangle_t& tri = tri_iter.second;
                add_triangle(tri);
            }
            assign_graph_ids();
        }

        node_t& get_or_create_node(int vertex_index)
        {
            auto& node = _nodes[vertex_index];
            node._vertex_index = vertex_index;
            return node;
        }

        void add_triangle(const triangle_t& tri)
        {
            auto& node0 = get_or_create_node(tri.i0);
            auto& node1 = get_or_create_node(tri.i1);
            auto& node2 = get_or_create_node(tri.i2);

            node0._edges.insert(&node1);
            node0._edges.insert(&node2);

            node1._edges.insert(&node0);
            node1._edges.insert(&node2);

            node2._edges.insert(&node0);
            node2._edges.insert(&node1);
        }

        void assign_graph_ids()
        {
            _num_subgraphs = 0;
            for (auto& node_iter : _nodes)
            {
                node_t& node = node_iter.second;
                if (assign_graph_ids(node, _num_subgraphs))
                    ++_num_subgraphs;
            }
        }

        bool assign_graph_ids(node_t& node, int graphid)
        {
            if (node._graphid < 0)
            {
                node._graphid = graphid;

                for (auto& edge : node._edges)
                {
                    assign_graph_ids(*edge, graphid);
                }
                return true;
            }
            return false;
        }

        int get_num_subgraphs() const
        {
            return _num_subgraphs;
        }

        // attempt to find the concave hull by walking the outside of subgraph.
        void get_hull(int graphid, const mesh_t& mesh, std::vector<int>& hull) const
        {
            const node_t* start = nullptr;
            vert_t::value_type min_y = DBL_MAX;
            vert_t::value_type min_x = DBL_MAX;

            // find the vertex with minimum y in graph graphid.
            // TODO: it would be faster to find a vertex in graph graphid,
            // and then traverse from there.
            for (const auto& node_iter : _nodes)
            {
                const node_t& node = node_iter.second;
                if (node._graphid == graphid)
                {
                    const vert_t& vert = mesh.get_vertex(node._vertex_index);
                    if (start == nullptr || vert.y() < min_y)
                    {
                        start = &node;
                        min_y = vert.y();
                    }
                }
            }

            if (start == nullptr)
                return;

            // next walk the boundary in a clockwise direction
            const node_t* curr_node = start;
            const vert_t& first = mesh.get_vertex(curr_node->_vertex_index);
            const node_t* prev_node = nullptr;
            vert_t prev_vert(first + vert_t(0, -1, 0));

            std::set<const node_t*> visited;
            
            while (true)
            {
                hull.push_back(curr_node->_vertex_index);

                double best_score = DBL_MAX;
                const node_t* best_edge = nullptr;
                const vert_t& curr_vert = mesh.get_vertex(curr_node->_vertex_index);

                vert_t invec = (curr_vert - prev_vert).normalize2d();

                for (auto& edge : curr_node->_edges)
                {
                    if (visited.find(edge) != visited.end())
                        continue;
                    //if (edge == prev_node) // no backtracking
                    //    continue;

                    const vert_t& next_vert = mesh.get_vertex(edge->_vertex_index);
                    vert_t outvec = (next_vert - curr_vert).normalize2d();

                    double turn = invec.cross2d(outvec) > 0.0 ? -1.0 : 1.0;
                    double dot = 1.0 - (0.5*(invec.dot2d(outvec) + 1.0));
                    double score = turn * dot;

                    if (score < best_score)
                    {
                        best_score = score;
                        best_edge = edge;
                    }
                }

                if (best_edge == nullptr)
                {
                    OE_WARN << "got stuck! :(" << std::endl;
                    break;
                }

                if (best_edge == start)
                {
                    // done!
                    break;
                }

                if (curr_node != start)
                    visited.insert(curr_node);

                prev_node = curr_node;
                prev_vert = curr_vert;
                curr_node = best_edge;
            }
        }
    };
}

#define addSkirtDataForIndex(INDEX, HEIGHT) \
{ \
    verts->push_back( (*verts)[INDEX] ); \
    normals->push_back( (*normals)[INDEX] ); \
    texCoords->push_back( (*texCoords)[INDEX] ); \
    texCoords->back().z() = (float)((int)texCoords->back().z() | VERTEX_SKIRT); \
    if ( neighbors ) neighbors->push_back( (*neighbors)[INDEX] ); \
    if ( neighborNormals ) neighborNormals->push_back( (*neighborNormals)[INDEX] ); \
    verts->push_back( (*verts)[INDEX] - ((*normals)[INDEX])*(HEIGHT) ); \
    normals->push_back( (*normals)[INDEX] ); \
    texCoords->push_back( (*texCoords)[INDEX] ); \
    texCoords->back().z() = (float)((int)texCoords->back().z() | VERTEX_SKIRT); \
    if ( neighbors ) neighbors->push_back( (*neighbors)[INDEX] - ((*normals)[INDEX])*(HEIGHT) ); \
    if ( neighborNormals ) neighborNormals->push_back( (*neighborNormals)[INDEX] ); \
}

#define addSkirtTriangles(PS, INDEX0, INDEX1) \
{ \
    PS->addElement((INDEX0));   \
    PS->addElement((INDEX0)+1); \
    PS->addElement((INDEX1));   \
    PS->addElement((INDEX1));   \
    PS->addElement((INDEX0)+1); \
    PS->addElement((INDEX1)+1); \
}

bool
MeshEditor::createTileMesh(
    SharedGeometry* sharedGeom,
    unsigned tileSize,
    double skirtHeightRatio)
{
    // uncomment for easier debugging
    //static Mutex m;
    //ScopedMutexLock lock(m);

    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    const GeoExtent& keyExtent = _key.getExtent();
    keyExtent.getCentroid(centroid);
    centroid.toWorld(centerWorld);
    osg::Matrix world2local, local2world;
    centroid.createWorldToLocal(world2local);
    local2world.invert(world2local);
    GeoLocator locator(keyExtent);
    const SpatialReference* tileSRS = keyExtent.getSRS();

    // calculate the bounding box of the tile in local coords,
    // for culling purposes:
    osg::Vec3d c[4];
    double xmin = DBL_MAX, ymin = DBL_MAX, xmax = -DBL_MAX, ymax = -DBL_MAX, zmin = DBL_MAX;
    locator.unitToWorld(osg::Vec3d(0, 0, 0), c[0]);
    locator.unitToWorld(osg::Vec3d(1, 0, 0), c[1]);
    locator.unitToWorld(osg::Vec3d(0, 1, 0), c[2]);
    locator.unitToWorld(osg::Vec3d(1, 1, 0), c[3]);
    for (int i = 0; i < 4; ++i) {
        c[i] = c[i] * world2local;
        xmin = std::min(xmin, c[i].x()), xmax = std::max(xmax, c[i].x());
        ymin = std::min(ymin, c[i].y()), ymax = std::max(ymax, c[i].y());
        zmin = std::min(zmin, c[i].z());
    }

    mesh_t mesh;
    mesh._verts.reserve(tileSize*tileSize);

    double xscale = -zmin / 0.5*(xmax - xmin);
    double yscale = -zmin / 0.5*(ymax - ymin);

    for (unsigned row = 0; row < tileSize; ++row)
    {
        double ny = (double)row / (double)(tileSize - 1);
        for (unsigned col = 0; col < tileSize; ++col)
        {
            double nx = (double)col / (double)(tileSize - 1);
            osg::Vec3d unit(nx, ny, 0.0);
            osg::Vec3d model;
            osg::Vec3d modelLTP;

            locator.unitToWorld(unit, model);
            modelLTP = model * world2local;

            int marker =
                VERTEX_VISIBLE |
                VERTEX_CONSTRAINT;

            // mark the perimeter as a boundary (for skirt generation)
            if (row == 0 || row == tileSize - 1 || col == 0 || col == tileSize - 1)
                marker |= VERTEX_BOUNDARY;

            int i = mesh.get_or_create_vertex(
                modelLTP,
                marker);

            if (row > 0 && col > 0)
            {
                mesh.add_triangle(i, i - 1, i - tileSize - 1);
                mesh.add_triangle(i, i - tileSize - 1, i - tileSize);
            }
        }
    }

    // Make the edits
    for (auto& edit : _edits)
    {
        // we're marking everything as a CONSTRAINT in order to disable morphing.
        int default_marker =
            VERTEX_VISIBLE |
            VERTEX_CONSTRAINT;

        // this will preserve a "burned-in" Z value.
        if (edit._layer->getHasElevation())
            default_marker |= VERTEX_HAS_ELEVATION;

        // track number of unused triangles so we can discard empty tiles
        int usused_tris = 0;

        for (auto& feature : edit._features)
        {
            GeometryIterator geom_iter(feature->getGeometry(), true);
            osg::Vec3d world, unit;
            while (geom_iter.hasMore())
            {
                Geometry* part = geom_iter.next();

                for (auto& point : *part)
                {
                    tileSRS->transformToWorld(point, world);
                    point = world * world2local;
                }

                int marker = default_marker;

                if (part->isPointSet())
                {
                    for (int i = 0; i < part->size(); ++i)
                    {
                        const vert_t& v = (*part)[i];
                        if (v.x() >= xmin && v.x() <= xmax &&
                            v.y() >= ymin && v.y() <= ymax)
                        {
                            mesh.insert(v, marker);
                        }
                    }
                }

                else
                {
                    // marking as BOUNDARY will allow skirt generation on this part
                    // for polygons with removed interior/exteriors
                    if (part->isRing() && (
                        edit._layer->getRemoveInterior() ||
                        edit._layer->getRemoveExterior()))
                    {
                        marker |= VERTEX_BOUNDARY;
                    }

                    // slice and dice the mesh.
                    // iterate over segments in the part, closing the loop if it's an open ring.
                    unsigned i = part->isRing() && part->isOpen() ? 0 : 1;
                    unsigned j = part->isRing() && part->isOpen() ? part->size() - 1 : 0;

                    for (; i < part->size(); j = i++)
                    {
                        const vert_t& p0 = (*part)[i];
                        const vert_t& p1 = (*part)[j];

                        // cull segment to tile
                        if ((p0.x() >= xmin || p1.x() >= xmin) &&
                            (p0.x() <= xmax || p1.x() <= xmax) &&
                            (p0.y() >= ymin || p1.y() >= ymin) &&
                            (p0.y() <= ymax || p1.y() <= ymax))
                        {
                            mesh.insert(segment_t(p0, p1), marker);
                        }
                    }
                }
            }

            // Find any triangles that we don't want to draw and 
            // mark them an "unused."
            if (edit._layer->getRemoveExterior() ||
                edit._layer->getRemoveInterior())
            {
                // Iterate without holes because Polygon::contains deals with them
                GeometryIterator mask_iter(
                    feature->getGeometry(),
                    false); // don't iterate into polygon holes

                while (mask_iter.hasMore())
                {
                    Geometry* part = mask_iter.next();
                    if (part->isPolygon())
                    {
                        for (auto& tri_iter : mesh._triangles)
                        {
                            triangle_t& tri = tri_iter.second;
                            vert_t c = (tri.p0 + tri.p1 + tri.p2) * (1.0 / 3.0);

                            bool inside = part->contains2D(c.x(), c.y());

                            if (((inside == true) && edit._layer->getRemoveInterior()) ||
                                ((inside == false) && edit._layer->getRemoveExterior()))
                            {
                                mesh.remove_triangle(tri);
                                //mesh.get_vertex(tri.i0).z() -= 25.0;
                                //mesh.get_vertex(tri.i1).z() -= 25.0;
                                //mesh.get_vertex(tri.i2).z() -= 25.0;
                                // Water: calculate the distance from the polygon for each triangle
                                // vertex and depress it accordingly; then add a surface polygon to match?
                            }

                            // this will remove "sliver" triangles that are coincident with
                            // the boundary, that would otherwise cause skirts to appear 
                            // where there are (apparently) no surface.
                            else if (tri.is_2d_degenerate)
                            {
                                mesh.remove_triangle(tri);
                            }
                        }
                    }
                }

                // if ALL triangles are unused, it's an empty tile.
                if (mesh._triangles.empty())
                {
                    _tileEmpty = true;
                    return false;
                }
            }
        }
    }

    // We have an edited mesh, now turn it back into something OSG can render.
    using Vec3Ptr = osg::ref_ptr<osg::Vec3Array>;
    Vec3Ptr verts = dynamic_cast<osg::Vec3Array*>(sharedGeom->getVertexArray());
    verts->reserve(mesh._verts.size());

    Vec3Ptr normals = dynamic_cast<osg::Vec3Array*>(sharedGeom->getNormalArray());
    normals->reserve(mesh._verts.size());

    Vec3Ptr texCoords = dynamic_cast<osg::Vec3Array*>(sharedGeom->getTexCoordArray());
    texCoords->reserve(mesh._verts.size());

    Vec3Ptr neighbors = dynamic_cast<osg::Vec3Array*>(sharedGeom->getNeighborArray());

    Vec3Ptr neighborNormals = dynamic_cast<osg::Vec3Array*>(sharedGeom->getNeighborNormalArray());

    osg::Vec3d world;
    osg::BoundingSphere tileBound;

    for(auto& vert : mesh._verts)
    {
        int marker = mesh.get_marker(vert);

        osg::Vec3d v(vert.x(), vert.y(), vert.z());
        osg::Vec3d unit;
        verts->push_back(v);
        world = v * local2world;
        locator.worldToUnit(world, unit);
        if (texCoords.valid())
            texCoords->push_back(osg::Vec3f(unit.x(), unit.y(), (float)marker));

        unit.z() += 1.0;
        osg::Vec3d modelPlusOne;
        locator.unitToWorld(unit, modelPlusOne);
        osg::Vec3d normal = (modelPlusOne*world2local) - v;
        normal.normalize();
        normals->push_back(normal);

        if (neighbors)
            neighbors->push_back(v);

        if (neighborNormals)
            neighborNormals->push_back(normal);

        tileBound.expandBy(verts->back());
    }

    // TODO: combine this with the skirt gen for speed
    osg::DrawElements* de = new osg::DrawElementsUShort(GL_TRIANGLES);
    de->reserveElements(mesh._triangles.size() * 3);
    for (const auto& tri : mesh._triangles)
    {
        de->addElement(tri.second.i0);
        de->addElement(tri.second.i1);
        de->addElement(tri.second.i2);
    }
    sharedGeom->setDrawElements(de);

    // make the skirts:
    if (skirtHeightRatio > 0.0)
    {
        double skirtHeight = skirtHeightRatio * tileBound.radius();

        // collect all edges marked as boundaries
        edgeset_t boundary_edges(mesh, VERTEX_BOUNDARY);

        // Add the skirt geometry. We don't share verts with the surface mesh
        // because we need to mark skirts verts so we can conditionally render
        // skirts in the shader.
        int mem = verts->size() + boundary_edges._edges.size() * 4;
        verts->reserve(mem);
        normals->reserve(mem);
        texCoords->reserve(mem);
        if (neighbors) neighbors->reserve(mem);
        if (neighborNormals) neighborNormals->reserve(mem);
        de->reserveElements(de->getNumIndices() + boundary_edges._edges.size() * 6);

        for (auto& edge : boundary_edges._edges)
        {
            addSkirtDataForIndex(edge._i0, skirtHeight);
            addSkirtDataForIndex(edge._i1, skirtHeight);
            addSkirtTriangles(de, verts->size() - 4, verts->size() - 2);
        }
    }

    return true;
}