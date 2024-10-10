#pragma once
#include "rtree.h"
#include <cmath>
#include <climits>
#include <map>
#include <unordered_map>
#include <iterator>
#include <set>
#include <unordered_set>

#define marker_is_set(INDEX, BITS) ((markers[INDEX] & BITS) != 0)
#define marker_not_set(INDEX, BITS) ((markers[INDEX] & BITS) == 0)
#define set_marker(INDEX, BITS) markers[INDEX] |= BITS;

namespace weemesh
{
    // MESHING SDK

    using UID = std::uint32_t;

    constexpr double DEFAULT_EPSILON = 0.00015;

    template<typename T>
    inline bool equivalent(T a, T b, T epsilon)
    {
        double d = b - a;
        return d < static_cast<T>(0.0) ? d >= -epsilon : d <= epsilon;
    }

    template<typename T>
    inline bool less_than(T a, T b, T epsilon)
    {
        return a < b && !equivalent(a, b, epsilon);
    }

    template<typename T>
    inline T clamp(T x, T lo, T hi)
    {
        return x < lo ? lo : x > hi ? hi : x;
    }

    // Adapted from Boost - see boost license
    // https://www.boost.org/users/license.html
    template <typename T>
    inline std::size_t hash_value_unsigned(T val)
    {
        const int size_t_bits = std::numeric_limits<std::size_t>::digits;
        const int length = (std::numeric_limits<T>::digits - 1) / size_t_bits;
        std::size_t seed = 0;
        for (unsigned int i = length * size_t_bits; i > 0; i -= size_t_bits)
            seed ^= (std::size_t)(val >> i) + (seed << 6) + (seed >> 2);
        seed ^= (std::size_t)val + (seed << 6) + (seed >> 2);
        return seed;
    }

    // 2.5D vertex. Holds a Z, but most operations only use X/Y
    struct vert_t
    {
        using value_type = double;
        double x = 0.0, y = 0.0, z = 0.0;
        vert_t() { }
        vert_t(value_type a, value_type b, value_type c) : x(a), y(b), z(c) { }
        vert_t(value_type* ptr) : x(ptr[0]), y(ptr[1]), z(ptr[2]) { }
        vert_t(const vert_t& rhs) : x(rhs.x), y(rhs.y), z(rhs.z) { }
        bool operator < (const vert_t& rhs) const {
            if (x < rhs.x) return true;
            if (x > rhs.x) return false;
            return y < rhs.y;
        }
        const value_type& operator[](int i) const {
            return i == 0 ? x : i == 1 ? y : z;
        }
        value_type& operator[](int i) {
            return i == 0 ? x : i == 1 ? y : z;
        }
        vert_t operator - (const vert_t& rhs) const {
            return vert_t(x - rhs.x, y - rhs.y, z - rhs.z);
        }
        vert_t operator + (const vert_t& rhs) const {
            return vert_t(x + rhs.x, y + rhs.y, z + rhs.z);
        }
        vert_t operator * (value_type a) const {
            return vert_t(x * a, y * a, z * a);
        }
        value_type dot2d(const vert_t& rhs) const {
            return x * rhs.x + y * rhs.y;
        }
        value_type cross2d(const vert_t& rhs) const {
            return x * rhs.y - rhs.x * y;
        }
        vert_t normalize2d() const {
            double len = length2d();
            return vert_t(x / len, y / len, z);
        }
        void set(value_type a, value_type b, value_type c) {
            x = a, y = b, z = c;
        }
        value_type length2d() const {
            return sqrt((x * x) + (y * y));
        }
        value_type length2d_squared() const {
            return (x * x) + (y * y);
        }
        // cast to another vec3 type
        template<typename VEC3, typename TYPE = typename VEC3::value_type>
        inline operator VEC3() const {
            return VEC3((TYPE)x, (TYPE)y, (TYPE)z);
        }
    };

    const vert_t::value_type zero(0.0);

    inline bool same_vert(const vert_t& a, const vert_t& b, vert_t::value_type epsilon)
    {
        return
            equivalent(a.x, b.x, epsilon) &&
            equivalent(a.y, b.y, epsilon);
    }


    // uniquely map vertices to indices
    using vert_table_t = std::map<vert_t, int>;

    // array of vert_t's
    using vert_array_t = std::vector<vert_t>;

    // line segment connecting two verts
    struct segment_t : std::pair<vert_t, vert_t>
    {
        segment_t(const vert_t& a, const vert_t& b) :
            std::pair<vert_t, vert_t>(a, b) { }

        template<class T>
        segment_t(const T& a, const T& b) :
            std::pair<vert_t, vert_t>({ a.x, a.y, a.z }, { b.x, b.y, b.z }) { }

        // true if 2 segments intersect; intersection point in "out"
        bool intersect(const segment_t& rhs, vert_t& out, vert_t::value_type& u) const
        {
            vert_t r = second - first;
            vert_t s = rhs.second - rhs.first;
            vert_t::value_type det = r.cross2d(s);

            if (equivalent(det, zero, 1e-5))
                return false;

            vert_t diff = rhs.first - first;
            u = diff.cross2d(s) / det;
            vert_t::value_type v = diff.cross2d(r) / det;

            out = first + (r * u);
            return (u > 0.0 && u < 1.0 && v > 0.0 && v < 1.0);
        }
    };

    struct triangle_t
    {
        UID uid; // unique id
        vert_t p0, p1, p2; // vertices
        vert_t centroid;
        unsigned i0, i1, i2; // indices
        vert_t::value_type a_min[2]; // bbox min
        vert_t::value_type a_max[2]; // bbox max
        bool is_2d_degenerate;

        // true if the triangle contains point P (in xy) within
        // a certain tolerance.
        inline bool contains_2d(const vert_t& P, vert_t::value_type epsilon) const
        {
#if 0
            vert_t bary;
            if (!get_barycentric(P, bary, epsilon))
                return false;

            return
                clamp(bary[0], 0.0, 1.0) == bary[0] &&
                clamp(bary[1], 0.0, 1.0) == bary[1] &&
                clamp(bary[2], 0.0, 1.0) == bary[2];
#else
            auto AB = p1 - p0, BC = p2 - p1, CA = p0 - p2;
            auto AP = P - p0, BP = P - p1, CP = P - p2;
            auto c1 = AB.cross2d(AP), c2 = BC.cross2d(BP), c3 = CA.cross2d(CP);
            return (c1 >= 0 && c2 >= 0 && c3 >= 0) || (c1 <= 0 && c2 <= 0 && c3 <= 0);
#endif
        }

        // true is index I is in this triangle.
        inline bool is_vertex(int i) const {
            return i == i0 || i == i1 || i == i2;
        }

        // true if point P is one of the triangle's verts
        inline bool is_vertex(const vert_t& p, vert_t::value_type epsilon) const
        {
            if (equivalent(p.x, p0.x, epsilon) &&
                equivalent(p.y, p0.y, epsilon))
                return true;
            if (equivalent(p.x, p1.x, epsilon) &&
                equivalent(p.y, p1.y, epsilon))
                return true;
            if (equivalent(p.x, p2.x, epsilon) &&
                equivalent(p.y, p2.y, epsilon))
                return true;

            return false;
        }

        inline int get_vertex(const vert_t& p, vert_t::value_type epsilon) const
        {
            if (equivalent(p.x, p0.x, epsilon) &&
                equivalent(p.y, p0.y, epsilon))
                return i0;
            if (equivalent(p.x, p1.x, epsilon) &&
                equivalent(p.y, p1.y, epsilon))
                return i1;
            if (equivalent(p.x, p2.x, epsilon) &&
                equivalent(p.y, p2.y, epsilon))
                return i2;

            return -1;
        }

        inline bool get_barycentric(const vert_t& p, vert_t& out, vert_t::value_type epsilon) const
        {
            vert_t v0 = p1 - p0, v1 = p2 - p0, v2 = p - p0;
            vert_t::value_type d00 = v0.dot2d(v0);
            vert_t::value_type d01 = v0.dot2d(v1);
            vert_t::value_type d11 = v1.dot2d(v1);
            vert_t::value_type d20 = v2.dot2d(v0);
            vert_t::value_type d21 = v2.dot2d(v1);
            vert_t::value_type denom = d00 * d11 - d01 * d01;

            // means that one of more of the triangles points are coincident:
            if (equivalent(denom, 0.0, epsilon))
                return false;

            out.y = (d11 * d20 - d01 * d21) / denom;
            out.z = (d00 * d21 - d01 * d20) / denom;
            out.x = 1.0 - out.y - out.z;

            return true;
        }
    };

    using spatial_index_t = RTree<UID, vert_t::value_type, 2>;

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
        int uidgen = 0;
        std::unordered_map<UID, triangle_t> triangles;
        vert_array_t verts;
        std::vector<int> markers;
        vert_t::value_type epsilon = DEFAULT_EPSILON;


        spatial_index_t _spatial_index;
        vert_table_t _vert_lut;
        int _num_edits = 0;
        int _boundary_marker = 1;
        int _constraint_marker = 16;
        int _has_elevation_marker = 4;

        mesh_t()
        {
            //nop
        }

        void set_boundary_marker(int value)
        {
            _boundary_marker = value;
        }

        void set_constraint_marker(int value)
        {
            _constraint_marker = value;
        }

        void set_has_elevation_marker(int value)
        {
            _has_elevation_marker = value;
        }

        // delete triangle from the mesh
        void remove_triangle(triangle_t& tri)
        {
            UID uid = tri.uid;
            _spatial_index.Remove(tri.a_min, tri.a_max, uid);
            triangles.erase(uid);

            ++_num_edits;
        }

        const double one_third = 1.0 / 3.0;

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
            tri.a_min[0] = std::min(tri.p0.x, std::min(tri.p1.x, tri.p2.x));
            tri.a_min[1] = std::min(tri.p0.y, std::min(tri.p1.y, tri.p2.y));
            tri.a_max[0] = std::max(tri.p0.x, std::max(tri.p1.x, tri.p2.x));
            tri.a_max[1] = std::max(tri.p0.y, std::max(tri.p1.y, tri.p2.y));
            tri.centroid = (tri.p0 + tri.p1 + tri.p2) * one_third;

            // "2d_degenerate" means that either a) at least 2 points are coincident, or
            // b) at least two edges are basically coincident (in the XY plane)
            tri.is_2d_degenerate =
                same_vert(tri.p0, tri.p1, epsilon) ||
                same_vert(tri.p1, tri.p2, epsilon) ||
                same_vert(tri.p2, tri.p0, epsilon) ||
                same_vert((tri.p1 - tri.p0).normalize2d(), (tri.p2 - tri.p0).normalize2d(), epsilon) ||
                same_vert((tri.p2 - tri.p1).normalize2d(), (tri.p0 - tri.p1).normalize2d(), epsilon) ||
                same_vert((tri.p0 - tri.p2).normalize2d(), (tri.p1 - tri.p2).normalize2d(), epsilon);

            triangles.emplace(uid, tri);
            _spatial_index.Insert(tri.a_min, tri.a_max, uid);

            ++_num_edits;

            return uid;
        }

        // find a vertex by its index
        const vert_t& get_vertex(unsigned i) const
        {
            return verts[i];
        }

        // find a vertex by its index
        vert_t& get_vertex(unsigned i)
        {
            return verts[i];
        }

        // find the marker for a vertex
        int& get_marker(const vert_t& vert)
        {
            return markers[_vert_lut[vert]];
        }

        // find the marker for a vertex index
        int get_marker(int i)
        {
            return markers[i];
        }

        // Add a new vertex (or lookup a matching one) and return its index.
        // If the vertex already exists, update its marker if necessary.
        int get_or_create_vertex(const vert_t& input, int marker)
        {
            int index;
            vert_table_t::iterator i = _vert_lut.find(input);
            if (i != _vert_lut.end())
            {
                index = i->second;
                markers[i->second] |= marker;
            }
            else if (verts.size() + 1 < 0xFFFF)
            {
                verts.push_back(input);
                markers.push_back(marker);
                _vert_lut[input] = verts.size() - 1;
                index = verts.size() - 1;
            }
            else
            {
                return -1;
            }

            return index;
        }

        // fetch a pointer to each triangle that intersects the bounding box
        unsigned get_triangles(vert_t::value_type xmin, vert_t::value_type ymin, vert_t::value_type xmax, vert_t::value_type ymax,
            std::vector<triangle_t*>& output)
        {
            output.clear();
            vert_t::value_type a_min[2] = { xmin, ymin };
            vert_t::value_type a_max[2] = { xmax, ymax };
            _spatial_index.Search(a_min, a_max, [&](const UID& uid) 
                {
                    output.emplace_back(&triangles[uid]);
                    return RTREE_KEEP_SEARCHING;
                });
            return output.size();
        }

        template<class T>
        int get_or_create_vertex_from_vec3(const T& vec, int marker) {
            return get_or_create_vertex(vert_t(vec.x, vec.y, vec.z), marker);
        }

        // insert a point into the mesh, cutting triangles as necessary
        void insert(const vert_t& vert, int marker)
        {
            // search for possible intersecting triangles (should only be one)
            vert_t::value_type a_min[2];
            vert_t::value_type a_max[2];
            a_min[0] = a_max[0] = vert.x;
            a_min[1] = a_max[1] = vert.y;

            std::vector<UID> uids;

            _spatial_index.Search(a_min, a_max, [&uids](const UID& u)
                {
                    uids.push_back(u);
                    return RTREE_KEEP_SEARCHING;
                });

            for (auto uid : uids)
            {
                triangle_t& tri = triangles[uid];

                if (tri.is_2d_degenerate)
                    continue;

                if (tri.contains_2d(vert, epsilon))
                {
                    inside_split(tri, vert, nullptr, marker);
                    // dont't break -- could split two triangles if on edge.
                    //break;
                }
            }
        }

        // insert a segment into the mesh, cutting triangles as necessary
        void insert(const segment_t& seg, int marker)
        {
            // search for possible intersecting triangles:
            vert_t::value_type a_min[2];
            vert_t::value_type a_max[2];
            a_min[0] = std::min(seg.first.x, seg.second.x);
            a_min[1] = std::min(seg.first.y, seg.second.y);
            a_max[0] = std::max(seg.first.x, seg.second.x);
            a_max[1] = std::max(seg.first.y, seg.second.y);
            std::vector<UID> uids;

            _spatial_index.Search(a_min, a_max, [&uids](const UID& u)
                {
                    uids.push_back(u);
                    return RTREE_KEEP_SEARCHING;
                });

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
                triangle_t& tri = triangles[uid];

                // check whether the triangle contains either endpoint on this segment
                // already. If so, update the markers.
                auto i_first = tri.get_vertex(seg.first, epsilon);
                if (i_first >= 0)
                    markers[i_first] |= marker;

                auto i_second = tri.get_vertex(seg.second, epsilon);
                if (i_second >= 0)
                    markers[i_second] |= marker;

                // if the exact segment already exists, we are done.
                if (i_first >= 0 && i_second >= 0)
                    continue;

                // skip triangles that are "degenerate" in 2D. We will keep them
                // because they may NOT be degenerate in 3D (e.g. steep slopes).
                if (tri.is_2d_degenerate)
                    continue;

                // first see if one of the endpoints falls within a triangle.
                // if so, split the triangle into 2 or 3 new ones, and mark
                // all of its verts as CONSTRAINT so they will not be subject
                // to morphing.
                if (tri.contains_2d(seg.first, epsilon))
                {
                    if (inside_split(tri, seg.first, &uid_list, marker))
                        continue;
                }

                if (tri.contains_2d(seg.second, epsilon))
                {
                    if (inside_split(tri, seg.second, &uid_list, marker))
                        continue;
                }

                // Next try to intersect the segment with a triangle edge.
                // In each case, make sure the intersection point isn't 
                // coincident with an existing vertex (in which case ignore it).
                // If we do split an edge, mark all of its verts as CONSTRAINT
                // so they will not be subject to morphing; furthermore, convery
                // any BOUNDARY markers to the new vert so we can maintain boundaries
                // for skirt generation.
                vert_t out;
                vert_t::value_type u;
                UID new_uid;
                int new_i;

                // does the segment cross first triangle edge?
                segment_t edge0(tri.p0, tri.p1);
                if (seg.intersect(edge0, out, u) && !tri.is_vertex(out, epsilon))
                {
                    int new_marker = marker;
                    if ((markers[tri.i0] & _boundary_marker) && (markers[tri.i1] & _boundary_marker))
                        new_marker |= _boundary_marker;

                    new_i = get_or_create_vertex(out, new_marker);
                    if (new_i < 0)
                        return;

                    int new_tris = 0;

                    new_uid = add_triangle(new_i, tri.i2, tri.i0);
                    if (new_uid >= 0) {
                        markers[tri.i2] |= _constraint_marker;
                        markers[tri.i0] |= _constraint_marker;
                        uid_list.push_back(new_uid);
                        ++new_tris;
                    }

                    new_uid = add_triangle(new_i, tri.i1, tri.i2);
                    if (new_uid >= 0) {
                        markers[tri.i1] |= _constraint_marker;
                        markers[tri.i2] |= _constraint_marker;
                        uid_list.push_back(new_uid);
                        ++new_tris;
                    }

                    if (new_tris > 0)
                    {
                        if (marker_not_set(new_i, _has_elevation_marker) &&
                            marker_is_set(tri.i0, _has_elevation_marker) && marker_is_set(tri.i1, _has_elevation_marker))
                        {
                            auto z0 = get_vertex(tri.i0).z, z1 = get_vertex(tri.i1).z;
                            double& new_z = get_vertex(new_i).z;
                            new_z = std::max(new_z, z0 + u * (z1 - z0));
                            set_marker(new_i, _has_elevation_marker);
                        }

                        remove_triangle(tri);
                        continue;
                    }
                }

                // does the segment cross second triangle edge?
                segment_t edge1(tri.p1, tri.p2);
                if (seg.intersect(edge1, out, u) && !tri.is_vertex(out, epsilon))
                {
                    int new_marker = marker;
                    if ((markers[tri.i1] & _boundary_marker) && (markers[tri.i2] & _boundary_marker))
                        new_marker |= _boundary_marker;

                    new_i = get_or_create_vertex(out, new_marker);
                    if (new_i < 0)
                        return;

                    int new_tris = 0;

                    new_uid = add_triangle(new_i, tri.i0, tri.i1);
                    if (new_uid >= 0) {
                        markers[tri.i0] |= _constraint_marker;
                        markers[tri.i1] |= _constraint_marker;
                        uid_list.push_back(new_uid);
                        ++new_tris;
                    }

                    new_uid = add_triangle(new_i, tri.i2, tri.i0);
                    if (new_uid >= 0) {
                        markers[tri.i2] |= _constraint_marker;
                        markers[tri.i0] |= _constraint_marker;
                        uid_list.push_back(new_uid);
                        ++new_tris;
                    }

                    if (new_tris > 0)
                    {
                        if (marker_not_set(new_i, _has_elevation_marker) &&
                            marker_is_set(tri.i1, _has_elevation_marker) && marker_is_set(tri.i2, _has_elevation_marker))
                        {
                            auto z0 = get_vertex(tri.i1).z, z1 = get_vertex(tri.i2).z;
                            double& new_z = get_vertex(new_i).z;
                            new_z = std::max(new_z, z0 + u * (z1 - z0));
                            set_marker(new_i, _has_elevation_marker);
                        }

                        remove_triangle(tri);
                        continue;
                    }
                }

                // does the segment cross third triangle edge?
                segment_t edge2(tri.p2, tri.p0);
                if (seg.intersect(edge2, out, u) && !tri.is_vertex(out, epsilon))
                {
                    int new_marker = marker;
                    if ((markers[tri.i2] & _boundary_marker) && (markers[tri.i0] & _boundary_marker))
                        new_marker |= _boundary_marker;

                    new_i = get_or_create_vertex(out, new_marker);
                    if (new_i < 0)
                        return;

                    int new_tris = 0;

                    new_uid = add_triangle(new_i, tri.i1, tri.i2);
                    if (new_uid >= 0) {
                        markers[tri.i1] |= _constraint_marker;
                        markers[tri.i2] |= _constraint_marker;
                        uid_list.push_back(new_uid);
                        ++new_tris;
                    }

                    new_uid = add_triangle(new_i, tri.i0, tri.i1);
                    if (new_uid >= 0) {
                        markers[tri.i0] |= _constraint_marker;
                        markers[tri.i1] |= _constraint_marker;
                        uid_list.push_back(new_uid);
                        ++new_tris;
                    }

                    if (new_tris > 0)
                    {
                        if (marker_not_set(new_i, _has_elevation_marker) &&
                            marker_is_set(tri.i2, _has_elevation_marker) && marker_is_set(tri.i0, _has_elevation_marker))
                        {
                            auto z0 = get_vertex(tri.i2).z, z1 = get_vertex(tri.i0).z;
                            double& new_z = get_vertex(new_i).z;
                            new_z = std::max(new_z, z0 + u * (z1 - z0));
                            set_marker(new_i, _has_elevation_marker);
                        }

                        remove_triangle(tri);
                        continue;
                    }
                }
            }
        }

        // inserts point "p" into the interior of triangle "tri",
        // adds three new triangles, and removes the original triangle.
        // return true if a split actual happened
        bool inside_split(triangle_t& tri, const vert_t& p, std::list<UID>* uid_list, int new_marker)
        {
            int new_i = get_or_create_vertex(p, new_marker);
            if (new_i < 0)
                return false;

            // if this vertex is already one of the triangle's verts, no split is necessary
            // and we are done.
            if (tri.is_vertex(new_i))
                return false;

            UID new_uid;
            int new_tris = 0;

            // calculate the barycentric coordinates of the new point
            // within the target triangle so we can detect any split
            // that occurs right on an edge and then bypass creating
            // a degenerate triangle.
            vert_t bary(1, 1, 1);
            if (tri.get_barycentric(p, bary, epsilon) == false)
                return false;

            if (!equivalent(bary[2], 0.0, epsilon)) {
                new_uid = add_triangle(tri.i0, tri.i1, new_i);
                if (new_uid >= 0 && uid_list) {
                    markers[tri.i0] |= _constraint_marker;
                    markers[tri.i1] |= _constraint_marker;
                    uid_list->push_back(new_uid);
                    ++new_tris;
                }
            }

            if (!equivalent(bary[0], 0.0, epsilon)) {
                new_uid = add_triangle(tri.i1, tri.i2, new_i);
                if (new_uid >= 0 && uid_list) {
                    markers[tri.i1] |= _constraint_marker;
                    markers[tri.i2] |= _constraint_marker;
                    uid_list->push_back(new_uid);
                    ++new_tris;
                }
            }

            if (!equivalent(bary[1], 0.0, epsilon)) {
                new_uid = add_triangle(tri.i2, tri.i0, new_i);
                if (new_uid >= 0 && uid_list) {
                    markers[tri.i2] |= _constraint_marker;
                    markers[tri.i0] |= _constraint_marker;
                    uid_list->push_back(new_uid);
                    ++new_tris;
                }
            }

            if (new_tris > 0)
            {
                // if all verts in the surrounding triangle have elevation data,
                // but this new one doesn't, set it via interpolation.
                if (marker_not_set(new_i, _has_elevation_marker) &&
                    marker_is_set(tri.i0, _has_elevation_marker) &&
                    marker_is_set(tri.i1, _has_elevation_marker) &&
                    marker_is_set(tri.i2, _has_elevation_marker))
                {
                    // interpolate the Z value from the triangle:
                    get_vertex(new_i).z =
                        get_vertex(tri.i0).z * bary[0] +
                        get_vertex(tri.i1).z * bary[1] +
                        get_vertex(tri.i2).z * bary[2];

                    set_marker(new_i, _has_elevation_marker);
                }

                remove_triangle(tri);
            }

            return (new_tris > 0);
        }

        vert_t point_on_edge_closest_to(const edge_t& edge, const vert_t& p) const {
            const vert_t& e1 = get_vertex(edge._i0);
            const vert_t& e2 = get_vertex(edge._i1);
            vert_t qp = e2 - e1;
            vert_t xp = p - e1;
            double u = xp.dot2d(qp) / qp.dot2d(qp);
            if (u < 0.0) return e1;
            else if (u > 1.0) return e2;
            else return e1 + qp * u;
        }
    };

    // a graph node
    struct node_t
    {
        int _vertex_index; // vertex index
        int _graphid;
        std::set<node_t*> _edges;
        node_t(int vi = 0) : _vertex_index(vi), _graphid(-1) { }
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
            for (auto& tri_iter : mesh.triangles)
            {
                const triangle_t& tri = tri_iter.second;
                add_triangle(tri, mesh, marker_mask);
            }
        }

        void add_triangle(const triangle_t& tri, const mesh_t& mesh, int marker_mask)
        {
            bool m0 = (mesh.markers[tri.i0] & marker_mask) != 0;
            bool m1 = (mesh.markers[tri.i1] & marker_mask) != 0;
            bool m2 = (mesh.markers[tri.i2] & marker_mask) != 0;

            if (m0 && m1)
                _edges.emplace(tri.i0, tri.i1);
            if (m1 && m2)
                _edges.emplace(tri.i1, tri.i2);
            if (m2 && m0)
                _edges.emplace(tri.i2, tri.i0);
        }

        bool point_on_any_edge_closest_to(const vert_t& p, const mesh_t& mesh, vert_t& closest) const
        {
            double min_distance2 = DBL_MAX;

            for (auto& edge : _edges)
            {
                vert_t c = mesh.point_on_edge_closest_to(edge, p);
                auto d2 = (c - p).length2d_squared();
                if (d2 < min_distance2)
                {
                    min_distance2 = d2;
                    closest = c;
                }
            }
            return min_distance2 < DBL_MAX;
        }
    };

    // each node paires with a vector of its neighbors, i.e. other
    // nodes with which is shares an edge.
    struct neighbors_t
    {
        edgeset_t _edgeset;
        std::unordered_map<int, std::vector<int>> _neighbormap;

        neighbors_t(const mesh_t& mesh) :
            _edgeset(mesh, ~0)
        {
            for (auto& edge : _edgeset._edges)
            {
                _neighbormap[edge._i0].push_back(edge._i1);
                _neighbormap[edge._i1].push_back(edge._i0);
            }
        }

        const std::vector<int>& operator()(int vertex_index)
        {
            return _neighbormap[vertex_index];
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
            for (auto& tri_iter : mesh.triangles)
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
                    if (start == nullptr || vert.y < min_y)
                    {
                        start = &node;
                        min_y = vert.y;
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
                    double dot = 1.0 - (0.5 * (invec.dot2d(outvec) + 1.0));
                    double score = turn * dot;

                    if (score < best_score)
                    {
                        best_score = score;
                        best_edge = edge;
                    }
                }

                if (best_edge == nullptr)
                {
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
