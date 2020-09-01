#include "MeshEditor"
#include "GeometryPool"

#include <osgEarth/Locators>
#include <osgEarth/Map>
#include <osgEarth/Math>
#include <osgEarth/WingedEdgeMesh>
#include <osgEarth/FeatureMeshEditLayer>
#include <algorithm>
#include <iostream>

#define LC "[MeshEditor] "

using namespace osgEarth;
using namespace osgEarth::REX;

MeshEditor::MeshEditor(const TileKey& key, unsigned tileSize, const Map* map) :
    _key( key ), 
    _tileSize(tileSize)
{
    std::vector<osg::ref_ptr<FeatureMeshEditLayer>> layers;
    map->getLayers(layers);

    for(auto& layer : layers)
    {
        if ( layer->getMinLevel() <= key.getLevelOfDetail() )
        {
            FeatureSource* fs = layer->getFeatureSource();
            if (fs)
            {
                osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(
                    key,
                    nullptr);

                while (cursor.valid() && cursor->hasMore())
                {
                    Edit edit;
                    cursor->fill(edit._features);

                    if (edit._features.empty() == false)
                    {
                        edit._layer = layer;
                        _edits.emplace_back(edit);
                    }
                }
            }
        }
    }
}

#if 0
struct TileVertex
{
    mutable bool isBorder = false;
    mutable int meshIndex = -1;
};

using TileMesh = Util::WingedEdgeMesh<osg::Vec3d, TileVertex>;

bool
MeshEditor::createTileMesh(SharedGeometry* sharedGeom, unsigned tileSize)
{
    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    _key.getExtent().getCentroid( centroid );
    centroid.toWorld( centerWorld );
    osg::Matrix world2local, local2world;
    centroid.createWorldToLocal( world2local );
    local2world.invert( world2local );
    // Attempt to calculate the number of verts in the surface geometry.
    bool needsSkirt = false; // _options.heightFieldSkirtRatio() > 0.0f;
    GeoLocator locator(_key.getExtent());
    auto tileSRS = _key.getExtent().getSRS();

    // Add one row at a time to the mesh. We will make triangles from
    // two rows as we go along.

    TileMesh wmesh;
    using RowVec = std::vector<const TileMesh::Vertex*>;
    RowVec bottomRow;
    for(unsigned row=0; row<tileSize; ++row)
    {
        float ny = (float)row/(float)(tileSize-1);
        RowVec topRow;
        for(unsigned col=0; col<tileSize; ++col)
        {
            float nx = (float)col/(float)(tileSize-1);
            osg::Vec3d unit(nx, ny, 0.0f);
            osg::Vec3d model;
            osg::Vec3d modelLTP;
            locator.unitToWorld(unit, model);
            modelLTP = model*world2local;
            const TileMesh::Vertex* v = wmesh.getVertex(modelLTP);
            v->isBorder = (row == 0 || row == tileSize - 1
                           || col == 0 || col == tileSize -1);
            topRow.push_back(v);
            // The mesh triangles
            if (row > 0 && col > 0)
            {
                const TileMesh::Vertex* t0[3] = {topRow[col - 1], bottomRow[col - 1], bottomRow[col]};
                const TileMesh::Vertex* t1[3] = {topRow[col - 1], bottomRow[col], topRow[col]};
                wmesh.addFace(&t0[0], &t0[3]);
                wmesh.addFace(&t1[0], &t1[3]);
            }
        }
        std::swap(topRow, bottomRow);
    }

    for (auto& editGeometry : _edits)
    {
        for ( auto arrayPtr : *editGeometry._)
        {
            // Cut in the segments
            if (arrayPtr->empty())
                continue;
            // Get points into tile coordinate system
            std::vector<osg::Vec3d> tileLocalPts;
            std::transform(arrayPtr->begin(), arrayPtr->end(), std::back_inserter(tileLocalPts),
                           [tileSRS,&world2local](const osg::Vec3d& worldPt)
                           {
                               osg::Vec3d result;
                               tileSRS->transformToWorld(worldPt, result);
                               return result * world2local;
                           });
            for (auto v0Itr = tileLocalPts.begin(), v1Itr = v0Itr + 1;
                 v1Itr != tileLocalPts.end();
                 v0Itr = v1Itr++)
            {
                Segment2d segment(*v0Itr, *v1Itr);
                wmesh.cutSegment(segment);
            }
        }
    }

    // We have an edited mesh, now turn it back into something OSG can
    // render.
    int vertexIndex = 0;
    using Vec3Ptr = osg::ref_ptr<osg::Vec3Array>;
    Vec3Ptr verts = dynamic_cast<osg::Vec3Array*>(sharedGeom->getVertexArray());
    Vec3Ptr normals = dynamic_cast<osg::Vec3Array*>(sharedGeom->getNormalArray());
    Vec3Ptr texCoords = dynamic_cast<osg::Vec3Array*>(sharedGeom->getTexCoordArray());
    for (auto& meshVertex : wmesh.vertices)
    {
        if (meshVertex.second.edges.empty())
            continue;
        meshVertex.second.meshIndex = vertexIndex++;
        verts->push_back(meshVertex.second.position); // convert to Vec3
        // Back to tile unit coords
        osg::Vec3d worldPos = meshVertex.second.position * local2world;
        osg::Vec3d unit;
        locator.worldToUnit(worldPos, unit);
        if (texCoords.valid())
        {
            texCoords->push_back(osg::Vec3f(unit.x(), unit.y(), VERTEX_MARKER_GRID));
        }
        unit.z() += 1.0f;
        osg::Vec3d modelPlusOne;
        locator.unitToWorld(unit, modelPlusOne);
        osg::Vec3d normal = (modelPlusOne*world2local) - meshVertex.second.position;
        normal.normalize();
        normals->push_back(normal);
        // Neighbors for morphing... or something else?
        // XXX skirts
    }
    
    osg::ref_ptr<osg::DrawElements> primSet(new osg::DrawElementsUShort(GL_TRIANGLES));
    primSet->reserveElements(wmesh.faces.size() * 3);
    for (auto& face : wmesh.faces)
    {
        if (!face.edge)
        {
            continue;
        }
        auto faceVerts = wmesh.getFaceVertices(&face);
        if (faceVerts.size() != 3)
        {
            OE_NOTICE << "face with " << faceVerts.size() << " vertices\n";
        }
        else
        {
            for (auto vertPtr : faceVerts)
            {
                primSet->addElement(vertPtr->meshIndex);
            }
        }
    }
    sharedGeom->setDrawElements(primSet.get());
    return true;
}

extern "C" void pfvs(void* vMesh, void* vFace)
{
    TileMesh* mesh = static_cast<TileMesh*>(vMesh);
    TileMesh::Face* face = static_cast<TileMesh::Face*>(vFace);
    auto faceVerts = mesh->getFaceVertices(face);
    for (auto vert: faceVerts)
    {
        std::cout << std::hex << vert << ": " << vert->position.x() << " " << vert->position.y() << '\n';
    }
}
#endif

namespace
{
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
        return osg::equivalent((float)a, (float)b);
    }


    typedef std::map<vert_t, int> vert_table_t;

    struct vert_array_t : public osg::MixinVector<vert_t>
    {
    };

    struct segment_t : std::pair<vert_t, vert_t>
    {
        segment_t(const vert_t& a, const vert_t& b) :
            std::pair<vert_t, vert_t>(a, b) { }

        // 2D cross product
        vert_t::value_type cross2d(const vert_t& a, const vert_t& b) const
        {
            return a.x()*b.y() - b.x()*a.y();
        }

        // true if 2 segments intersect; intersection point in "out"
        bool intersect(const segment_t& rhs, vert_t& out) const
        {
            vert_t r = second - first;
            vert_t s = rhs.second - rhs.first;
            vert_t::value_type det = cross2d(r, s);

            if (equivalent(det, zero))
                return false;

            vert_t::value_type u = cross2d(rhs.first - first, s) / det;
            vert_t::value_type v = cross2d(rhs.first - first, r) / det;

            out = first + (r * u);
            return (u > 0.0 && u < 1.0 && v > 0.0 && v < 1.0);
        }
    };

    struct tri_t
    {
        UID uid; // unique id
        vert_t p0, p1, p2; // vertices
        unsigned i0, i1, i2; // indices
        vert_t::value_type e01, e12, e20; // edge lengths
        vert_t::value_type a_min[2]; // bbox min
        vert_t::value_type a_max[2]; // bbox max
        bool _draw;

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
    typedef RTree<UID, vert_t::value_type, 2> tri_lut_t;

    struct mesh_t
    {
        int uidgen;
        std::unordered_map<UID, tri_t> _triangles;
        tri_lut_t _spatial_index;
        vert_table_t _vert_lut;
        vert_array_t _verts;
        std::vector<int> _markers;

        mesh_t() : uidgen(0) {
        }

        // delete triangle from the mesh
        void remove_triangle(tri_t& tri)
        {
            UID uid = tri.uid;
            _spatial_index.Remove(tri.a_min, tri.a_max, uid);
            _triangles.erase(uid);
        }

        // add new triangle to the mesh from 3 indices
        tri_t& add_triangle(int i0, int i1, int i2)
        {
            UID uid(uidgen++);
            tri_t& tri = _triangles[uid];
            tri.uid = uid;
            tri._draw = true;
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

            _spatial_index.Insert(tri.a_min, tri.a_max, tri.uid);
            return tri;
        }

        const vert_t& get_vertex(unsigned i) const
        {
            return _verts[i];
        }

        int& get_marker(const vert_t& vert)
        {
            return _markers[_vert_lut[vert]];
        }

        int get_or_create_vertex(const vert_t& input, int marker)
        {
            vert_table_t::iterator i = _vert_lut.find(input);
            if (i != _vert_lut.end())
            {
                int index = i->second;
                _markers[i->second] = marker;
                return index;
            }
            else
            {
                _verts.push_back(input);
                _markers.push_back(marker);
                _vert_lut[input] = _verts.size() - 1;
                return _verts.size() - 1;
            }
        }

        // insert a segment into the mesh, cutting triangles as necessary
        void insert(const segment_t& seg, int marker)
        {
            // restrict edge length to a fraction of the original edge lengths
            // (hueristic value)
            vert_t::value_type min_edge =
                (_triangles.begin()->second.e01) * 0.1; // 25;

            vert_t::value_type a_min[2];
            vert_t::value_type a_max[2];
            a_min[0] = std::min(seg.first.x(), seg.second.x());
            a_min[1] = std::min(seg.first.y(), seg.second.y());
            a_max[0] = std::max(seg.first.x(), seg.second.x());
            a_max[1] = std::max(seg.first.y(), seg.second.y());
            std::vector<UID> uids;
            _spatial_index.Search(a_min, a_max, &uids, ~0);
            std::list<UID> uid_list;
            std::copy(uids.begin(), uids.end(), std::back_inserter(uid_list));
            for (auto uid : uid_list)
            {
                tri_t& tri = _triangles[uid];

                // is the first segment endpoint inside a triangle?
                if (tri.contains2d(seg.first) &&
                    tri.dist_to_nearest_vertex(seg.first) > min_edge)
                {
                    inside_split(tri, seg.first, uid_list, marker);
                    continue;
                }

                // is the second segment endpoint inside a triangle?
                if (tri.contains2d(seg.second) &&
                    tri.dist_to_nearest_vertex(seg.second) > min_edge)
                {
                    inside_split(tri, seg.second, uid_list, marker);
                    continue;
                }

                // interect the segment with each triangle edge:
                vert_t out;
                UID new_uid;
                int new_i;

                // does the segment cross first triangle edge?
                if (tri.e01 > min_edge)
                {
                    segment_t edge0(tri.p0, tri.p1);
                    if (seg.intersect(edge0, out))
                    {
                        new_i = get_or_create_vertex(out, marker);

                        if (!tri.is_vertex(out))
                        {
                            new_uid = add_triangle(new_i, tri.i2, tri.i0).uid;
                            uid_list.push_back(new_uid);

                            new_uid = add_triangle(new_i, tri.i1, tri.i2).uid;
                            uid_list.push_back(new_uid);

                            remove_triangle(tri);
                            continue;
                        }
                    }
                }

                // does the segment cross second triangle edge?
                if (tri.e12 > min_edge)
                {
                    segment_t edge1(tri.p1, tri.p2);
                    if (seg.intersect(edge1, out))
                    {
                        new_i = get_or_create_vertex(out, marker);

                        if (!tri.is_vertex(out))
                        {
                            new_uid = add_triangle(new_i, tri.i0, tri.i1).uid;
                            uid_list.push_back(new_uid);

                            new_uid = add_triangle(new_i, tri.i2, tri.i0).uid;
                            uid_list.push_back(new_uid);

                            remove_triangle(tri);
                            continue;
                        }
                    }
                }

                // does the segment cross third triangle edge?
                if (tri.e20 > min_edge)
                {
                    segment_t edge2(tri.p2, tri.p0);
                    if (seg.intersect(edge2, out))
                    {
                        new_i = get_or_create_vertex(out, marker);

                        if (!tri.is_vertex(out))
                        {
                            new_uid = add_triangle(new_i, tri.i1, tri.i2).uid;
                            uid_list.push_back(new_uid);

                            new_uid = add_triangle(new_i, tri.i0, tri.i1).uid;
                            uid_list.push_back(new_uid);

                            remove_triangle(tri);
                            continue;
                        }
                    }
                }
            }
        }

        // inserts point "p" into the interior of triangle "tri",
        // adds three new triangles, and removes the original triangle.
        void inside_split(tri_t& tri, const vert_t& p, std::list<UID>& uid_list, int new_marker)
        {
            int new_i = get_or_create_vertex(p, new_marker);

            UID new_uid;

            new_uid = add_triangle(tri.i0, tri.i1, new_i).uid;
            uid_list.push_back(new_uid);

            new_uid = add_triangle(tri.i1, tri.i2, new_i).uid;
            uid_list.push_back(new_uid);

            new_uid = add_triangle(tri.i2, tri.i0, new_i).uid;
            uid_list.push_back(new_uid);

            remove_triangle(tri);
        }
    };

    struct edge_t
    {
        int _i0, _i1; // indices of node endpoints
        edge_t() : _i0(0), _i1(0) { }
        edge_t(int i0, int i1) : _i0(i0), _i1(i1) { }
        bool operator == (const edge_t& rhs) const {
            return _i0 == rhs._i0 && _i1 == rhs._i1;
        }
        bool operator < (const edge_t& rhs) const {
            if (_i0 < rhs._i0) return true;
            if (_i0 > rhs._i0) return false;
            return (_i1 < rhs._i1);
        }
    };

    struct node_t
    {
        int _vi; // vertex index
        std::vector<std::set<edge_t>::iterator> _edges; // edges terminating at this node
        node_t(int vi=0) : _vi(vi) { }
        bool operator == (const node_t& rhs) const {
            return _vi == rhs._vi;
        }
    };

    struct graph_t
    {
        std::unordered_map<int, node_t> _nodes;
        std::set<edge_t> _edges;
    };

    struct noder
    {
        noder(const mesh_t& mesh)
        {
            graph_t graph;
            for (auto& tri_iter : mesh._triangles)
            {
                const tri_t& tri = tri_iter.second;

                auto& node0 = graph._nodes[tri.i0];
                node0._vi = tri.i0;
                auto& node1 = graph._nodes[tri.i1];
                node1._vi = tri.i1;
                auto& node2 = graph._nodes[tri.i2];
                node2._vi = tri.i2;
                
                auto& edge01 = graph._edges.emplace(edge_t(tri.i0, tri.i1)).first;
                auto& edge12 = graph._edges.emplace(edge_t(tri.i1, tri.i2)).first;
                auto& edge20 = graph._edges.emplace(edge_t(tri.i2, tri.i0)).first;

                node0._edges.emplace_back(edge01);
                node0._edges.emplace_back(edge20);
                node1._edges.emplace_back(edge12);
                node1._edges.emplace_back(edge01);
                node2._edges.emplace_back(edge20);
                node2._edges.emplace_back(edge12);
            }
        }
    };
}

//#define USE_UNIT_SPACE

bool
MeshEditor::createTileMesh2(SharedGeometry* sharedGeom, unsigned tileSize)
{
    //static Mutex m;
    //ScopedMutexLock lock(m);

    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    _key.getExtent().getCentroid(centroid);
    centroid.toWorld(centerWorld);
    osg::Matrix world2local, local2world;
    centroid.createWorldToLocal(world2local);
    local2world.invert(world2local);
    GeoLocator locator(_key.getExtent());
    auto tileSRS = _key.getExtent().getSRS();

    mesh_t mesh;
    mesh._verts.reserve(tileSize*tileSize);

    double xmin, ymin, xmax, ymax;

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

#ifdef USE_UNIT_SPACE
            modelLTP.set(unit.x(), unit.y(), modelLTP.z());
#endif
            int i = mesh.get_or_create_vertex(modelLTP, VERTEX_NORMAL);

            if (row > 0 && col > 0)
            {
                mesh.add_triangle(i, i - 1, i - tileSize - 1);
                mesh.add_triangle(i, i - tileSize - 1, i - tileSize);
            }

            if (row == 0 && col == 0)
                xmin = modelLTP.x(), ymin = modelLTP.y();
            else if (row == tileSize - 1 && col == tileSize - 1)
                xmax = modelLTP.x(), ymax = modelLTP.y();
        }
    }

    unsigned num_segments = 0;

    // Make the edits
    for (auto& edit : _edits)
    {
        int marker =
            edit._layer->getHasElevation() ? VERTEX_CONSTRAINT :
            VERTEX_NORMAL;

        for (auto& f : edit._features)
        {
            // first transform it to the local SRS:
            osg::ref_ptr<Feature> feature = new Feature(*f, osg::CopyOp::DEEP_COPY_ALL);
            feature->transform(tileSRS);

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

                // make sure the part is closed so we get all segments
                part->close();

                for (auto v0Itr = part->begin(), v1Itr = v0Itr + 1;
                    v1Itr != part->end();
                    v0Itr = v1Itr++)
                {
                    const vert_t& p0 = *v0Itr;
                    const vert_t& p1 = *v1Itr;

                    // cull to tile
                    if ((p0.x() >= xmin || p1.x() >= xmin) &&
                        (p0.x() <= xmax || p1.x() <= xmax) &&
                        (p0.y() >= ymin || p1.y() >= ymin) &&
                        (p0.y() <= ymax || p1.y() <= ymax))
                    {
                        mesh.insert(segment_t(p0, p1), marker);
                        num_segments++;
                    }
                }
            }

            // iterate without holes for tri-in-polygon test:
            if (num_segments > 0 && (
                    edit._layer->getRemoveInterior() ||
                    edit._layer->getRemoveExterior()))
            {
                ConstGeometryIterator mask_iter(feature->getGeometry(), false);
                while (mask_iter.hasMore())
                {
                    const Geometry* part = mask_iter.next();
                    //if (part->isPolygon())
                    {
                        for (auto& tri_iter : mesh._triangles)
                        {
                            tri_t& tri = tri_iter.second;
                            vert_t c = (tri.p0 + tri.p1 + tri.p2) * (1.0 / 3.0);

                            bool inside = part->contains2D(c.x(), c.y());

                            if (
                                ((inside==true) && edit._layer->getRemoveInterior()) ||
                                ((inside==false) && edit._layer->getRemoveExterior()))
                            {
                                tri._draw = false;
                            }
                        }
                    }
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

    osg::Vec3d world;

    for(auto& vert : mesh._verts)
    {
        int marker = mesh.get_marker(vert);

        osg::Vec3d v(vert.x(), vert.y(), vert.z());
        osg::Vec3d unit;

#ifdef USE_UNIT_SPACE
        unit.set(vert.x(), vert.y(), 0.0);
        locator.unitToWorld(unit, world);
        v = world * world2local;
        verts->push_back(v);

        if (texCoords.valid())
            texCoords->push_back(osg::Vec3f(unit.x(), unit.y(), (float)marker));

        unit.z() = v.z();

#else
        verts->push_back(v);
        world = v * local2world;
        locator.worldToUnit(world, unit);
        if (texCoords.valid())
            texCoords->push_back(osg::Vec3f(unit.x(), unit.y(), (float)marker));
#endif


        unit.z() += 1.0;
        osg::Vec3d modelPlusOne;
        locator.unitToWorld(unit, modelPlusOne);
        osg::Vec3d normal = (modelPlusOne*world2local) - v;
        normal.normalize();
        normals->push_back(normal);
    }

    osg::DrawElements* de = new osg::DrawElementsUShort(GL_TRIANGLES);
    de->reserveElements(mesh._triangles.size() * 3);
    for (const auto& tri : mesh._triangles)
    {
        if (tri.second._draw)
        {
            de->addElement(tri.second.i0);
            de->addElement(tri.second.i1);
            de->addElement(tri.second.i2);
        }
    }
    sharedGeom->setDrawElements(de);

    return true;
}