#include "MeshEditor"
#include "GeometryPool"

#include <osgEarth/Locators>
#include <osgEarth/Map>
#include <osgEarth/Math>
#include <osgEarth/WingedEdgeMesh>
#include <algorithm>
#include <iostream>

#define LC "[MeshEditor] "

using namespace osgEarth;
using namespace osgEarth::REX;

MeshEditor::MeshEditor(const TileKey& key, unsigned tileSize, const Map* map) :
    _key( key ), _tileSize(tileSize), _ndcMin(DBL_MAX, DBL_MAX, DBL_MAX), _ndcMax(-DBL_MAX, -DBL_MAX, -DBL_MAX)
{
    MeshEditLayerVector editLayers;
    map->getLayers(editLayers);

    for(MeshEditLayerVector::const_iterator it = editLayers.begin();
        it != editLayers.end(); 
        ++it)
    {
        MeshEditLayer* layer = it->get();
        if ( layer->getMinLevel() <= key.getLevelOfDetail() )
        {
            addEditGeometry(layer->getOrCreateEditGeometry( 1.0, key, (ProgressCallback*)0L ) );
        }
    }
}

void
MeshEditor::addEditGeometry(MeshEditLayer::EditVector *geometry)
{
    // Make a "locator" for this key so we can do coordinate conversion:
    GeoLocator geoLocator(_key.getExtent());

    if ( geometry )
    {
        osg::ref_ptr<MeshEditLayer::EditVector> ev = new MeshEditLayer::EditVector;
        osg::Vec3d min_ndc, max_ndc;
        for (auto geomString : *geometry)
        {
            // Calculate the axis-aligned bounding box of the boundary polygon:
            osg::BoundingBoxd bbox = polygonBBox2d(*geomString);
            // convert that bounding box to "unit" space (0..1 across the tile)
            geoLocator.mapToUnit(bbox._min, min_ndc);
            geoLocator.mapToUnit(bbox._max, max_ndc);

            // true if boundary overlaps tile in X dimension:
            bool x_match = min_ndc.x() < 1.0 && max_ndc.x() >= 0.0;
            // true if boundary overlaps tile in Y dimension:
            bool y_match = min_ndc.y() < 1.0 && max_ndc.y() >= 0.0;
            if (x_match && y_match)
            {
                // yes, boundary overlaps tile, so expand the global NDC bounding
                // box to include the new mask:
                _ndcMin.x() = std::min(_ndcMin.x(), min_ndc.x());
                _ndcMin.y() = std::min(_ndcMin.y(), min_ndc.y());
                _ndcMax.x() = std::max(_ndcMax.x(), max_ndc.x());
                _ndcMax.y() = std::max(_ndcMax.y(), max_ndc.y());
                // and add this mask to the list.
                ev->push_back(geomString);
            }
        }
        if (!ev->empty())
        {
            _edits.push_back(EditGeometry(ev.get(), min_ndc, max_ndc));
        }
    }
}

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
    // Make the cuts
    for (auto& editGeometry : _edits)
    {
        for ( auto arrayPtr : *editGeometry.geometry)
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
    vert_t(value_type a, value_type b, value_type c) : _x(a), _y(b), _z(c) { }
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

typedef std::map<vert_t, int> vert_table_t;


typedef osg::Vec3dArray vert_array_t_base;

struct vert_array_t : public osg::MixinVector<vert_t>
{
};

struct segment_t : std::pair<vert_t, vert_t>
{
    segment_t(const vert_t& a, const vert_t& b) :
        std::pair<vert_t, vert_t>(a, b) { }

    // 2D cross product
    vert_t::value_type cross2d(const vert_t& a, const vert_t& b) const {
        return a.x()*b.y() - b.x()*a.y();
    }

    // true if 2 segments intersect; intersection point in "out"
    bool intersect(const segment_t& rhs, vert_t& out) const
    {
        vert_t r = second - first;
        vert_t s = rhs.second - rhs.first;
        vert_t::value_type det = cross2d(r, s);

        if (osg::equivalent(det, static_cast<vert_t::value_type>(0.0)))
            return false;

        vert_t::value_type u = cross2d(rhs.first - first, s) / det;
        vert_t::value_type v = cross2d(rhs.first - first, r) / det;

        out = first + (r * u);
        return (u > 0.0 && u < 1.0 && v > 0.0 && v < 1.0);
    }
};

struct tri_t
{
    UID uid;
    vert_t p0, p1, p2; // vertices
    unsigned i0, i1, i2; // indices
    vert_t::value_type a_min[2];
    vert_t::value_type a_max[2];
    vert_t::value_type e01, e12, e20;

    // true id the triangle contains point P
    bool contains2d(const vert_t& P) const 
    {
        vert_t c = p2 - p0;
        vert_t b = p1 - p0;
        vert_t p = P - p0;

        vert_t::value_type cc = c.dot2d(c), bc = b.dot2d(c), pc = c.dot2d(p), bb = b.dot2d(b), pb = b.dot2d(p);
        vert_t::value_type demon = cc * bb - bc * bc;
        if (osg::equivalent(demon, static_cast<vert_t::value_type>(0.0)))
            return false;

        float u = (bb*pc - bc * pb) / demon;
        float v = (cc*pb - bc * pc) / demon;

        return u >= 0 && v >= 0 && (u + v < 1.0);
    }

    // true if point P is one of the triangle's verts
    bool matches_vertex(const vert_t& p) const
    {
        if (osg::equivalent(p.x(), p0.x()) &&
            osg::equivalent(p.y(), p0.y()))
            return true;
        if (osg::equivalent(p.x(), p1.x()) &&
            osg::equivalent(p.y(), p1.y()))
            return true;
        if (osg::equivalent(p.x(), p2.x()) &&
            osg::equivalent(p.y(), p2.y()))
            return true;
        return false;
    }
};
typedef RTree<UID, vert_t::value_type, 2> MySpatialIndex;

struct mesh_t
{
    int uidgen;
    std::unordered_map<UID, tri_t> _triangles;
    MySpatialIndex _spatial_index;
    vert_table_t _vert_lut;
    vert_array_t _verts;

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

    int get_or_create_vertex(const vert_t& input)
    {
        vert_table_t::iterator i = _vert_lut.find(input);
        if (i != _vert_lut.end())
        {
            return i->second;
        }
        else
        {
            _verts.push_back(input);
            _vert_lut[input] = _verts.size() - 1;
            return _verts.size() - 1;
        }
    }

    // insert a segment into the mesh, cutting triangles as necessary
    void insert(const segment_t& seg)
    {
        vert_t::value_type min_edge =
            (_triangles.begin()->second.e01) * 0.25;

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
                !tri.matches_vertex(seg.first) &&
                (seg.first-tri.p0).length() > min_edge &&
                (seg.first-tri.p1).length() > min_edge &&
                (seg.first-tri.p2).length() > min_edge)
            {
                inside_split(tri, seg.first, uid_list);
                continue;
            }

            // is the second segment endpoint inside a triangle?
            if (tri.contains2d(seg.second) && 
                !tri.matches_vertex(seg.second) &&
                (seg.second - tri.p0).length() > min_edge &&
                (seg.second - tri.p1).length() > min_edge &&
                (seg.second - tri.p2).length() > min_edge)
            {
                inside_split(tri, seg.second, uid_list);
                continue;
            }

            // interect the segment with each triangle edge:
            vert_t out;
            UID new_uid;

            // does the segment cross first triangle edge?
            if (tri.e01 > min_edge)
            {
                segment_t edge0(tri.p0, tri.p1);
                if (seg.intersect(edge0, out) && !tri.matches_vertex(out))
                {
                    int new_i = get_or_create_vertex(out);

                    UID uid0 = add_triangle(new_i, tri.i2, tri.i0).uid;
                    uid_list.push_back(uid0);

                    UID uid1 = add_triangle(new_i, tri.i1, tri.i2).uid;
                    uid_list.push_back(uid1);

                    remove_triangle(tri);

                    continue;
                }
            }

            // does the segment cross second triangle edge?
            if (tri.e12 > min_edge)
            {
                segment_t edge1(tri.p1, tri.p2);
                if (seg.intersect(edge1, out) && !tri.matches_vertex(out))
                {
                    int new_i = get_or_create_vertex(out);

                    new_uid = add_triangle(new_i, tri.i0, tri.i1).uid;
                    uid_list.push_back(new_uid);

                    new_uid = add_triangle(new_i, tri.i2, tri.i0).uid;
                    uid_list.push_back(new_uid);

                    remove_triangle(tri);

                    continue;
                }
            }

            // does the segment cross third triangle edge?
            if (tri.e20 > min_edge)
            {
                segment_t edge2(tri.p2, tri.p0);
                if (seg.intersect(edge2, out) && !tri.matches_vertex(out))
                {
                    int new_i = get_or_create_vertex(out);

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

    // inserts point "p" into the interior of triangle "tri",
    // adds three new triangles, and removes the original triangle.
    void inside_split(tri_t& tri, const vert_t& p, std::list<UID>& uid_list)
    {
        //todo check if p already exists?

        int new_i = get_or_create_vertex(p);

        UID uid;

        uid = add_triangle(tri.i0, tri.i1, new_i).uid;
        uid_list.push_back(uid);

        uid = add_triangle(tri.i1, tri.i2, new_i).uid;
        uid_list.push_back(uid);

        uid = add_triangle(tri.i2, tri.i0, new_i).uid;
        uid_list.push_back(uid);

        remove_triangle(tri);
    }
};

bool
MeshEditor::createTileMesh2(SharedGeometry* sharedGeom, unsigned tileSize)
{
    static Mutex m;
    ScopedMutexLock lock(m);

    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    _key.getExtent().getCentroid(centroid);
    centroid.toWorld(centerWorld);
    osg::Matrix world2local, local2world;
    centroid.createWorldToLocal(world2local);
    local2world.invert(world2local);
    bool needsSkirt = false;
    GeoLocator locator(_key.getExtent());
    auto tileSRS = _key.getExtent().getSRS();

    mesh_t mesh;
    mesh._verts.reserve(tileSize*tileSize);

    using RowVec = std::vector<vert_t*>;
    RowVec bottomRow;
    for (unsigned row = 0; row < tileSize; ++row)
    {
        double ny = (double)row / (double)(tileSize - 1);
        RowVec topRow;
        for (unsigned col = 0; col < tileSize; ++col)
        {
            double nx = (double)col / (double)(tileSize - 1);
            osg::Vec3d unit(nx, ny, 0.0);
            osg::Vec3d model;
            osg::Vec3d modelLTP;
            locator.unitToWorld(unit, model);
            modelLTP = model * world2local;
            int i = mesh.get_or_create_vertex(modelLTP);

            if (row > 0 && col > 0)
            {
                //int i = mesh._verts.size() - 1;
                mesh.add_triangle(i, i - 1, i - tileSize - 1);
                mesh.add_triangle(i, i - tileSize - 1, i - tileSize);
            }
        }
        std::swap(topRow, bottomRow);
    }


    for (auto& editGeometry : _edits)
    {
        for (auto arrayPtr : *editGeometry.geometry)
        {
            // Cut in the segments
            if (arrayPtr->empty())
                continue;

            // Get points into tile coordinate system
            std::vector<vert_t> tileLocalPts;

            std::transform(
                arrayPtr->begin(), arrayPtr->end(), std::back_inserter(tileLocalPts),

                [tileSRS, &world2local](const osg::Vec3d& worldPt)
                {
                    osg::Vec3d result;
                    tileSRS->transformToWorld(worldPt, result);
                    return result * world2local;
                });

            for (auto v0Itr = tileLocalPts.begin(), v1Itr = v0Itr + 1;
                v1Itr != tileLocalPts.end();
                v0Itr = v1Itr++)
            {
                mesh.insert(segment_t(*v0Itr, *v1Itr));
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

    for(auto& vert : mesh._verts)
    {
        osg::Vec3d v(vert.x(), vert.y(), vert.z());
        verts->push_back(v);

        osg::Vec3d worldPos = v * local2world;
        osg::Vec3d unit;
        locator.worldToUnit(worldPos, unit);
        float marker = VERTEX_MARKER_GRID;
        if (texCoords.valid())
            texCoords->push_back(osg::Vec3f(unit.x(), unit.y(), marker));
       
        unit.z() += 1.0;
        osg::Vec3d modelPlusOne;
        locator.unitToWorld(unit, modelPlusOne);
        osg::Vec3d normal = (modelPlusOne*world2local) - v;
        normal.normalize();
        normals->push_back(normal);
    }

    osg::DrawElements* de = new osg::DrawElementsUShort(GL_TRIANGLES);
    de->reserveElements(mesh._triangles.size() * 3);
    for (auto& tri : mesh._triangles)
    {
        de->addElement(tri.second.i0);
        de->addElement(tri.second.i1);
        de->addElement(tri.second.i2);
    }
    sharedGeom->setDrawElements(de);

    return true;
}