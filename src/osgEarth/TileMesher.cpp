/* osgEarth
* Copyright 2008-2014 Pelican Mapping
* MIT License
*/
#include "TileMesher"
#include "Locators"
#include "weemesh.h"

using namespace osgEarth;

TileMesh::TileMesh(const TileMesh& m)
{
    this->operator=(m);
}

TileMesh&
TileMesh::operator=(const TileMesh& m)
{
    localToWorld = m.localToWorld;
    verts = m.verts;
    normals = m.normals;
    uvs = m.uvs;
    vert_neighbors = m.vert_neighbors;
    normal_neighbors = m.normal_neighbors;
    indices = m.indices;
    hasConstraints = m.hasConstraints;
    return *this;
}

TileMesh::TileMesh(TileMesh&& m)
{
    localToWorld = m.localToWorld; m.localToWorld = osg::Matrix::identity();
    verts = m.verts; m.verts = { };
    normals = m.normals; m.normals = { };
    uvs = m.uvs; m.uvs = { };
    vert_neighbors = m.vert_neighbors; m.vert_neighbors = { };
    normal_neighbors = m.normal_neighbors; m.normal_neighbors = { };
    indices = m.indices; m.indices = { };
    hasConstraints = m.hasConstraints;
}

TileMesher::TileMesher()
{
    //nop
}

void
TileMesher::setTerrainOptions(const TerrainOptionsAPI& options)
{
    _options = options;
}

namespace
{
    void addSkirtTriangles(unsigned INDEX0, unsigned INDEX1, osg::DrawElements* primset)
    {
        primset->addElement((INDEX0));
        primset->addElement((INDEX0)+1);
        primset->addElement((INDEX1));
        primset->addElement((INDEX1));
        primset->addElement((INDEX0)+1);
        primset->addElement((INDEX1)+1);
    }

    void addSkirtDataForIndex(unsigned INDEX, float HEIGHT, TileMesh& geom)
    {
        geom.verts->push_back((*geom.verts)[INDEX]);
        geom.normals->push_back((*geom.normals)[INDEX]);
        geom.uvs->push_back((*geom.uvs)[INDEX]);
        geom.uvs->back().z() = (float)((int)geom.uvs->back().z() | VERTEX_SKIRT);
        if (geom.vert_neighbors) geom.vert_neighbors->push_back((*geom.vert_neighbors)[INDEX]);
        if (geom.normal_neighbors) geom.normal_neighbors->push_back((*geom.normal_neighbors)[INDEX]);
        geom.verts->push_back((*geom.verts)[INDEX] - ((*geom.normals)[INDEX]) * (HEIGHT));
        geom.normals->push_back((*geom.normals)[INDEX]);
        geom.uvs->push_back((*geom.uvs)[INDEX]);
        geom.uvs->back().z() = (float)((int)geom.uvs->back().z() | VERTEX_SKIRT);
        if (geom.vert_neighbors) geom.vert_neighbors->push_back((*geom.vert_neighbors)[INDEX] - ((*geom.normals)[INDEX]) * (HEIGHT));
        if (geom.normal_neighbors) geom.normal_neighbors->push_back((*geom.normal_neighbors)[INDEX]);
    }

    int getMorphNeighborIndexOffset(unsigned col, unsigned row, int rowSize)
    {
        if ((col & 0x1) == 1 && (row & 0x1) == 1) return rowSize + 2;
        if ((row & 0x1) == 1) return rowSize + 1;
        if ((col & 0x1) == 1) return 2;
        return 1;
    }
}

osg::DrawElements*
TileMesher::getOrCreateStandardIndices() const
{
    if (!_standardIndices.valid())
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_standardIndices.valid())
        {
            unsigned tileSize = _options.getTileSize();
            float skirtRatio = _options.getHeightFieldSkirtRatio();

            // Attempt to calculate the number of verts in the surface geometry.
            bool needsSkirt = skirtRatio > 0.0f;

            unsigned numVertsInSurface = (tileSize * tileSize);
            unsigned numVertsInSkirt = needsSkirt ? (tileSize - 1) * 2u * 4u : 0;
            unsigned numVerts = numVertsInSurface + numVertsInSkirt;
            unsigned numIndiciesInSurface = (tileSize - 1) * (tileSize - 1) * 6;
            unsigned numIncidesInSkirt = skirtRatio > 0.0f ? (tileSize - 1) * 4 * 6 : 0;

            GLenum mode = _options.getGPUTessellation() == true ? GL_PATCHES : GL_TRIANGLES;

            auto primset = new osg::DrawElementsUInt(mode);
            primset->reserveElements(numIndiciesInSurface + numIncidesInSkirt);

            // add the elements for the surface:
            for (unsigned j = 0; j < tileSize - 1; ++j)
            {
                for (unsigned i = 0; i < tileSize - 1; ++i)
                {
                    int i00 = j * tileSize + i;
                    int i01 = i00 + tileSize;
                    int i10 = i00 + 1;
                    int i11 = i01 + 1;

                    primset->addElement(i01);
                    primset->addElement(i00);
                    primset->addElement(i11);

                    primset->addElement(i00);
                    primset->addElement(i10);
                    primset->addElement(i11);
                }
            }

            if (needsSkirt)
            {
                // add the elements for the skirt:
                int skirtBegin = numVertsInSurface;
                int skirtEnd = skirtBegin + numVertsInSkirt;
                int i;
                for (i = skirtBegin; i < (int)skirtEnd - 3; i += 2)
                {
                    addSkirtTriangles(i, i + 2, primset);
                }
                addSkirtTriangles(i, skirtBegin, primset);
            }

            primset->setElementBufferObject(new osg::ElementBufferObject());

            _standardIndices = primset;
        }
    }

    return _standardIndices.get();
}

TileMesh
TileMesher::createMesh(const TileKey& key, const MeshConstraints& edits, Cancelable* progress) const
{
    if (edits.empty())
    {
        return createMeshStandard(key, progress);
    }
    else
    {
        return createMeshWithConstraints(key, {}, edits, progress);
    }
}

TileMesh
TileMesher::createMesh(const TileKey& key, const TileMesh& mesh, const MeshConstraints& edits, Cancelable* progress) const
{
    if (edits.empty())
    {
        return mesh;
    }
    else
    {
        return createMeshWithConstraints(key, mesh, edits, progress);
    }
}

TileMesh
TileMesher::createMeshStandard(const TileKey& key, Cancelable* progress) const
{
    // Establish a local reference frame for the tile:
    GeoPoint centroid_world = key.getExtent().getCentroid();
    osg::Matrix world2local, local2world;
    centroid_world.createWorldToLocal(world2local);
    local2world.invert(world2local);

    unsigned tileSize = _options.getTileSize();
    float skirtRatio = _options.getHeightFieldSkirtRatio();

    // Attempt to calculate the number of verts in the surface geometry.
    bool needsSkirt = skirtRatio > 0.0f;
    unsigned numVertsInSurface = (tileSize * tileSize);
    unsigned numVertsInSkirt = needsSkirt ? (tileSize - 1) * 2u * 4u : 0;
    unsigned numVerts = numVertsInSurface + numVertsInSkirt;
    unsigned numIndiciesInSurface = (tileSize - 1) * (tileSize - 1) * 6;
    unsigned numIncidesInSkirt = skirtRatio > 0.0f ? (tileSize - 1) * 4 * 6 : 0;

    osg::BoundingSphere tileBound;

    // the geometry:
    TileMesh geom;
    geom.localToWorld = local2world;

    osg::ref_ptr<osg::VertexBufferObject> vbo = new osg::VertexBufferObject();

    // the initial vertex locations:
    geom.verts = new osg::Vec3Array();
    geom.verts->setVertexBufferObject(vbo.get());
    geom.verts->reserve(numVerts);
    geom.verts->setBinding(osg::Array::BIND_PER_VERTEX);

    // the surface normals (i.e. extrusion vectors)
    geom.normals = new osg::Vec3Array();
    geom.normals->setVertexBufferObject(vbo.get());
    geom.normals->reserve(numVerts);
    geom.normals->setBinding(osg::Array::BIND_PER_VERTEX);

    if (_options.getMorphTerrain() == true)
    {
        // neighbor positions (for morphing)
        geom.vert_neighbors = new osg::Vec3Array();
        geom.vert_neighbors->setBinding(osg::Array::BIND_PER_VERTEX);
        geom.vert_neighbors->setVertexBufferObject(vbo.get());
        geom.vert_neighbors->reserve(numVerts);

        geom.normal_neighbors = new osg::Vec3Array();
        geom.normal_neighbors->setBinding(osg::Array::BIND_PER_VERTEX);
        geom.normal_neighbors->setVertexBufferObject(vbo.get());
        geom.normal_neighbors->reserve(numVerts);
    }

    // tex coord is [0..1] across the tile. The 3rd dimension tracks whether the
    // vert is masked: 0=yes, 1=no
    bool populateTexCoords = true;
    geom.uvs = new osg::Vec3Array();
    geom.uvs->setBinding(osg::Array::BIND_PER_VERTEX);
    geom.uvs->setVertexBufferObject(vbo.get());
    geom.uvs->reserve(numVerts);

    osg::Vec3d unit;
    osg::Vec3d model;
    osg::Vec3d modelLTP;
    osg::Vec3d modelPlusOne;
    osg::Vec3d normal;

    GeoLocator locator(key.getExtent());

    for (unsigned row = 0; row < tileSize; ++row)
    {
        float ny = (float)row / (float)(tileSize - 1);
        for (unsigned col = 0; col < tileSize; ++col)
        {
            float nx = (float)col / (float)(tileSize - 1);

            unit.set(nx, ny, 0.0f);
            locator.unitToWorld(unit, model);
            modelLTP = model * world2local;
            geom.verts->push_back(modelLTP);

            tileBound.expandBy(geom.verts->back());

            if (populateTexCoords)
            {
                // Use the Z coord as a type marker
                float marker = VERTEX_VISIBLE;
                geom.uvs->push_back(osg::Vec3f(nx, ny, marker));
            }

            unit.z() = 1.0f;
            locator.unitToWorld(unit, modelPlusOne);
            normal = (modelPlusOne * world2local) - modelLTP;
            normal.normalize();
            geom.normals->push_back(normal);

            // neighbor:
            if (geom.vert_neighbors.valid())
            {
                const osg::Vec3& modelNeighborLTP = (*geom.verts)[geom.verts->size() - getMorphNeighborIndexOffset(col, row, tileSize)];
                geom.vert_neighbors->push_back(modelNeighborLTP);
            }

            if (geom.normal_neighbors.valid())
            {
                const osg::Vec3& modelNeighborNormalLTP = (*geom.normals)[geom.normals->size() - getMorphNeighborIndexOffset(col, row, tileSize)];
                geom.normal_neighbors->push_back(modelNeighborNormalLTP);
            }
        }
    }

    if (needsSkirt)
    {
        // calculate the skirt extrusion height
        double height = tileBound.radius() * skirtRatio;

        // Normal tile skirt first:
        unsigned skirtIndex = geom.verts->size();

        // first, create all the skirt verts, normals, and texcoords.
        for (int c = 0; c < (int)tileSize - 1; ++c)
            addSkirtDataForIndex(c, height, geom); //south

        for (int r = 0; r < (int)tileSize - 1; ++r)
            addSkirtDataForIndex(r * tileSize + (tileSize - 1), height, geom); //east

        for (int c = tileSize - 1; c > 0; --c)
            addSkirtDataForIndex((tileSize - 1) * tileSize + c, height, geom); //north

        for (int r = tileSize - 1; r > 0; --r)
            addSkirtDataForIndex(r * tileSize, height, geom); //west
    }

    //geom.indices = getOrCreateStandardIndices(options);

    return geom;
}

namespace
{
    void build_regular_gridded_mesh(weemesh::mesh_t& mesh, unsigned tileSize, const GeoLocator& locator, const osg::Matrix& world2local)
    {
        mesh.set_boundary_marker(VERTEX_BOUNDARY);
        mesh.set_constraint_marker(VERTEX_CONSTRAINT);
        mesh.set_has_elevation_marker(VERTEX_HAS_ELEVATION);

        mesh.verts.reserve(tileSize * tileSize);

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

                int marker = VERTEX_VISIBLE;

                // mark the perimeter as a boundary (for skirt generation)
                if (row == 0 || row == tileSize - 1 || col == 0 || col == tileSize - 1)
                    marker |= VERTEX_BOUNDARY;

                int i = mesh.get_or_create_vertex(
                    weemesh::vert_t(modelLTP.x(), modelLTP.y(), modelLTP.z()),
                    marker);

                if (row > 0 && col > 0)
                {
                    mesh.add_triangle(i, i - 1, i - tileSize - 1);
                    mesh.add_triangle(i, i - tileSize - 1, i - tileSize);
                }
            }
        }
    }

    void load_mesh(weemesh::mesh_t& mesh, const TileMesh& input)
    {
        mesh.set_boundary_marker(VERTEX_BOUNDARY);
        mesh.set_constraint_marker(VERTEX_CONSTRAINT);
        mesh.set_has_elevation_marker(VERTEX_HAS_ELEVATION);

        mesh.verts.reserve(input.verts->getNumElements());

        for (unsigned i = 0; i < input.indices->getNumIndices(); i += 3)
        {
            auto i1 = input.indices->getElement(i);
            auto i2 = input.indices->getElement(i + 1);
            auto i3 = input.indices->getElement(i + 2);

            int marker1 = (int)(*input.uvs)[i1].z();
            int marker2 = (int)(*input.uvs)[i2].z();
            int marker3 = (int)(*input.uvs)[i3].z();

            auto v1 = mesh.get_or_create_vertex(weemesh::vert_t((*input.verts)[i1].x(), (*input.verts)[i1].y(), (*input.verts)[i1].z()), marker1);
            auto v2 = mesh.get_or_create_vertex(weemesh::vert_t((*input.verts)[i2].x(), (*input.verts)[i2].y(), (*input.verts)[i2].z()), marker2);
            auto v3 = mesh.get_or_create_vertex(weemesh::vert_t((*input.verts)[i3].x(), (*input.verts)[i3].y(), (*input.verts)[i3].z()), marker3);

            mesh.add_triangle(v1, v2, v3);
        }
    }
}

TileMesh
TileMesher::createMeshWithConstraints(
    const TileKey& key,
    const TileMesh& input_mesh,
    const MeshConstraints& edits,
    Cancelable* cancelable) const
{
    auto& keyExtent = key.getExtent();
    auto tileSRS = keyExtent.getSRS();

    // Establish a local reference frame for the tile:
    GeoPoint centroid_world = keyExtent.getCentroid();
    osg::Matrix world2local, local2world;
    centroid_world.createWorldToLocal(world2local);
    local2world.invert(world2local);
    
    GeoLocator locator(keyExtent);

    unsigned tileSize = _options.getTileSize();
    float skirtRatio = _options.getHeightFieldSkirtRatio();

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
    Bounds localBounds(xmin, ymin, -FLT_MAX, xmax, ymax, FLT_MAX);

    TileMesh geom; // final output.
    geom.localToWorld = local2world;

    weemesh::mesh_t mesh;

    // if we have an input mesh, use it. Otherwise, build a regular gridded mesh.
    if (input_mesh.verts.valid())
    {
        load_mesh(mesh, input_mesh);
    }
    else
    {
        build_regular_gridded_mesh(mesh, tileSize, locator, world2local);
    }

    // keep it real
    int max_num_triangles = mesh.triangles.size() * 1024;
    
    bool have_any_removal_requests = false;

    // First transform all our constraint geometry to the local tile system.
    for (auto& edit : edits)
    {
        osg::Vec3d world;
        for (auto& feature : edit.features)
        {
            feature->transform(tileSRS);

            GeometryIterator geom_iter(feature->getGeometry(), true);
            while (geom_iter.hasMore())
            {
                Geometry* part = geom_iter.next();

                // transform the constraint (in place) from world coordinates
                // to tile-local coordinates
                for (auto& point : *part)
                {
                    tileSRS->transformToWorld(point, world);
                    point = world * world2local;
                }
            }
        }
    }    

    // Make the edits
    for (auto& edit : edits)
    {
        if (edit.removeExterior || edit.removeInterior)
        {
            have_any_removal_requests = true;
        }

        // we're marking all new verts CONSTRAINT in order to disable morphing.
        int default_marker = VERTEX_VISIBLE | VERTEX_CONSTRAINT;

        // this will preserve a "burned-in" Z value in the shader.
        if (edit.hasElevation)
        {
            default_marker |= VERTEX_HAS_ELEVATION;
        }

        for (auto& feature : edit.features)
        {
            GeometryIterator geom_iter(feature->getGeometry(), true);
            osg::Vec3d world, unit;
            while (geom_iter.hasMore())
            {
                if (mesh.triangles.size() >= max_num_triangles)
                {
                    // just stop it
                    //OE_WARN << "WARNING, breaking out of the meshing process. Too many tris bro!" << std::endl;
                    break;
                }

                Geometry* part = geom_iter.next();

                if (intersects2d(part->getBounds(), localBounds))
                {
                    if (part->isPointSet())
                    {
                        for (int i = 0; i < part->size(); ++i)
                        {
                            const weemesh::vert_t v((*part)[i].ptr());

                            if (v.x >= xmin && v.x <= xmax && v.y >= ymin && v.y <= ymax)
                            {
                                mesh.insert(v, default_marker);
                            }
                        }
                    }

                    else
                    {
                        // marking as BOUNDARY will allow skirt generation on this part
                        // for polygons with removed interior/exteriors
                        int marker = default_marker;
                        if (part->isRing() && (edit.removeInterior || edit.removeExterior))
                        {
                            marker |= VERTEX_BOUNDARY;
                        }

                        // slice and dice the mesh.
                        // iterate over segments in the part, closing the loop if it's an open ring.
                        unsigned i = part->isRing() && part->isOpen() ? 0 : 1;
                        unsigned j = part->isRing() && part->isOpen() ? part->size() - 1 : 0;

                        for (; i < part->size(); j = i++)
                        {
                            const weemesh::vert_t p0((*part)[i].ptr());
                            const weemesh::vert_t p1((*part)[j].ptr());

                            // cull segment to tile
                            if ((p0.x >= xmin || p1.x >= xmin) &&
                                (p0.x <= xmax || p1.x <= xmax) &&
                                (p0.y >= ymin || p1.y >= ymin) &&
                                (p0.y <= ymax || p1.y <= ymax))
                            {
                                mesh.insert(weemesh::segment_t(p0, p1), marker);
                            }
                        }
                    }
                }

                if (cancelable && cancelable->canceled())
                    return {};
            }
        }
    }

    // Now that meshing is complete, remove interior or exterior triangles
    // if we find any.
    // IDEAS:
    // - remove tri entirely
    // - change clamping u/v of elevation;
    // - alter elevation offset based on distance from feature;
    // - duplicate tris to make water surface+bed
    // ... pluggable behavior ?
    if (have_any_removal_requests)
    {
        std::unordered_set<weemesh::triangle_t*> insiders;
        std::unordered_set<weemesh::triangle_t*> insiders_to_remove;
        std::unordered_set<weemesh::triangle_t*> outsiders_to_possibly_remove;
        weemesh::vert_t centroid;
        const double one_third = 1.0 / 3.0;
        std::vector<weemesh::triangle_t*> tris;

        for (auto& edit : edits)
        {
            if (edit.removeInterior || edit.removeExterior)
            {
                for (auto& feature : edit.features)
                {
                    // skip the polygon holes.
                    GeometryIterator geom_iter(feature->getGeometry(), false);
                    while (geom_iter.hasMore())
                    {
                        Geometry* part = geom_iter.next();

                        // Note: the part was already transformed in a previous step.

                        const auto& bb = part->getBounds();

                        if (part->isPolygon() && intersects2d(bb, localBounds))
                        {
                            if (edit.removeExterior)
                            {
                                // expensive path, much check ALL triangles when removing exterior.
                                for (auto& tri_iter : mesh.triangles)
                                {
                                    weemesh::triangle_t* tri = &tri_iter.second;

                                    bool inside = part->contains2D(tri->centroid.x, tri->centroid.y);

                                    if (inside)
                                    {
                                        insiders.insert(tri);
                                        if (edit.removeInterior)
                                        {
                                            insiders_to_remove.insert(tri);
                                        }
                                    }
                                    else if (edit.removeExterior)
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

                        if (cancelable && cancelable->canceled())
                            return {};
                    }
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

#if 0
        // do we want to add the constraint triangles back in?
        if (!insiders_to_remove.empty())
        {
            std::set<std::tuple<int, int, int>> unique_tris;

            for (auto& edit : edits)
            {
                if (edit.removeInterior)
                {
                    for (auto& feature : edit.features)
                    {
                        // skip the polygon holes.
                        GeometryIterator geom_iter(feature->getGeometry(), false);
                        while (geom_iter.hasMore())
                        {
                            Geometry* part = geom_iter.next();

                            if (localBounds.contains(part->getBounds().center()))
                            {
                                if (part->isPolygon() && part->size() == 3)
                                {
                                    auto i1 = mesh.get_or_create_vertex(weemesh::vert_t((*part)[0].ptr()), VERTEX_CONSTRAINT);
                                    auto i2 = mesh.get_or_create_vertex(weemesh::vert_t((*part)[1].ptr()), VERTEX_CONSTRAINT);
                                    auto i3 = mesh.get_or_create_vertex(weemesh::vert_t((*part)[2].ptr()), VERTEX_CONSTRAINT);

                                    auto unique = std::make_tuple(i1, i2, i3);
                                    if (unique_tris.count(unique) == 0)
                                    {
                                        unique_tris.insert(unique);
                                        mesh.add_triangle(i1, i2, i3);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
#endif
    }

    // if ALL triangles are gone, it's an empty tile.
    if (mesh.triangles.empty())
    {
        geom.hasConstraints = true;
        return geom;
    }

    // Process any fill-elevation requests:
    for (auto& edit : edits)
    {
        if (edit.hasElevation && edit.fillElevations)
        {
            // This algorithm looks for verts with no elevation data, and assigns each
            // one an elevation based on the closest edge that HAS elevation data.
            // Typical use case: remove exterior polygons, leaving you with an interior
            // mesh in which the edge points have set elevations. This will then interpolate
            // the elevations of any new interior points giving you a "flat" surface for
            // a road or river (for example).
            // TODO: do some kind of smoothing for exterior points to make nice transitions...?
            weemesh::vert_t closest;

            // collect every edge that has valid elevation data in Z.
            // usually the means the caller assigned Z values to the constraint geometry
            // and set edit.hasElevation to true.
            weemesh::edgeset_t elevated_edges(mesh, VERTEX_HAS_ELEVATION);

            // find every vertex without elevation, and set its elevation to the same value
            // as that of the closest point on the nearest constrained edge
            for (int i = 0; i < mesh.verts.size(); ++i)
            {
                if ((mesh.markers[i] & VERTEX_HAS_ELEVATION) == 0)
                {
                    if (elevated_edges.point_on_any_edge_closest_to(mesh.verts[i], mesh, closest))
                    {
                        mesh.verts[i].z = closest.z;
                        mesh.markers[i] |= (VERTEX_CONSTRAINT | VERTEX_HAS_ELEVATION);
                    }
                }
            }
        }
    }
    // Time to assemble the resulting TileMesh structure.
   
    // TODO:
    // This geometry/index set is now sparse. Any verts that were
    // orphaned due to triangle removal are still present, just not
    // addressed in the index buffer. This is obviously a huge waste
    // of space and we should compress it down.

    geom.verts = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    geom.verts->reserve(mesh.verts.size());

    geom.normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    geom.normals->reserve(mesh.verts.size());

    geom.uvs = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    geom.uvs->reserve(mesh.verts.size());

    if (_options.getMorphTerrain())
    {
        geom.vert_neighbors = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
        geom.vert_neighbors->reserve(mesh.verts.size());

        geom.normal_neighbors = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
        geom.normal_neighbors->reserve(mesh.verts.size());
    }

    osg::Vec3d world;
    osg::BoundingSphere tileBound;

    int ptr = 0;
    int original_grid_size = tileSize * tileSize;

    // generate UVs and neighbor data:
    for (auto& vert : mesh.verts)
    {
        int marker = mesh.get_marker(vert);

        osg::Vec3d v(vert.x, vert.y, vert.z);
        osg::Vec3d unit;
        geom.verts->push_back(v);
        world = v * local2world;
        locator.worldToUnit(world, unit);
        if (geom.uvs.valid())
            geom.uvs->push_back(osg::Vec3f(unit.x(), unit.y(), (float)marker));

        unit.z() += 1.0;
        osg::Vec3d modelPlusOne;
        locator.unitToWorld(unit, modelPlusOne);
        osg::Vec3d normal = (modelPlusOne * world2local) - v;
        normal.normalize();
        geom.normals->push_back(normal);

        // assign "neighbors" (for morphing) to any "orignal grid" vertex 
        // that is NOT marked as a constraint.
        if (ptr < original_grid_size && !(marker & VERTEX_CONSTRAINT))
        {
            int row = (ptr / tileSize);
            int col = (ptr % tileSize);

            if (geom.vert_neighbors.valid())
            {
                geom.vert_neighbors->push_back(
                    (*geom.verts)[geom.verts->size() - getMorphNeighborIndexOffset(col, row, tileSize)]);
            }

            if (geom.normal_neighbors.valid())
            {
                geom.normal_neighbors->push_back(
                    (*geom.normals)[geom.normals->size() - getMorphNeighborIndexOffset(col, row, tileSize)]);
            }
        }

        else
        {
            // all new indices...just copy.
            if (geom.vert_neighbors)
                geom.vert_neighbors->push_back(v);

            if (geom.normal_neighbors)
                geom.normal_neighbors->push_back(normal);
        }

        tileBound.expandBy(geom.verts->back());

        ++ptr;
    }

    // the index set, discarding any degenerate triangles.
    auto mode = _options.getGPUTessellation() == true ? GL_PATCHES : GL_TRIANGLES;
    geom.indices = new osg::DrawElementsUInt(mode);
    geom.indices->reserveElements(mesh.triangles.size() * 3);
    for (const auto& tri : mesh.triangles)
    {
        if (!tri.second.is_2d_degenerate)
        {
            geom.indices->addElement(tri.second.i0);
            geom.indices->addElement(tri.second.i1);
            geom.indices->addElement(tri.second.i2);
        }
    }

    // make the skirts:
    if (_options.getHeightFieldSkirtRatio() > 0.0)
    {
        double skirtHeight = _options.getHeightFieldSkirtRatio() * tileBound.radius();

        // collect all edges marked as boundaries
        weemesh::edgeset_t boundary_edges(mesh, VERTEX_BOUNDARY);

        // Add the skirt geometry. We don't share verts with the surface mesh
        // because we need to mark skirts verts so we can conditionally render
        // skirts in the shader.
        int mem = geom.verts->size() + boundary_edges._edges.size() * 4;
        geom.verts->reserve(mem);
        geom.normals->reserve(mem);
        geom.uvs->reserve(mem);
        if (geom.vert_neighbors.valid())
            geom.vert_neighbors->reserve(mem);
        if (geom.normal_neighbors.valid())
            geom.normal_neighbors->reserve(mem);
        geom.indices->reserveElements(geom.indices->getNumIndices() + boundary_edges._edges.size() * 6);

        for (auto& edge : boundary_edges._edges)
        {
            // bail if we run out of UShort space
            if (geom.verts->size() + 4 > 0xFFFF)
                break;

            addSkirtDataForIndex(edge._i0, skirtHeight, geom);
            addSkirtDataForIndex(edge._i1, skirtHeight, geom);
            addSkirtTriangles(geom.verts->size() - 4, geom.verts->size() - 2, geom.indices.get());
        }
    }

    // Mark the geometry appropriately
    geom.hasConstraints = (mesh._num_edits > 0);

    // Assign buffer objects
    auto vbo = new osg::VertexBufferObject();
    for (auto& array : { geom.verts, geom.normals, geom.uvs, geom.vert_neighbors, geom.normal_neighbors })
        if (array)
            array->setVertexBufferObject(vbo);

    if (geom.indices)
        geom.indices->setElementBufferObject(new osg::ElementBufferObject());

    return geom;
}
