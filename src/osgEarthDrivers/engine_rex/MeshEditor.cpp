#include "MeshEditor"
#include "GeometryPool"

#include <osgEarth/Locators>
#include <osgEarth/Map>
#include <osgEarth/Math>
#include <osgEarth/TerrainConstraintLayer>
//#include <osgEarth/rtree.h>
#include <osgEarth/weemesh.h>
#include <algorithm>
#include <iostream>

#define LC "[MeshEditor] "

using namespace osgEarth;
using namespace osgEarth::REX;
using namespace weemesh;

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
        if (!layer->isOpen() || !layer->getVisible())
            continue;

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
                layer->getFilters(),
                nullptr,
                progress);

            Edit edit;
            while (cursor.valid() && cursor->hasMore())
            {
                if (progress && progress->isCanceled())
                    return;

                Feature* f = cursor->nextFeature();
                if (f->getExtent().intersects(keyExtent))
                {
                    f->transform(keyExtent.getSRS());
                    edit._features.push_back(f);
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
    double skirtHeightRatio,
    GLenum mode,
    Cancelable* progress)
{
    // uncomment for easier debugging
    //static Mutex m;
    //ScopedMutexLock lock(m);

    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    const GeoExtent& keyExtent = _key.getExtent();
    GeoPoint centroid = keyExtent.getCentroid();
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
    mesh.set_boundary_marker(VERTEX_BOUNDARY);
    mesh.set_constraint_marker(VERTEX_CONSTRAINT);

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
                VERTEX_VISIBLE; // | VERTEX_CONSTRAINT;

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

    if (progress && progress->isCanceled())
        return false;

    // keep it real
    int max_num_triangles = mesh._triangles.size() * 100;

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

        for (auto& feature : edit._features)
        {
            GeometryIterator geom_iter(feature->getGeometry(), true);
            osg::Vec3d world, unit;
            while (geom_iter.hasMore())
            {
                if (mesh._triangles.size() >= max_num_triangles)
                {
                    // just stop it
                    break;
                }

                if (progress && progress->isCanceled())
                    return false;

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
                        const vert_t v((*part)[i].ptr());

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
                    if (part->isRing() && edit._layer->getRemoveInterior())
                    {
                        marker |= VERTEX_BOUNDARY;
                    }

                    // slice and dice the mesh.
                    // iterate over segments in the part, closing the loop if it's an open ring.
                    unsigned i = part->isRing() && part->isOpen() ? 0 : 1;
                    unsigned j = part->isRing() && part->isOpen() ? part->size() - 1 : 0;

                    for (; i < part->size(); j = i++)
                    {
                        const vert_t p0((*part)[i].ptr());
                        const vert_t p1((*part)[j].ptr());

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
            if (edit._layer->getRemoveInterior())
            {
                // Iterate without holes because Polygon::contains deals with them
                GeometryIterator mask_iter(
                    feature->getGeometry(),
                    false); // don't iterate into polygon holes

                while (mask_iter.hasMore())
                {
                    if (progress && progress->isCanceled())
                        return false;

                    Geometry* part = mask_iter.next();
                    if (part->isPolygon())
                    {
                        std::list<triangle_t*> trisToRemove;

                        for (auto& tri_iter : mesh._triangles)
                        {
                            triangle_t& tri = tri_iter.second;
                            vert_t c = (tri.p0 + tri.p1 + tri.p2) * (1.0 / 3.0);

                            bool inside = part->contains2D(c.x(), c.y());

                            if ((inside == true) && edit._layer->getRemoveInterior())
                            {
                                trisToRemove.push_back(&tri);

                                //OPTIONS:
                                // - remove tri entirely
                                // - change clamping u/v of elevation;
                                // - alter elevation offset based on distance from feature;
                                // - duplicate tris to make water surface+bed
                                // ... pluggable behavior ?
                            }

                            // this will remove "sliver" triangles that are coincident with
                            // the boundary, that would otherwise cause skirts to appear 
                            // where there are (apparently) no surface.
                            else if (tri.is_2d_degenerate)
                            {
                                trisToRemove.push_back(&tri);
                            }
                        }

                        for (auto tri : trisToRemove)
                        {
                            mesh.remove_triangle(*tri);
                        }
                    }
                }

                // if ALL triangles are unused, it's an empty tile.
                if (mesh._triangles.empty())
                {
                    _tileEmpty = true;
                    sharedGeom->setHasConstraints(true);
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

    int ptr = 0;
    int original_grid_size = tileSize * tileSize;

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

        // assign "neighbors" (for morphing) to any "orignal grid" vertex 
        // that is NOT marked as a constraint.
        if (ptr < original_grid_size && !(marker & VERTEX_CONSTRAINT))
        {
            int row = (ptr / tileSize);
            int col = (ptr % tileSize);

            if (neighbors)
            {
                neighbors->push_back(
                    (*verts)[verts->size() - getMorphNeighborIndexOffset(col, row, tileSize)]);
            }

            if (neighborNormals)
            {
                neighborNormals->push_back(
                    (*normals)[normals->size() - getMorphNeighborIndexOffset(col, row, tileSize)]);
            }
        }

        else
        {
            // all new indices...just copy.
            if (neighbors)
                neighbors->push_back(v);
            if (neighborNormals)
                neighborNormals->push_back(normal);
        }

        tileBound.expandBy(verts->back());

        ++ptr;
    }

    // TODO: combine this with the skirt gen for speed
    SharedDrawElements* de = new SharedDrawElements(mode);
    de->reserveElements(mesh._triangles.size() * 3);
    for (const auto& tri : mesh._triangles)
    {
        if (!tri.second.is_2d_degenerate)
        {
            de->addElement(tri.second.i0);
            de->addElement(tri.second.i1);
            de->addElement(tri.second.i2);
        }
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
            // bail if we run out of UShort space
            if (verts->size() + 4 > 0xFFFF)
                break;

            addSkirtDataForIndex(edge._i0, skirtHeight);
            addSkirtDataForIndex(edge._i1, skirtHeight);
            addSkirtTriangles(de, verts->size() - 4, verts->size() - 2);
        }
    }

    // Mark the geometry appropriately
    sharedGeom->setHasConstraints(mesh._num_edits > 0);

    return true;
}