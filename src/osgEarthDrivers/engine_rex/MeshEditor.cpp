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
        for (auto geomString : *geometry)
        {
            // Calculate the axis-aligned bounding box of the boundary polygon:
            osg::BoundingBoxd bbox = polygonBBox2d(*geomString);
            // convert that bounding box to "unit" space (0..1 across the tile)
            osg::Vec3d min_ndc, max_ndc;
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
                _edits.push_back(EditGeometry(geometry, min_ndc, max_ndc));
            }
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
        unit.z() = 1.0f;
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
        auto faceVerts = wmesh.getFaceVertices(&face);
        if (faceVerts.size() > 3)
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
    sharedGeom->setDrawElements(primSet);
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
