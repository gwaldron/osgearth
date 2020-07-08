/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "MaskGenerator"

#include <osgEarth/Array>
#include <osgEarth/MaskLayer>
#include <osgEarth/Locators>
#include <osgEarth/Map>
#include <osgEarth/ModelLayer>
#include <osgEarth/Geometry>
#include <osgEarth/Math>
#include <osgUtil/DelaunayTriangulator>

#define LC "[MaskGenerator] "

using namespace osgEarth;
using namespace osgEarth::REX;

#define MATCH_TOLERANCE 0.000001

#define EQUIVALENT(X,Y) (osg::equivalent((double)(X), (double)(Y), MATCH_TOLERANCE))

#define EQUIVALENT_2D(A, B) (EQUIVALENT(A->x(), B->x()) && EQUIVALENT(A->y(), B->y()))

namespace
{
    void resample(Geometry* geom, double maxLen)
    {
        GeometryIterator i(geom);
        while (i.hasMore())
        {
            Geometry* part = i.next();
            if (part->size() < 2) continue;

            std::vector<osg::Vec3d> newGeom;
            ConstSegmentIterator csi(part, true);
            while(csi.hasMore())
            {
                Segment seg = csi.next();
                newGeom.push_back(seg.first);
                osg::Vec3d vec3d = seg.second - seg.first;
                osg::Vec2d vec2d(vec3d.x(), vec3d.y());
                double len2d = vec2d.length();

                if (len2d > maxLen)
                {
                    double numNewPoints = ::floor(len2d/maxLen);
                    double frac = 1.0 / (numNewPoints + 1.0);
                    for (double fj = 1.0; fj <= numNewPoints; ++fj)
                    {
                        double t = fj * frac;
                        osg::Vec3d newPoint(
                            seg.first.x() + vec2d.x()*t,
                            seg.first.y() + vec2d.y()*t,
                            seg.first.z() + vec3d.z()*t);
                        newGeom.push_back(newPoint);
                    }
                }
            }
            // This part is a closed loop, so the last point is
            // already in newGeom as the first point.
            if (newGeom.empty() == false)
            {
                part->swap(newGeom);
            }
        }

        geom->removeDuplicates();
    }

    //! Compares two 3D points ignoring the Z value.
    struct less_2d
    {
        bool operator()(const osg::Vec3& lhs, const osg::Vec3& rhs) const
        {
            if (lhs[0] < rhs[0]) return true;
            else if (lhs[0] > rhs[0]) return false;
            else return lhs[1] < rhs[1];
        }
    };
}

// Use our own high-performance (hah!) tests for point-in-polygon

class FastDelaunayConstraint : public osgUtil::DelaunayConstraint
{
public:
    bool contains(const osg::Vec3 &testpoint) const;
    bool outside(const osg::Vec3 &testpoint) const;
};

bool FastDelaunayConstraint::contains(const osg::Vec3 &testpoint) const
{
    return pointInPoly2d(testpoint, this, MATCH_TOLERANCE);
}

bool FastDelaunayConstraint::outside(const osg::Vec3 &testpoint) const
{
    return !pointInPoly2d(testpoint, this, MATCH_TOLERANCE);
}

MaskGenerator::MaskGenerator(const TileKey& key, unsigned tileSize, const Map* map) :
    _key( key ), _tileSize(tileSize), _ndcMin(DBL_MAX, DBL_MAX, DBL_MAX), _ndcMax(-DBL_MAX, -DBL_MAX, -DBL_MAX),
    _tileLength(static_cast<double>(tileSize) - 1.0)
{
    MaskLayerVector maskLayers;
    map->getLayers(maskLayers);

    for(MaskLayerVector::const_iterator it = maskLayers.begin();
        it != maskLayers.end(); 
        ++it)
    {
        MaskLayer* layer = it->get();
        if ( layer->getMinLevel() <= key.getLevelOfDetail() )
        {
            setupMaskRecord(layer->getOrCreateMaskBoundary( 1.0, key.getExtent().getSRS(), (ProgressCallback*)0L ) );
        }

        // add masks from model layers with embedded masks?
        ModelLayerVector modelLayers;
        map->getLayers(modelLayers);
        for(ModelLayerVector::const_iterator i = modelLayers.begin(); i != modelLayers.end(); ++i)
        {
            ModelLayer* layer = i->get();
            if (layer->getMaskSource() && layer->getMaskMinLevel() <= key.getLevelOfDetail())
            {
                setupMaskRecord(layer->getOrCreateMaskBoundary(1.0f, key.getExtent().getSRS(), (ProgressCallback*)0L) );
            }
        }
    }
}

void
MaskGenerator::setupMaskRecord(osg::Vec3dArray* boundary)
{
    // Make a "locator" for this key so we can do coordinate conversion:
    GeoLocator geoLocator(_key.getExtent());

    if ( boundary )
    {
        // Calculate the axis-aligned bounding box of the boundary polygon:
        osg::BoundingBoxd bbox = polygonBBox2d(*boundary);

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
            _maskRecords.push_back( MaskRecord(boundary, min_ndc, max_ndc, 0L) );
        }
    }
}

void MaskGenerator::patchIndicesFromBounds(const osg::Vec3d& ndcMin, const osg::Vec3d& ndcMax,
                                           int& minX, int& minY, int& maxX, int& maxY)
{
    // Figure out the indices representing the area we need to "cut out"
    // of the tile to accommadate the mask:
    minX = static_cast<int>(floor(clamp(ndcMin.x(), 0.0, 1.0) * _tileLength));
    minY = static_cast<int>(floor(clamp(ndcMin.y(), 0.0, 1.0) * _tileLength));
    maxX = static_cast<int>(ceil(clamp(ndcMax.x(), 0.0, 1.0) * _tileLength));
    maxY = static_cast<int>(ceil(clamp(ndcMax.y(), 0.0, 1.0) * _tileLength));
}

MaskGenerator::Result
MaskGenerator::createMaskPrimitives(osg::Vec3Array* verts, osg::Vec3Array* texCoords, 
                                    osg::Vec3Array* normals, osg::Vec3Array* neighbors,
                                    osg::Vec3Array* neighborNormals,
                                    osg::ref_ptr<osg::DrawElements>& out_elements)
{
    if (_maskRecords.size() <= 0)
    {
        return R_BOUNDARY_DOES_NOT_INTERSECT_TILE;
    }
    
    GeoLocator geoLocator(_key.getExtent());

    // Configure up a local tangent plane at the centroid of the tile:
    GeoPoint centroid;
    _key.getExtent().getCentroid( centroid );
    osg::Matrix world2local, local2world;
    centroid.createWorldToLocal( world2local );
    local2world.invert( world2local );

    // This array holds the NDC grid of points inside the patch polygon
    // (built later in this method)
    osg::ref_ptr<osg::Vec3Array> coordsArray = new osg::Vec3Array();

    // Calculate the combined axis-aligned NDC bounding box for all masks:
    // (gw: didn't we already do this in setupMaskRecord?)
    const double minndcx = _ndcMin.x();
    const double minndcy = _ndcMin.y();
    const double maxndcx = _ndcMax.x();
    const double maxndcy = _ndcMax.y();

    // Figure out the indices representing the area we need to "cut out"
    // of the tile to accommadate the mask:
    int min_i = (int)floor(minndcx * _tileLength);
    if (min_i < 0) min_i = 0;
    if (min_i >= (int)_tileSize) min_i = _tileSize - 1;

    int min_j = (int)floor(minndcy * _tileLength);
    if (min_j < 0) min_j = 0;
    if (min_j >= (int)_tileSize) min_j = _tileSize - 1;

    int max_i = (int)ceil(maxndcx * _tileLength);
    if (max_i < 0) max_i = 0;
    if (max_i >= (int)_tileSize) max_i = _tileSize - 1;

    int max_j = (int)ceil(maxndcy * _tileLength);
    if (max_j < 0) max_j = 0;
    if (max_j >= (int)_tileSize) max_j = _tileSize - 1;

    // XXX This isn't right.
    if (min_i < 0 || max_i < 0 || min_j < 0 || max_j < 0)
    {
        return R_BOUNDARY_DOES_NOT_INTERSECT_TILE;
    }

    
    // The "patch polygon" is the region that stitches the normal tile geometry to the mask boundary.
    // The patch will be in NDC coordinates:

    // Number of verts wide (i) and height(i) of the patch polygon:
    int num_i = max_i - min_i + 1;
    int num_j = max_j - min_j + 1;

    // Ndc coordinates can be easily computed from indices into the
    // patch. So, for the points that come from the patch grid,
    // calculate their coordinates based on the indices and keep track
    // of whether or not they should be included in the triangulated
    // points.
    Array::Matrix<bool> coordsStorage(num_j, num_i, true);
    Array::View<bool> coords(coordsStorage, -min_j, -min_i, _tileSize, _tileSize);
    osg::ref_ptr<Polygon> patchPoly = new Polygon();
    patchPoly->resize(num_i * 2 + num_j * 2 - 4);

    // top and bottom verts:
    for (int i = 0; i < num_i; i++)
    {
        {
            osg::Vec3d ndc( ((double)(i + min_i))/_tileLength, ((double)min_j)/_tileLength, 0.0);
            (*patchPoly)[i] = ndc;
        }

        {
            // bottom:
            osg::Vec3d ndc( ((double)(i + min_i))/_tileLength, ((double)max_j)/_tileLength, 0.0);
            (*patchPoly)[i + (2 * num_i + num_j - 3) - 2 * i] = ndc;
        }
    }

    // left and right verts:
    for (int j = 0; j < num_j - 2; j++)
    {
        {
            // right:
            osg::Vec3d ndc( ((double)max_i)/_tileLength, ((double)(min_j + j + 1))/_tileLength, 0.0);
            (*patchPoly)[j + num_i] = ndc;
        }

        {
            osg::Vec3d ndc( ((double)min_i)/_tileLength, ((double)(min_j + j + 1))/_tileLength, 0.0);
            (*patchPoly)[j + (2 * num_i + 2 * num_j - 5) - 2 * j] = ndc;
        }
    }

    std::set<osg::Vec3d, less_2d> boundaryVerts;

    osg::ref_ptr<FastDelaunayConstraint> dc = new FastDelaunayConstraint();
    osg::Vec3Array* constraintVerts = new osg::Vec3Array();
    dc->setVertexArray(constraintVerts);

    // Use delaunay triangulation for stitching:
    for (MaskRecordVector::iterator mr = _maskRecords.begin();mr != _maskRecords.end();mr++)
    {
        //Create local polygon representing mask
        mr->_boundaryPoly = new Polygon();
        mr->_boundaryPoly->reserve(mr->_boundary->size());
        for (osg::Vec3dArray::iterator it = (*mr)._boundary->begin(); it != (*mr)._boundary->end(); ++it)
        {
            osg::Vec3d local;
            geoLocator.mapToUnit(*it, local);
            mr->_boundaryPoly->push_back(local);
        }
            
        // Resample the masking polygon to closely match the resolution of the 
        // current tile grid, which will result in a better tessellation.
        // Ideally we would do this after cropping, but that is causing some
        // triangulation errors. TODO -gw
        if (!mr->_boundaryPoly->empty())
        {
            const double interval = 1.0 / double(_tileSize-1);
            resample(mr->_boundaryPoly.get(), interval);
        }

        // Crop the boundary to the patch polygon (i.e. the bounding box)
        // for case where mask crosses tile edges
        osg::ref_ptr<Geometry> boundaryPolyCroppedToTile;
        mr->_boundaryPoly->crop(patchPoly.get(), boundaryPolyCroppedToTile);

        // See the comment for the call to resample above. -gw
        //if (boundaryPolyCroppedToTile.valid() && !boundaryPolyCroppedToTile->empty())
        //{
        //    const double interval = 1.0 / double(_tileSize-1);
        //    resample(boundaryPolyCroppedToTile.get(), interval);
        //}

        // Add the cropped boundary geometry as a Triangulation Constraint.
        unsigned start = constraintVerts->size();

        GeometryIterator i( boundaryPolyCroppedToTile.get(), false );
        while( i.hasMore() )
        {
            Geometry* part = i.next();
            if (!part)
                continue;

            if (part->getType() == Geometry::TYPE_POLYGON)
            {
                osg::ref_ptr<osg::Vec3Array> partVerts = part->createVec3Array();
                int offset = constraintVerts->size();
                constraintVerts->reserve(constraintVerts->size() + partVerts->size());
                constraintVerts->insert(constraintVerts->end(), partVerts->begin(), partVerts->end());
                dc->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, offset, partVerts->size()));
            }
        }

        // Cropping strips z-values! so we need reassign them.            
        osg::Vec3Array::iterator it_start = constraintVerts->begin() + start;
        std::vector<int> isZSet;
        for (osg::Vec3Array::iterator it = it_start; it != constraintVerts->end(); ++it)
        {
            int zSet = 0;

            // Search the patch (bounding box) polygon for matching points:
            for (Polygon::iterator mit = patchPoly->begin(); mit != patchPoly->end(); ++mit)
            {
                if (EQUIVALENT_2D(mit, it))
                {
                    //(*it).z() = (*mit).z(); // commented out by Jeff...why?
                    zSet += 1;

                    // Remove duplicate point from coordsArray to avoid duplicate point warnings
                    int ix = static_cast<int>(round(mit->x() * _tileLength));
                    int iy = static_cast<int>(round(mit->y() * _tileLength));
                    coords(iy, ix) = false;
                    break;
                }
            }

            // Search the original uncropped boundary polygon for matching points,
            // and build a set of boundary vertices.
            for (Polygon::iterator mit = mr->_boundaryPoly->begin(); mit != mr->_boundaryPoly->end(); ++mit)
            {
                if (EQUIVALENT_2D(mit, it))
                {
                    (*it).z() = (*mit).z();
                    zSet += 2;

                    boundaryVerts.insert( *it );
                    break;
                }
            }

            isZSet.push_back(zSet);
        }

        // Any mask patch verts that are still unset are newly created verts where the patch
        // meets the mask. (Do you mean, where the boundary crosses the tile edge? -gw) 
        // Find the mask segment the point lies along and calculate the
        // appropriate z value for the point.
        int count = 0;
        for (osg::Vec3Array::iterator it = it_start; it != constraintVerts->end(); ++it)
        {
            //If the z-value was set from a mask vertex there is no need to change it.  If
            //it was set from a vertex from the patch polygon it may need to be overridden if
            //the vertex lies along a mask edge.  Or if it is unset, it will need to be set.
            //if (isZSet[count] < 2)
            if (!isZSet[count])
            {
                osg::Vec3d p2 = *it;
                double closestZ = 0.0;
                double closestRatio = DBL_MAX;
                for (Polygon::iterator mit = mr->_boundaryPoly->begin(); mit != mr->_boundaryPoly->end(); ++mit)
                {
                    osg::Vec3d p1 = *mit;
                    osg::Vec3d p3 = mit == --mr->_boundaryPoly->end() ? mr->_boundaryPoly->front() : (*(mit + 1));

                    //Truncated vales to compensate for accuracy issues
                    double p1x = ((int)(p1.x() * 1000000)) / 1000000.0L;
                    double p3x = ((int)(p3.x() * 1000000)) / 1000000.0L;
                    double p2x = ((int)(p2.x() * 1000000)) / 1000000.0L;

                    double p1y = ((int)(p1.y() * 1000000)) / 1000000.0L;
                    double p3y = ((int)(p3.y() * 1000000)) / 1000000.0L;
                    double p2y = ((int)(p2.y() * 1000000)) / 1000000.0L;

                    if ((p1x < p3x ? p2x >= p1x && p2x <= p3x : p2x >= p3x && p2x <= p1x) &&
                        (p1y < p3y ? p2y >= p1y && p2y <= p3y : p2y >= p3y && p2y <= p1y))
                    {
                        double l1 =(osg::Vec2d(p2.x(), p2.y()) - osg::Vec2d(p1.x(), p1.y())).length();
                        double lt = (osg::Vec2d(p3.x(), p3.y()) - osg::Vec2d(p1.x(), p1.y())).length();
                        double zmag = p3.z() - p1.z();

                        double foundZ = (l1 / lt) * zmag + p1.z();

                        double mRatio = 1.0;
                        if (EQUIVALENT(p1x, p3x))
                        {
                            if (EQUIVALENT(p1x, p2x))
                                mRatio = 0.0;
                        }
                        else
                        {
                            double m1 = p1x == p2x ? 0.0 : (p2y - p1y) / (p2x - p1x);
                            double m2 = p1x == p3x ? 0.0 : (p3y - p1y) / (p3x - p1x);
                            mRatio = m2 == 0.0 ? m1 : osg::absolute(1.0L - m1 / m2);
                        }

                        if (mRatio < 0.01)
                        {
                            (*it).z() = foundZ;
                            isZSet[count] = 2;

                            boundaryVerts.insert( *it );
                            break;
                        }
                        else if (mRatio < closestRatio)
                        {
                            closestRatio = mRatio;
                            closestZ = foundZ;
                        }
                    }
                }

                if (!isZSet[count] && closestRatio < DBL_MAX)
                {
                    (*it).z() = closestZ;
                    isZSet[count] = 2;
                    boundaryVerts.insert( *it );
                }
            }

            if (isZSet[count] == 0)
            {
                OE_INFO << LC << "Z-value not set for mask constraint vertex" << std::endl;
            }

            count++;
        }
        // Any grid points that are inside this mask poly should be
        // eliminated too.
        int minX, minY, maxX, maxY;
        patchIndicesFromBounds(mr->_ndcMin, mr->_ndcMax, minX, minY, maxX, maxY);
        const double scale = 1.0 / _tileLength;
        for (int j = minY; j <= maxY; ++j)
        {
            for (int i = minX; i <= maxX; ++i)
            {
                if (coords(j, i))
                {
                    osg::Vec3d gridPt(i * scale, j * scale, 0.0);
                    coords(j, i) = !pointInPoly2d(gridPt, *mr->_boundaryPoly, MATCH_TOLERANCE);
                }
            }
        }
    }

    // If we collected no constraints, that means the boundary geometry
    // does not intersect the tile at all. Bail out now.
    if (constraintVerts->empty())
    {
        return R_BOUNDARY_DOES_NOT_INTERSECT_TILE;
    }
    // XXX
    const double scale = 1.0 / _tileLength;
    for (int j = min_j; j <= max_j; ++j)
    {
        for (int i = min_i; i <= max_i; ++i)
        {
            if (coords(j, i))
            {
                coordsArray->push_back(osg::Vec3d(i * scale, j * scale, 0.0));
            }
        }
    }

    if (coordsArray->empty())
    {
        return R_BOUNDARY_CONTAINS_ENTIRE_TILE;
    }
    // Set up a triangulator with the patch coordinates:
    osg::ref_ptr<osgUtil::DelaunayTriangulator> trig = new osgUtil::DelaunayTriangulator();
    trig->setInputPointArray(coordsArray.get());
    trig->addInputConstraint(dc.get());

    // Create array to hold vertex normals
    //osg::Vec3Array* norms = new osg::Vec3Array();
    //trig->setOutputNormalArray(norms);

    // Triangulate! 
    trig->triangulate();

    // Remove any triangles interior to the boundaries.
    // Note: an alternative here would be to flatten them all to a common height -gw
    trig->removeInternalTriangles(dc.get());

    // Now build the new geometry.
    const osg::Vec3Array* trigPoints = trig->getInputPointArray();

    // Reserve space; pre-allocating space is faster
    verts->reserve(verts->size() + trigPoints->size());
    texCoords->reserve(texCoords->size() + trigPoints->size());
    normals->reserve(normals->size() + trigPoints->size());
    if ( neighbors )
        neighbors->reserve(neighbors->size() + trigPoints->size()); 
    if ( neighborNormals )
        neighborNormals->reserve(neighborNormals->size() + trigPoints->size()); 

    // Iterate through point to convert to model coords, calculate normals, and set up tex coords
    //osg::ref_ptr<GeoLocator> locator = GeoLocator::createForKey( _key, mapInfo );

    unsigned vertsOffset = verts->size();

    for (osg::Vec3Array::const_iterator it = trigPoints->begin(); it != trigPoints->end(); ++it)
    {
        // check to see if point is a part of the original mask boundary
        bool isBoundary = boundaryVerts.find(*it) != boundaryVerts.end();

        // get local coords
        osg::Vec3d local;
        geoLocator.unitToWorld(osg::Vec3d(it->x(), it->y(), 0.0f), local);
        local = local * world2local;

        // calc normals
        osg::Vec3d localPlusOne;
        geoLocator.unitToWorld(osg::Vec3d(it->x(), it->y(), 1.0f), localPlusOne);
        osg::Vec3d normal = (localPlusOne*world2local)-local;                
        normal.normalize();
        normals->push_back( normal );

        // set elevation if this is a point along the mask boundary
        if (isBoundary)
            local += normal*it->z();

        verts->push_back(local);

        // use same data for neighbor to prevent morphing
        if ( neighbors )
            neighbors->push_back( local );
        if ( neighborNormals )
            neighborNormals->push_back( normal );

        // set up text coords
        texCoords->push_back( osg::Vec3f(it->x(), it->y(), isBoundary ? VERTEX_MARKER_BOUNDARY : VERTEX_MARKER_PATCH) );
    }

    // Get triangles from triangulator and add as primitive set to the geometry
    osg::DrawElementsUInt* tris = trig->getTriangles();

    // If something went wrong, just bail out. This should never happen
    if (tris == 0L || tris->getNumIndices() < 3)
    {
        //OE_INFO << LC << "* Triangulation resulted in no geometry\n";
        return R_BOUNDARY_CONTAINS_ENTIRE_TILE;
    }

    // Construct the output triangle set.
    unsigned numIndicies = tris->size() * 6;
    out_elements =
        numIndicies > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUInt(tris->getMode()) :
        (osg::DrawElements*)new osg::DrawElementsUShort(tris->getMode());

    out_elements->reserveElements(numIndicies);

    const osg::MixinVector<GLuint> ins = tris->asVector();

    for (osg::MixinVector<GLuint>::const_iterator it = ins.begin(); it != ins.end(); ++it)
    {
        unsigned i0 = vertsOffset + *it++;
        unsigned i1 = vertsOffset + *it++;
        unsigned i2 = vertsOffset + *it;

        const osg::Vec3d& v0 = (*verts)[i0];
        const osg::Vec3d& v1 = (*verts)[i1];
        const osg::Vec3d& v2 = (*verts)[i2];

        // check the winding order. Triangles don't always come out in the right orientation
        if (((v0 - v1) ^ (v2 - v1)).z() < 0)
        {
            out_elements->addElement(i0);
            out_elements->addElement(i1);
            out_elements->addElement(i2);
        }
        else
        {
            out_elements->addElement(i0);
            out_elements->addElement(i2);
            out_elements->addElement(i1);
        }
    }

    return R_BOUNDARY_INTERSECTS_TILE;
}

void
MaskGenerator::getMinMax(osg::Vec3d& min, osg::Vec3d& max)
{
    min = _ndcMin;
    max = _ndcMax;
}

float
MaskGenerator::getMarker(float nx, float ny) const
{
    float marker = VERTEX_MARKER_GRID;

    if (_maskRecords.size() > 0)
    {
        int min_i = (int)floor(_ndcMin.x() * _tileLength);
        int min_j = (int)floor(_ndcMin.y() * _tileLength);
        int max_i = (int)ceil(_ndcMax.x() * _tileLength);
        int max_j = (int)ceil(_ndcMax.y() * _tileLength);

        int i = nx * _tileLength;
        int j = ny * _tileLength;

        if (i > min_i && i < max_i && j > min_j && j < max_j)
        {
            marker = VERTEX_MARKER_DISCARD; // contained by patch
        }
        else if ((i == min_i && j >= min_j && j <= max_j) ||
                 (i == max_i && j >= min_j && j <= max_j) ||
                 (j == min_j && i >= min_i && i <= max_i) ||
                 (j == max_j && i >= min_i && i <= max_i))
        {
            marker = VERTEX_MARKER_PATCH;
        }
    }

    return marker;
}
