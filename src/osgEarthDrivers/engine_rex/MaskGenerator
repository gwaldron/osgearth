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
#ifndef OSGEARTH_DRIVERS_REX_MASK_GENERATOR
#define OSGEARTH_DRIVERS_REX_MASK_GENERATOR 1

#include "Common"
#include <osgEarth/TileKey>
#include <osgEarth/MapInfo>
#include <osg/Geometry>

#define VERTEX_MARKER_DISCARD   1    // do not draw
#define VERTEX_MARKER_GRID      2    // regular grid vertex (not part of mask)
#define VERTEX_MARKER_PATCH     4    // not subject to morphing
#define VERTEX_MARKER_BOUNDARY  8    // not subject to elevation texture
#define VERTEX_MARKER_SKIRT    16    // it's a skirt vertex (bitmask)

namespace osgEarth {
    class Map;
}

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    using namespace osgEarth;

    /**
     * Record that stores the data for a single masking region.
     */
    struct MaskRecord
    {
        osg::ref_ptr<osg::Vec3dArray> _boundary;
        osg::Vec3d                    _ndcMin, _ndcMax;
        osg::Geometry*                _geom;
        osg::ref_ptr<osg::Vec3Array>  _internal;

        MaskRecord(osg::Vec3dArray* boundary, osg::Vec3d& ndcMin, osg::Vec3d& ndcMax, osg::Geometry* geom) 
            : _boundary(boundary), _ndcMin(ndcMin), _ndcMax(ndcMax), _geom(geom), _internal(new osg::Vec3Array()) { }
    };

    typedef std::vector<MaskRecord> MaskRecordVector;


    /**
     * Creates geometry for the part of a tile containing mask data.
     * Used internally by GeometryPool.
     */
    class MaskGenerator : public osg::Referenced
    {
    public:
        enum Result {
            R_BOUNDARY_DOES_NOT_INTERSECT_TILE,
            R_BOUNDARY_CONTAINS_ENTIRE_TILE,
            R_BOUNDARY_INTERSECTS_TILE
        };

    public:
        MaskGenerator(const TileKey& key, unsigned tileSize, const Map* map);

        //! True if this tile has masking data at all
        bool hasMasks() const
        {
            return _maskRecords.size() > 0;
        }

        //! whether a texcoord indicates that the corresponding vert is masked.
        bool isMasked(const osg::Vec3f& texCoord) const
        {
            return (VERTEX_MARKER_DISCARD & (int)texCoord.z()) != 0;
        }

        //! returns once of the VERTEX_MARKER_* defines for the given NDC location
        float getMarker(float nx, float ny) const;

        //! Gets the LL and UR corners of the "patch rectangle" in NDC space
        void getMinMax(osg::Vec3d& min, osg::Vec3d& max);

        //! Generates all the masking geometry and appened it to the passed-in arrays.
        Result createMaskPrimitives(
            const MapInfo&  mapInfo, 
            osg::Vec3Array* verts, 
            osg::Vec3Array* texCoords,
            osg::Vec3Array* normals,
            osg::Vec3Array* neighbors,
            osg::ref_ptr<osg::DrawElementsUInt>& out_elements);

    protected:
        void setupMaskRecord(const MapInfo& mapInfo, osg::Vec3dArray* boundary);

    protected:
        const TileKey _key;
        unsigned _tileSize;
        MaskRecordVector _maskRecords;
        osg::Vec3d _ndcMin, _ndcMax;
    };

} } } // namespace osgEarth::Drivers::RexTerrainEngine

#endif // OSGEARTH_DRIVERS_REX_MASK_GENERATOR
