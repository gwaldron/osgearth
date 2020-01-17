/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#ifndef OSGEARTH_ELEVATION_QUERY_H
#define OSGEARTH_ELEVATION_QUERY_H 1

#include <osgEarth/ElevationPool>
#include <osgEarth/Containers>
#include <osgEarth/ModelLayer>
#include <osgUtil/LineSegmentIntersector>

namespace osgEarth
{
    /**
     * @deprecated after 2.10 - Use ElevationPool instead, via Map::getElevationPool to
     *    query elevation data, and use Terrain::getHeight to query the in-scene
     *    terrain graph.
     *
     * ElevationQuery (EQ) lets you query the elevation at any point on a map.
     *
     * Rather than intersecting with a loadeEd scene graph, EQ uses the osgEarth
     * engine to directly access the best terrain tile for elevation query. You
     * give it the DEM resolution at which you want an elevation point, and it will
     * access the necessary tile and sample it.
     *
     * NOTE: EQ does NOT take into account rendering properties like vertical scale or
     * skirts. If you need a vertical scale, for example, simply scale the resulting
     * elevation value.
     *
     * ElevationQuery is not thread-safe. So not use the same instance of ElevationQuery
     * from multiple threads without mutexing.
     */
    class OSGEARTH_EXPORT ElevationQuery
    {
    public:
        ElevationQuery();

        /**
         * Constructs a new elevation manager.
         *
         * @param map
         *      Map against which to perform elevation queries.
         */
        ElevationQuery(const Map* map);

        /**
         * Sets a new map
         */
        void setMap(const Map* map);

        /**
         * Gets the elevation data at a point, given a terrain resolution.
         *
         * @param point
         *      Coordinates for which to query elevation.
         * @param desiredResolution
         *      Optimal resolution of elevation data to use for the query (if available).
         *      Pass in 0 (zero) to use the best available resolution.
         * @param out_resolution
         *      Resolution of the resulting elevation value (if the method returns true).
         *
         * @return Sampled elevation value (or NO_DATA_VALUE)
         */
        float getElevation(
            const GeoPoint& point,
            double          desiredResolution    =0.0,
            double*         out_actualResolution =0L );

        /**
         * Gets elevations for a whole array of points, storing the result in the
         * "z" element. If "ignoreZ" is false, the new Z value will be offset by
         * the original Z value.
         */
        bool getElevations(
            std::vector<osg::Vec3d>& points,
            const SpatialReference*  pointsSRS,
            bool                     ignoreZ = true,
            double                   desiredResolution =0.0 );

        /**
         * Gets elevations for a whole array of points, storing the results in the
         * "out_elevations" vector.
         */
        bool getElevations(
            const std::vector<osg::Vec3d>& points,
            const SpatialReference*        pointsSRS,
            std::vector<float>&            out_elevations,
            double                         desiredResolution =0.0 );

        /** dtor */
        virtual ~ElevationQuery() { }

    private:

        // Map model layers marked as terrain patches
        LayerVector _terrainModelLayers;
        ElevationLayerVector _elevationLayers;

        // Intersector to query the patch layers
        osg::ref_ptr<osgUtil::LineSegmentIntersector> _lsi;

        // Callback to force the intersector to materialize paged model nodes
        osg::ref_ptr<osgUtil::IntersectionVisitor::ReadCallback> _ivrc;

        // Active elevation sampler
        osg::ref_ptr<ElevationEnvelope> _envelope;

    private:
        void reset();
        void sync();
        void gatherTerrainModelLayers(const Map*);

        bool getElevationImpl(
            const GeoPoint& point,
            float&          out_elevation,
            double          desiredResolution,
            double*         out_actualResolution );

        osg::observer_ptr<const Map> _map;
        Revision _mapRevision;
    };

} // namespace osgEarth

#endif // OSGEARTH_ELEVATION_QUERY_H
