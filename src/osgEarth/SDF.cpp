/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include "SDF"
#include "Math"
#include "Metrics"
#include <osgEarth/rtree.h>

using namespace osgEarth;
using namespace osgEarth::Util;

void
SDFGenerator::encodeSDF(
    const FeatureList& features,
    osg::Image* image,
    const GeoExtent& extent,
    GLenum channel,
    FilterContext& fctx,
    const NumericExpression& min_dist_meters,
    const NumericExpression& max_dist_meters,
    bool invert,
    Cancelable* progress)
{
    OE_SOFT_ASSERT_AND_RETURN(image != nullptr, __func__, );
    OE_SOFT_ASSERT_AND_RETURN(extent.isValid(), __func__, );

    OE_PROFILING_ZONE;

    int c = clamp((int)(channel - GL_RED), 0, 3);
    osg::Vec3d p;
    osg::Vec4 pixel;
    //GeoImage gi(image, extent);

    NumericExpression mindist(min_dist_meters);
    NumericExpression maxdist(max_dist_meters);

    ImageUtils::PixelReader read(image);
    ImageUtils::PixelWriter write(image);

    // Poor man's degrees-to-meters conversion
    double toMeters = 1.0;
    if (extent.getSRS()->isGeographic())
    {
        double R = extent.getSRS()->getEllipsoid()->getRadiusEquator();
        toMeters = (2.0 * osg::PI * R / 360.0) * cos(osg::DegreesToRadians(extent.yMin()));
    }

    // Table of min and max SDF distances per feature:
    std::unordered_map<Feature*, std::pair<double, double>> distLUT;

    // Build a spatial index of features we are considering.
    // This is WAY faster than just iterating over all features.
    RTree<Feature*, double, 2> index;

    // The search radius, to constrain our search, will be equal
    // to the highest "max distance" taken from the feature set.
    double searchRadius = 0.0;

    double a_min[2], a_max[2];
    for (auto& feature : features)
    {
        const GeoExtent& e = feature->getExtent();
        a_min[0] = e.xMin(), a_min[1] = e.yMin();
        a_max[0] = e.xMax(), a_max[1] = e.yMax();
        index.Insert(a_min, a_max, feature.get());

        std::pair<double, double> limits(
            feature->eval(mindist, &fctx),
            feature->eval(maxdist, &fctx));

        distLUT.emplace(feature.get(), limits);

        searchRadius = std::max(limits.second, searchRadius);
    }
    searchRadius *= 1.1;

    std::vector<Feature*> hits;
    std::vector<double> ranges_squared;

    GeoImageIterator iter(image, extent);

    iter.forEachPixelOnCenter([&]()
        {
            read(pixel, iter.s(), iter.t());
            if (pixel[c] > 0.0)
            {
                p.x() = iter.x(), p.y() = iter.y();
                double nearest = 1.0;

                // Find all features within the search radius. We can't just say "find the 
                // one closest element" because the RTree only operates on the bounding-box
                // level. So instead we have to grab everything within the radius and manually
                // find the closest one.
                if (index.KNNSearch(p.ptr(), &hits, &ranges_squared, 0, searchRadius) > 0)
                {
                    for (int i = 0; i < hits.size() && nearest > 0.0; ++i)
                    {
                        Feature* feature = hits[i];
                        double range_squared = ranges_squared[i];
                        std::pair<double, double>& limits = distLUT[feature];

                        if (range_squared*toMeters <= limits.second*limits.second)
                        {
                            double sd = feature->getGeometry()->getSignedDistance2D(p) * toMeters;
                            if (invert) sd = -sd;
                            double sd_unit = unitremap(sd, limits.first, limits.second);
                            nearest = std::min(nearest, sd_unit);
                        }
                    }
                }

                if (nearest < pixel[c])
                {
                    pixel[c] = nearest;
                    write(pixel, iter.s(), iter.t());
                }
            }
        });
}
