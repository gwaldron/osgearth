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
    Cancelable* progress)
{
    OE_SOFT_ASSERT_AND_RETURN(image != nullptr, __func__, );
    OE_SOFT_ASSERT_AND_RETURN(extent.isValid(), __func__, );

    int c = clamp((int)(channel - GL_RED), 0, 3);
    osg::Vec3d p;
    osg::Vec4 pixel;
    GeoImage gi(image, extent);

    NumericExpression mindist(min_dist_meters);
    NumericExpression maxdist(max_dist_meters);

    ImageUtils::PixelReader read(image);
    ImageUtils::PixelWriter write(image);

    double toMeters = 1.0;
    if (extent.getSRS()->isGeographic())
    {
        double R = extent.getSRS()->getEllipsoid()->getRadiusEquator();
        toMeters = (2.0 * osg::PI * R / 360.0) * cos(osg::DegreesToRadians(extent.yMin()));
    }

    std::vector<std::pair<double, double>> distLUT;
    distLUT.reserve(features.size());
    for (auto& f_ptr : features)
    {
        distLUT.emplace_back(
            f_ptr->eval(mindist, &fctx),
            f_ptr->eval(maxdist, &fctx));
    }

    for (int t = 0; t < image->t(); ++t)
    {
        if (progress && progress->isCanceled())
            return;

        for (int s = 0; s < image->s(); ++s)
        {
            double best = DBL_MAX;

            gi.getCoord(s, t, p.x(), p.y());

            int i = 0;
            for (auto& f_ptr : features)
            {
                double sd = f_ptr->getGeometry()->getSignedDistance2D(p) * toMeters;
                double sd_unit = unitremap(sd, distLUT[i].first, distLUT[i].second);
                if (sd_unit < best)
                    best = sd_unit;
                if (best == 0.0)
                    break;
                ++i;
            }
            read(pixel, s, t);
            pixel[c] = best;
            write(pixel, s, t);
        }
    }
}
