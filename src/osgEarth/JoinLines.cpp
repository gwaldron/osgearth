/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "JoinLines"

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    inline bool eq2d(const osg::Vec3d& lhs, const osg::Vec3d& rhs, double EPS = 1e-3)
    {
        return
            equivalent(lhs.x(), rhs.x(), EPS) &&
            equivalent(lhs.y(), rhs.y(), EPS);
    }
}

FilterContext
JoinLinesFilter::push(FeatureList& input, FilterContext& context)
{        
    // first, collect all linear features as single linestrings.
    FeatureList lines;
    lines.reserve(input.size());

    for (auto& feature : input) {
        auto* g = feature->getGeometry();
        if (g && g->isLinear()) {
            GeometryIterator iter(g, false);
            iter.forEach([&](auto* geom)
                {
                    if (geom->size() >= 2) {
                        auto* new_feature = new Feature(*feature);
                        new_feature->setGeometry(geom->cloneAs(Geometry::TYPE_LINESTRING));
                        lines.emplace_back(new_feature);
                    }
                });
        }
    }

    const double epsilon = 1e-3;

    // combine linestrings with common endpoints:
    for (int changes = 1; changes > 0; )
    {
        changes = 0;
        for (auto& feature : lines)
        {
            if (!feature.valid())
                continue;

            auto* geom = feature->getGeometry();

            for (auto& other : lines)
            {
                if (other.valid() && other != feature)
                {
                    auto* other_geom = other->getGeometry();

                    if (eq2d(geom->back(), other_geom->front(), epsilon))
                    {
                        geom->resize(geom->size() - 1);
                        geom->insert(geom->end(), other_geom->begin(), other_geom->end());
                        changes++;
                        other = nullptr;
                    }

                    else if (eq2d(geom->back(), other_geom->back(), epsilon))
                    {
                        geom->resize(geom->size() - 1);
                        geom->insert(geom->end(), other_geom->rbegin(), other_geom->rend());
                        changes++;
                        other = nullptr;
                    }

                    else if (eq2d(other_geom->back(), geom->front(), epsilon))
                    {
                        other_geom->resize(other_geom->size() - 1);
                        other_geom->insert(other_geom->end(), geom->begin(), geom->end());
                        changes++;
                        feature = nullptr;
                        break;
                    }

                    else if (eq2d(other_geom->back(), geom->back(), epsilon))
                    {
                        other_geom->resize(other_geom->size() - 1);
                        other_geom->insert(other_geom->end(), geom->rbegin(), geom->rend());
                        changes++;
                        feature = nullptr;
                        break;
                    }
                }
            }
        }
    }

    // remove the ones that were null'd out during connection:
    input.clear();

    for (auto& feature : lines)
        if (feature.valid())
            input.emplace_back(feature);

    return context;
}
