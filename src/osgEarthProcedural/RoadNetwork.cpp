/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2025 Pelican Mapping
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
#include "RoadNetwork"
#include <osgEarth/Feature>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#undef LC
#define LC "[RoadNetwork] "

namespace
{
    //template<class A, class B>
    //inline bool equivalent(const A& lhs, const B& rhs)
    //{
    //    auto LX = (std::int64_t)(lhs.x() * RoadNetwork::precision);
    //    auto RX = (std::int64_t)(rhs.x() * RoadNetwork::precision);
    //    if (LX != RX) return false;
    //    auto LY = (std::int64_t)(lhs.y() * RoadNetwork::precision);
    //    auto RY = (std::int64_t)(rhs.y() * RoadNetwork::precision);
    //    return LY == RY;
    //}
}

void
RoadNetwork::addFeature(Feature* feature)
{
    OE_SOFT_ASSERT_AND_RETURN(feature, void());

    auto* geom = feature->getGeometry();
    geom->forEachPart([&](Geometry* part)
        {
            if (part->size() >= 2)
            {
                auto& j0 = addJunction(part->front());
                auto& j1 = addJunction(part->back());
                j0.ways.emplace_back(ways.size());
                j1.ways.emplace_back(ways.size());
                ways.emplace_back(j0, j1, feature, part);
                ways.back().length = part->getLength();
            }
        });
}

const RoadNetwork::Junction&
RoadNetwork::addJunction(const osg::Vec3d& point)
{
    auto i = junctions.insert(Junction(point));
    return *i.first;
}

void
RoadNetwork::buildRelations()
{
    // Keep track of nodes already traversed.
    std::set<const Junction*> endpoints_traversed;
    std::set<const Junction*> midpoints_traversed;

    for (auto& junction : junctions)
    {
        // collect nodes that are NOT endpoints.
        if (junction.is_midpoint())
        {
            midpoints.insert(junction);
        }

        // Is this junction an endpoint that does not already belong to a relation?
        else if (junction.is_endpoint() && endpoints_traversed.count(&junction) == 0)
        {
            endpoints_traversed.insert(&junction);

            Relation relation;
            int incoming_way_idx = -1;
            auto* current_junction = &junction;

            while (current_junction)
            {
                int outgoing_way_idx = -1;

                // if the user set the "nextWay" functor, AND this is NOT the first junction
                // (because the first junction has to incoming Way):
                if (nextWayInRelation && incoming_way_idx >= 0)
                {
                    outgoing_way_idx = nextWayInRelation(*current_junction, incoming_way_idx, relation.ways);
                }

                // if we did NOT compute an outgoing way in the previous step,
                // pick the first valid one in the junction:
                if (outgoing_way_idx < 0)
                {
                    for (int k = 0; k < current_junction->ways.size(); ++k)
                    {
                        int w = current_junction->ways[k];
                        if (std::find(relation.ways.begin(), relation.ways.end(), outgoing_way_idx) == relation.ways.end())
                        {
                            outgoing_way_idx = w;
                            break;
                        }
                    }
                }

                // safety catch to disallow doubling back:
                if (outgoing_way_idx == incoming_way_idx)
                {
                    outgoing_way_idx = -1;
                }

                // still good? keep going:
                if (outgoing_way_idx >= 0)
                {
                    // add this new way to our relation:
                    relation.ways.emplace_back(outgoing_way_idx);

                    // update the relation's total length:
                    Way& outgoing_way = ways[outgoing_way_idx];
                    relation.length += outgoing_way.length;

                    // Make sure the geometry's points flow in the same direction as the relation
                    // as a whole. This is critical for clamping interpolation later on.
                    if (*current_junction != outgoing_way.geometry->front())
                    {
                        std::reverse(outgoing_way.geometry->begin(), outgoing_way.geometry->end());
                    }

                    // record whether the front and back vertixes of this geometry are endpoints.
                    geometryEndpointFlags[outgoing_way.geometry] = { current_junction->is_endpoint(), outgoing_way.end->is_endpoint() };

                    // iterate to the next junction:
                    current_junction = outgoing_way.end;
                    incoming_way_idx = outgoing_way_idx;

                    // If the next junction is already part of another relation,
                    // we are done. This can happen for merge lanes (for example)
                    if (midpoints_traversed.count(current_junction) > 0)
                    {
                        current_junction = nullptr;
                    }

                    // If we reached an endpoint, we're at the end of the relation
                    // and can go no further.
                    if (current_junction->is_endpoint())
                    {
                        endpoints_traversed.insert(current_junction);
                        current_junction = nullptr;
                    }
                }
                else
                {
                    // no ways left, we are done.
                    current_junction = nullptr;
                }
            }

            if (!relation.ways.empty())
            {
                relations.emplace_back(std::move(relation));
            }
        }
    }
}

void
RoadNetwork::mergeRelations(FeatureList& output)
{
    if (canMerge)
    {
        std::set<Feature*> added_to_output;

        // merging is all-or-nothing for a relation.
        for (auto& relation : relations)
        {
            bool ok_to_merge = false;
            int total_size = 0;

            if (relation.ways.size() > 1)
            {
                ok_to_merge = true;

                // make sure that all the features comprising this relation are compatible,
                // whatever that means -- it's up the the implementor of the "canMerge" lambda.
                for (int i = 0; ok_to_merge && i < relation.ways.size() - 1; ++i)
                {
                    if (canMerge(ways[relation.ways[i]].feature, ways[relation.ways[i + 1]].feature))
                    {
                        total_size += ways[relation.ways[i]].geometry->size();
                    }
                    else
                    {
                        ok_to_merge = false;
                    }
                }
            }

            // if all features passed the test, create one new feature that clones the first
            // feature in the relation and string together all its points into a single geometry.
            if (ok_to_merge)
            {
                total_size += ways[relation.ways.back()].geometry->size();

                auto* line = new LineString();
                line->reserve(total_size);

                osg::ref_ptr<Feature> new_feature;

                for (int i = 0; i < relation.ways.size(); ++i)
                {
                    auto& way = ways[relation.ways[i]];
                    auto* geom = way.geometry;
                    int start = (i == 0) ? 0 : 1; // so we don't duplicate endpoints.
                    for (int j = start; j < geom->size(); ++j)
                    {
                        line->push_back((*geom)[j]);
                    }

                    if (!new_feature.valid())
                    {
                        new_feature = new Feature(*way.feature);
                        new_feature->setGeometry(line);
                    }
                }

                if (new_feature.valid())
                {
                    output.emplace_back(new_feature);
                }
            }

            else
            {
                for (int i = 0; i < relation.ways.size(); ++i)
                {
                    auto* feature = ways[relation.ways[i]].feature;
                    if (added_to_output.insert(feature).second == true)
                    {
                        output.emplace_back(feature);
                    }
                }
            }
        }
    }
}

void
RoadNetwork::getFeatures(const GeoExtent& extent, std::unordered_set<FeatureID>& output) const
{
    output.clear();

    for(std::size_t i = 0; i<ways.size(); ++i)
    {
        auto& way = ways[i];
        auto center = way.geometry->getBounds().center();
        if (extent.contains(center.x(), center.y()))
        {
            output.insert(way.feature->getFID());
        }
    }
}
