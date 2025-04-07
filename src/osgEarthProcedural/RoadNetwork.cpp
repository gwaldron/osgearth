/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "RoadNetwork"
#include <osgEarth/Feature>
#include <osgEarth/GeoData>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#undef LC
#define LC "[RoadNetwork] "

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
                ways.emplace_back(j0, j1, feature, part);
                ways.back().length = part->getLength();
                j0.ways.emplace_back(&ways.back());
                j1.ways.emplace_back(&ways.back());
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
            Way* incoming_way = nullptr;
            auto* current_junction = &junction;

            while (current_junction)
            {
                Way* outgoing_way = nullptr;

                // if the user set the "nextWay" functor, AND this is NOT the first junction
                // (because the first junction has to incoming Way):
                if (nextWayInRelation && incoming_way)
                {
                    outgoing_way = nextWayInRelation(*current_junction, incoming_way, relation.ways);
                }

                // if we did NOT compute an outgoing way in the previous step,
                // pick the first valid one in the junction:
                if (outgoing_way == nullptr)
                {
                    for(auto* way : current_junction->ways)
                    {
                        if (std::find(relation.ways.begin(), relation.ways.end(), way) == relation.ways.end())
                        {
                            outgoing_way = way;
                            break;
                        }
                    }
                }

                // safety catch to disallow doubling back:
                if (outgoing_way == incoming_way)
                {
                    outgoing_way = nullptr;
                }

                // still good? keep going:
                if (outgoing_way)
                {
                    // add this new way to our relation:
                    relation.ways.emplace_back(outgoing_way);

                    // update the relation's total length:
                    relation.length += outgoing_way->length;

                    // Make sure the relation all flows in the same direction; this include not only 
                    // the Way start and end junctions, but also the geometry's points themselves.
                    if (*current_junction != *outgoing_way->start)
                    {
                        std::swap(outgoing_way->start, outgoing_way->end);
                    }
                    if (*current_junction != outgoing_way->geometry->front())
                    {
                        std::reverse(outgoing_way->geometry->begin(), outgoing_way->geometry->end());
                    }

                    // record whether the front and back vertixes of this geometry are endpoints.
                    geometryEndpointFlags[outgoing_way->geometry] = { 
                        current_junction->is_endpoint(), 
                        outgoing_way->end->is_endpoint()
                    };

                    // iterate to the next junction:
                    current_junction = outgoing_way->end;
                    incoming_way = outgoing_way;

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
RoadNetwork::mergeRelations(std::vector<osg::ref_ptr<Feature>>& output)
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
                    if (canMerge(relation.ways[i]->feature, relation.ways[i + 1]->feature))
                    {
                        total_size += relation.ways[i]->geometry->size();
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
                total_size += relation.ways.back()->geometry->size();

                auto* line = new LineString();
                line->reserve(total_size);

                osg::ref_ptr<Feature> new_feature;

                for (int i = 0; i < relation.ways.size(); ++i)
                {
                    auto& way = *relation.ways[i];
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
                for (auto& way : relation.ways)
                {
                    if (added_to_output.insert(way->feature).second == true)
                    {
                        output.emplace_back(way->feature);
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





namespace
{
    Geometry* line_to_polygon(const Geometry* geom, double width)
    {
        MultiGeometry* mg = new MultiGeometry();
        double halfWidth = 0.5 * width;

        ConstGeometryIterator i(geom, false);
        while (i.hasMore())
        {
            auto* line = i.next();
            if (line->size() < 2)
                continue;

            Polygon* poly = new Polygon();
            poly->resize(line->size() * 2);

            for (int i = 0; i < line->size(); ++i)
            {
                osg::Vec3d dir;
                if (i == 0)
                    dir = (*line)[i + 1] - (*line)[i];
                else if (i == line->size() - 1)
                    dir = (*line)[i] - (*line)[i - 1];
                else
                    dir = (*line)[i + 1] - (*line)[i - 1];
                dir.normalize();

                osg::Vec3d right = dir ^ osg::Vec3d(0, 0, 1);

                (*poly)[i] = (*line)[i] + right * halfWidth;
                (*poly)[line->size() * 2 - i - 1] = (*line)[i] - right * halfWidth;
            }

            mg->add(poly);
        }

        return mg;
    }
}
