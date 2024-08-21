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

#include <osgEarth/TerrainConstraintLayer>
#include <osgEarth/Map>
#include <osgEarth/Progress>
#include <osgEarth/Utils>
#include <osgEarth/SimplePager>

using namespace osgEarth;

#define LC "[TerrainConstraintLayer] "
REGISTER_OSGEARTH_LAYER(terrainconstraint, TerrainConstraintLayer);
REGISTER_OSGEARTH_LAYER(featuremask, TerrainConstraintLayer);
REGISTER_OSGEARTH_LAYER(mask, TerrainConstraintLayer);

//........................................................................

void
TerrainConstraintLayer::Options::fromConfig(const Config& conf)
{
    // backwards compatability
    if (ciEquals(conf.key(), "featuremask"))
    {
        removeInterior() = true;
        hasElevation() = true;
    }

    featureSource().get(conf, "features");
    model().get(conf, "model");
    conf.get("remove_interior", removeInterior());
    conf.get("remove_exterior", removeExterior());
    conf.get("has_elevation", hasElevation());
    conf.get("min_level", minLevel());

    const Config& filtersConf = conf.child("filters");
    for (auto& child : filtersConf.children())
        filters().push_back(ConfigOptions(child));
}

Config
TerrainConstraintLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    featureSource().set(conf, "features");
    model().set(conf, "model");
    conf.set("remove_interior", removeInterior());
    conf.set("remove_exterior", removeExterior());
    conf.set("has_elevation", hasElevation());
    conf.set("min_level", minLevel());

    if (filters().empty() == false)
    {
        Config temp;
        for (unsigned i = 0; i < filters().size(); ++i)
            temp.add(filters()[i].getConfig());
        conf.set("filters", temp);
    }
    return conf;
}

//........................................................................

namespace
{
    using triangle_t = std::tuple<osg::Vec3d, osg::Vec3d, osg::Vec3d>;
    using triangle_set_t = std::set<triangle_t>;

    void addNode(osg::Node* node, const SpatialReference* map_srs, triangle_set_t& tris, MeshConstraint& constraint)
    {
        OE_SOFT_ASSERT_AND_RETURN(node != nullptr, void());

        if (node->getUserDataContainer())
        {
            auto wrapper = dynamic_cast<WrapperObject<osg::ref_ptr<Feature>>*>(
                node->getUserDataContainer()->getUserObject("features"));

            if (wrapper)
            {
                constraint.features.push_back(wrapper->value.get());
            }

            return;
        }


        OE_SOFT_ASSERT_AND_RETURN(map_srs != nullptr, void());

        const SpatialReference* ecef = map_srs->getGeocentricSRS();
        auto* working_srs = SpatialReference::get("spherical-mercator");

        auto multipolygon = new MultiGeometry();
        auto feature = new Feature(multipolygon, map_srs);

        auto functor = [&](osg::Geometry& geom, unsigned i0, unsigned i1, unsigned i2, const osg::Matrix& xform)
            {
                auto verts = dynamic_cast<osg::Vec3Array*>(geom.getVertexArray());
                osg::Vec3d v[3] = { (*verts)[i0], (*verts)[i1], (*verts)[i2] };

                for (unsigned i = 0; i < 3; ++i)
                {
                    v[i] = v[i] * xform; // to ECEF
                    ecef->transform(v[i], map_srs, v[i]); // to Map SRS
                    
                    // zero out elevation if necessary
                    if (constraint.hasElevation == false)
                        v[i].z() = 0.0;
                }

                auto triangle = std::make_tuple(v[0], v[1], v[2]);
                if (tris.find(triangle) == tris.end())
                {
                    tris.insert(triangle);

                    osg::ref_ptr<osgEarth::Polygon> poly = new osgEarth::Polygon();
                    poly->push_back(v[0]);
                    poly->push_back(v[1]);
                    poly->push_back(v[2]);
                    multipolygon->getComponents().push_back(poly);
                }
            };

        TriangleVisitor visitor(functor);
        node->accept(visitor);

        if (!multipolygon->getComponents().empty())
        {
            constraint.features.emplace_back(feature);
        }
    }
}

void
TerrainConstraintLayer::setFeatureSource(FeatureSource* layer)
{
    if (layer != getFeatureSource())
    {
        bool is_open = isOpen();
        if (is_open) close();
        options().featureSource().setLayer(layer);
        if (is_open) open();
    }
}

FeatureSource*
TerrainConstraintLayer::getFeatureSource() const
{
    return options().featureSource().getLayer();
}

void
TerrainConstraintLayer::setModelLayer(TiledModelLayer* value)
{
    if (value != getModelLayer())
    {
        bool is_open = isOpen();
        if (is_open) close();
        options().model().setLayer(value);
        if (is_open) open();
    }
}

TiledModelLayer*
TerrainConstraintLayer::getModelLayer() const
{
    return options().model().getLayer();
}

void
TerrainConstraintLayer::init()
{
    VisibleLayer::init();

    // open and visible are the same thing for constraint layers layers
    _visibleTiedToOpen = true;
}

Status
TerrainConstraintLayer::openImplementation()
{
    Status parent = VisibleLayer::openImplementation();
    if (parent.isError())
        return parent;

    if (!options().featureSource().isSet() && !options().model().isSet())
    {
        return Status(Status::ConfigurationError, "Missing either features or model constraint source");
    }

    if (options().featureSource().isSet())
    {
        Status fsStatus = options().featureSource().open(getReadOptions());
        if (fsStatus.isError())
            return fsStatus;
    }

    if (options().model().isSet())
    {
        Status modelStatus = options().model().open(getReadOptions());
        if (modelStatus.isError())
            return modelStatus;
    }

    _filterChain = FeatureFilterChain::create(options().filters(), getReadOptions());

    return Status::NoError;
}

const GeoExtent&
TerrainConstraintLayer::getExtent() const
{
    if (getFeatureSource())
        return getFeatureSource()->getExtent();
    else if (options().model().getLayer())
        return options().model().getLayer()->getExtent();
    else
        return Layer::getExtent();
}

void
TerrainConstraintLayer::addedToMap(const Map* map)
{
    VisibleLayer::addedToMap(map);
    options().featureSource().addedToMap(map);
    options().model().addedToMap(map);
    create();
}

void
TerrainConstraintLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);
    options().model().removedFromMap(map);
    VisibleLayer::removedFromMap(map);
}

void
TerrainConstraintLayer::create()
{
    setStatus(Status::OK());

    FeatureSource* fs = getFeatureSource();
    if (fs)
    {
        if (!fs->getFeatureProfile())
        {
            setStatus(Status::ConfigurationError, "Feature source cannot report profile (is it open?)");
        }
        return;
    }

    auto* model = options().model().getLayer();
    if (model)
    {
        return;
    }

    setStatus(Status(Status::ConfigurationError, "No data source available"));
}

void
TerrainConstraintLayer::setRemoveInterior(bool value)
{
    setOptionThatRequiresReopen(options().removeInterior(), value);
}

void
TerrainConstraintLayer::setRemoveExterior(bool value)
{
    setOptionThatRequiresReopen(options().removeExterior(), value);
}

void
TerrainConstraintLayer::setHasElevation(bool value)
{
    setOptionThatRequiresReopen(options().hasElevation(), value);
}

void
TerrainConstraintLayer::setMinLevel(unsigned value)
{
    setOptionThatRequiresReopen(options().minLevel(), value);
}

void
TerrainConstraintLayer::getFeatureConstraint(const TileKey& key, FilterContext* context, MeshConstraint& constraint, ProgressCallback* progress) const
{
    auto cursor = getFeatureSource()->createFeatureCursor(key, _filterChain, context, progress);

    if (cursor.valid() && cursor->hasMore())
    {
        const GeoExtent& keyExtent = key.getExtent();

        while (cursor->hasMore())
        {
            if (progress && progress->isCanceled())
                return;

            Feature* f = cursor->nextFeature();

            if (f && f->getExtent().intersects(keyExtent))
            {
                f->transform(keyExtent.getSRS());
                constraint.features.emplace_back(f);
            }
        }
    }
}

void
TerrainConstraintLayer::getModelConstraint(const TileKey& key, MeshConstraint& constraint, ProgressCallback* progress) const
{
    auto layer = options().model().getLayer();
    OE_SOFT_ASSERT_AND_RETURN(layer, void());

    auto layer_profile = layer->getProfile();
    OE_SOFT_ASSERT_AND_RETURN(layer_profile, void());

    triangle_set_t triangles;

#if 0
    osg::ref_ptr<osg::Node> node = layer->createTile(key, progress);
    if (node.valid())
    {
        addNode(node.get(), key.getProfile()->getSRS(), triangles, constraint);
    }
#else 
    for (int i = -1; i <= 1; ++i)
    {
        for (int j = -1; j <= 1; ++j)
        {
            auto part_key = key.createNeighborKey(i, j);
            osg::ref_ptr<osg::Node> node = layer->createTile(part_key, progress);
            if (node.valid())
            {
                addNode(node.get(), key.getProfile()->getSRS(), triangles, constraint);
            }
        }
    }
#endif
}


MeshConstraint
TerrainConstraintLayer::getConstraint(const TileKey& key, FilterContext* context, ProgressCallback* progress) const
{

    if (!isOpen() || !getVisible() || getMinLevel() > key.getLOD())
        return {};

    const GeoExtent& keyExtent = key.getExtent();
    if (getExtent().isValid() && !getExtent().intersects(keyExtent))
        return {};

    MeshConstraint result;
    result.hasElevation = getHasElevation();
    result.removeExterior = getRemoveExterior();
    result.removeInterior = getRemoveInterior();

#if 0
    // testing :)
    result.hasElevation = true;
    result.removeExterior = false;
    result.removeInterior = true;
    result.cutMesh = false;
    result.flattenMesh = true;
#endif

    if (options().model().getLayer())
    {
        getModelConstraint(key, result, progress);
    }

    else if (getFeatureSource())
    {
        getFeatureConstraint(key, context, result, progress);
    }

    return result;
}



void
TerrainConstraintQuery::setup(const Map* map)
{
    layers.clear();
    if (map)
    {
        map->getOpenLayers(layers);
        session = new Session(map);
    }
}

bool
TerrainConstraintQuery::getConstraints(const TileKey& key, MeshConstraints& output, ProgressCallback* progress) const
{
    output.clear();

    if (!layers.empty())
    {
        const GeoExtent& keyExtent = key.getExtent();

        FilterContext context(session.get());

        for (auto& layer : layers)
        {
            auto constraint = layer->getConstraint(key, &context, progress);
            if (!constraint.features.empty())
            {
                output.push_back(std::move(constraint));
            }
        }
    }

    return !output.empty();
}
