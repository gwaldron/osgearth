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

using namespace osgEarth;

#define LC "[TerrainConstraintLayer] "
REGISTER_OSGEARTH_LAYER(terrainconstraint, TerrainConstraintLayer);
REGISTER_OSGEARTH_LAYER(featuremask, TerrainConstraintLayer);
REGISTER_OSGEARTH_LAYER(mask, TerrainConstraintLayer);

//........................................................................

void
TerrainConstraintLayer::Options::fromConfig(const Config& conf)
{
    minLevel().setDefault(8u);

    // backwards compatability
    if (ciEquals(conf.key(), "featuremask"))
    {
        removeInterior() = true;
        hasElevation() = true;
    }

    featureSource().get(conf, "features");
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

Status
TerrainConstraintLayer::openImplementation()
{
    Status parent = VisibleLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    _filterchain = FeatureFilterChain::create(
        options().filters(),
        getReadOptions());

    return Status::NoError;
}

const GeoExtent&
TerrainConstraintLayer::getExtent() const
{
    return getFeatureSource() ?
        getFeatureSource()->getExtent() : Layer::getExtent();
}

void
TerrainConstraintLayer::setVisible(bool value)
{
    //VisibleLayer::setVisible(value); // do NOT call base class
    if (value && !isOpen())
        open();
    else if (!value && isOpen())
        close();
}

void
TerrainConstraintLayer::addedToMap(const Map* map)
{
    OE_DEBUG << LC << "addedToMap\n";
    VisibleLayer::addedToMap(map);
    options().featureSource().addedToMap(map);
    create();
}

void
TerrainConstraintLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);
    VisibleLayer::removedFromMap(map);
}

void
TerrainConstraintLayer::create()
{
    FeatureSource* fs = getFeatureSource();

    if (!fs)
    {
        setStatus(Status(Status::ConfigurationError, "No feature source available"));
        return;
    }

    if (!fs->getFeatureProfile())
    {
        setStatus(Status(Status::ConfigurationError, "Feature source cannot report profile (is it open?)"));
        return;
    }

    setStatus(Status::OK());
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
TerrainConstraintQuery::setMap(const Map* map)
{
    _layers.clear();
    if (map)
        map->getOpenLayers(_layers);
}

void
TerrainConstraintQuery::addLayer(TerrainConstraintLayer* layer)
{
    _layers.push_back(layer);
}

bool
TerrainConstraintQuery::getConstraints(
    const TileKey& key,
    std::vector<TerrainConstraint>& output,
    ProgressCallback* progress) const
{
    output.clear();

    if (!_layers.empty())
    {
        const GeoExtent& keyExtent = key.getExtent();

        for (auto& layer : _layers)
        {
            if (!layer->isOpen() || !layer->getVisible())
                continue;

            // not to the min LOD yet?
            if (layer->getMinLevel() > key.getLOD())
                continue;

            // extents don't intersect?
            if (!layer->getExtent().intersects(keyExtent))
                continue;

            // For each feature, check that it intersects the tile key,
            // and then xform it to the correct SRS and clone it for
            // editing.
            FeatureSource* fs = layer->getFeatureSource();
            if (fs)
            {
                osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(
                    key,
                    layer->getFilters(),
                    nullptr,
                    progress);

                if (cursor.valid() && cursor->hasMore())
                {
                    TerrainConstraint constraint;
                    constraint.layer = layer;

                    while (cursor->hasMore())
                    {
                        if (progress && progress->isCanceled())
                            return false;

                        Feature* f = cursor->nextFeature();

                        if (f && f->getExtent().intersects(keyExtent))
                        {
                            f->transform(keyExtent.getSRS());
                            constraint.features.push_back(f);
                        }
                    }

                    output.push_back(std::move(constraint));
                }
            }
        }
    }
    return true;
}
