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
#include <osgEarth/FeatureCursor>
#include <osgEarth/Map>
#include <osgEarth/Progress>
#include <osgEarth/AltitudeFilter>

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
    VisibleLayer::setVisible(value);
    if (value)
        open();
    else
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
TerrainConstraintLayer::setHasElevation(bool value)
{
    setOptionThatRequiresReopen(options().hasElevation(), value);
}

void
TerrainConstraintLayer::setMinLevel(unsigned value)
{
    setOptionThatRequiresReopen(options().minLevel(), value);
}
