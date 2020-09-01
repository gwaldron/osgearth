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

#include <osgEarth/FeatureMeshEditLayer>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Map>
#include <osgEarth/Progress>
#include <osgEarth/AltitudeFilter>

using namespace osgEarth;

#define LC "[FeatureMeshEditLayer] "
REGISTER_OSGEARTH_LAYER(terrainconstraint, FeatureMeshEditLayer);
REGISTER_OSGEARTH_LAYER(featuremeshedit, FeatureMeshEditLayer);
REGISTER_OSGEARTH_LAYER(feature_mesh_edit, FeatureMeshEditLayer);

//........................................................................

void
FeatureMeshEditLayer::Options::fromConfig(const Config& conf)
{
    featureSource().get(conf, "features");
    conf.get("remove_interior", removeInterior());
    conf.get("remove_exterior", removeExterior());
    conf.get("has_elevation", hasElevation());
}

Config
FeatureMeshEditLayer::Options::getConfig() const
{
    Config conf = MeshEditLayer::Options::getConfig();
    featureSource().set(conf, "features");
    conf.set("remove_interior", removeInterior());
    conf.set("remove_exterior", removeExterior());
    conf.set("has_elevation", hasElevation());
    return conf;
}

//........................................................................

void
FeatureMeshEditLayer::setFeatureSource(FeatureSource* layer)
{
    options().featureSource().setLayer(layer);
}

FeatureSource*
FeatureMeshEditLayer::getFeatureSource() const
{
    return options().featureSource().getLayer();
}

Status
FeatureMeshEditLayer::openImplementation()
{
    Status parent = MeshEditLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    return Status::NoError;
}

Config
FeatureMeshEditLayer::getConfig() const
{
    Config c = MeshEditLayer::getConfig();
    return c;
}

void
FeatureMeshEditLayer::addedToMap(const Map* map)
{
    OE_DEBUG << LC << "addedToMap\n";
    MeshEditLayer::addedToMap(map);
    options().featureSource().addedToMap(map);
    create();
}

void
FeatureMeshEditLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);
    MeshEditLayer::removedFromMap(map);
}

void
FeatureMeshEditLayer::create()
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
