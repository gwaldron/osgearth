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
#include "FilteredFeatureSource"

#define LC "[FilteredFeatureSource] " << getName() << ": "

using namespace osgEarth;

//...................................................................

REGISTER_OSGEARTH_LAYER(filteredfeatures, FilteredFeatureSource);

Config
FilteredFeatureSource::Options::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    featureSource().set(conf, "features");

    if (filters().empty() == false)
    {
        Config temp;
        for (auto& filter : filters())
            temp.add(filter.getConfig());
        conf.set("filters", temp);
    }
    return conf;
}

void
FilteredFeatureSource::Options::fromConfig(const Config& conf)
{
    featureSource().get(conf, "features");

    auto& filters_conf = conf.child("filters");
    for(auto& i : filters_conf.children())
        filters().push_back(ConfigOptions(i));
}

Status FilteredFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    _filters = FeatureFilterChain::create(options().filters(), getReadOptions());

    Status fsStatus = options().featureSource().open(getReadOptions()); 
    return fsStatus;
}

void FilteredFeatureSource::addedToMap(const Map* map)
{
    options().featureSource().addedToMap(map);

    if (!getFeatureSource())
    {
        setStatus(Status(Status::ResourceUnavailable, "Cannot find feature source"));
        return;
    }

    if (!getFeatureSource()->getFeatureProfile())
    {
        setStatus(Status(Status::ConfigurationError, "Feature source does not report a valid profile"));
        return;
    }

    setFeatureProfile(getFeatureSource()->getFeatureProfile());

    FeatureSource::addedToMap(map);
}

void FilteredFeatureSource::removedFromMap(const Map* map)
{    
    options().featureSource().removedFromMap(map);
}

void
FilteredFeatureSource::setFeatureSource(FeatureSource* source)
{
    if (getFeatureSource() != source)
    {
        options().featureSource().setLayer(source);

        if (source && source->getStatus().isError())
        {
            setStatus(source->getStatus());
            return;
        }

        dirty();
    }
}

FeatureSource*
FilteredFeatureSource::getFeatureSource() const
{
    return options().featureSource().getLayer();
}

FeatureCursor*
FilteredFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress) const
{
    if (isOpen() && getFeatureSource())
    {
        osg::ref_ptr<FeatureCursor> cursor = getFeatureSource()->createFeatureCursor(query, {}, nullptr, progress);
        if (cursor.valid())
        {
            FeatureList features;
            cursor->fill(features);

            if (!features.empty())
            {
                FilterContext cx(getFeatureProfile(), query);
                cx = _filters.push(features, cx);
            }

            return new FeatureListCursor(features);
        }
        else
        {
            return nullptr;
        }
    }
    return nullptr;
}
