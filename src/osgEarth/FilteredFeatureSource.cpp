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
#include <osgEarth/FilteredFeatureSource>

#define LC "[FilteredFeatureSource] " << getName() << ": "

using namespace osgEarth;

//...................................................................

REGISTER_OSGEARTH_LAYER(filteredfeatures, FilteredFeatureSource);

Config
FilteredFeatureSource::Options::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    featureSource().set(conf, "features");
    return conf;
}

void
FilteredFeatureSource::Options::fromConfig(const Config& conf)
{
    featureSource().get(conf, "features");    
}

Status FilteredFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    Status fsStatus = options().featureSource().open(getReadOptions()); 
    return fsStatus;
}

void FilteredFeatureSource::addedToMap(const Map* map)
{
    options().featureSource().addedToMap(map);
    if (getFeatureSource())
    {
        setFeatureProfile(getFeatureSource()->getFeatureProfile());
    }    
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

FeatureCursorImplementation*
FilteredFeatureSource::createFeatureCursorImplementation(
    const Query& query,
    ProgressCallback* progress) const
{
    if (getFeatureSource())
    {
        auto impl = getFeatureSource()->createFeatureCursorImplementation(query, progress);
        if (impl && !_filters.empty())
        {
            FeatureCursor temp(impl);
            return new FilteredFeatureCursorImpl(temp, _filters, nullptr);
        }
        else return impl;
    }
    return nullptr;
}