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

FeatureCursor* FilteredFeatureSource::createFeatureCursorImplementation(
    const Query& query,
    ProgressCallback* progress)
{
    if (getFeatureSource())
    {
        osg::ref_ptr< FeatureCursor > cursor = getFeatureSource()->createFeatureCursor(query, progress);
        if (cursor.valid())
        {
            if (_filters.valid())
            {
                FilterContext* cx = new FilterContext;
                cx->setProfile(getFeatureProfile());
                if (query.tileKey().isSet())
                {
                    cx->extent() = query.tileKey()->getExtent();
                }
                else if (query.bounds().isSet())
                {
                    cx->extent() = GeoExtent(getFeatureProfile()->getSRS(), query.bounds().get());
                }
                else
                {
                    cx->extent() = getFeatureProfile()->getExtent();
                }
                return new FilteredFeatureCursor(cursor.get(), _filters, cx, true);
            }
            else
            {
                return cursor.release();
            }
        }             
    }
    return nullptr;
}