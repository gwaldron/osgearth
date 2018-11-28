/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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
#include <osgEarthFeatures/FeatureMaskLayer>
#include <osgEarthFeatures/FeatureCursor>
#include <osgEarth/Map>
#include <osgEarth/Progress>

using namespace osgEarth;
using namespace osgEarth::Features;

#define LC "[FeatureMaskLayer] "

REGISTER_OSGEARTH_LAYER(featuremask, FeatureMaskLayer);

//........................................................................

void
FeatureMaskLayerOptions::fromConfig(const Config& conf)
{
    conf.get("feature_source", _featureSource);
}

Config
FeatureMaskLayerOptions::getConfig() const
{
    Config conf = MaskLayerOptions::getConfig();
    conf.set("feature_source", _featureSource);
    return conf;
}

//........................................................................

void
FeatureMaskLayer::setFeatureSource(FeatureSource* layer)
{
    if (layer && layer->getStatus().isError())
    {
        setStatus(Status::Error(Status::ResourceUnavailable, "Feature layer is unavailable; check for error"));
        return;
    }

    if (layer)
        OE_INFO << LC << "Feature source layer is \"" << layer->getName() << "\"\n";

    _featureSource = layer;
}

const Status&
FeatureMaskLayer::open()
{
    if (_featureSource.valid())
    {
        _featureSource->setReadOptions(getReadOptions());
        _featureSource->open();
    }
    else
    {
        setStatus(Status(Status::ConfigurationError, "Cannot create feature source"));
    }

    return MaskLayer::open();
}

osg::Vec3dArray*
FeatureMaskLayer::getOrCreateMaskBoundary(float heightScale,
                                          const SpatialReference* srs,
                                          ProgressCallback* progress)
{
    if (!_featureSource.valid())
        return 0L;

    if (!_boundary.valid())
    {
        Threading::ScopedMutexLock lock(_boundaryMutex);
        if (!_boundary.valid())
        {
            osg::ref_ptr<FeatureCursor> cursor = _featureSource->createFeatureCursor(progress);
            if (cursor.valid() && cursor->hasMore())
            {
                Feature* f = cursor->nextFeature();
                if (f && f->getGeometry())
                {
                    f->transform(srs);
                    _boundary = f->getGeometry()->createVec3dArray();
                }
            }
        }
    }

    return _boundary.get();
}

void
FeatureMaskLayer::addedToMap(const Map* map)
{
    OE_DEBUG << LC << "addedToMap\n";

    if (options().featureSource().isSet())
    {
        _featureLayerListener.clear();

        _featureLayerListener.listen(
            map,
            options().featureSource().get(),
            this,
            &FeatureMaskLayer::setFeatureSource);
    }

    create();
}

void
FeatureMaskLayer::removedFromMap(const Map* map)
{
    _featureLayerListener.clear();
}

void
FeatureMaskLayer::create()
{
    if (!_featureSource.valid())
    {
        setStatus(Status(Status::ConfigurationError, "No feature source available"));
        return;
    }

    if (!_featureSource->getFeatureProfile())
    {
        setStatus(Status(Status::ConfigurationError, "Feature source cannot report profile (is it open?)"));
        return;
    }

    setStatus(Status::OK());
}
