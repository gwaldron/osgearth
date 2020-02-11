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
#include <osgEarth/FeatureMaskLayer>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Map>
#include <osgEarth/Progress>

using namespace osgEarth;

#define LC "[FeatureMaskLayer] "

REGISTER_OSGEARTH_LAYER(featuremask, FeatureMaskLayer);
REGISTER_OSGEARTH_LAYER(feature_mask, FeatureMaskLayer);

//........................................................................

void
FeatureMaskLayer::Options::fromConfig(const Config& conf)
{
    LayerReference<FeatureSource>::get(conf, "features", _featureSourceLayer, _featureSource);
}

Config
FeatureMaskLayer::Options::getConfig() const
{
    Config conf = MaskLayer::Options::getConfig();
    LayerReference<FeatureSource>::set(conf, "features", _featureSourceLayer, _featureSource);
    return conf;
}

//........................................................................

void
FeatureMaskLayer::setFeatureSource(FeatureSource* layer)
{
    _featureSource.setLayer(layer);
}

FeatureSource*
FeatureMaskLayer::getFeatureSource() const
{
    return _featureSource.getLayer();
}

Status
FeatureMaskLayer::openImplementation()
{
    Status parent = MaskLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status fsStatus = _featureSource.open(options().featureSource(), getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    return Status::NoError;
}

osg::Vec3dArray*
FeatureMaskLayer::getOrCreateMaskBoundary(float heightScale,
                                          const SpatialReference* srs,
                                          ProgressCallback* progress)
{
    FeatureSource* fs = getFeatureSource();

    if (fs == NULL)
        return 0L;

    if (!_boundary.valid())
    {
        Threading::ScopedMutexLock lock(_boundaryMutex);
        if (!_boundary.valid())
        {
            osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(progress);
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
    MaskLayer::addedToMap(map);
    _featureSource.connect(map, options().featureSourceLayer());
    create();
}

void
FeatureMaskLayer::removedFromMap(const Map* map)
{
    MaskLayer::removedFromMap(map);
    _featureSource.disconnect(map);
}

void
FeatureMaskLayer::create()
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
