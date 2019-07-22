/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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

REGISTER_OSGEARTH_LAYER(feature_mask, FeatureMaskLayer);

//........................................................................

FeatureMaskLayerOptions::FeatureMaskLayerOptions(const ConfigOptions& options) :
MaskLayerOptions(options)
{
    fromConfig(_conf);
}

void
FeatureMaskLayerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

void
FeatureMaskLayerOptions::fromConfig(const Config& conf)
{
    conf.get("feature_source", _featureSourceLayer);
    conf.get("features", _featureSource);
}

Config
FeatureMaskLayerOptions::getConfig() const
{
    Config conf = MaskLayerOptions::getConfig();
    conf.set("feature_source", _featureSourceLayer);
    conf.set("features", _featureSource);
    return conf;
}

//........................................................................

FeatureMaskLayer::FeatureMaskLayer(const FeatureMaskLayerOptions& options) :
MaskLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

FeatureMaskLayer::FeatureMaskLayer(FeatureMaskLayerOptions* optionsPtr) :
MaskLayer(optionsPtr),
_options(optionsPtr)
{
    //nop - init called from base class
}

FeatureMaskLayer::~FeatureMaskLayer()
{
    //nop
}

void
FeatureMaskLayer::setFeatureSourceLayer(FeatureSourceLayer* layer)
{
    if (layer && layer->getStatus().isError())
    {
        setStatus(Status::Error(Status::ResourceUnavailable, "Feature source layer is unavailable; check for error"));
        return;
    }

    if (layer)
        OE_INFO << LC << "Feature source layer is \"" << layer->getName() << "\"\n";

    setFeatureSource(layer ? layer->getFeatureSource() : 0L);
}

void
FeatureMaskLayer::setFeatureSource(FeatureSource* source)
{
    if (_featureSource != source)
    {
        if (source)
            OE_INFO << LC << "Setting feature source \"" << source->getName() << "\"\n";

        _featureSource = source;

        if (source && source->getStatus().isError())
        {
            setStatus(source->getStatus());
            return;
        }

        create();
    }
}

const Status&
FeatureMaskLayer::open()
{
    if (options().featureSource().isSet())
    {
        FeatureSource* fs = FeatureSourceFactory::create(options().featureSource().get());
        if (fs)
        {
            fs->setReadOptions(getReadOptions());
            fs->open();
            setFeatureSource(fs);
        }
        else
        {
            setStatus(Status(Status::ConfigurationError, "Cannot create feature source"));
        }
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

    if (options().featureSourceLayer().isSet())
    {
        _featureSourceLayerListener.clear();

        _featureSourceLayerListener.listen(
            map,
            options().featureSourceLayer().get(),
            this,
            &FeatureMaskLayer::setFeatureSourceLayer);
    }

    create();
}

void
FeatureMaskLayer::removedFromMap(const Map* map)
{
    _featureSourceLayerListener.clear();
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
