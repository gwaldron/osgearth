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
#include <osgEarthFeatures/FeatureSourceLayer>

#define LC "[FeatureSourceLayer] " << getName() << ": "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

namespace osgEarth {
    namespace Features {
        REGISTER_OSGEARTH_LAYER(feature_source, FeatureSourceLayer);
    }
}

FeatureSourceLayer::FeatureSourceLayer() :
Layer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

FeatureSourceLayer::FeatureSourceLayer(const FeatureSourceLayerOptions& inOptions) :
Layer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(inOptions)
{
    init();
}

FeatureSourceLayer::FeatureSourceLayer(FeatureSourceLayerOptions* optionsPtr) :
Layer(optionsPtr? optionsPtr : &_optionsConcrete),
_options(optionsPtr? optionsPtr : &_optionsConcrete)
{
    // init() will be called by base class
}

void
FeatureSourceLayer::setFeatureSource(FeatureSource* value)
{
    _featureSource = value;
}

FeatureSource*
FeatureSourceLayer::getFeatureSource() const
{ 
    return _featureSource.get();
}

/**
 * Open the feature source set this layer's status to its status.
 */
const Status& 
FeatureSourceLayer::open()
{
    if (!_featureSource.valid())
    {
        _featureSource = FeatureSourceFactory::create(options());
        if (!_featureSource.valid())
            return setStatus(Status::Error(Status::ServiceUnavailable, "Unable to create feature source"));
    }

    Status fsStatus = _featureSource->open(getReadOptions());
    if (fsStatus.isError())
    {
        setStatus(fsStatus);
        _featureSource = 0L;
    }
    else
    {
        OE_INFO << LC << "Opened feature source OK.\n";
    }

    return getStatus();
}
