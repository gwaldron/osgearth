/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/ConvertTypeFilter>

using namespace osgEarth;
using namespace osgEarth::Util;

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(convert, ConvertTypeFilter)


ConvertTypeFilter::ConvertTypeFilter(const Geometry::Type& toType) :
    _toType(toType)
{
    // NOP
}

ConvertTypeFilter::ConvertTypeFilter(const Config& conf)
{
    if (conf.key() == "convert")
    {
        conf.get("type", "point", _toType, Geometry::TYPE_POINTSET);
        conf.get("type", "line", _toType, Geometry::TYPE_LINESTRING);
        conf.get("type", "polygon", _toType, Geometry::TYPE_POLYGON);
    }
}

Config ConvertTypeFilter::getConfig() const
{
    Config config("convert");
    config.set("type", "point", _toType, Geometry::TYPE_POINTSET);
    config.set("type", "line", _toType, Geometry::TYPE_LINESTRING);
    config.set("type", "polygon", _toType, Geometry::TYPE_POLYGON);
    return config;
}

FilterContext
ConvertTypeFilter::push( FeatureList& input, FilterContext& context )
{
    if ( !isSupported() )
    {
        OE_WARN << "ConvertTypeFilter support not enabled" << std::endl;
        return context;
    }

    if (_toType == Geometry::TYPE_UNKNOWN)
    {
        return context;
    }

    bool ok = true;
    for (auto& feature : input)
    {
        if (feature && feature->getGeometry() && feature->getGeometry()->getComponentType() != _toType.value())
        {
            auto cloned = feature->getGeometry()->cloneAs(_toType.value());
            if (cloned)
            {
                OE_SOFT_ASSERT(cloned->isValid());
                feature->setGeometry(cloned);
            }
        }
    }

    return context;
}
