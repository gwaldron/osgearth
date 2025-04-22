/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/SimplifyFilter>
#include <osgEarth/FilterContext>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(simplify, SimplifyFilter);

bool
SimplifyFilter::isSupported()
{
    return true;
}

SimplifyFilter::SimplifyFilter(const Config& conf) :
    _options(conf)
{
    //nop
}

FilterContext
SimplifyFilter::push(FeatureList& input, FilterContext& context)
{
    if (!isSupported())
    {
        OE_WARN << "SimplifyFilter support not enabled" << std::endl;
        return context;
    }

    FeatureList output;
    output.reserve(input.size());

    double t = options().tolerance().value();

    if (options().toleranceIsPercentage() == true && context.extent().isSet())
    {
        // 0.01 = percentage to value
        auto w = context.extent()->width() / 2.0;
        t = w * options().tolerance().value() * 0.01;
    }

    for (auto& feature : input)
    {
        if (feature.valid())
        {
            auto geometry = feature->getGeometry();

            if (options().preserveAllFeatures() == false &&
                !feature->getGeometry()->isPointSet() &&
                feature->getGeometry()->getLength() < t)
            {
                continue;
            }

            auto simplifiedGeometry = geometry->simplify(t, options().preserveTopology().value());  
            if (simplifiedGeometry.valid())
            {
                feature->setGeometry(simplifiedGeometry.get());
                output.emplace_back(feature);
            }
        }
    }
    output.swap(input);

    return context;
}
