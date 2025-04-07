/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/AttributesFilter>
#include <osgEarth/FilterContext>
#include <osgEarth/Notify>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(attributes, AttributesFilter)

AttributesFilter::AttributesFilter()
{
}

AttributesFilter::AttributesFilter(const std::vector<std::string>& attributes) :
    _attributes(attributes)
{
}

AttributesFilter::AttributesFilter(const AttributesFilter& rhs) :
    _attributes(rhs._attributes)
{
}

AttributesFilter::AttributesFilter(const Config& conf)
{
    if (conf.key() == "attributes")
    {
        _attributes = StringTokenizer().delim(",").tokenize(conf.value());
    }
}

Config AttributesFilter::getConfig() const
{
    Config config("attributes");

    std::stringstream buf;
    for (unsigned int i = 0; i < _attributes.size(); i++)
    {
        if (i > 0)
        {
            buf << ",";
        }
        buf << _attributes[i];
    }
    config.setValue(buf.str());
    return config;
}

FilterContext
AttributesFilter::push(FeatureList& input, FilterContext& context)
{
    if (!isSupported())
    {
        OE_WARN << "AttributeFilter support not enabled" << std::endl;
        return context;
    }

    FeatureList output;
    output.reserve(input.size());

    bool ok = true;
    for(auto& feature : input)
    {
        bool passed = false;
        if (feature.valid())
        {
            for (auto& a : _attributes)
            {
                if (feature->hasAttr(a))
                {
                    output.emplace_back(feature);
                    break;
                }
            }
        }
    }
    output.swap(input);

    return context;
}
