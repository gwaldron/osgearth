/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/ScriptFilter>
#include <osgEarth/FilterContext>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(script, ScriptFilter);

#define LC "[ScriptFilter] "

bool
ScriptFilter::isSupported()
{
    return true;
}

ScriptFilter::ScriptFilter() :
ScriptFilterOptions()
{
    ctor();
}

ScriptFilter::ScriptFilter( const Config& conf ):
ScriptFilterOptions( conf )
{
    ctor();
}

void
ScriptFilter::ctor()
{
    _engine = ScriptEngineFactory::create(language().get());
    if (_engine.valid())
    {
        if (profile().isSet())
        {
            _engine->setProfile(profile().get());
        }
    }
}

bool
ScriptFilter::push(Feature* input, FilterContext& context)
{
    bool keep = true;

    if (!input || !input->getGeometry() || !_engine.valid())
        return false;

    ScriptResult result = _engine->run(_expression.get(), input, &context);
    keep = result.asBool();

    return keep;
}


FilterContext
ScriptFilter::push( FeatureList& input, FilterContext& context )
{
    if ( !isSupported() )
    {
        OE_WARN << "ScriptFilter support not enabled" << std::endl;
        return context;
    }

    if (!_engine.valid())
    {
        OE_WARN << "No scripting engine\n";
        return context;
    }

    FeatureList output;

    std::vector<ScriptResult> results;
    results.reserve(input.size());

    if (_engine->run(_expression.get(), input, results, &context) &&
        results.size() == input.size())
    {
        unsigned i = 0;
        for (auto& feature : input)
        {
            if (results[i].asBool() == true)
            {
                output.emplace_back(feature.get());
            }
            ++i;
        }
    }

    input.swap(output);
    return context;
}
