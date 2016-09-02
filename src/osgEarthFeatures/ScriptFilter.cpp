/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarthFeatures/ScriptFilter>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

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

    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); )
    {
        if ( push( i->get(), context ) )
        {
            ++i;
        }
        else
        {
            i = input.erase(i);
        }
    }

    return context;
}
