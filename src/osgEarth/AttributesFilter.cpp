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
        StringTokenizer tok(",");
        tok.tokenize(conf.value(), _attributes);
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

    bool ok = true;
    for (FeatureList::iterator i = input.begin(); i != input.end(); )
    {
        bool passed = false;
        for (auto& a : _attributes)
        {
            if (i->get()->hasAttr(a))
            {
                ++i;
                passed = true;
                break;
            }
        }

        if (!passed)
        {
            i = input.erase(i);
        }
    }

    return context;
}
