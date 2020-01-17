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
#include "SplatCoverageLegend"
#include <osgEarth/Config>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[SplatCoverageLegend] "

//............................................................................

SplatCoverageLegend::SplatCoverageLegend()
{
    //nop
}

bool
SplatCoverageLegend::getPredicatesForClass(const std::string& className,
                                           std::vector<const CoverageValuePredicate*>& output) const
{
    output.clear();
    for(Predicates::const_iterator i = _predicates.begin(); i != _predicates.end(); ++i)
    {
        if ( i->get()->_mappedClassName == className )
        {
            output.push_back( i->get() );
        }
    }
    return output.size() > 0;
}

void
SplatCoverageLegend::fromConfig(const Config& conf)
{
    conf.get("name",   _name);
    conf.get("source", _source);

    ConfigSet predicatesConf = conf.child("mappings").children();
    for(ConfigSet::const_iterator i = predicatesConf.begin(); i != predicatesConf.end(); ++i)
    {
        osg::ref_ptr<CoverageValuePredicate> p = new CoverageValuePredicate();

        i->get( "name",  p->_description );
        i->get( "value", p->_exactValue );
        i->get( "class", p->_mappedClassName );
        
        if ( p->_mappedClassName.isSet() )
        {
            _predicates.push_back( p.get() );
        }
    }
}

Config
SplatCoverageLegend::getConfig() const
{
    Config conf;
    
    conf.set("name",   _name);
    conf.set("source", _source);

    Config preds;
    for(Predicates::const_iterator i = _predicates.begin(); i != _predicates.end(); ++i)
    {
        CoverageValuePredicate* p = i->get();
        Config pred;
        pred.set( "name",  p->_description );
        pred.set( "value", p->_exactValue );
        pred.set( "class", p->_mappedClassName );

        preds.add( "mapping", pred );
    }
    conf.set( "mappings", preds );

    return conf;
}

