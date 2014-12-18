/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include "CoverageLegend"
#include <osgEarth/Config>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#define LC "[CoverageLegend] "

//............................................................................

CoverageLegend::CoverageLegend()
{
    //nop
}

void
CoverageLegend::fromConfig(const Config& conf)
{
    conf.getIfSet("name",   _name);
    conf.getIfSet("source", _source);

    ConfigSet predicatesConf = conf.child("mappings").children();
    for(ConfigSet::const_iterator i = predicatesConf.begin(); i != predicatesConf.end(); ++i)
    {
        osg::ref_ptr<CoverageValuePredicate> p = new CoverageValuePredicate();

        i->getIfSet( "name",  p->_description );
        i->getIfSet( "value", p->_exactValue );
        i->getIfSet( "class", p->_mappedClassName );
        
        if ( p->_mappedClassName.isSet() )
        {
            _predicates.push_back( p.get() );
        }
    }
}

Config
CoverageLegend::getConfig() const
{
    Config conf;
    
    conf.addIfSet("name",   _name);
    conf.addIfSet("source", _source);

    Config preds;
    for(Predicates::const_iterator i = _predicates.begin(); i != _predicates.end(); ++i)
    {
        CoverageValuePredicate* p = i->get();
        Config pred;
        pred.addIfSet( "name",  p->_description );
        pred.addIfSet( "value", p->_exactValue );
        pred.addIfSet( "class", p->_mappedClassName );
        preds.add(pred);
    }
    conf.add( "mappings", preds );

    return conf;
}
