/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthFeatures/Filter>

using namespace osgEarth;
using namespace osgEarthFeatures;

FilterContext::FilterContext()
{
    //NOP
}

FilterContext::FilterContext(const FilterContext& rhs) :
_extent( rhs._extent ),
_profile( rhs._profile )
{
    //NOP
}

FilterChain::FilterChain( const FilterContext& context ) :
_prototypeContext( context )
{
    //NOP
}

osg::Referenced*
FilterChain::process( Feature* feature )
{
    osg::ref_ptr<osg::Referenced> input = feature;
    for( std::list< osg::ref_ptr<Filter> >::iterator i = begin(); i != end(); i++ )
    {
        FilterContext context( _prototypeContext );
        osg::ref_ptr<osg::Referenced> output = i->get()->process( input, context );
        if ( !output.valid() )
            return 0L;
        else
            input = output.get();
    }

    return input.get() == feature? input.get() : input.release();
}
