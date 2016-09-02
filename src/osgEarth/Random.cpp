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
#include <osgEarth/Random>
#include <time.h>
#include <limits.h>

#define LC "[Random] "

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    void fast_rand( unsigned& next )
    {
        // note: this is not a "good" PRNG, but it is good enough for some applications
        // and it is wicked fast.
        next = next * 1103515245 + 12345;
    }
}

//------------------------------------------------------------------------

Random::Random( Random::Method method ) :
_method( method ),
_seed  ( (unsigned)::time(0L) )
{
    _next = _seed;
}

Random::Random( unsigned seed, Random::Method method ) :
_method( method ),
_seed  ( seed )
{
    _next = _seed;
}

Random::Random( const Random& rhs ) :
_method( rhs._method ),
_seed  ( rhs._seed ),
_next  ( rhs._next )
{
    //nop
}

void
Random::seed(unsigned value)
{
    _seed = value;
    reset();
}

void
Random::reset()
{
    _next = _seed;
}

unsigned
Random::next( unsigned mod )
{
    if ( _method == METHOD_FAST )
    {
        fast_rand( _next );
    }    
    return mod == UINT_MAX ? _next : _next % mod;
}

double
Random::next()
{
    return (double)next(UINT_MAX) / (double)UINT_MAX;
}
