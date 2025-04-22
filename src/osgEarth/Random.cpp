/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/Random>
#include <ctime>
#include <climits>

#define LC "[Random] "

using namespace osgEarth;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

namespace
{
    void fast_rand( unsigned& next )
    {
        // note: this is not a "good" PRNG, but it is good enough for some applications
        // and it is wicked fast.
        //next = next * 1103515245 + 1234;
        next = ((214013*next+2531011));
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
