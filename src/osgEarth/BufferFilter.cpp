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
#include <osgEarth/BufferFilter>
#include <osgEarth/FilterContext>

#define LC "[BufferFilter] "

using namespace osgEarth;

bool
BufferFilter::isSupported()
{
#ifdef OSGEARTH_HAVE_GEOS
    static bool s_isSupported = true;
#else
    static bool s_isSupported = false;
#endif

    return s_isSupported;
}

#define ASSERT_SUPPORT() \
    if ( !BufferFilter::isSupported() ) { \
        OE_NOTICE << "BufferFilter NOT SUPPORTED - please compile osgEarth with GEOS" << std::endl; }

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(buffer, BufferFilter );

BufferFilter::BufferFilter() :
_distance   ( 1.0 ),
_numQuadSegs( 0 ),
_capStyle   ( Stroke::LINECAP_SQUARE )
{
    //NOP
}

BufferFilter::BufferFilter( const BufferFilter& rhs ) :
_distance   ( rhs._distance ),
_numQuadSegs( rhs._numQuadSegs ),
_capStyle   ( rhs._capStyle )
{
    //NOP
}

BufferFilter::BufferFilter( const Config& conf ) :
_distance   ( 1.0 ),
_numQuadSegs( 0 ),
_capStyle   ( Stroke::LINECAP_SQUARE )
{
    if (conf.key() == "buffer")
    {
        conf.get( "distance", _distance );
    }
}

Config BufferFilter::getConfig() const
{
    Config config( "buffer" );
    config.set( "distance", _distance);
    return config;
}

FilterContext
BufferFilter::push( FeatureList& input, FilterContext& context )
{
    if ( !isSupported() )
    {
        OE_WARN << "BufferFilter support not enabled - please compile osgEarth with GEOS" << std::endl;
        return context;
    }

    //OE_NOTICE << "Buffer: input = " << input.size() << " features" << std::endl;
    for( FeatureList::iterator i = input.begin(); i != input.end(); )
    {
        Feature* feature = i->get();
        if ( !feature || !feature->getGeometry() )
            continue;

        osg::ref_ptr<Geometry> output;

        BufferParameters params;
        
        params._capStyle =
                _capStyle == Stroke::LINECAP_ROUND  ? BufferParameters::CAP_ROUND :
                _capStyle == Stroke::LINECAP_SQUARE ? BufferParameters::CAP_SQUARE :
                _capStyle == Stroke::LINECAP_FLAT   ? BufferParameters::CAP_FLAT :
                                                      BufferParameters::CAP_SQUARE;

        params._cornerSegs = _numQuadSegs;

        if ( feature->getGeometry()->buffer( _distance.value(), output, params ) )
        {
            feature->setGeometry( output.get() );
            ++i;
        }
        else
        {
            i = input.erase( i );
            OE_DEBUG << LC << "feature " << feature->getFID() << " yielded no geometry" << std::endl;
        }
    }

    return context;
}
