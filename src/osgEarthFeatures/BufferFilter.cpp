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
#include <osgEarthFeatures/BufferFilter>

#ifdef OSGEARTH_HAVE_GEOS
#  include <osgEarthFeatures/GEOS>
#endif

using namespace osgEarth;
using namespace osgEarth::Features;

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
    if ( !BufferFilter::isSupported() ) { osg::notify(osg::NOTICE) << "[osgEarth] BufferFilter NOT SUPPORTED" << std::endl; }

BufferFilter::BufferFilter() :
_distance( 1.0 ),
_numSegsPer90deg( 4 )
{
    //NOP
}

BufferFilter::BufferFilter( double distance, int numSegsPer90deg ) :
_distance( distance ),
_numSegsPer90deg( _numSegsPer90deg )
{
    // NOP
}

BufferFilter::BufferFilter( const BufferFilter& rhs ) :
_distance( rhs._distance ),
_numSegsPer90deg( rhs._numSegsPer90deg )
{
    //NOP
}

bool
BufferFilter::push( Feature* input, const FilterContext& context )
{
    bool success = false;

#ifdef OSGEARTH_HAVE_GEOS

    FeatureGeometry output;
    bool ok = GEOSUtils::buffer( _distance, input->getGeometry(), output, context.profile() );
    if ( ok )
    {
        input->setGeometry( output );
    }
    else
    {
        osg::notify(osg::NOTICE) << "[osgEarth] Buffer: op failed for FID " << input->getFID() << std::endl;
    }
    
    success = ok;

#else // OSGEARTH_HAVE_GEOS

    double tolerance = context.profile()->getSRS()->isGeographic() ? 0.000001 : 0.001;

    if ( context.profile()->getGeometryType() == FeatureProfile::GEOM_LINE )
    {
        FeatureGeometry output;
//        bufferLinesToPolygons( input->getGeometry(), _distance, tolerance, output );
        input->setGeometry( output );
        success = true;
    }
    else if ( context.profile()->getGeometryType() == FeatureProfile::GEOM_POLYGON )
    {
        FeatureGeometry output;
        bufferPolys( input->getGeometry(), _distance, tolerance, output );
        input->setGeometry( output );
        success = true;
    }

#endif
    
    return success;
}

FilterContext
BufferFilter::push( FeatureList& input, const FilterContext& context )
{
    if ( !isSupported() )
    {
        osg::notify(osg::NOTICE) << "[osgEarth] BufferFilter support not enabled" << std::endl;
        return context;
    }

    //osg::notify(osg::NOTICE) << "[osgEarth] Buffer: input = " << input.size() << " features" << std::endl;
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); ++i )
        if ( !push( i->get(), context ) )
            ok = false;

    // construct a new context -- output of buffering is always polygons.
    FilterContext outCx( context );

    outCx.profile() = new FeatureProfile(
        context.profile()->getSRS(),
        FeatureProfile::GEOM_POLYGON,
        context.profile()->getDimensionality(),
        context.profile()->isMultiGeometry() );

    return context;
}
