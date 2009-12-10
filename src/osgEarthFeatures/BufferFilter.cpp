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
#  include <geos/geom/Geometry.h>
#  include <geos/geom/GeometryFactory.h>
#  include <geos/operation/buffer/BufferOp.h>
   using namespace geos;
   using namespace geos::operation;
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
_numQuadSegs( 0 ),
_capStyle( Stroke::LINECAP_DEFAULT )
{
    //NOP
}

BufferFilter::BufferFilter( const BufferFilter& rhs ) :
_distance( rhs._distance ),
_numQuadSegs( rhs._numQuadSegs ),
_capStyle( rhs._capStyle )
{
    //NOP
}

bool
BufferFilter::push( Feature* input, const FilterContext& context )
{
    bool ok = false;

#ifdef OSGEARTH_HAVE_GEOS

    FeatureGeometry output;

    geom::Geometry* inGeom = GEOSUtils::importGeometry( input->getGeometry(), context.profile() );
    if ( inGeom )
    {
        int geosCapStyle = 
            //buffer::BufferOp::BufferParameters::EndCapStyle geosEndCap =
            _capStyle == Stroke::LINECAP_ROUND  ? buffer::BufferOp::CAP_ROUND : //buffer::BufferParameters::CAP_ROUND :
            _capStyle == Stroke::LINECAP_SQUARE ? buffer::BufferOp::CAP_SQUARE : //buffer::BufferParameters::CAP_SQUARE :
            _capStyle == Stroke::LINECAP_BUTT   ? buffer::BufferOp::CAP_BUTT : //buffer::BufferParameters::CAP_BUTT :
            buffer::BufferOp::CAP_ROUND;
            //buffer::BufferParameters::CAP_ROUND;

        int geosQuadSegs = _numQuadSegs > 0 ? _numQuadSegs : buffer::OffsetCurveBuilder::DEFAULT_QUADRANT_SEGMENTS;

        geom::Geometry* outGeom = buffer::BufferOp::bufferOp(
            inGeom,
            _distance,
            geosQuadSegs,
            geosCapStyle );

        if ( outGeom )
        {
            GEOSUtils::exportGeometry( outGeom, output, context.profile() );
            outGeom->getFactory()->destroyGeometry( outGeom );
            ok = true;
        }
        else
        {
            osg::notify(osg::NOTICE) << "[osgEarth] Buffer: no output geometry.." << std::endl;
            ok = false;
        }

        inGeom->getFactory()->destroyGeometry( inGeom );
    }
    else
    {
        osg::notify(osg::NOTICE) << "[osgEarth] Buffer: importGeom failed" << std::endl;
        ok = false;
    }

    if ( ok )
    {
        input->setGeometry( output );

        if ( context.profile()->getGeometryType() == FeatureProfile::GEOM_POLYGON )
        {   
            output.normalizePolygon();
        }
    }

#endif
    
    return ok;
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
