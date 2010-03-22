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
#include <osgEarthFeatures2/BufferFilter>

#ifdef OSGEARTH_HAVE_GEOS
#  include <osgEarthFeatures2/GEOS>
#  include <geos/geom/Geometry.h>
#  include <geos/geom/GeometryFactory.h>
#  include <geos/operation/buffer/BufferOp.h>
   using namespace geos;
   using namespace geos::operation;
#endif

using namespace osgEarth;
using namespace osgEarth::Features2;
using namespace osgEarth::Symbology;

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
    if ( !BufferFilter::isSupported() ) { OE_NOTICE << "BufferFilter NOT SUPPORTED" << std::endl; }

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

    if ( !input || !input->getGeometry() )
        return true;

    osg::ref_ptr<Symbology::Geometry> output;

#ifdef OSGEARTH_HAVE_GEOS   

    geom::Geometry* inGeom = GEOSUtils::importGeometry( input->getGeometry() );
    if ( inGeom )
    {
        buffer::BufferParameters::EndCapStyle geosEndCap =
            _capStyle == Stroke::LINECAP_ROUND  ? buffer::BufferParameters::CAP_ROUND :
            _capStyle == Stroke::LINECAP_SQUARE ? buffer::BufferParameters::CAP_SQUARE :
            _capStyle == Stroke::LINECAP_BUTT   ? buffer::BufferParameters::CAP_FLAT :
            buffer::BufferParameters::CAP_SQUARE;

        //buffer::BufferParameters::JoinStyle geosJoin =
        //    _joinStyle

        int geosQuadSegs = _numQuadSegs > 0 
            ? _numQuadSegs
            : buffer::BufferParameters::DEFAULT_QUADRANT_SEGMENTS;

        geom::Geometry* outGeom = buffer::BufferOp::bufferOp(
            inGeom,
            _distance.value(),
            geosQuadSegs,
            geosEndCap );

        if ( outGeom )
        {
            output = GEOSUtils::exportGeometry( outGeom );
            outGeom->getFactory()->destroyGeometry( outGeom );
        }
        else
        {
            OE_NOTICE << "Buffer: no output geometry.." << std::endl;
        }

        inGeom->getFactory()->destroyGeometry( inGeom );
    }
    else
    {
        OE_NOTICE << "Buffer: importGeom failed" << std::endl;
    }

    if ( output.valid() )
    {
        input->setGeometry( output.get() );

        //if ( context.profile()->getGeometryType() == FeatureProfile::GEOM_POLYGON )
        //{   
        //    output.normalizePolygon();
        //}
    }

#endif // OSGEARTH_HAVE_GEOS
    return output.valid();
}

FilterContext
BufferFilter::push( FeatureList& input, const FilterContext& context )
{
    if ( !isSupported() )
    {
        OE_NOTICE << "BufferFilter support not enabled" << std::endl;
        return context;
    }

    //OE_NOTICE << "Buffer: input = " << input.size() << " features" << std::endl;
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); ++i )
        if ( !push( i->get(), context ) )
            ok = false;

    // construct a new context -- output of buffering is always polygons.
    //FilterContext outCx( context );

    //outCx.profile() = new FeatureProfile(
    //    context.profile()->getSRS(),
    //    FeatureProfile::GEOM_POLYGON,
    //    context.profile()->getDimensionality(),
    //    context.profile()->isMultiGeometry() );

    return context;
}
