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
#include <osgEarthSymbology/GeometryRasterizer>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PolygonSymbol>
#include <osgEarthSymbology/AGG.h>

using namespace osgEarth::Symbology;

#define LC "[GeometryRasterizer] "

// --------------------------------------------------------------------------

struct AggState : public osg::Referenced
{
    AggState( osg::Image* image )
        : _rbuf( image->data(), image->s(), image->t(), image->s()*4 ),
          _ren( _rbuf )
    {
        _ras.gamma( 1.3 );
        _ras.filling_rule( agg::fill_even_odd );

        // pre-clear the buffer....
        _ren.clear(agg::rgba8(0,0,0,0));
    }

    agg::rendering_buffer           _rbuf;
    agg::renderer<agg::span_abgr32> _ren;
    agg::rasterizer                 _ras;
};

// --------------------------------------------------------------------------

GeometryRasterizer::GeometryRasterizer( int width, int height, const Style& style ) :
_style( style )
{
    _image = new osg::Image();
    _image->allocateImage( width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE );
    _image->setAllocationMode( osg::Image::USE_NEW_DELETE );
    _state = new AggState( _image.get() );
}

GeometryRasterizer::GeometryRasterizer( osg::Image* image, const Style& style ) :
_image( image ),
_style( style )
{
    _state = new AggState( _image.get() );
}

GeometryRasterizer::~GeometryRasterizer()
{
}

osg::Image*
GeometryRasterizer::finalize()
{
    //convert from ABGR to RGBA
    unsigned char* pixel = _image->data();
    for(int i=0; i<_image->s()*_image->t()*4; i+=4, pixel+=4)
    {
        std::swap( pixel[0], pixel[3] );
        std::swap( pixel[1], pixel[2] );
    }
    osg::Image* result = _image.release();
    _image = 0L;
    return result;
}

void
GeometryRasterizer::draw( const Geometry* geom, const osg::Vec4f& c )
{
    if ( !_image.valid() ) return;

    AggState* state = static_cast<AggState*>( _state.get() );

    osg::Vec4f color = c;
    osg::ref_ptr<const Geometry> geomToRender = geom;

    if ( _style.has<PolygonSymbol>() )
    {
        color = _style.get<const PolygonSymbol>()->fill()->color();
    }
    else
    {
        const LineSymbol* ls = _style.getSymbol<const LineSymbol>();
        float distance = ls ? ls->stroke()->width().value() * 0.5f : 1.0f;
        osg::ref_ptr<Geometry> bufferedGeom;
        if ( !geom->buffer( distance, bufferedGeom ) )
        {
            OE_WARN << LC << "Failed to draw line; buffer op not available" << std::endl;
            return;
        }
        geomToRender = bufferedGeom.get();
        if ( ls )
            color = ls->stroke()->color();
    }

    float a = 127+(color.a()*255)/2; // scale alpha up
    agg::rgba8 fgColor = agg::rgba8( (unsigned int)(color.r()*255), (unsigned int)(color.g()*255), (unsigned int)(color.b()*255), (unsigned int)a );

    ConstGeometryIterator gi( geomToRender.get() );
    while( gi.hasMore() )
    {
        const Vec3dVector& g = gi.next()->asVector();

        for( Vec3dVector::const_iterator p = g.begin(); p != g.end(); p++ )
        {
            const osg::Vec3d& p0 = *p;

            if ( p == g.begin() )
                state->_ras.move_to_d( p0.x(), p0.y() );
            else
                state->_ras.line_to_d( p0.x(), p0.y() );
        }
    }
    state->_ras.render( state->_ren, fgColor );
    state->_ras.reset();
}

