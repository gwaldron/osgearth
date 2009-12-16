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
#include <osgEarthFeatures/BuildGeometryFilter>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/PolygonOffset>
#include <osgUtil/Tessellator>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Features::Styling;


BuildGeometryFilter::BuildGeometryFilter()
{
    reset();
}

BuildGeometryFilter::BuildGeometryFilter( const StyleClass& styleClass ) :
_styleClass( styleClass )
{
    reset();
}

void
BuildGeometryFilter::reset()
{
    _geode = new osg::Geode();
    _hasLines = false;
    _hasPoints = false;
}

bool
BuildGeometryFilter::push( Feature* input, const FilterContext& context )
{
    if ( !input || !input->getGeometry() )
        return true;

    GeometryIterator parts( input->getGeometry() );
    parts.traversePolygonHoles() = false;
    while( parts.hasMore() )
    {
        Geometry* part = parts.next();
        
        osg::PrimitiveSet::Mode primMode = osg::PrimitiveSet::POINTS;

        Geometry::Type renderType = _geomTypeOverride.isSet() ? _geomTypeOverride.get() : part->getType();

        //osg::notify(osg::NOTICE)
        //    << "[osgEarth] BuildGeomFilter: part type = "
        //    << Geometry::toString( part->getType() ) << ", renderType = "
        //    << Geometry::toString( renderType ) << std::endl;

        switch( renderType )
        {
        case Geometry::TYPE_POINTSET:
            _hasPoints = true;
            primMode = osg::PrimitiveSet::POINTS;
            break;

        case Geometry::TYPE_LINESTRING:
            _hasLines = true;
            primMode = osg::PrimitiveSet::LINE_STRIP;
            break;

        case Geometry::TYPE_RING:
            _hasLines = true;
            primMode = osg::PrimitiveSet::LINE_LOOP;
            break;

        case Geometry::TYPE_POLYGON:
            primMode = osg::PrimitiveSet::LINE_LOOP; // loop will tessellate into polys
            break;
        }

        osg::Vec4ub color;
        if ( renderType == Geometry::TYPE_POLYGON )
        {
            color = _styleClass.get().polygonSymbolizer().fill().color();
            color.a() = (int)(255.0f * _styleClass.get().polygonSymbolizer().fill().opacity());
        }
        else
        {
            color = _styleClass->lineSymbolizer().stroke().color();
            color.a() = (int)(255.0f * _styleClass->lineSymbolizer().stroke().opacity());
        }
    
        osg::Geometry* osgGeom = new osg::Geometry();

        osg::Vec4ubArray* colors = new osg::Vec4ubArray(1);
        (*colors)[0] = color;
        osgGeom->setColorArray( colors );
        osgGeom->setColorBinding( osg::Geometry::BIND_OVERALL );
        
        if ( renderType == Geometry::TYPE_POLYGON && part->getType() == Geometry::TYPE_POLYGON && static_cast<Polygon*>(part)->getHoles().size() > 0 )
        {
            Polygon* poly = static_cast<Polygon*>(part);
            int totalPoints = poly->getTotalPointCount();
            osg::Vec3Array* allPoints = new osg::Vec3Array( totalPoints );
            int offset = 0;
            for( RingCollection::const_iterator h = poly->getHoles().begin(); h != poly->getHoles().end(); ++h )
            {
                Geometry* hole = h->get();
                std::copy( hole->begin(), hole->end(), allPoints->begin() + offset );
                osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, offset, hole->size() ) );
                offset += hole->size();
            }
            osgGeom->setVertexArray( allPoints );
        }
        else
        {
            osgGeom->setVertexArray( part->toVec3Array() );
            osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, 0, part->size() ) );
        }

        // tessellate all polygon geometries. Tessellating each geometry separately
        // with TESS_TYPE_GEOMETRY is much faster than doing the whole bunch together
        // using TESS_TYPE_DRAWABLE.

        if ( renderType == Geometry::TYPE_POLYGON )
        {
            osgUtil::Tessellator tess;
            tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
            tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
            tess.retessellatePolygons( *osgGeom );
        }

        // add the part to the geode.
        _geode->addDrawable( osgGeom );
    }

    return true;
}

FilterContext
BuildGeometryFilter::push( FeatureList& input, osg::ref_ptr<osg::Node>& output, const FilterContext& context )
{
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        if ( !push( i->get(), context ) )
            ok = false;

    if ( ok )
    {
        if ( _styleClass.isSet() && _geode.valid() )
        {
            // could optimize this to only happen is lines or points were created ..
            float size = _styleClass->lineSymbolizer().stroke().width();
            _geode->getOrCreateStateSet()->setAttribute( new osg::Point(size), osg::StateAttribute::ON );

            float width = _styleClass->lineSymbolizer().stroke().width();
            _geode->getOrCreateStateSet()->setAttribute( new osg::LineWidth(width), osg::StateAttribute::ON );
        }

        output = _geode.release();
    }
    else
    {
        output = 0L;
    }

    FilterContext outCx( context );
    return outCx;
}
