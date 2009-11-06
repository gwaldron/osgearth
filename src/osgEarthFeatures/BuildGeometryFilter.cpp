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
using namespace osgEarthFeatures;
using namespace osgEarthFeatures::Styling;


BuildGeometryFilter::BuildGeometryFilter()
{
    reset();
}

BuildGeometryFilter::BuildGeometryFilter( const Rule& rule ) :
_styleRule( rule )
{
    reset();
}

void
BuildGeometryFilter::setStyleRule( const Rule& rule )
{
    _styleRule = rule;
}

void
BuildGeometryFilter::reset()
{
    _geode = new osg::Geode();
}

bool
BuildGeometryFilter::push( Feature* input, FilterContext& context )
{
    FeatureProfile::GeometryType geomType = context._profile->getGeometryType();
    GLenum primType =
        geomType == FeatureProfile::GEOM_LINE ? GL_LINE_STRIP :
        geomType == FeatureProfile::GEOM_POINT ? GL_POINTS :
        geomType == FeatureProfile::GEOM_POLYGON ? GL_LINE_LOOP : // for later tessellation
        GL_POINTS;

    osg::Vec4ub color;
    
    if (geomType == FeatureProfile::GEOM_POLYGON)
    {
        color = _styleRule.polygonSymbolizer().fill().color();
        color.a() = (int)(255.0f * _styleRule.polygonSymbolizer().fill().opacity());
    }
    else
    {
        color = _styleRule.lineSymbolizer().stroke().color();
        color.a() = (int)(255.0f * _styleRule.lineSymbolizer().stroke().opacity());
    }

    osg::Geometry* geom = new osg::Geometry();
    osg::Vec4ubArray* colors = new osg::Vec4ubArray(1);
    (*colors)[0] = color;
    geom->setColorArray( colors );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    osg::Vec3Array* allverts = new osg::Vec3Array();
    geom->setVertexArray( allverts );

    for( int p=0, partPtr=0; p<input->getNumParts(); p++ )
    {                
        osg::Vec3dArray* part = input->getPart( p );
        allverts->reserve( allverts->size() + part->size() );
        for( int v=0; v<part->size(); v++ )
            allverts->push_back( (*part)[v] );
        geom->addPrimitiveSet( new osg::DrawArrays( primType, partPtr, part->size() ) );
        partPtr += part->size();
    }

    // tessellate all polygon geometries. Tessellating each geometry separately
    // with TESS_TYPE_GEOMETRY is much faster than doing the whole bunch together
    // using TESS_TYPE_DRAWABLE.
    if ( geomType == FeatureProfile::GEOM_POLYGON )
    {
        osgUtil::Tessellator tess;
        tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
        tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
        tess.retessellatePolygons( *geom );
    }

    _geode->addDrawable( geom );

    return true;
}

bool
BuildGeometryFilter::push( FeatureList& input, FilterContext& context )
{
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        if ( !push( i->get(), context ) )
            ok = false;
    return ok;
}

osg::Node*
BuildGeometryFilter::getOutput( FilterContext& context )
{
    FeatureProfile::GeometryType geomType = context._profile->getGeometryType();

    if ( geomType == FeatureProfile::GEOM_POLYGON )
    {
        //NOP
    }
    else if ( geomType == FeatureProfile::GEOM_POINT )
    {
        float size = _styleRule.lineSymbolizer().stroke().width();
        _geode->getOrCreateStateSet()->setAttribute( new osg::Point(size), osg::StateAttribute::ON );
    }
    else // GEOM_LINE
    {
        float width = _styleRule.lineSymbolizer().stroke().width();
        _geode->getOrCreateStateSet()->setAttribute( new osg::LineWidth(width), osg::StateAttribute::ON );
    }

    return _geode.get();
}
