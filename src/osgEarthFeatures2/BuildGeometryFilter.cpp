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
#include <osgEarthFeatures2/BuildGeometryFilter>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/ClusterCullingCallback>
#include <osgText/Text>
#include <osgUtil/Tessellator>

using namespace osgEarth;
using namespace osgEarth::Features2;
using namespace osgEarth::Symbology;


BuildGeometryFilter::BuildGeometryFilter() :
_style( new Style() ),
_geomTypeOverride( Symbology::Geometry::TYPE_UNKNOWN )
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
BuildGeometryFilter::pushTextAnnotation( TextAnnotation* anno, const FilterContext& context )
{
    // find the centroid
    osg::Vec3d centroid = anno->getGeometry()->getBounds().center();

    osgText::Text* t = new osgText::Text();
    t->setText( anno->text() );
    t->setFont( "fonts/arial.ttf" );
    t->setAutoRotateToScreen( true );
    t->setCharacterSizeMode( osgText::TextBase::SCREEN_COORDS );
    t->setCharacterSize( 32.0f );
    //t->setCharacterSizeMode( osgText::TextBase::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT );
    //t->setCharacterSize( 300000.0f );
    t->setPosition( centroid );
    t->setAlignment( osgText::TextBase::CENTER_CENTER );
    t->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS), osg::StateAttribute::ON );
    t->getOrCreateStateSet()->setRenderBinDetails( 99999, "RenderBin" );

    // apply styling as appropriate:
    osg::Vec4f textColor(1,1,1,1);
    osg::Vec4f haloColor(0,0,0,1);

    TextSymbol* textSymbolizer = getStyle()->getSymbol<TextSymbol>();
    if ( textSymbolizer )
    {
        textColor = textSymbolizer->fill()->color();
        if ( textSymbolizer->halo().isSet() )
        {
            haloColor = textSymbolizer->halo()->color();
        }
    }

    t->setColor( textColor );
    t->setBackdropColor( haloColor );
    t->setBackdropType( osgText::Text::OUTLINE );

    if ( context.isGeocentric() )
    {
        // install a cluster culler: note that the CCC control point and normal must be
        // in world coordinates
        const osg::EllipsoidModel* ellip = context.profile()->getSRS()->getEllipsoid();
        osg::Vec3d cp = centroid * context.inverseReferenceFrame();
        osg::Vec3d normal = ellip->computeLocalUpVector( cp.x(), cp.y(), cp.z() );
        osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback( cp, normal, 0.0f );
        t->setCullCallback( ccc );
    }

    _geode->addDrawable( t );

    return true;    
}

bool
BuildGeometryFilter::pushRegularFeature( Feature* input, const FilterContext& context )
{
    GeometryIterator parts( input->getGeometry() );
    parts.traversePolygonHoles() = false;
    while( parts.hasMore() )
    {
        Geometry* part = parts.next();
        
        osg::PrimitiveSet::Mode primMode = osg::PrimitiveSet::POINTS;

        Geometry::Type renderType = _geomTypeOverride.isSet() ? _geomTypeOverride.get() : part->getType();

        //OE_NOTICE
        //    << "BuildGeomFilter: part type = "
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

        // Cedric Pinson : how we should fix that ????
        //osg::Vec4f color = _style->getColor( renderType );
        osg::Vec4f color = osg::Vec4(1,0,1,1);
    
        osg::Geometry* osgGeom = new osg::Geometry();

        osg::Vec4Array* colors = new osg::Vec4Array(1);
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

bool
BuildGeometryFilter::push( Feature* input, const FilterContext& context )
{
    if ( !input || !input->getGeometry() )
        return true;
    else if ( dynamic_cast<TextAnnotation*>(input) )
        return pushTextAnnotation( static_cast<TextAnnotation*>(input), context );
    else
        return pushRegularFeature( input, context );
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
        if ( _style.valid() && _geode.valid() )
        {
            // could optimize this to only happen is lines or points were created ..
            LineSymbol* lineSymbol = _style->getSymbol<LineSymbol>();
            float size = 1.0;
            if (lineSymbol)
                size = lineSymbol->stroke()->width().value();
            _geode->getOrCreateStateSet()->setAttribute( new osg::Point(size), osg::StateAttribute::ON );
            _geode->getOrCreateStateSet()->setAttribute( new osg::LineWidth(size), osg::StateAttribute::ON );
        }

        output = _geode.release();

        if ( context.hasReferenceFrame() )
        {
            osg::MatrixTransform* delocalizer = new osg::MatrixTransform(
                context.inverseReferenceFrame() );
            delocalizer->addChild( output.get() );
            output = delocalizer;
        }
    }
    else
    {
        output = 0L;
    }

    FilterContext outCx( context );
    outCx.setReferenceFrame( osg::Matrixd::identity() ); // clear the ref frame.
    return outCx;
}
