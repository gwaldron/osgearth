/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthSymbology/TextSymbol>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PolygonSymbol>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthSymbology/MeshConsolidator>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/LineStipple>
#include <osg/Point>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/ClusterCullingCallback>
#include <osgText/Text>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>
#include <osgDB/WriteFile>
#include <osg/Version>

#define LC "[BuildGeometryFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

BuildGeometryFilter::BuildGeometryFilter( const Style& style ) :
_style( style ),
_geomTypeOverride( Symbology::Geometry::TYPE_UNKNOWN ),
_maxAngle_deg( 5.0 ),
_mergeGeometry( false )
{
    reset();
}

void
BuildGeometryFilter::reset()
{
    _result = 0L;
    _geode = new osg::Geode();
    _hasLines = false;
    _hasPoints = false;
    _hasPolygons = false;
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

    const TextSymbol* textSymbolizer = getStyle().getSymbol<TextSymbol>();
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
    GeometryIterator parts( input->getGeometry(), false );
    while( parts.hasMore() )
    {
        Geometry* part = parts.next();
        
        osg::PrimitiveSet::Mode primMode = osg::PrimitiveSet::POINTS;

        const Style& myStyle = input->style().isSet() ? *input->style() : _style;
        //const Style* myStyle = input->style().isSet() ? input->style()->get() : _style.get();

        osg::Vec4f color = osg::Vec4(1,1,1,1);
        bool tessellatePolys = true;

        bool setWidth = input->style().isSet(); // otherwise it will be set globally, we assume
        float width = 1.0f;

        switch( part->getType() )
        {
        case Geometry::TYPE_POINTSET:
            {
                _hasPoints = true;
                primMode = osg::PrimitiveSet::POINTS;
                const PointSymbol* point = myStyle.getSymbol<PointSymbol>();
                if (point)
                {
                    color = point->fill()->color();
                }
            }
            break;

        case Geometry::TYPE_LINESTRING:
            {
                _hasLines = true;
                primMode = osg::PrimitiveSet::LINE_STRIP;
                const LineSymbol* lineSymbol = myStyle.getSymbol<LineSymbol>();
                if (lineSymbol)
                {
                    color = lineSymbol->stroke()->color();
                    width = lineSymbol->stroke()->width().isSet() ? *lineSymbol->stroke()->width() : 1.0f;
                }
            }
            break;

        case Geometry::TYPE_RING:
            {
                _hasLines = true;
                primMode = osg::PrimitiveSet::LINE_LOOP;
                const LineSymbol* lineSymbol = myStyle.getSymbol<LineSymbol>();
                if (lineSymbol)
                {
                    color = lineSymbol->stroke()->color();
                    width = lineSymbol->stroke()->width().isSet() ? *lineSymbol->stroke()->width() : 1.0f;
                }
            }
            break;

        case Geometry::TYPE_POLYGON:
            {
                primMode = osg::PrimitiveSet::LINE_LOOP; // loop will tessellate into polys
                const PolygonSymbol* poly = myStyle.getSymbol<PolygonSymbol>();
                if (poly)
                {
                    _hasPolygons = true;
                    color = poly->fill()->color();
                }
                else
                {
                    // if we have a line symbol and no polygon symbol, draw as an outline.
                    _hasLines = true;
                    const LineSymbol* line = myStyle.getSymbol<LineSymbol>();
                    if ( line )
                    {
                        color = line->stroke()->color();
                        width = line->stroke()->width().isSet() ? *line->stroke()->width() : 1.0f;
                        tessellatePolys = false;
                    }
                }
            }
            break;
		default:
			break;
        }
        
        osg::Geometry* osgGeom = new osg::Geometry();

        if ( _featureNameExpr.isSet() )
        {
            const std::string& name = input->eval( _featureNameExpr.mutable_value() );
            osgGeom->setName( name );
        }

        // NOTE: benchmarking reveals VBOs to be much slower (for static data, at least)
        //osgGeom->setUseVertexBufferObjects( true );
        //osgGeom->setUseDisplayList( false );

        if ( setWidth && width != 1.0f )
        {
            osgGeom->getOrCreateStateSet()->setAttributeAndModes(
                new osg::LineWidth( width ), osg::StateAttribute::ON );
        }

        if (_hasLines)
        {
            const LineSymbol* line = myStyle.getSymbol<LineSymbol>();
            if (line && line->stroke().isSet() && line->stroke()->stipple().isSet())
            {
                osg::LineStipple* lineStipple = new osg::LineStipple;
                lineStipple->setPattern( *line->stroke()->stipple() );            
                osgGeom->getOrCreateStateSet()->setAttributeAndModes( lineStipple, osg::StateAttribute::ON );
            }
        }
        
        if (part->getType() == Geometry::TYPE_POLYGON && static_cast<Polygon*>(part)->getHoles().size() > 0 )
        {
            Polygon* poly = static_cast<Polygon*>(part);
            int totalPoints = poly->getTotalPointCount();
            osg::Vec3Array* allPoints = new osg::Vec3Array( totalPoints );

            std::copy( part->begin(), part->end(), allPoints->begin() );
            osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, 0, part->size() ) );

            int offset = part->size();

            for( RingCollection::const_iterator h = poly->getHoles().begin(); h != poly->getHoles().end(); ++h )
            {
                Geometry* hole = h->get();
                if ( hole->isValid() )
                {
                    std::copy( hole->begin(), hole->end(), allPoints->begin() + offset );
                    osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, offset, hole->size() ) );
                    offset += hole->size();
                }
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

        if ( part->getType() == Geometry::TYPE_POLYGON && tessellatePolys )
        {
            osgUtil::Tessellator tess;
            //tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_DRAWABLE );
            //tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
            tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
            tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );

            tess.retessellatePolygons( *osgGeom );

            // the tessellator results in a collection of trifans, strips, etc. This step will
            // consolidate those into one (or more if necessary) GL_TRIANGLES primitive.
            //NOTE: this now happens elsewhere 
            //MeshConsolidator::run( *osgGeom );

            // mark this geometry as DYNAMIC because otherwise the OSG optimizer will destroy it.
            //osgGeom->setDataVariance( osg::Object::DYNAMIC );
        }

        if ( context.isGeocentric() && part->getType() != Geometry::TYPE_POINTSET )
        {
            double threshold = osg::DegreesToRadians( *_maxAngle_deg );

            MeshSubdivider ms( context.referenceFrame(), context.inverseReferenceFrame() );
            //ms.setMaxElementsPerEBO( INT_MAX );
            ms.run( threshold, *osgGeom );
        }

        // set the color array. We have to do this last, otherwise it screws up any modifications
        // make by the MeshSubdivider. No idea why. gw

        //osg::Vec4Array* colors = new osg::Vec4Array( osgGeom->getVertexArray()->getNumElements() );
        //for( unsigned c = 0; c < colors->size(); ++c )
        //    (*colors)[c] = color;
        //osgGeom->setColorArray( colors );
        //osgGeom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

        // NOTE! per-vertex colors makes the optimizer destroy the geometry....

        osg::Vec4Array* colors = new osg::Vec4Array(1);
        (*colors)[0] = color;
        osgGeom->setColorArray( colors );
        osgGeom->setColorBinding( osg::Geometry::BIND_OVERALL );

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
BuildGeometryFilter::push( FeatureList& input, const FilterContext& context )
{
    reset();

    OE_DEBUG << LC 
        << context.toString() << std::endl;

    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
    {
        if ( !push( i->get(), context ) )
            ok = false;
    }

    // In a feature class with one point-per-feature, you end up with one geometry per point,
    // which results is (a) very bad performance and (b) geometries with a zero bbox that therefore
    // don't draw. This is not a total solution (won't work for a single point, isn't friendly for
    // doing feature-selection, etc.) but is a workable temporary fix. In the future we're going
    // to replace this filter anyway with something more highly optimized (a la osgGIS).
    //
    // however...seems that MERGE_GEOMETRY destroys almost everything except for points!!
    //if ( _mergeGeometry == true )
    //{
    //    osgUtil::Optimizer optimizer;
    //    optimizer.optimize( _geode.get(), osgUtil::Optimizer::MERGE_GEOMETRY );
    //}

    if ( !_featureNameExpr.isSet() )
    {
        MeshConsolidator::run( *_geode.get() );
    }

    if ( ok )
    {
        if ( !_style.empty() && _geode.valid() )
        {
            // could optimize this to only happen is lines or points were created ..
            const LineSymbol* lineSymbol = _style.getSymbol<LineSymbol>();
            float size = 1.0;
            if (lineSymbol)
                size = lineSymbol->stroke()->width().value();

            _geode->getOrCreateStateSet()->setAttribute( new osg::Point(size), osg::StateAttribute::ON );
            _geode->getOrCreateStateSet()->setAttribute( new osg::LineWidth(size), osg::StateAttribute::ON );

            const PointSymbol* pointSymbol = _style.getSymbol<PointSymbol>();
            if ( pointSymbol && pointSymbol->size().isSet() )
                _geode->getOrCreateStateSet()->setAttribute( 
                    new osg::Point( *pointSymbol->size() ), osg::StateAttribute::ON );
        }

        _result = _geode.release();

        if ( context.hasReferenceFrame() )
        {
            osg::MatrixTransform* delocalizer = new osg::MatrixTransform( context.inverseReferenceFrame() );
            delocalizer->addChild( _result.get() );
            _result = delocalizer;
        }
    }
    else
    {
        _result = 0L;
    }

    FilterContext outCx( context );
    outCx.setReferenceFrame( osg::Matrixd::identity() ); // clear the ref frame.
    return outCx;
}
