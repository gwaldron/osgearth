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
#include <osgEarthFeatures/FeatureSourceMeshConsolidator>
#include <osgEarthSymbology/TextSymbol>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PolygonSymbol>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarth/ECEF>
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
_style        ( style ),
_maxAngle_deg ( 5.0 ),
_geoInterp    ( GEOINTERP_RHUMB_LINE ),
_mergeGeometry( false )
{
    reset();
}

void
BuildGeometryFilter::reset()
{
    _result = 0L;
    _geode = new osg::Geode();
    _featureNode = new FeatureSourceMultiNode;
    _hasLines = false;
    _hasPoints = false;
    _hasPolygons = false;
}

bool
BuildGeometryFilter::process( FeatureList& features, const FilterContext& context )
{
    bool makeECEF = false;
    const SpatialReference* featureSRS = 0L;

    if ( context.isGeoreferenced() )
    {
        makeECEF = context.getSession()->getMapInfo().isGeocentric();
        featureSRS = context.extent()->getSRS();
    }

    for( FeatureList::iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();

        GeometryIterator parts( input->getGeometry(), false );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();
            
            osg::PrimitiveSet::Mode primMode = osg::PrimitiveSet::POINTS;

            const Style& myStyle = input->style().isSet() ? *input->style() : _style;

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
            }
            
            osg::Geometry* osgGeom = new osg::Geometry();

            if ( _featureNameExpr.isSet() )
            {
                const std::string& name = input->eval( _featureNameExpr.mutable_value(), &context );
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
                osg::Vec3Array* allPoints; // = new osg::Vec3Array( totalPoints );

                if ( makeECEF )
                {
                    allPoints = new osg::Vec3Array();
                    ECEF::transformAndLocalize( part->asVector(), featureSRS, allPoints, _world2local );
                }
                else
                {
                    allPoints = new osg::Vec3Array( totalPoints );
                    std::copy( part->begin(), part->end(), allPoints->begin() );
                }
                osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, 0, part->size() ) );

                int offset = part->size();

                for( RingCollection::const_iterator h = poly->getHoles().begin(); h != poly->getHoles().end(); ++h )
                {
                    Geometry* hole = h->get();
                    if ( hole->isValid() )
                    {
                        if ( makeECEF )
                            ECEF::transformAndLocalize( hole->asVector(), featureSRS, allPoints, _world2local );
                        else
                            std::copy( hole->begin(), hole->end(), allPoints->begin() + offset );

                        osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, offset, hole->size() ) );
                        offset += hole->size();
                    }
                }
                osgGeom->setVertexArray( allPoints );
            }
            else
            {
                if ( makeECEF )
                {
                    osg::Vec3Array* newPart = new osg::Vec3Array();
                    ECEF::transformAndLocalize( part->asVector(), featureSRS, newPart, _world2local );
                    osgGeom->setVertexArray( newPart );
                }
                else
                {
                    osgGeom->setVertexArray( part->toVec3Array() );
                }
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

            if ( makeECEF && part->getType() != Geometry::TYPE_POINTSET )
            {
                double threshold = osg::DegreesToRadians( *_maxAngle_deg );

                MeshSubdivider ms( _world2local, _local2world );
                //ms.setMaxElementsPerEBO( INT_MAX );
                if ( input->geoInterp().isSet() )
                    ms.run( *osgGeom, threshold, *input->geoInterp() );
                else
                    ms.run( *osgGeom, threshold, *_geoInterp );
            }

            // NOTE! per-vertex colors makes the optimizer destroy the geometry....
            osg::Vec4Array* colors = new osg::Vec4Array(1);
            (*colors)[0] = color;
            osgGeom->setColorArray( colors );
            osgGeom->setColorBinding( osg::Geometry::BIND_OVERALL );

            // add the part to the geode.
            _featureNode->addDrawable(osgGeom, input->getFID());

            _geode->addDrawable( osgGeom );
        }
    }
    
    return true;
}

osg::Node*
BuildGeometryFilter::push( FeatureList& input, FilterContext& context )
{
    reset();

    computeLocalizers( context );

    bool ok = process( input, context );

    // convert all geom to triangles and consolidate into minimal set of Geometries
    if ( !_featureNameExpr.isSet() )
    {
        FeatureSourceMeshConsolidator::run( *_geode.get(), _featureNode );
    }

    osg::Node* result = 0L;

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

        _featureNode->addChild(_geode.release());

        result = delocalize( _featureNode.release() );
    }
    else
    {
        result = 0L;
    }

    return result;
}
