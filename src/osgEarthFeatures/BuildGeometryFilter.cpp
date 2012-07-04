/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthSymbology/TextSymbol>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PolygonSymbol>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarth/ECEF>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/LineStipple>
#include <osg/Point>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>
#include <osgDB/WriteFile>
#include <osg/Version>

#define LC "[BuildGeometryFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define USE_SINGLE_COLOR 1

namespace
{
    void applyLineAndPointSymbology( osg::StateSet* stateSet, const LineSymbol* line, const PointSymbol* point )
    {
        if ( line )
        {
            float width = std::max( 1.0f, *line->stroke()->width() );
            stateSet->setAttributeAndModes(new osg::LineWidth(width), 1);
            if ( line->stroke()->stipple().isSet() )
            {
                stateSet->setAttributeAndModes( new osg::LineStipple(1, *line->stroke()->stipple()) );
            }
        }

        if ( point )
        {
            float size = std::max( 0.1f, *point->size() );
            stateSet->setAttributeAndModes(new osg::Point(size), 1);
        }
    }
}


BuildGeometryFilter::BuildGeometryFilter( const Style& style ) :
_style        ( style ),
_maxAngle_deg ( 1.0 ),
_geoInterp    ( GEOINTERP_RHUMB_LINE ),
_mergeGeometry( false ),
_useVertexBufferObjects( true )
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
BuildGeometryFilter::process( FeatureList& features, const FilterContext& context )
{
    bool makeECEF = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* mapSRS = 0L;

    if ( context.isGeoreferenced() )
    {
        makeECEF   = context.getSession()->getMapInfo().isGeocentric();
        featureSRS = context.extent()->getSRS();
        mapSRS     = context.getSession()->getMapInfo().getProfile()->getSRS();
    }

    for( FeatureList::iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();

        GeometryIterator parts( input->getGeometry(), false );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();

            // skip empty geometry
            if ( part->size() == 0 )
                continue;

            const Style& myStyle = input->style().isSet() ? *input->style() : _style;

            bool  setLinePropsHere   = input->style().isSet(); // otherwise it will be set globally, we assume
            float width              = 1.0f;
            bool  hasPolyOutline     = false;

            const PointSymbol*   pointSymbol = myStyle.get<PointSymbol>();
            const LineSymbol*    lineSymbol  = myStyle.get<LineSymbol>();
            const PolygonSymbol* polySymbol  = myStyle.get<PolygonSymbol>();

            // resolve the geometry type from the component type and the symbology:
            Geometry::Type renderType = Geometry::TYPE_UNKNOWN;

            // First priority is a matching part type and symbol:
            if ( polySymbol != 0L && part->getType() == Geometry::TYPE_POLYGON )
            {
                renderType = Geometry::TYPE_POLYGON;
            }
            else if ( lineSymbol != 0L && part->isLinear() )
            {
                renderType = part->getType();
            }
            else if ( pointSymbol != 0L && part->getType() == Geometry::TYPE_POINTSET )
            {
                renderType = Geometry::TYPE_POINTSET;
            }

            // Second priority is the symbol:
            else if ( polySymbol != 0L )
            {
                renderType = Geometry::TYPE_POLYGON;
            }
            else if ( lineSymbol != 0L )
            {
                if ( part->getType() == Geometry::TYPE_POLYGON )
                    renderType = Geometry::TYPE_RING;
                else
                    renderType = Geometry::TYPE_LINESTRING;
            }
            else if ( pointSymbol != 0L )
            {
                renderType = Geometry::TYPE_POINTSET;
            }

            // No symbol? just use the geometry type.
            else
            {
                renderType = part->getType();
            }

            // validate the geometry:
            if ( renderType == Geometry::TYPE_POLYGON && part->size() < 3 )
                continue;
            else if ( (renderType == Geometry::TYPE_LINESTRING || renderType == Geometry::TYPE_RING) && part->size() < 2 )
                continue;

            // resolve the color:
            osg::Vec4f primaryColor =
                polySymbol ? polySymbol->fill()->color() :
                lineSymbol ? lineSymbol->stroke()->color() :
                pointSymbol ? pointSymbol->fill()->color() :
                osg::Vec4f(1,1,1,1);
            
            osg::Geometry* osgGeom = new osg::Geometry();
            osgGeom->setUseVertexBufferObjects( _useVertexBufferObjects.value() );

            if ( _featureNameExpr.isSet() )
            {
                const std::string& name = input->eval( _featureNameExpr.mutable_value(), &context );
                osgGeom->setName( name );
            }

            // build the geometry:
            osg::Vec3Array* allPoints = 0L;

            if ( renderType == Geometry::TYPE_POLYGON )
            {
                buildPolygon(part, featureSRS, mapSRS, makeECEF, true, osgGeom);
                allPoints = static_cast<osg::Vec3Array*>( osgGeom->getVertexArray() );
            }
            else
            {
                // line or point geometry
                GLenum primMode = 
                    renderType == Geometry::TYPE_LINESTRING ? GL_LINE_STRIP :
                    renderType == Geometry::TYPE_RING       ? GL_LINE_LOOP :
                    GL_POINTS;
                allPoints = new osg::Vec3Array();
                transformAndLocalize( part->asVector(), featureSRS, allPoints, mapSRS, _world2local, makeECEF );
                osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, 0, part->size() ) );
                osgGeom->setVertexArray( allPoints );

                applyLineAndPointSymbology( osgGeom->getOrCreateStateSet(), lineSymbol, pointSymbol );

                if ( primMode == GL_POINTS && allPoints->size() == 1 )
                {
                    const osg::Vec3d& center = (*allPoints)[0];
                    osgGeom->setInitialBound( osg::BoundingBox(center-osg::Vec3(.5,.5,.5), center+osg::Vec3(.5,.5,.5)) );
                }
            }

            if (allPoints->getVertexBufferObject())
                allPoints->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);
            
            // subdivide the mesh if necessary to conform to an ECEF globe:
            if ( makeECEF && renderType != Geometry::TYPE_POINTSET )
            {
                // check for explicit tessellation disable:
                const LineSymbol* line = _style.get<LineSymbol>();
                bool disableTess = line && line->tessellation().isSetTo(0);

                if ( makeECEF && !disableTess )
                {                    
                    double threshold = osg::DegreesToRadians( *_maxAngle_deg );
                    OE_DEBUG << "Running mesh subdivider with threshold " << *_maxAngle_deg << std::endl;

                    MeshSubdivider ms( _world2local, _local2world );
                    //ms.setMaxElementsPerEBO( INT_MAX );
                    if ( input->geoInterp().isSet() )
                        ms.run( *osgGeom, threshold, *input->geoInterp() );
                    else
                        ms.run( *osgGeom, threshold, *_geoInterp );
                }
            }


            // assign the primary color:
#if USE_SINGLE_COLOR            
            osg::Vec4Array* colors = new osg::Vec4Array( 1 );
            (*colors)[0] = primaryColor;
            osgGeom->setColorBinding( osg::Geometry::BIND_OVERALL );
#else

            osg::Vec4Array* colors = new osg::Vec4Array( osgGeom->getVertexArray()->getNumElements() ); //allPoints->size() );
            for(unsigned c=0; c<colors->size(); ++c)
                (*colors)[c] = primaryColor;
            osgGeom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
#endif


            osgGeom->setColorArray( colors );
            

            _geode->addDrawable( osgGeom );

            // record the geometry's primitive set(s) in the index:
            if ( context.featureIndex() )
                context.featureIndex()->tagPrimitiveSets( osgGeom, input->getFID() );

            // build secondary geometry, if necessary (polygon outlines)
            if ( renderType == Geometry::TYPE_POLYGON && lineSymbol )
            {
                // polygon offset on the poly so the outline doesn't z-fight
                osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::PolygonOffset(1,1), 1 );

                osg::Geometry* outline = new osg::Geometry();
                outline->setUseVertexBufferObjects( _useVertexBufferObjects.value() );

                buildPolygon(part, featureSRS, mapSRS, makeECEF, false, outline);

                if ( outline->getVertexArray()->getVertexBufferObject() )
                    outline->getVertexArray()->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);                
                
                osg::Vec4f outlineColor = lineSymbol->stroke()->color();                

                osg::Vec4Array* outlineColors = new osg::Vec4Array();                
#if USE_SINGLE_COLOR
                outlineColors->reserve(1);
                outlineColors->push_back( outlineColor );
                outline->setColorBinding( osg::Geometry::BIND_OVERALL );
#else
                unsigned pcount = part->getTotalPointCount();                
                outlineColors->reserve( pcount );
                for( unsigned c=0; c < pcount; ++c )
                    outlineColors->push_back( outlineColor );
                outline->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
#endif
                outline->setColorArray(outlineColors);

                // check for explicit tessellation disable:                
                bool disableTess = lineSymbol && lineSymbol->tessellation().isSetTo(0);

                // subdivide if necessary.                
                if ( makeECEF && !disableTess )
                {
                    double threshold = osg::DegreesToRadians( *_maxAngle_deg );
                    OE_DEBUG << "Running mesh subdivider for outlines with threshold " << *_maxAngle_deg << std::endl;
                    MeshSubdivider ms( _world2local, _local2world );
                    if ( input->geoInterp().isSet() )
                        ms.run( *outline, threshold, *input->geoInterp() );
                    else
                        ms.run( *outline, threshold, *_geoInterp );
                }

                applyLineAndPointSymbology( outline->getOrCreateStateSet(), lineSymbol, 0L );

                _geode->addDrawable( outline );

                //_featureNode->addDrawable( outline, input->getFID() );

                // Mark each primitive set with its feature ID.
                if ( context.featureIndex() )
                    context.featureIndex()->tagPrimitiveSets( outline, input->getFID() );
            }

        }
    }
    
    return true;
}

// builds and tessellates a polygon (with or without holes)
void
BuildGeometryFilter::buildPolygon(Geometry*               ring,
                                  const SpatialReference* featureSRS,
                                  const SpatialReference* mapSRS,
                                  bool                    makeECEF,
                                  bool                    tessellate,
                                  osg::Geometry*          osgGeom)
{
    if ( !ring->isValid() )
        return;

    int totalPoints = ring->getTotalPointCount();
    osg::Vec3Array* allPoints = new osg::Vec3Array();
    transformAndLocalize( ring->asVector(), featureSRS, allPoints, mapSRS, _world2local, makeECEF );

    GLenum mode = GL_LINE_LOOP;
    osgGeom->addPrimitiveSet( new osg::DrawArrays( mode, 0, ring->size() ) );

    Polygon* poly = dynamic_cast<Polygon*>(ring);
    if ( poly )
    {
        int offset = ring->size();

        for( RingCollection::const_iterator h = poly->getHoles().begin(); h != poly->getHoles().end(); ++h )
        {
            Geometry* hole = h->get();
            if ( hole->isValid() )
            {
                transformAndLocalize( hole->asVector(), featureSRS, allPoints, mapSRS, _world2local, makeECEF );

                osgGeom->addPrimitiveSet( new osg::DrawArrays( mode, offset, hole->size() ) );
                offset += hole->size();
            }

            _geode->addDrawable( osgGeom );
        }
    }
    osgGeom->setVertexArray( allPoints );

    if ( tessellate )
    {
        osgUtil::Tessellator tess;
        tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
        tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
        //tess.setBoundaryOnly( true );
        tess.retessellatePolygons( *osgGeom );
    }
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
        MeshConsolidator::run( *_geode.get() );
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
                size = std::max(1.0f, lineSymbol->stroke()->width().value());

            _geode->getOrCreateStateSet()->setAttribute( new osg::Point(size), osg::StateAttribute::ON );
            _geode->getOrCreateStateSet()->setAttribute( new osg::LineWidth(size), osg::StateAttribute::ON );

            const PointSymbol* pointSymbol = _style.getSymbol<PointSymbol>();
            if ( pointSymbol && pointSymbol->size().isSet() )
                _geode->getOrCreateStateSet()->setAttribute( 
                    new osg::Point( *pointSymbol->size() ), osg::StateAttribute::ON );
        }

        // apply the delocalization matrix for no-jitter
        result = delocalize( _geode.release() );
    }
    else
    {
        result = 0L;
    }

    return result;
}
