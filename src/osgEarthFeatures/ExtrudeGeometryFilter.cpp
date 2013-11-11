/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarth/ECEF>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Utils>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>
#include <osg/LineWidth>
#include <osg/PolygonOffset>

#define LC "[ExtrudeGeometryFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

namespace
{
    // Calculates the rotation angle of a shape. This conanically applies to
    // buildings; it finds the longest edge and compares its angle to the
    // x-axis to determine a rotation value. This method is used so we can 
    // properly rotate textures for rooftop application.
    float getApparentRotation( const Geometry* geom )
    {
        Segment n;
        double  maxLen2 = 0.0;
        ConstSegmentIterator i( geom, true );
        while( i.hasMore() )
        {
            Segment s = i.next();
            double len2 = (s.second - s.first).length2();
            if ( len2 > maxLen2 ) 
            {
                maxLen2 = len2;
                n = s;
            }
        }

        const osg::Vec3d& p1 = n.first.x() < n.second.x() ? n.first : n.second;
        const osg::Vec3d& p2 = n.first.x() < n.second.x() ? n.second : n.first;

        return atan2( p2.x()-p1.x(), p2.y()-p1.y() );
    }
}

//------------------------------------------------------------------------

ExtrudeGeometryFilter::ExtrudeGeometryFilter() :
_maxAngle_deg       ( 5.0 ),
_mergeGeometry      ( true ),
_wallAngleThresh_deg( 60.0 ),
_styleDirty         ( true ),
_makeStencilVolume  ( false ),
_useVertexBufferObjects( true )
{
    //NOP
}

void
ExtrudeGeometryFilter::setStyle( const Style& style )
{
    _style      = style;
    _styleDirty = true;
}

void
ExtrudeGeometryFilter::reset( const FilterContext& context )
{
    _cosWallAngleThresh = cos( _wallAngleThresh_deg );
    _geodes.clear();
    
    if ( _styleDirty )
    {
        const StyleSheet* sheet = context.getSession() ? context.getSession()->styles() : 0L;

        _wallSkinSymbol    = 0L;
        _wallPolygonSymbol = 0L;
        _roofSkinSymbol    = 0L;
        _roofPolygonSymbol = 0L;
        _extrusionSymbol   = 0L;
        _outlineSymbol     = 0L;

        _extrusionSymbol = _style.get<ExtrusionSymbol>();
        if ( _extrusionSymbol.valid() )
        {
            // make a copy of the height expression so we can use it:
            if ( _extrusionSymbol->heightExpression().isSet() )
            {
                _heightExpr = *_extrusionSymbol->heightExpression();
            }

            // If there is no height expression, and we have either absolute or terrain-relative
            // clamping, THAT means that we want to extrude DOWN from the geometry to the ground
            // (instead of from the geometry.)
            AltitudeSymbol* alt = _style.get<AltitudeSymbol>();
            if ( alt && !_extrusionSymbol->heightExpression().isSet() && !_extrusionSymbol->height().isSet() )
            {
                if (alt->clamping() == AltitudeSymbol::CLAMP_ABSOLUTE ||
                    alt->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN )
                {
                    _heightExpr = NumericExpression( "0-[__max_hat]" );
                }
            }
            
            // attempt to extract the wall symbols:
            if ( _extrusionSymbol->wallStyleName().isSet() && sheet != 0L )
            {
                const Style* wallStyle = sheet->getStyle( *_extrusionSymbol->wallStyleName(), false );
                if ( wallStyle )
                {
                    _wallSkinSymbol = wallStyle->get<SkinSymbol>();
                    _wallPolygonSymbol = wallStyle->get<PolygonSymbol>();
                }
            }

            // attempt to extract the rooftop symbols:
            if ( _extrusionSymbol->roofStyleName().isSet() && sheet != 0L )
            {
                const Style* roofStyle = sheet->getStyle( *_extrusionSymbol->roofStyleName(), false );
                if ( roofStyle )
                {
                    _roofSkinSymbol = roofStyle->get<SkinSymbol>();
                    _roofPolygonSymbol = roofStyle->get<PolygonSymbol>();
                }
            }

            // if there's a line symbol, use it to outline the extruded data.
            _outlineSymbol = _style.get<LineSymbol>();
        }

        // backup plan for skin symbols:
        const SkinSymbol* skin = _style.get<SkinSymbol>();
        if ( skin )
        {
            if ( !_wallSkinSymbol.valid() )
                _wallSkinSymbol = skin;
            if ( !_roofSkinSymbol.valid() )
                _roofSkinSymbol = skin;
        }

        // backup plan for poly symbols:
        const PolygonSymbol* poly = _style.get<PolygonSymbol>();
        if ( poly )
        {
            if ( !_wallPolygonSymbol.valid() )
                _wallPolygonSymbol = poly;
            if ( !_roofPolygonSymbol.valid() )
                _roofPolygonSymbol = poly;
        }

        _styleDirty = false;
    }
}

bool
ExtrudeGeometryFilter::extrudeGeometry(const Geometry*         input,
                                       double                  height,
                                       double                  heightOffset,
                                       bool                    flatten,
                                       osg::Geometry*          walls,
                                       osg::Geometry*          roof,
                                       osg::Geometry*          base,
                                       osg::Geometry*          outline,
                                       const osg::Vec4&        wallColor,
                                       const osg::Vec4&        wallBaseColor,
                                       const osg::Vec4&        roofColor,
                                       const osg::Vec4&        outlineColor,
                                       const SkinResource*     wallSkin,
                                       const SkinResource*     roofSkin,
                                       FilterContext&          cx )
{
    bool makeECEF = false;
    const SpatialReference* srs = 0L;
    const SpatialReference* mapSRS = 0L;

    if ( cx.isGeoreferenced() )
    {
       srs = cx.extent()->getSRS();
       makeECEF = cx.getSession()->getMapInfo().isGeocentric();
       mapSRS = cx.getSession()->getMapInfo().getProfile()->getSRS();
    }

    bool made_geom = false;

    double tex_width_m   = wallSkin ? *wallSkin->imageWidth() : 1.0;
    double tex_height_m  = wallSkin ? *wallSkin->imageHeight() : 1.0;
    bool   tex_repeats_y = wallSkin ? *wallSkin->isTiled() : false;
    bool   useColor      = (!wallSkin || wallSkin->texEnvMode() != osg::TexEnv::DECAL) && !_makeStencilVolume;

    bool isPolygon = input->getComponentType() == Geometry::TYPE_POLYGON;

    unsigned pointCount = input->getTotalPointCount();
    
    // If we are extruding a polygon, and applying a wall texture, we need an extra
    // point in the geometry in order to close the polygon and generate a unique
    // texture coordinate for that final point.
    bool isSkinnedPolygon = isPolygon && wallSkin != 0L;

    // Total number of verts. Add 2 to close a polygon (necessary so the first and last
    // points can have unique texture coordinates)
    unsigned numWallVerts = 2 * pointCount + (isSkinnedPolygon? (2 * input->getNumGeometries()) : 0);

    // create all the OSG geometry components
    osg::Vec3Array* verts = new osg::Vec3Array( numWallVerts );
    walls->setVertexArray( verts );

    osg::Vec2Array* wallTexcoords = 0L;
    if ( wallSkin )
    { 
        wallTexcoords = new osg::Vec2Array( numWallVerts );
        walls->setTexCoordArray( 0, wallTexcoords );
    }

    osg::Vec4Array* colors = 0L;
    if ( useColor )
    {
        // per-vertex colors are necessary if we are going to use the MeshConsolidator -gw
        colors = new osg::Vec4Array();
        colors->reserve( numWallVerts );
        colors->assign( numWallVerts, wallColor );
        walls->setColorArray( colors );
        walls->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }

    // set up rooftop tessellation and texturing, if necessary:
    osg::Vec3Array* roofVerts     = 0L;
    osg::Vec2Array* roofTexcoords = 0L;
    float           roofRotation  = 0.0f;
    Bounds          roofBounds;
    float           sinR = 0.0f, cosR = 0.0f;
    double          roofTexSpanX = 0.0, roofTexSpanY = 0.0;
    osg::ref_ptr<const SpatialReference> roofProjSRS;

    if ( roof )
    {
        roofVerts = new osg::Vec3Array( pointCount );
        roof->setVertexArray( roofVerts );

        // per-vertex colors are necessary if we are going to use the MeshConsolidator -gw
        if ( useColor )
        {
            osg::Vec4Array* roofColors = new osg::Vec4Array();
            roofColors->reserve( pointCount );
            roofColors->assign( pointCount, roofColor );
            roof->setColorArray( roofColors );
            roof->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
        }

        if ( roofSkin )
        {
            roofTexcoords = new osg::Vec2Array( pointCount );
            roof->setTexCoordArray( 0, roofTexcoords );

            roofBounds = input->getBounds();

            // if our data is lat/long, we need to reproject the geometry and the bounds into a projected
            // coordinate system in order to properly generate tex coords.
            if ( srs && srs->isGeographic() )
            {
                osg::Vec2d geogCenter = roofBounds.center2d();
                roofProjSRS = srs->createUTMFromLonLat( Angular(geogCenter.x()), Angular(geogCenter.y()) );
                if ( roofProjSRS.valid() )
                {
                    roofBounds.transform( srs, roofProjSRS.get() );
                    osg::ref_ptr<Geometry> projectedInput = input->clone();
                    srs->transform( projectedInput->asVector(), roofProjSRS.get() );
                    roofRotation = getApparentRotation( projectedInput.get() );
                }
            }
            else
            {
                roofRotation = getApparentRotation( input );
            }
            
            sinR = sin(roofRotation);
            cosR = cos(roofRotation);

            if ( !roofSkin->isTiled().value() )
            {
                //note: doesn't really work
                roofTexSpanX = cosR*roofBounds.width() - sinR*roofBounds.height();
                roofTexSpanY = sinR*roofBounds.width() + cosR*roofBounds.height();
            }
            else
            {
                roofTexSpanX = roofSkin->imageWidth().isSet() ? *roofSkin->imageWidth() : roofSkin->imageHeight().isSet() ? *roofSkin->imageHeight() : 10.0;
                if ( roofTexSpanX <= 0.0 ) roofTexSpanX = 10.0;
                roofTexSpanY = roofSkin->imageHeight().isSet() ? *roofSkin->imageHeight() : roofSkin->imageWidth().isSet() ? *roofSkin->imageWidth() : 10.0;
                if ( roofTexSpanY <= 0.0 ) roofTexSpanY = 10.0;
            }
        }
    }

    osg::Vec3Array* baseVerts = NULL;
    if ( base )
    {
        baseVerts = new osg::Vec3Array( pointCount );
        base->setVertexArray( baseVerts );
    }

    osg::Vec3Array* outlineVerts = 0L;
    osg::Vec3Array* outlineNormals = 0L;
    if ( outline )
    {
        outlineVerts = new osg::Vec3Array( numWallVerts );
        outline->setVertexArray( outlineVerts );

        osg::Vec4Array* outlineColors = new osg::Vec4Array();
        outlineColors->reserve( numWallVerts );
        outlineColors->assign( numWallVerts, outlineColor );
        outline->setColorArray( outlineColors );
        outline->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

        // cop out, just point all the outline normals up. fix this later.
        outlineNormals = new osg::Vec3Array();
        outlineNormals->reserve( numWallVerts );
        outlineNormals->assign( numWallVerts, osg::Vec3(0,0,1) );
        outline->setNormalArray( outlineNormals );
    }

    unsigned wallVertPtr    = 0;
    unsigned roofVertPtr    = 0;
    unsigned baseVertPtr    = 0;

    double     targetLen = -DBL_MAX;
    osg::Vec3d minLoc(DBL_MAX, DBL_MAX, DBL_MAX);
    double     minLoc_len = DBL_MAX;
    osg::Vec3d maxLoc(0,0,0);
    double     maxLoc_len = 0;

    // Initial pass over the geometry does two things:
    // 1: Calculate the minimum Z across all parts.
    // 2: Establish a "target length" for extrusion

    double absHeight = fabs(height);

    ConstGeometryIterator zfinder( input );
    while( zfinder.hasMore() )
    {
        const Geometry* geom = zfinder.next();
        for( Geometry::const_iterator m = geom->begin(); m != geom->end(); ++m )
        {
            osg::Vec3d m_point = *m;

            if ( m_point.z() + absHeight > targetLen )
                targetLen = m_point.z() + absHeight;

            if (m_point.z() < minLoc.z())
                minLoc = m_point;

            if (m_point.z() > maxLoc.z())
                maxLoc = m_point;
        }
    }

    // apply the height offsets
    height    -= heightOffset;
    targetLen -= heightOffset;

    // now generate the extruded geometry.
    ConstGeometryIterator iter( input );
    while( iter.hasMore() )
    {
        const Geometry* part = iter.next();

        double tex_height_m_adj = tex_height_m;

        unsigned wallPartPtr = wallVertPtr;
        unsigned roofPartPtr = roofVertPtr;
        unsigned basePartPtr = baseVertPtr;
        double   partLen     = 0.0;
        double   maxHeight   = 0.0;

        maxHeight = targetLen - minLoc.z();

        // Adjust the texture height so it is a multiple of the maximum height
        double div = osg::round(maxHeight / tex_height_m);
        if (div == 0) div = 1; //Prevent divide by zero
        tex_height_m_adj = maxHeight / div;

        //osg::DrawElementsUShort* idx = new osg::DrawElementsUShort( GL_TRIANGLES );
        osg::DrawElementsUInt* idx = new osg::DrawElementsUInt( GL_TRIANGLES );

        for( Geometry::const_iterator m = part->begin(); m != part->end(); ++m )
        {
            osg::Vec3d basePt = *m;
            osg::Vec3d roofPt;

            if ( height >= 0 )
            {
                if ( flatten )
                    roofPt.set( basePt.x(), basePt.y(), targetLen );
                else
                    roofPt.set( basePt.x(), basePt.y(), basePt.z() + height );
            }
            else // height < 0
            {
                roofPt = *m;
                basePt.z() += height;
            }

            // add to the approprate vertex lists:
            int p = wallVertPtr;

            // figure out the rooftop texture coordinates before doing any
            // transformations:
            if ( roofSkin && srs )
            {
                double xr, yr;

                if ( srs && srs->isGeographic() && roofProjSRS )
                {
                    osg::Vec3d projRoofPt;
                    srs->transform( roofPt, roofProjSRS.get(), projRoofPt );
                    xr = (projRoofPt.x() - roofBounds.xMin());
                    yr = (projRoofPt.y() - roofBounds.yMin());
                }
                else
                {
                    xr = (roofPt.x() - roofBounds.xMin());
                    yr = (roofPt.y() - roofBounds.yMin());
                }

                float u = (cosR*xr - sinR*yr) / roofTexSpanX;
                float v = (sinR*xr + cosR*yr) / roofTexSpanY;

                (*roofTexcoords)[roofVertPtr].set( u, v );
            }

            transformAndLocalize( basePt, srs, basePt, mapSRS, _world2local, makeECEF );
            transformAndLocalize( roofPt, srs, roofPt, mapSRS, _world2local, makeECEF );


            if ( base )
            {
                (*baseVerts)[baseVertPtr] = basePt;
            }

            if ( roof )
            {
                (*roofVerts)[roofVertPtr] = roofPt;
            }

            baseVertPtr++;
            roofVertPtr++;

            (*verts)[p] = roofPt;
            (*verts)[p+1] = basePt;

            if ( useColor )
            {
                (*colors)[p+1] = wallBaseColor;
            }

            if ( outline )
            {
                (*outlineVerts)[p] = roofPt;
                (*outlineVerts)[p+1] = basePt;
            }
            
            partLen += wallVertPtr > wallPartPtr ? ((*verts)[p] - (*verts)[p-2]).length() : 0.0;
            double h = tex_repeats_y ? -((*verts)[p] - (*verts)[p+1]).length() : -tex_height_m_adj;

            if ( wallSkin )
            {
                (*wallTexcoords)[p].set( partLen/tex_width_m, 0.0f );
                (*wallTexcoords)[p+1].set( partLen/tex_width_m, h/tex_height_m_adj );
            }

            // form the 2 triangles
            if ( (m+1) == part->end() )
            {
                if ( isPolygon )
                {
                    // end of the wall; loop around to close it off.
                    if ( isSkinnedPolygon )
                    {
                        // if we requested an extra geometry point, that means we are generating
                        // a polygon-closing line so we can have a unique texcoord for it. 
                        idx->push_back(wallVertPtr);
                        idx->push_back(wallVertPtr+1);
                        idx->push_back(wallVertPtr+2);

                        idx->push_back(wallVertPtr+1);
                        idx->push_back(wallVertPtr+3);
                        idx->push_back(wallVertPtr+2);

                        (*verts)[p+2] = (*verts)[wallPartPtr];
                        (*verts)[p+3] = (*verts)[wallPartPtr+1];

                        if ( wallSkin )
                        {
                            partLen += ((*verts)[p+2] - (*verts)[p]).length();
                            double h = tex_repeats_y ? -((*verts)[p+2] - (*verts)[p+3]).length() : -tex_height_m_adj;
                            (*wallTexcoords)[p+2].set( partLen/tex_width_m, 0.0f );
                            (*wallTexcoords)[p+3].set( partLen/tex_width_m, h/tex_height_m_adj );
                        }

                        wallVertPtr += 2;
                    }
                    else
                    {
                        // either not a poly, or no wall skin, so we can share the polygon-closing
                        // loop point.
                        idx->push_back(wallVertPtr); 
                        idx->push_back(wallVertPtr+1);
                        idx->push_back(wallPartPtr);

                        idx->push_back(wallVertPtr+1);
                        idx->push_back(wallPartPtr+1);
                        idx->push_back(wallPartPtr);
                    }
                }
                else
                {
                    //nop - no elements required at the end of a line
                }
            }
            else
            {
                idx->push_back(wallVertPtr); 
                idx->push_back(wallVertPtr+1);
                idx->push_back(wallVertPtr+2); 

                idx->push_back(wallVertPtr+1);
                idx->push_back(wallVertPtr+3);
                idx->push_back(wallVertPtr+2);
            }

            wallVertPtr += 2;
            made_geom = true;
        }

        walls->addPrimitiveSet( idx );

        if ( roof )
        {
            roof->addPrimitiveSet( new osg::DrawArrays(
                osg::PrimitiveSet::LINE_LOOP,
                roofPartPtr, roofVertPtr - roofPartPtr ) );
        }

        if ( base )
        {
            // reverse the base verts:
            int len = baseVertPtr - basePartPtr;
            for( int i=basePartPtr; i<len/2; i++ )
                std::swap( (*baseVerts)[i], (*baseVerts)[basePartPtr+(len-1)-i] );

            base->addPrimitiveSet( new osg::DrawArrays(
                osg::PrimitiveSet::LINE_LOOP,
                basePartPtr, baseVertPtr - basePartPtr ) );
        }

        if ( outline )
        {
            unsigned len = baseVertPtr - basePartPtr;

            GLenum roofLineMode = isPolygon ? GL_LINE_LOOP : GL_LINE_STRIP;
            osg::DrawElementsUInt* roofLine = new osg::DrawElementsUInt( roofLineMode );
            roofLine->reserveElements( len );
            for( unsigned i=0; i<len; ++i )
                roofLine->addElement( basePartPtr + i*2 );
            outline->addPrimitiveSet( roofLine );

            // if the outline is tessellated, we only want outlines on the original 
            // points (not the inserted points)
            unsigned step = std::max( 1u, 
                _outlineSymbol->tessellation().isSet() ? *_outlineSymbol->tessellation() : 1u );

            osg::DrawElementsUInt* wallLines = new osg::DrawElementsUInt( GL_LINES );
            wallLines->reserve( len*2 );
            for( unsigned i=0; i<len; i+=step )
            {
                wallLines->push_back( basePartPtr + i*2 );
                wallLines->push_back( basePartPtr + i*2 + 1 );
            }
            outline->addPrimitiveSet( wallLines );

            applyLineSymbology( outline->getOrCreateStateSet(), _outlineSymbol.get() );
        }
    }

    return made_geom;
}

void
ExtrudeGeometryFilter::addDrawable(osg::Drawable*      drawable,
                                   osg::StateSet*      stateSet,
                                   const std::string&  name,
                                   Feature*            feature,
                                   FeatureSourceIndex* index )
{
    // find the geode for the active stateset, creating a new one if necessary. NULL is a 
    // valid key as well.
    osg::Geode* geode = _geodes[stateSet].get();
    if ( !geode )
    {
        geode = new osg::Geode();
        geode->setStateSet( stateSet );
        _geodes[stateSet] = geode;
    }

    geode->addDrawable( drawable );

    if ( !name.empty() )
    {
        drawable->setName( name );
    }

    if ( index )
    {
        index->tagPrimitiveSets( drawable, feature );
    }
}

bool
ExtrudeGeometryFilter::process( FeatureList& features, FilterContext& context )
{
    // seed our random number generators
    Random wallSkinPRNG( _wallSkinSymbol.valid()? *_wallSkinSymbol->randomSeed() : 0, Random::METHOD_FAST );
    Random roofSkinPRNG( _roofSkinSymbol.valid()? *_roofSkinSymbol->randomSeed() : 0, Random::METHOD_FAST );

    for( FeatureList::iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();

        GeometryIterator iter( input->getGeometry(), false );
        while( iter.hasMore() )
        {
            Geometry* part = iter.next();

            osg::ref_ptr<osg::Geometry> walls = new osg::Geometry();
            walls->setUseVertexBufferObjects( _useVertexBufferObjects.get() );
            
            osg::ref_ptr<osg::Geometry> rooflines = 0L;
            osg::ref_ptr<osg::Geometry> baselines = 0L;
            osg::ref_ptr<osg::Geometry> outlines  = 0L;
            
            if ( part->getType() == Geometry::TYPE_POLYGON )
            {
                rooflines = new osg::Geometry();
                rooflines->setUseVertexBufferObjects( _useVertexBufferObjects.get() );

                // prep the shapes by making sure all polys are open:
                static_cast<Polygon*>(part)->open();
            }

            // fire up the outline geometry if we have a line symbol.
            if ( _outlineSymbol != 0L )
            {
                outlines = new osg::Geometry();
                outlines->setUseVertexBufferObjects( _useVertexBufferObjects.get() );
            }

            // make a base cap if we're doing stencil volumes.
            if ( _makeStencilVolume )
            {
                baselines = new osg::Geometry();
                baselines->setUseVertexBufferObjects( _useVertexBufferObjects.get() );
            }

            // calculate the extrusion height:
            float height;

            if ( _heightCallback.valid() )
            {
                height = _heightCallback->operator()(input, context);
            }
            else if ( _heightExpr.isSet() )
            {
                height = input->eval( _heightExpr.mutable_value(), &context );
            }
            else
            {
                height = *_extrusionSymbol->height();
            }

            // calculate the height offset from the base:
            float offset = 0.0;
            if ( _heightOffsetExpr.isSet() )
            {
                offset = input->eval( _heightOffsetExpr.mutable_value(), &context );
            }

            osg::ref_ptr<osg::StateSet> wallStateSet;
            osg::ref_ptr<osg::StateSet> roofStateSet;

            // calculate the wall texturing:
            SkinResource* wallSkin = 0L;
            if ( _wallSkinSymbol.valid() )
            {
                if ( _wallResLib.valid() )
                {
                    SkinSymbol querySymbol( *_wallSkinSymbol.get() );
                    querySymbol.objectHeight() = fabs(height) - offset;
                    wallSkin = _wallResLib->getSkin( &querySymbol, wallSkinPRNG, context.getDBOptions() );
                }

                else
                {
                    //TODO: simple single texture?
                }
            }

            // calculate the rooftop texture:
            SkinResource* roofSkin = 0L;
            if ( _roofSkinSymbol.valid() )
            {
                if ( _roofResLib.valid() )
                {
                    SkinSymbol querySymbol( *_roofSkinSymbol.get() );
                    roofSkin = _roofResLib->getSkin( &querySymbol, roofSkinPRNG, context.getDBOptions() );
                }

                else
                {
                    //TODO: simple single texture?
                }
            }

            // calculate the colors:
            osg::Vec4f wallColor(1,1,1,0), wallBaseColor(1,1,1,0), roofColor(1,1,1,0), outlineColor(1,1,1,1);

            if ( _wallPolygonSymbol.valid() )
            {
                wallColor = _wallPolygonSymbol->fill()->color();
                if ( _extrusionSymbol->wallGradientPercentage().isSet() )
                {
                    wallBaseColor = Color(wallColor).brightness( 1.0 - *_extrusionSymbol->wallGradientPercentage() );
                }
                else
                {
                    wallBaseColor = wallColor;
                }
            }
            if ( _roofPolygonSymbol.valid() )
            {
                roofColor = _roofPolygonSymbol->fill()->color();
            }
            if ( _outlineSymbol.valid() )
            {
                outlineColor = _outlineSymbol->stroke()->color();
            }

            // Create the extruded geometry!
            if (extrudeGeometry( 
                    part, height, offset, 
                    *_extrusionSymbol->flatten(),
                    walls.get(), rooflines.get(), baselines.get(), outlines.get(),
                    wallColor, wallBaseColor, roofColor, outlineColor,
                    wallSkin, roofSkin,
                    context ) )
            {      
                if ( wallSkin )
                {
                    context.resourceCache()->getStateSet( wallSkin, wallStateSet );
                }

                // generate per-vertex normals, altering the geometry as necessary to avoid
                // smoothing around sharp corners
                osgUtil::SmoothingVisitor::smooth(
                    *walls.get(), 
                    osg::DegreesToRadians(_wallAngleThresh_deg) );

                // tessellate and add the roofs if necessary:
                if ( rooflines.valid() )
                {
                    osgUtil::Tessellator tess;
                    tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
                    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
                    tess.retessellatePolygons( *(rooflines.get()) );

                    // generate default normals (no crease angle necessary; they are all pointing up)
                    // TODO do this manually; probably faster
                    if ( !_makeStencilVolume )
                        osgUtil::SmoothingVisitor::smooth( *rooflines.get() );

                    if ( roofSkin )
                    {
                        context.resourceCache()->getStateSet( roofSkin, roofStateSet );
                    }
                }

                if ( baselines.valid() )
                {
                    osgUtil::Tessellator tess;
                    tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
                    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
                    tess.retessellatePolygons( *(baselines.get()) );
                }

                std::string name;
                if ( !_featureNameExpr.empty() )
                    name = input->eval( _featureNameExpr, &context );

                FeatureSourceIndex* index = context.featureIndex();

                addDrawable( walls.get(), wallStateSet.get(), name, input, index );

                if ( rooflines.valid() )
                {
                    addDrawable( rooflines.get(), roofStateSet.get(), name, input, index );
                }

                if ( baselines.valid() )
                {
                    addDrawable( baselines.get(), 0L, name, input, index );
                }

                if ( outlines.valid() )
                {
                    addDrawable( outlines.get(), 0L, name, input, index );
                }
            }   
        }
    }

    return true;
}

osg::Node*
ExtrudeGeometryFilter::push( FeatureList& input, FilterContext& context )
{
    reset( context );

    // minimally, we require an extrusion symbol.
    if ( !_extrusionSymbol.valid() )
    {
        OE_WARN << LC << "Missing required extrusion symbolology; geometry will be empty" << std::endl;
        return new osg::Group();
    }

    // establish the active resource library, if applicable.
    _wallResLib = 0L;
    _roofResLib = 0L;

    const StyleSheet* sheet = context.getSession() ? context.getSession()->styles() : 0L;

    if ( sheet != 0L )
    {
        if ( _wallSkinSymbol.valid() && _wallSkinSymbol->libraryName().isSet() )
        {
            _wallResLib = sheet->getResourceLibrary( *_wallSkinSymbol->libraryName() );

            if ( !_wallResLib.valid() )
            {
                OE_WARN << LC << "Unable to load resource library '" << *_wallSkinSymbol->libraryName() << "'"
                    << "; wall geometry will not be textured." << std::endl;
                _wallSkinSymbol = 0L;
            }
        }

        if ( _roofSkinSymbol.valid() && _roofSkinSymbol->libraryName().isSet() )
        {
            _roofResLib = sheet->getResourceLibrary( *_roofSkinSymbol->libraryName() );
            if ( !_roofResLib.valid() )
            {
                OE_WARN << LC << "Unable to load resource library '" << *_roofSkinSymbol->libraryName() << "'"
                    << "; roof geometry will not be textured." << std::endl;
                _roofSkinSymbol = 0L;
            }
        }
    }

    // calculate the localization matrices (_local2world and _world2local)
    computeLocalizers( context );

    // push all the features through the extruder.
    bool ok = process( input, context );

    // convert everything to triangles and combine drawables.
    if ( _mergeGeometry == true && _featureNameExpr.empty() )
    {
        for( SortedGeodeMap::iterator i = _geodes.begin(); i != _geodes.end(); ++i )
        {
            if ( context.featureIndex() )
            {
                // The MC will recognize the presence of feature indexing tags and
                // preserve them. The Cache optimizer however will not, so it is
                // out for now.
                MeshConsolidator::run( *i->second.get() );

                //VertexCacheOptimizer vco;
                //i->second->accept( vco );
            }
            else
            {
                //TODO: try this -- issues: it won't work on lines, and will it screw up
                // feature indexing?
                osgUtil::Optimizer o;
                o.optimize( i->second.get(),
                    osgUtil::Optimizer::MERGE_GEOMETRY |
                    osgUtil::Optimizer::VERTEX_PRETRANSFORM |
                    osgUtil::Optimizer::INDEX_MESH |
                    osgUtil::Optimizer::VERTEX_POSTTRANSFORM );
            }
        }
    }

    // parent geometry with a delocalizer (if necessary)
    osg::Group* group = createDelocalizeGroup();
    
    // combines geometries where the statesets are the same.
    for( SortedGeodeMap::iterator i = _geodes.begin(); i != _geodes.end(); ++i )
    {
        group->addChild( i->second.get() );
    }
    _geodes.clear();

    // if we drew outlines, apply a poly offset too.
    if ( _outlineSymbol.valid() )
    {
        osg::StateSet* groupStateSet = group->getOrCreateStateSet();
        groupStateSet->setAttributeAndModes( new osg::PolygonOffset(1,1), 1 );
        if ( _outlineSymbol->stroke()->width().isSet() )
            groupStateSet->setAttributeAndModes( new osg::LineWidth(*_outlineSymbol->stroke()->width()), 1 );
    }

    return group;
}
