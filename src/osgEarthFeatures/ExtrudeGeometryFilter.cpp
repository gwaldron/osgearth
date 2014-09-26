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
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthFeatures/Session>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarthSymbology/ResourceCache>
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
_useVertexBufferObjects( true ),
_useTextureArrays( true )
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
ExtrudeGeometryFilter::buildStructure(const Geometry*         input,
                                      double                  height,
                                      double                  heightOffset,
                                      bool                    flatten,
                                      const SkinResource*     wallSkin,
                                      const SkinResource*     roofSkin,
                                      Structure&              structure,
                                      FilterContext&          cx )
{
    bool  makeECEF                 = false;
    const SpatialReference* srs    = 0L;
    const SpatialReference* mapSRS = 0L;

    if ( cx.isGeoreferenced() )
    {
       srs      = cx.extent()->getSRS();
       makeECEF = cx.getSession()->getMapInfo().isGeocentric();
       mapSRS   = cx.getSession()->getMapInfo().getProfile()->getSRS();
    }

    // whether this is a closed polygon structure.
    structure.isPolygon = (input->getComponentType() == Geometry::TYPE_POLYGON);

    // extrusion working variables
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
    
    float   roofRotation  = 0.0f;
    Bounds  roofBounds;
    float   sinR = 0.0f, cosR = 0.0f;
    double  roofTexSpanX = 0.0, roofTexSpanY = 0.0;
    osg::ref_ptr<const SpatialReference> roofProjSRS;

    if ( roofSkin )
    {
        roofBounds = input->getBounds();

        // if our data is lat/long, we need to reproject the geometry and the bounds into a projected
        // coordinate system in order to properly generate tex coords.
        if ( srs && srs->isGeographic() )
        {
            osg::Vec2d geogCenter = roofBounds.center2d();
            roofProjSRS = srs->createUTMFromLonLat( Angle(geogCenter.x()), Angle(geogCenter.y()) );
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
            //note: non-tiled roofs don't really work atm.
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

    // prep for wall texture coordinate generation.
    double texWidthM  = wallSkin ? *wallSkin->imageWidth() : 0.0;
    double texHeightM = wallSkin ? *wallSkin->imageHeight() : 1.0;

    ConstGeometryIterator iter( input );
    while( iter.hasMore() )
    {
        const Geometry* part = iter.next();

        // skip a part that's too small
        if (part->size() < 2)
            continue;

        // add a new wall.
        structure.elevations.push_back(Elevation());
        Elevation& elevation = structure.elevations.back();

        double maxHeight = targetLen - minLoc.z();

        // Adjust the texture height so it is a multiple of the maximum height
        double div = osg::round(maxHeight / texHeightM);
        elevation.texHeightAdjustedM = div > 0.0 ? maxHeight / div : maxHeight;

        // Step 1 - Create the real corners and transform them into our target SRS.
        Corners corners;
        for(Geometry::const_iterator m = part->begin(); m != part->end(); ++m)
        {
            Corners::iterator corner = corners.insert(corners.end(), Corner());
            
            // mark as "from source", as opposed to being inserted by the algorithm.
            corner->isFromSource = true;
            corner->base = *m;

            // extrude:
            if ( height >= 0 ) // extrude up
            {
                if ( flatten )
                    corner->roof.set( corner->base.x(), corner->base.y(), targetLen );
                else
                    corner->roof.set( corner->base.x(), corner->base.y(), corner->base.z() + height );
            }
            else // height < 0 .. extrude down
            {
                corner->roof = *m;
                corner->base.z() += height;
            }
            
            // figure out the rooftop texture coords before doing any transformation:
            if ( roofSkin && srs )
            {
                double xr, yr;

                if ( srs && srs->isGeographic() && roofProjSRS )
                {
                    osg::Vec3d projRoofPt;
                    srs->transform( corner->roof, roofProjSRS.get(), projRoofPt );
                    xr = (projRoofPt.x() - roofBounds.xMin());
                    yr = (projRoofPt.y() - roofBounds.yMin());
                }
                else
                {
                    xr = (corner->roof.x() - roofBounds.xMin());
                    yr = (corner->roof.y() - roofBounds.yMin());
                }

                corner->roofTexU = (cosR*xr - sinR*yr) / roofTexSpanX;
                corner->roofTexV = (sinR*xr + cosR*yr) / roofTexSpanY;
            }
            
            // transform into target SRS.
            transformAndLocalize( corner->base, srs, corner->base, mapSRS, _world2local, makeECEF );
            transformAndLocalize( corner->roof, srs, corner->roof, mapSRS, _world2local, makeECEF );
        }

        // Step 2 - Insert intermediate Corners as needed to satify texturing
        // requirements (if necessary) and record each corner offset (horizontal distance
        // from the beginning of the part geometry to the corner.)
        double cornerOffset    = 0.0;
        double nextTexBoundary = texWidthM;

        for(Corners::iterator c = corners.begin(); c != corners.end(); ++c)
        {
            Corners::iterator this_corner = c;

            Corners::iterator next_corner = c;
			bool isLastEdge = false;
			if ( ++next_corner == corners.end() )
			{
				isLastEdge = true;
				next_corner = corners.begin();
			}

            osg::Vec3d base_vec = next_corner->base - this_corner->base;
            double span = base_vec.length();

            this_corner->offsetX = cornerOffset;

            if (wallSkin)
            {
                base_vec /= span; // normalize
                osg::Vec3d roof_vec = next_corner->roof - this_corner->roof;
                roof_vec.normalize();

                while(nextTexBoundary < cornerOffset+span)
                {
                    // insert a new fake corner.
					Corners::iterator new_corner;

                    if ( isLastEdge )
                    {
						corners.push_back(Corner());
						new_corner = c;
						new_corner++;
                    }
                    else
                    {
						new_corner = corners.insert(next_corner, Corner());
					}

                    new_corner->isFromSource = false;
                    double advance = nextTexBoundary-cornerOffset;
                    new_corner->base = this_corner->base + base_vec*advance;
                    new_corner->roof = this_corner->roof + roof_vec*advance;
                    new_corner->offsetX = cornerOffset + advance;
                    nextTexBoundary += texWidthM;

                    // advance the main iterator
                    c = new_corner;
                }
            }

            cornerOffset += span;
        }

        // Step 3 - Calculate the angle of each corner.
        osg::Vec3d prev_vec;
        for(Corners::iterator c = corners.begin(); c != corners.end(); ++c)
        {
            Corners::const_iterator this_corner = c;

            Corners::const_iterator next_corner = c;
            if ( ++next_corner == corners.end() )
                next_corner = corners.begin();

            if ( this_corner == corners.begin() )
            {
                Corners::const_iterator prev_corner = corners.end();
                --prev_corner;
                prev_vec = this_corner->roof - prev_corner->roof;
                prev_vec.normalize();
            }

            osg::Vec3d this_vec = next_corner->roof - this_corner->roof;
            this_vec.normalize();
            if ( c != corners.begin() )
            {
                c->cosAngle = prev_vec * this_vec;
            }
        }

        // Step 4 - Create faces connecting each pair of Posts.
        Faces& faces = elevation.faces;
        for(Corners::const_iterator c = corners.begin(); c != corners.end(); ++c)
        {
            Corners::const_iterator this_corner = c;

            Corners::const_iterator next_corner = c;
            if ( ++next_corner == corners.end() )
                next_corner = corners.begin();
            
            // only close the shape for polygons.
            if (next_corner != corners.begin() || structure.isPolygon)
            {
                faces.push_back(Face());
                Face& face = faces.back();
                face.left  = *this_corner;
                face.right = *next_corner;

                // recalculate the final offset on the last face
                if ( next_corner == corners.begin() )
                {
                    osg::Vec3d vec = next_corner->roof - this_corner->roof;
                    face.right.offsetX = face.left.offsetX + vec.length();
                }

                face.widthM = next_corner->offsetX - this_corner->offsetX;
            }
        }
    }

    return true;
}


bool
ExtrudeGeometryFilter::buildWallGeometry(const Structure&     structure,
                                         osg::Geometry*       walls,
                                         const osg::Vec4&     wallColor,
                                         const osg::Vec4&     wallBaseColor,
                                         const SkinResource*  wallSkin)
{
    bool madeGeom = true;

    // 6 verts per face total (3 triangles)
    unsigned numWallVerts = 6 * structure.getNumPoints();

    double texWidthM   = wallSkin ? *wallSkin->imageWidth()  : 1.0;
    double texHeightM  = wallSkin ? *wallSkin->imageHeight() : 1.0;
    bool   useColor    = (!wallSkin || wallSkin->texEnvMode() != osg::TexEnv::DECAL) && !_makeStencilVolume;
    
    // Scale and bias:
    osg::Vec2f scale, bias;
    float layer;
    if ( wallSkin )
    {
        bias.set (wallSkin->imageBiasS().get(),  wallSkin->imageBiasT().get());
        scale.set(wallSkin->imageScaleS().get(), wallSkin->imageScaleT().get());
        layer = (float)wallSkin->imageLayer().get();
    }

    // create all the OSG geometry components
    osg::Vec3Array* verts = new osg::Vec3Array( numWallVerts );
    walls->setVertexArray( verts );
    
    osg::Vec3Array* tex = 0L;
    if ( wallSkin )
    { 
        tex = new osg::Vec3Array( numWallVerts );
        walls->setTexCoordArray( 0, tex );
    }

    osg::Vec4Array* colors = 0L;
    if ( useColor )
    {
        // per-vertex colors are necessary if we are going to use the MeshConsolidator -gw
        colors = new osg::Vec4Array( numWallVerts );
        walls->setColorArray( colors );
        walls->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }

    unsigned vertptr = 0;
    bool     tex_repeats_y = wallSkin && wallSkin->isTiled() == true;

    for(Elevations::const_iterator elev = structure.elevations.begin(); elev != structure.elevations.end(); ++elev)
    {
        osg::DrawElements* de = 
            numWallVerts > 0xFFFF ? (osg::DrawElements*) new osg::DrawElementsUInt  ( GL_TRIANGLES ) :
            numWallVerts > 0xFF   ? (osg::DrawElements*) new osg::DrawElementsUShort( GL_TRIANGLES ) :
                                    (osg::DrawElements*) new osg::DrawElementsUByte ( GL_TRIANGLES );

        // pre-allocate for speed
        de->reserveElements( numWallVerts );

        walls->addPrimitiveSet( de );

        for(Faces::const_iterator f = elev->faces.begin(); f != elev->faces.end(); ++f, vertptr+=6)
        {
            // set the 6 wall verts.
            (*verts)[vertptr+0] = f->left.roof;
            (*verts)[vertptr+1] = f->left.base;
            (*verts)[vertptr+2] = f->right.base;
            (*verts)[vertptr+3] = f->right.base;
            (*verts)[vertptr+4] = f->right.roof;
            (*verts)[vertptr+5] = f->left.roof;

            // Assign wall polygon colors.
            if (useColor)
            {
#if 0
                // experimental: apply some ambient occlusion to tight inside corners                
                float bL = f->left.cosAngle > 0.0 ? 1.0 : (1.0+f->left.cosAngle);
                float bR = f->right.cosAngle > 0.0 ? 1.0 : (1.0+f->right.cosAngle);

                osg::Vec4f leftColor      = Color(wallColor).brightness(bL);
                osg::Vec4f leftBaseColor  = Color(wallBaseColor).brightness(bL);
                osg::Vec4f rightColor     = Color(wallColor).brightness(bR);
                osg::Vec4f rightBaseColor = Color(wallBaseColor).brightness(bR);

                (*colors)[vertptr+0] = leftColor;
                (*colors)[vertptr+1] = leftBaseColor;
                (*colors)[vertptr+2] = rightBaseColor;
                (*colors)[vertptr+3] = rightBaseColor;
                (*colors)[vertptr+4] = rightColor;
                (*colors)[vertptr+5] = leftColor;
#else
                (*colors)[vertptr+0] = wallColor;
                (*colors)[vertptr+1] = wallBaseColor;
                (*colors)[vertptr+2] = wallBaseColor;
                (*colors)[vertptr+3] = wallBaseColor;
                (*colors)[vertptr+4] = wallColor;
                (*colors)[vertptr+5] = wallColor;
#endif
            }

            // Calculate texture coordinates:
            if (wallSkin)
            {
                // Calculate left and right corner V coordinates:
                double hL = tex_repeats_y ? (f->left.roof - f->left.base).length()   : elev->texHeightAdjustedM;
                double hR = tex_repeats_y ? (f->right.roof - f->right.base).length() : elev->texHeightAdjustedM;
                
                // Calculate the texture coordinates at each corner. The structure builder
                // will have spaced the verts correctly for this to work.
                float uL = fmod( f->left.offsetX, texWidthM ) / texWidthM;
                float uR = fmod( f->right.offsetX, texWidthM ) / texWidthM;

                // Correct for the case in which the rightmost corner is exactly on a
                // texture boundary.
                if ( uR < uL || (uL == 0.0 && uR == 0.0))
                    uR = 1.0f;

                osg::Vec2f texBaseL( uL, 0.0f );
                osg::Vec2f texBaseR( uR, 0.0f );
                osg::Vec2f texRoofL( uL, hL/elev->texHeightAdjustedM );
                osg::Vec2f texRoofR( uR, hR/elev->texHeightAdjustedM );

                texRoofL = bias + osg::componentMultiply(texRoofL, scale);
                texRoofR = bias + osg::componentMultiply(texRoofR, scale);
                texBaseL = bias + osg::componentMultiply(texBaseL, scale);
                texBaseR = bias + osg::componentMultiply(texBaseR, scale);

                (*tex)[vertptr+0].set( texRoofL.x(), texRoofL.y(), layer );
                (*tex)[vertptr+1].set( texBaseL.x(), texBaseL.y(), layer );
                (*tex)[vertptr+2].set( texBaseR.x(), texBaseR.y(), layer );
                (*tex)[vertptr+3].set( texBaseR.x(), texBaseR.y(), layer );
                (*tex)[vertptr+4].set( texRoofR.x(), texRoofR.y(), layer );
                (*tex)[vertptr+5].set( texRoofL.x(), texRoofL.y(), layer );
            }

            for(int i=0; i<6; ++i)
                de->addElement( vertptr+i );
        }
    }
    
    // generate per-vertex normals, altering the geometry as necessary to avoid
    // smoothing around sharp corners

    // TODO: reconsider this, given the new Structure setup
    // it won't actual smooth corners since we don't have shared edges.
    osgUtil::SmoothingVisitor::smooth(
        *walls,
        osg::DegreesToRadians(_wallAngleThresh_deg) );

    return madeGeom;
}


bool
ExtrudeGeometryFilter::buildRoofGeometry(const Structure&     structure,
                                         osg::Geometry*       roof,
                                         const osg::Vec4&     roofColor,
                                         const SkinResource*  roofSkin)
{    
    osg::Vec3Array* verts = new osg::Vec3Array();
    roof->setVertexArray( verts );

    osg::Vec4Array* color = new osg::Vec4Array();
    roof->setColorArray( color );
    roof->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    osg::Vec3Array* tex = 0L;
    if ( roofSkin )
    {
        tex = new osg::Vec3Array();
        roof->setTexCoordArray(0, tex);
    }

    // Create a series of line loops that the tessellator can reorganize
    // into polygons.
    unsigned vertptr = 0;
    for(Elevations::const_iterator e = structure.elevations.begin(); e != structure.elevations.end(); ++e)
    {
        unsigned elevptr = vertptr;
        for(Faces::const_iterator f = e->faces.begin(); f != e->faces.end(); ++f)
        {
            // Only use source verts; we skip interim verts inserted by the 
            // structure building since they are co-linear anyway and thus we don't
            // need them for the roof line.
            if ( f->left.isFromSource )
            {
                verts->push_back( f->left.roof );
                color->push_back( roofColor );
                if ( tex )
                {
                    tex->push_back( osg::Vec3f(f->left.roofTexU, f->left.roofTexV, (float)0.0f) );
                }
                ++vertptr;
            }
        }
        roof->addPrimitiveSet( new osg::DrawArrays(GL_LINE_LOOP, elevptr, vertptr-elevptr) );
    }
    

    osg::Vec3Array* normal = new osg::Vec3Array(verts->size());
    roof->setNormalArray( normal );
    roof->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    normal->assign( verts->size(), osg::Vec3(0,0,1) );

    // Tessellate the roof lines into polygons.
    osgUtil::Tessellator tess;
    tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
    tess.retessellatePolygons( *roof );

    return true;
}


bool
ExtrudeGeometryFilter::buildOutlineGeometry(const Structure&  structure,
                                            osg::Geometry*    outline,
                                            const osg::Vec4&  outlineColor,
                                            float             minCreaseAngleDeg)
{
    // minimum angle between adjacent faces for which to draw a post.
    const float cosMinAngle = cos(osg::DegreesToRadians(minCreaseAngleDeg));

    osg::Vec3Array* verts = new osg::Vec3Array();
    outline->setVertexArray( verts );

    osg::Vec4Array* color = new osg::Vec4Array();
    outline->setColorArray( color );
    outline->setColorBinding( osg::Geometry::BIND_OVERALL );
    color->push_back( outlineColor );

    osg::DrawElements* de = new osg::DrawElementsUInt(GL_LINES);
    outline->addPrimitiveSet(de);

    unsigned vertptr = 0;
    for(Elevations::const_iterator e = structure.elevations.begin(); e != structure.elevations.end(); ++e)
    {
        osg::Vec3d prev_vec;
        unsigned elevptr = vertptr;
        for(Faces::const_iterator f = e->faces.begin(); f != e->faces.end(); ++f)
        {
            // Only use source verts for posts.
            bool drawPost     = f->left.isFromSource;
            bool drawCrossbar = true;

            osg::Vec3d this_vec = f->right.roof - f->left.roof;
            this_vec.normalize();

            if (f->left.isFromSource && f != e->faces.begin())
            {
                drawPost = (this_vec * prev_vec) < cosMinAngle;
            }

            if ( drawPost || drawCrossbar )
            {
                verts->push_back( f->left.roof );
            }

            if ( drawPost )
            {
                verts->push_back( f->left.base );
                de->addElement(vertptr);
                de->addElement(verts->size()-1);
            }

            if ( drawCrossbar )
            {
                verts->push_back( f->right.roof );

                de->addElement(vertptr);
                de->addElement(verts->size()-1);
            }

            vertptr = verts->size();

            prev_vec = this_vec;
        }

        // Draw an end-post if this isn't a closed polygon.
        if ( !structure.isPolygon )
        {
            Faces::const_iterator last = e->faces.end()-1;
            verts->push_back( last->right.roof );
            de->addElement( verts->size()-1 );
            verts->push_back( last->right.base );
            de->addElement( verts->size()-1 );
        }
    }

    return true;
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

        // run a symbol script if present.
        if ( _extrusionSymbol->script().isSet() )
        {
            StringExpression temp( _extrusionSymbol->script().get() );
            input->eval( temp, &context );
        }

        // iterator over the parts.
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

            // Build the data model for the structure.
            Structure structure;

            buildStructure(
                part, 
                height, 
                offset, 
                _extrusionSymbol->flatten().get(),
                wallSkin,
                roofSkin,
                structure,
                context);

            // Create the walls.
            if ( walls.valid() )
            {
                osg::Vec4f wallColor(1,1,1,1), wallBaseColor(1,1,1,1);

                if ( _wallPolygonSymbol.valid() )
                {
                    wallColor = _wallPolygonSymbol->fill()->color();
                }

                if ( _extrusionSymbol->wallGradientPercentage().isSet() )
                {
                    wallBaseColor = Color(wallColor).brightness( 1.0 - *_extrusionSymbol->wallGradientPercentage() );
                }
                else
                {
                    wallBaseColor = wallColor;
                }

                buildWallGeometry(structure, walls.get(), wallColor, wallBaseColor, wallSkin);

                if ( wallSkin )
                {
                    // Get a stateset for the individual wall stateset
                    context.resourceCache()->getOrCreateStateSet( wallSkin, wallStateSet );
                }
            }

            // tessellate and add the roofs if necessary:
            if ( rooflines.valid() )
            {
                osg::Vec4f roofColor(1,1,1,1);
                if ( _roofPolygonSymbol.valid() )
                {
                    roofColor = _roofPolygonSymbol->fill()->color();
                }

                buildRoofGeometry(structure, rooflines.get(), roofColor, roofSkin);

                if ( roofSkin )
                {
                    // Get a stateset for the individual roof skin
                    context.resourceCache()->getOrCreateStateSet( roofSkin, roofStateSet );
                }
            }

            if ( outlines.valid() )
            {
                osg::Vec4f outlineColor(1,1,1,1);
                if ( _outlineSymbol.valid() )
                {
                    outlineColor = _outlineSymbol->stroke()->color();
                }

                float minCreaseAngle = _outlineSymbol->creaseAngle().value();
                buildOutlineGeometry(structure, outlines.get(), outlineColor, minCreaseAngle);
            }

            if ( baselines.valid() )
            {
                //TODO.
                osgUtil::Tessellator tess;
                tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
                tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
                tess.retessellatePolygons( *(baselines.get()) );
            }

            // Set up for feature naming and feature indexing:
            std::string name;
            if ( !_featureNameExpr.empty() )
                name = input->eval( _featureNameExpr, &context );

            FeatureSourceIndex* index = context.featureIndex();

            if ( walls.valid() )
            {
                addDrawable( walls.get(), wallStateSet.get(), name, input, index );
            }

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
            if ( context.featureIndex() || _outlineSymbol.valid())
            {
                // The MC will recognize the presence of feature indexing tags and
                // preserve them. The Cache optimizer however will not, so it is
                // out for now.
                // The Optimizer also doesn't work with line geometry, so if we have outlines
                // then we need to use MC.                
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
