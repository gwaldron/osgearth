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
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthSymbology/MeshConsolidator>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ClusterCullingCallback>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>
#include <osg/Version>
#include <osgEarth/Version>

#define LC "[ExtrudeGeometryFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


ExtrudeGeometryFilter::ExtrudeGeometryFilter() :
_maxAngle_deg( 5.0 ),
_mergeGeometry( false ),
_height( 10.0 ),
_flatten( true ),
_wallAngleThresh_deg( 60.0 ),
_color( osg::Vec4f(1, 1, 1, 1) )
{
    reset();
}

void
ExtrudeGeometryFilter::reset()
{
    _geode = new osg::Geode();
    _cosWallAngleThresh = cos( _wallAngleThresh_deg );
}

#undef USE_TEX

bool
ExtrudeGeometryFilter::extrudeGeometry(const Geometry*         input,
                                       double                  height,
                                       bool                    flatten,
                                       osg::Geometry*          walls,
                                       osg::Geometry*          topCap,
                                       osg::Geometry*          bottomCap,
                                       const osg::Vec4&        color,
                                       const FilterContext&    cx )
{
    bool made_geom = false;

#ifdef USE_TEX
    double tex_width_m = skin? skin->getTextureWidthMeters() : 1.0;
    double tex_height_m = skin? skin->getTextureHeightMeters() : 1.0;
    //Adjust the texture height so it is a multiple of the extrusion height
    bool   tex_repeats_y = skin? skin->getRepeatsVertically() : true;
#else
    double tex_width_m = 1.0;
    double tex_height_m = 1.0;
    bool   tex_repeats_y = true;
#endif

    bool isPolygon = input->getComponentType() == Geometry::TYPE_POLYGON;

    unsigned pointCount = input->getTotalPointCount();
    unsigned numVerts = 2 * pointCount;

    // create all the OSG geometry components
    osg::Vec3Array* verts = new osg::Vec3Array( numVerts );
    walls->setVertexArray( verts );

#ifdef USE_TEX
    osg::Vec2Array* texcoords = new osg::Vec2Array( numVerts );
    walls->setTexCoordArray( 0, texcoords );
#endif

    osg::Vec4Array* colors = new osg::Vec4Array( numVerts );
    walls->setColorArray( colors );
    walls->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    osg::Vec3Array* topVerts = NULL;
    osg::Vec4Array* topColors = NULL;
    if ( topCap )
    {
        topVerts = new osg::Vec3Array( pointCount );
        topCap->setVertexArray( topVerts );

        topColors = new osg::Vec4Array( pointCount );
        topCap->setColorArray( topColors );
        topCap->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }

    osg::Vec3Array* bottomVerts = NULL;
    if ( bottomCap )
    {
        bottomVerts = new osg::Vec3Array( pointCount );
        bottomCap->setVertexArray( bottomVerts );
    }

    unsigned wallVertPtr   = 0;
    unsigned topVertPtr    = 0;
    unsigned bottomVertPtr = 0;

    double targetLen = -DBL_MAX;
    osg::Vec3d minLoc(DBL_MAX, DBL_MAX, DBL_MAX);


    // Initial pass over the geometry does two things:
    // 1: Calculate the minimum Z across all parts.
    // 2: Establish a "target length" for extrusion
    ConstGeometryIterator zfinder( input );
    while( zfinder.hasMore() )
    {
        const Geometry* geom = zfinder.next();
        for( Geometry::const_iterator m = geom->begin(); m != geom->end(); ++m )
        {
            // Find the minimum Z
            osg::Vec3d m_world = cx.toWorld( *m );
            if ( cx.isGeocentric() )
            {
                osg::Vec3d p_vec = m_world;
                osg::Vec3d e_vec = p_vec;
                e_vec.normalize();
                p_vec = p_vec + (e_vec * height);

                if ( m_world.length() < minLoc.length() )
                    minLoc = m_world;

                double p_ex_len = p_vec.length();
                if ( p_ex_len > targetLen )
                    targetLen = p_ex_len;
            }
            else
            {
                if ( m_world.z() + height > targetLen )
                    targetLen = m_world.z() + height;

                if (m_world.z() < minLoc.z())
                    minLoc = m_world;
            }
        }
    }

    // now generate the extruded geometry.
    ConstGeometryIterator iter( input );
    while( iter.hasMore() )
    {
        const Geometry* part = iter.next();

        double tex_height_m_adj = tex_height_m;

        unsigned wallPartPtr = wallVertPtr;
        unsigned topPartPtr  = topVertPtr;
        unsigned bottomPartPtr = bottomVertPtr;
        double   partLen = 0.0;
        double   maxHeight = 0.0;

        if ( cx.isGeocentric() )
            maxHeight = targetLen - minLoc.length();
        else
            maxHeight = targetLen - minLoc.z();

        //Adjust the texture height so it is a multiple of the maximum height
        double div = osg::round(maxHeight / tex_height_m);
        if (div == 0) div = 1; //Prevent divide by zero
        tex_height_m_adj = maxHeight / div;

        osg::DrawElementsUInt* idx = new osg::DrawElementsUInt( GL_TRIANGLES );

        for( Geometry::const_iterator m = part->begin(); m != part->end(); ++m )
        {
            osg::Vec3d m_world = cx.toWorld( *m );

            // calculate our extrusion vector for this point.
            osg::Vec3d extrudeVec;
            if ( cx.isGeocentric() )
            {
                osg::Vec3d p_vec = m_world;

                if ( flatten )
                {
                    double p_len = p_vec.length();
                    double ratio = targetLen/p_len;
                    p_vec *= ratio;
                }
                else
                {
                    osg::Vec3d unitVec = p_vec;
                    unitVec.normalize();
                    p_vec = p_vec + unitVec*height;
                }

                extrudeVec = p_vec * cx.referenceFrame();
            }
            else
            {
                if ( flatten )
                    extrudeVec.set( m_world.x(), m_world.y(), targetLen );
                else
                    extrudeVec.set( m_world.x(), m_world.y(), m_world.z() + height );

                extrudeVec = extrudeVec * cx.referenceFrame();
            }

            // build the top and bottom caps
            if ( topCap )
            {
                (*topColors)[topVertPtr]  = color;
                (*topVerts)[topVertPtr++] = extrudeVec;
            }
            if ( bottomCap )
            {
                (*bottomVerts)[bottomVertPtr++] = *m;
            }
             
            partLen += wallVertPtr > wallPartPtr ?
                (extrudeVec - (*verts)[wallVertPtr-2]).length() :
                0.0;

            double h;
            if ( tex_repeats_y )
                h = -(extrudeVec - *m).length();
            else
                h = -tex_height_m_adj;

            int p;

            p = wallVertPtr; // ++
            (*colors)[p] = color;
            (*verts)[p] = extrudeVec;
#ifdef USE_TEX
            (*texcoords)[p].set( part_len/tex_width_m, 0.0f );
#endif

            p = wallVertPtr+1; // ++
            (*colors)[p] = color;
            (*verts)[p] = *m;
#ifdef USE_TEX
            (*texcoords)[p].set( part_len/tex_width_m, h/tex_height_m_adj );
#endif

            // form the 2 triangles
            if ( (m+1) == part->end() )
            {
                if ( isPolygon )
                {
                    // end of the wall; loop around to close it off.
                    idx->push_back(wallVertPtr); 
                    idx->push_back(wallVertPtr+1);
                    idx->push_back(wallPartPtr);

                    idx->push_back(wallVertPtr+1);
                    idx->push_back(wallPartPtr+1);
                    idx->push_back(wallPartPtr);
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

        if ( topCap )
        {
            topCap->addPrimitiveSet( new osg::DrawArrays(
                osg::PrimitiveSet::LINE_LOOP,
                topPartPtr, topVertPtr - topPartPtr ) );
        }
        if ( bottomCap )
        {
            // reverse the bottom verts:
            int len = bottomVertPtr - bottomPartPtr;
            for( int i=bottomPartPtr; i<len/2; i++ )
                std::swap( (*bottomVerts)[i], (*bottomVerts)[bottomPartPtr+(len-1)-i] );

            bottomCap->addPrimitiveSet( new osg::DrawArrays(
                osg::PrimitiveSet::LINE_LOOP,
                bottomPartPtr, bottomVertPtr - bottomPartPtr ) );
        }
    }

    return made_geom;
}

bool
ExtrudeGeometryFilter::pushFeature( Feature* input, const FilterContext& context )
{
    GeometryIterator iter( input->getGeometry(), false );
    while( iter.hasMore() )
    {
        Geometry* part = iter.next();

        osg::ref_ptr<osg::Geometry> walls = new osg::Geometry();
        
        osg::ref_ptr<osg::Geometry> rooflines = 0L;
        
        if ( part->getType() == Geometry::TYPE_POLYGON )
        {
            rooflines = new osg::Geometry();

            // prep the shapes by making sure all polys are open:
            static_cast<Polygon*>(part)->open();
        }

        float height;

        if ( _heightCallback.valid() )
        {
            height = _heightCallback->operator()(input, context);
        }
        else if ( _heightAttr.isSet() )
        {
            height = as<float>(input->getAttr(*_heightAttr), _height);
        }
        else if ( _heightExpr.isSet() )
        {
            height = input->eval( _heightExpr.mutable_value() );
        }
        else
        {
            height = _height;
        }

        if ( extrudeGeometry( part, height, _flatten, walls.get(), rooflines.get(), 0L, _color, context ) )
        {      
#ifdef USE_TEX
            if ( skin )
            {
                osg::StateSet* wall_ss = env->getResourceCache()->getStateSet( skin );
                if ( wall_ss )
                {
                    walls->setStateSet( wall_ss );
                }
                //env->getSession()->getResources()->getStateSet( skin ) );
                //env->getSession()->markResourceUsed( skin );
            }
            else
#endif
            {
                //There is no skin, so disable texturing for the walls to prevent other textures from being applied to the walls
                if ( !_noTextureStateSet.valid() )
                {
                    _noTextureStateSet = new osg::StateSet();
                    _noTextureStateSet->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::OFF);
                }

                walls->setStateSet( _noTextureStateSet.get() );
            }

            // generate per-vertex normals, altering the geometry as necessary to avoid
            // smoothing around sharp corners
#if OSG_MIN_VERSION_REQUIRED(2,9,9)
            //Crease angle threshold wasn't added until
            osgUtil::SmoothingVisitor::smooth(
                *walls.get(), 
                osg::DegreesToRadians(_wallAngleThresh_deg) );            
#else
            osgUtil::SmoothingVisitor::smooth(*walls.get());            
#endif

            // tessellate and add the roofs if necessary:
            if ( rooflines.valid() )
            {
                osgUtil::Tessellator tess;
                tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
                tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD ); //POSITIVE );
                tess.retessellatePolygons( *(rooflines.get()) );

                // generate default normals (no crease angle necessary; they are all pointing up)
                osgUtil::SmoothingVisitor::smooth( *rooflines.get() );

                // texture the rooflines if necessary
                //applyOverlayTexturing( rooflines.get(), input, env );
                
                // reorganize the drawable into a single triangle set.
                MeshConsolidator::run( *rooflines.get() );

                // mark this geometry as DYNAMIC because otherwise the OSG optimizer will destroy it.
                rooflines->setDataVariance( osg::Object::DYNAMIC );
            }

            //applyFragmentName( new_fragment, input, env );

            _geode->addDrawable( walls.get() );

            if ( rooflines.valid() )
                _geode->addDrawable( rooflines.get() );
        }   
    }


    return true;
}

namespace 
{
    struct EnableVBO : public osg::NodeVisitor 
    {
        EnableVBO() : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ) { }

        void accept( osg::Geode& geode )
        {
            for( unsigned i=0; i<geode.getNumDrawables(); ++i )
            {
                osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
                if ( geom )
                {
                    geom->setUseVertexBufferObjects( true );
                }
            }
            traverse( geode );
        }
    };
}

osg::Node*
ExtrudeGeometryFilter::push( FeatureList& input, const FilterContext& context )
{
    reset();

    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        pushFeature( i->get(), context );

    // BREAKS if you use VBOs - make sure they're disabled
    osgUtil::Optimizer optimizer;
    optimizer.optimize( _geode.get(), osgUtil::Optimizer::MERGE_GEOMETRY );

    //optimizer.optimize( _geode.get(), osgUtil::Optimizer::INDEX_MESH );

    //TODO: figure out whether this helps
    //optimizer.optimize( _geode, osgUtil::Optimizer::VERTEX_PRETRANSFORM | osgUtil::Optimizer::VERTEX_POSTTRANSFORM );
    
    // activate the VBOs after optimization
    EnableVBO visitor;
    _geode->accept( visitor );

    return _geode.release();
}
