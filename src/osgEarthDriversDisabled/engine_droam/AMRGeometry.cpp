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
#include "Common"
#include "AMRGeometry"
#include <osg/State>
#include <osg/Uniform>
#include <osgEarth/Notify>

#define LC "[AMRGeometry] "

// --------------------------------------------------------------------------

#include "AMRShaders.h"

// --------------------------------------------------------------------------

AMRTriangle::AMRTriangle()
{
    _stateSet = new osg::StateSet();

    // should this be INT_SAMPLER_2D?
    _stateSet->getOrCreateUniform( "tex0", osg::Uniform::INT )->set( 0 );
}

#define SET_UNIFORM(X,Y,Z) \
    _stateSet->getOrCreateUniform( X , Y )->set( Z )


AMRTriangle::AMRTriangle(const MeshNode& n0, const osg::Vec2& t0,
                         const MeshNode& n1, const osg::Vec2& t1, 
                         const MeshNode& n2, const osg::Vec2& t2) :
_node0(n0), _node1(n1), _node2(n2)
{
    _stateSet = new osg::StateSet();
    // should this be INT_SAMPLER_2D?
    SET_UNIFORM( "tex0", osg::Uniform::INT, 0 );

    SET_UNIFORM( "c0", osg::Uniform::FLOAT_VEC3, _node0._geodeticCoord );
    SET_UNIFORM( "c1", osg::Uniform::FLOAT_VEC3, _node1._geodeticCoord );
    SET_UNIFORM( "c2", osg::Uniform::FLOAT_VEC3, _node2._geodeticCoord );

    SET_UNIFORM( "v0", osg::Uniform::FLOAT_VEC3, _node0._vertex );
    SET_UNIFORM( "v1", osg::Uniform::FLOAT_VEC3, _node1._vertex );
    SET_UNIFORM( "v2", osg::Uniform::FLOAT_VEC3, _node2._vertex );

    SET_UNIFORM( "t0", osg::Uniform::FLOAT_VEC2, t0 );
    SET_UNIFORM( "t1", osg::Uniform::FLOAT_VEC2, t1 );
    SET_UNIFORM( "t2", osg::Uniform::FLOAT_VEC2, t2 );

    SET_UNIFORM( "n0", osg::Uniform::FLOAT_VEC3, _node0._normal );
    SET_UNIFORM( "n1", osg::Uniform::FLOAT_VEC3, _node1._normal );
    SET_UNIFORM( "n2", osg::Uniform::FLOAT_VEC3, _node2._normal );

    SET_UNIFORM( "r0", osg::Uniform::FLOAT_VEC4, _node0._geodeticRot.asVec4() );
    SET_UNIFORM( "r1", osg::Uniform::FLOAT_VEC4, _node1._geodeticRot.asVec4() );
    SET_UNIFORM( "r2", osg::Uniform::FLOAT_VEC4, _node2._geodeticRot.asVec4() );
}

void
AMRTriangle::expand( osg::BoundingBox& box )
{
    box.expandBy( _node0._vertex );
    box.expandBy( _node1._vertex );
    box.expandBy( _node2._vertex );
}

// --------------------------------------------------------------------------

AMRDrawable::AMRDrawable()
{
    _stateSet = new osg::StateSet();
}

// --------------------------------------------------------------------------

AMRGeometry::AMRGeometry()
{
    initShaders();
    initPatterns();

    //this->setBound( osg::BoundingBox(-1e10, -1e10, -1e10, 1e10, 1e10, 1e10) );
}

AMRGeometry::AMRGeometry( const AMRGeometry& rhs, const osg::CopyOp& op ) :
osg::Drawable( rhs, op ) //osg::Geometry( rhs, op )
{
    //todo
    setInitialBound( osg::BoundingBox(-1e10, -1e10, -1e10, 1e10, 1e10, 1e10) );
}

osg::BoundingBox
AMRGeometry::computeBound() const
{
    osg::BoundingBox box;
    for( AMRDrawableList::const_iterator i = _drawList.begin(); i != _drawList.end(); ++i )
    {
        const AMRTriangleList& prims = i->get()->_triangles;
        for( AMRTriangleList::const_iterator j = prims.begin(); j != prims.end(); ++j )
        {
            j->get()->expand( box );
        }
    } 
    return box;
}

void
AMRGeometry::clearDrawList()
{
    if ( _drawList.size() > 0 )
    {
        _drawList.clear();
        dirtyBound();
    }
}

void
AMRGeometry::setDrawList( const AMRDrawableList& drawList )
{
    _drawList = drawList;
    dirtyBound();
}

void
AMRGeometry::initShaders()
{
    // initialize the shader program.
    _program = new osg::Program();
    _program->setName( "AMRGeometry" );

    osg::Shader* vertexShader = new osg::Shader( osg::Shader::VERTEX,
        //std::string( source_vertShaderMain_flatMethod )
        std::string( source_vertShaderMain_geocentricMethod ) +
        std::string( source_geodeticToXYZ ) +
        std::string( source_rotVecToGeodetic )
        //std::string( source_vertShaderMain_latLonMethod )
        //std::string( source_vertShaderMain_slerpMethod )
        );

    vertexShader->setName( "AMR Vert Shader" );
    _program->addShader( vertexShader );

    osg::Shader* fragmentShader = new osg::Shader( osg::Shader::FRAGMENT,
        std::string( source_fragShaderMain )
        );

    fragmentShader->setName( "AMR Frag Shader" );
    _program->addShader( fragmentShader );

    // the shader program:
    this->getOrCreateStateSet()->setAttribute( _program.get(), osg::StateAttribute::ON );
}

static void
toBarycentric(const osg::Vec3& p1, const osg::Vec3& p2, const osg::Vec3& p3, 
              const osg::Vec3& in,
              osg::Vec3& outVert, osg::Vec2& outTex )
{
    //from: http://forums.cgsociety.org/archive/index.php/t-275372.html
    osg::Vec3 
        v1 = in - p1,
        v2 = in - p2,
        v3 = in - p3;

    double 
        area1 = 0.5 * (v2 ^ v3).length(),
        area2 = 0.5 * (v1 ^ v3).length(),
        area3 = 0.5 * (v1 ^ v2).length();

    double fullArea = area1 + area2 + area3;

    double u = area1/fullArea;
    double v = area2/fullArea;
    double w = area3/fullArea; 

    outVert.set( u, v, w );

    // tex coords
    osg::Vec2 t1( p1.x(), p1.y() );
    osg::Vec2 t2( p2.x(), p2.y() );
    osg::Vec2 t3( p3.x(), p3.y() );
    outTex = t1*w + t2*v + t3*u;
}


void
AMRGeometry::initPatterns()
{
    _numPatternVerts = 0;
    _numPatternElements = 0;
    _numPatternStrips = 0;
    _numPatternTriangles = 0;

    this->setUseVertexBufferObjects( true );
    this->setUseDisplayList( false );

    _patternVBO = new osg::VertexBufferObject();

    _verts = new osg::Vec3Array();
    _verts->setVertexBufferObject( _patternVBO.get() );

    _texCoords = new osg::Vec2Array();
    _texCoords->setVertexBufferObject( _patternVBO.get() );
 
    // build a right-triangle pattern. (0,0) is the lower-left (90d),
    // (0,1) is the lower right (45d) and (1,0) is the upper-left (45d)
    osg::Vec3f p1(0,0,0), p2(0,1,0), p3(1,0,0);

    for( int r=AMR_PATCH_ROWS-1; r >=0; --r )
    {
        int cols = AMR_PATCH_ROWS-r;
        //OE_INFO << "ROW " << r << std::endl;
        for( int c=0; c<cols; ++c )
        {
            osg::Vec3 point( (float)c/(float)(AMR_PATCH_ROWS-1), (float)r/(float)(AMR_PATCH_ROWS-1), 0 );
            osg::Vec3 baryVert;
            osg::Vec2 baryTex;
            toBarycentric( p1, p2, p3, point, baryVert, baryTex );
            _verts->push_back( baryVert );
            _texCoords->push_back( baryTex );
        }
    }
    _numPatternVerts = _verts->size();

    unsigned short off = 0;
    unsigned short rowptr = off;

    _patternEBO = new osg::ElementBufferObject();

    for( int r=1; r<AMR_PATCH_ROWS; ++r )
    {
        rowptr += r;
        osg::DrawElementsUShort* e = new osg::DrawElementsUShort( GL_TRIANGLE_STRIP );
        e->setElementBufferObject( _patternEBO.get() );            

        for( int c=0; c<=r; ++c )
        {
            e->push_back( rowptr + c );               
            if ( c < r )
                e->push_back( rowptr + c - r );
        }
        OE_INFO << std::endl;
        _pattern.push_back( e );

        _numPatternStrips++;
        _numPatternElements += e->size();
        _numPatternTriangles += (e->size()-1)/2;     
    }

    OE_INFO << LC
        << "Pattern: "   << std::dec
        << "verts="      << _numPatternVerts
        << ", strips="   << _numPatternStrips
        << ", tris="     << _numPatternTriangles
        << ", elements=" << _numPatternElements
        << std::endl;
}

static int s_numTemplates = 0;

void
AMRGeometry::drawImplementation( osg::RenderInfo& renderInfo ) const
{   
    osg::State& state = *renderInfo.getState();
    
    // bind the VBO:
    state.setVertexPointer( _verts.get() );

    // bind the texture coordinate arrrays:
    state.setTexCoordPointer( 0, _texCoords.get() );

    // this will enable the amr geometry's stateset (and activate the Program)
    state.pushStateSet( this->getStateSet() );
    //state.pushStateSet(0L);
    //_program->apply( state );

    int numTemplates = 0;

    for( AMRDrawableList::const_iterator i = _drawList.begin(); i != _drawList.end(); ++i )
    {
        const AMRDrawable* drawable = i->get();

        // apply the drawable's state changes:
        state.pushStateSet( drawable->_stateSet.get() );

        for( AMRTriangleList::const_iterator j = drawable->_triangles.begin(); j != drawable->_triangles.end(); ++j )
        {
            const AMRTriangle* dtemplate = j->get();

            // apply the primitive's state changes:
            state.apply( dtemplate->_stateSet.get() );

            // render the pattern (a collection of primitive sets)
            for( Pattern::const_iterator p = _pattern.begin(); p != _pattern.end(); ++p )
            {
                p->get()->draw( state, true );
            }

            numTemplates++;
        }

        state.popStateSet();
    }

    if ( s_numTemplates != numTemplates )
    {
        s_numTemplates = numTemplates;
        OE_INFO << LC << std::dec 
            << "templates="  << numTemplates
            << ", verts="    << numTemplates*_numPatternVerts
            << ", strips="   << numTemplates*_numPatternStrips
            << ", tris="     << numTemplates*_numPatternTriangles
            << ", elements=" << numTemplates*_numPatternElements
            << std::endl;
    }

    // unbind the buffer objects.
    state.unbindVertexBufferObject();
    state.unbindElementBufferObject();

    // undo the program.
    state.popStateSet();
}
