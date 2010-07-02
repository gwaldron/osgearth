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
#include "Common"
#include "AMRGeometry"
#include <osg/State>
#include <osg/Uniform>
#include <osgEarth/Notify>

#define LC "[osgEarth::AMRGeometry] "

// --------------------------------------------------------------------------

#include "AMRShaders.h"

// --------------------------------------------------------------------------

AMRTriangle::AMRTriangle()
{
    _stateSet = new osg::StateSet();

    // should this be INT_SAMPLER_2D?
    _stateSet->getOrCreateUniform( "tex0", osg::Uniform::INT )->set( 0 );
}

void
AMRTriangle::setVerts( const osg::Vec3& p0, const osg::Vec3& p1, const osg::Vec3& p2 )
{
    _p0 = p0, _p1 = p1, _p2 = p2;
    setUniform( "p0", p0 );
    setUniform( "p1", p1 );
    setUniform( "p2", p2 );
}

//void
//AMRTriangle::setHeights( float h0, float h1, float h2 )
//{
//    _stateSet->getOrCreateUniform( "h0", osg::Uniform::FLOAT )->set( h0 );
//    _stateSet->getOrCreateUniform( "h1", osg::Uniform::FLOAT )->set( h1 );
//    _stateSet->getOrCreateUniform( "h2", osg::Uniform::FLOAT )->set( h2 );
//}

void 
AMRTriangle::setNormals(  const osg::Vec3& n0, const osg::Vec3& n1, const osg::Vec3& n2 )
{
    _n0 = n0, _n1 = n1, _n2 = n2;
    setUniform( "n0", n0 );
    setUniform( "n1", n1 );
    setUniform( "n2", n2 );
}

void
AMRTriangle::setTexCoords( int unit, const osg::Vec2& t0, const osg::Vec2& t1, const osg::Vec2& t2 )
{
    setUniform( "t0", t0 );
    setUniform( "t1", t1 );
    setUniform( "t2", t2 );
}

void
AMRTriangle::setUniform( const std::string& name, const osg::Vec3& value )
{
    _stateSet->getOrCreateUniform( name, osg::Uniform::FLOAT_VEC3 )->set( value );
}

void
AMRTriangle::setUniform( const std::string& name, const osg::Vec2& value )
{
    _stateSet->getOrCreateUniform( name, osg::Uniform::FLOAT_VEC2 )->set( value );
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

    this->setBound( osg::BoundingBox(-1e10, -1e10, -1e10, 1e10, 1e10, 1e10) );
}

AMRGeometry::AMRGeometry( const AMRGeometry& rhs, const osg::CopyOp& op ) :
osg::Geometry( rhs, op )
{
    //todo
}

void
AMRGeometry::clearDrawList()
{
    _drawList.clear();
}

void
AMRGeometry::setDrawList( const AMRDrawableList& drawList )
{
    _drawList = drawList;

    osg::BoundingBox box;
    for( AMRDrawableList::const_iterator i = _drawList.begin(); i != _drawList.end(); ++i )
    {
        const AMRTriangleList& prims = i->get()->_primitives;
        for( AMRTriangleList::const_iterator j = prims.begin(); j != prims.end(); ++j )
        {
            box.expandBy( j->get()->_p0 );
            box.expandBy( j->get()->_p1 );
            box.expandBy( j->get()->_p2 );
        }
    }
    setBound( box );
}

void
AMRGeometry::initShaders()
{
    // initialize the shader program.
    _program = new osg::Program();
    _program->setName( "AMRGeometry" );

    osg::Shader* vertexShader = new osg::Shader( osg::Shader::VERTEX,
        std::string( source_slerp ) +
        std::string( source_vertShaderMain )
        );

    vertexShader->setName( "AMR Vert Shader" );
    _program->addShader( vertexShader );

    osg::Shader* fragmentShader = new osg::Shader( osg::Shader::FRAGMENT,
        std::string( source_fragShaderMain )
        );

    fragmentShader->setName( "AMR Frag Shader" );
    _program->addShader( fragmentShader );

    // the shader program:
    this->getOrCreateStateSet()->setAttributeAndModes( _program.get(), osg::StateAttribute::ON );
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
    _patternSize = 0;

    this->setUseVertexBufferObjects( true );

    osg::Vec3Array* v = new osg::Vec3Array();
    this->setVertexArray( v );

    osg::Vec2Array* t0 = new osg::Vec2Array();
    this->setTexCoordArray( 0, t0 );
 
    // build a right-triangle pattern. (0,0) is the lower-left (90d),
    // (0,1) is the lower right (45d) and (1,0) is the upper-left (45d)
    {
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
                v->push_back( baryVert );
                t0->push_back( baryTex );

                //OE_INFO << "   " << std::fixed 
                //    << ec.x() << "," << ec.y() << "," << ec.z() << " ==> "
                //    << bc.x() << "," << bc.y() << "," << bc.z() << std::endl;
            }
        }

        unsigned short off = 0;
        unsigned short rowptr = off;

        for( int r=1; r<AMR_PATCH_ROWS; ++r )
        {
            //OE_INFO << "ROW " << r << std::endl;
            rowptr += r;
            osg::DrawElementsUShort* e = new osg::DrawElementsUShort( GL_TRIANGLE_STRIP );
            e->setElementBufferObject( this->getOrCreateElementBufferObject() );

            for( int c=0; c<=r; ++c )
            {
                e->push_back( rowptr + c );
                //OE_INFO << " " << rowptr+c << ",";
                
                if ( c < r ) {
                    e->push_back( rowptr + c - r );
                    //OE_INFO << rowptr+c-r << ",";
                }
            }
            OE_INFO << std::endl;
            _pattern.push_back( e );

            //OE_INFO << LC << "Added element to pattern, size = " << e->size() << std::endl;
            _patternSize += e->size();
        }
    }
}


void
AMRGeometry::drawImplementation( osg::RenderInfo& renderInfo ) const
{   
    // Copied all this from Geometry::drawImplementation. 
    // TODO: cull it down to just what we need.
    osg::State& state = *renderInfo.getState();

    // bind the VBO:
    state.setVertexPointer(_vertexData.array.get());

    // bind the texture coordinate arrrays:
    for( unsigned int unit = 0; unit < _texCoordList.size(); ++unit )
    {
        const osg::Array* array = _texCoordList[unit].array.get();
        if (array) state.setTexCoordPointer( unit, array );
    }

    //OE_INFO << LC 
    //    << "Rendering " << _drawList.size() << " tris, "
    //    << _drawList.size()*_patternSize << " verts" << std::endl;

    // this will enable the amrgeometry's stateset (and activate the Program)
    state.pushStateSet( this->getStateSet() );

    for( AMRDrawableList::const_iterator i = _drawList.begin(); i != _drawList.end(); ++i )
    {
        const AMRDrawable* drawable = i->get();

        // apply the drawable's state changes:
        state.pushStateSet( drawable->_stateSet.get() );

        for( AMRTriangleList::const_iterator j = drawable->_primitives.begin(); j != drawable->_primitives.end(); ++j )
        {
            const AMRTriangle* prim = j->get();

            // apply the primitive's state changes:
            state.apply( prim->_stateSet.get() );

            // render the pattern (a collection of primitive sets)
            for( Pattern::const_iterator p = _pattern.begin(); p != _pattern.end(); ++p )
            {
                p->get()->draw( state, true );
            }
        }

        state.popStateSet();
    }

    // unbind the buffer objects.
    state.unbindVertexBufferObject();
    state.unbindElementBufferObject();

    // undo the program.
    state.popStateSet();
}
