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
#include "AMRGeometry"
#include <osg/State>
#include <osg/Uniform>
#include <osgEarth/Notify>

#define LC "[osgEarth::AMRGeometry] "

// --------------------------------------------------------------------------

#include "AMRShaders.h"

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
AMRGeometry::setDrawList( const AMRTriangleList& triangles )
{
    _drawList = triangles;
}

void
AMRGeometry::drawImplementation( osg::RenderInfo& renderInfo ) const
{   
    // Copied all this from Geometry::drawImplementation. 
    // TODO: cull it down to just what we need.
    osg::State& state = *renderInfo.getState();

    bool checkForGLErrors = state.getCheckForGLErrors()==osg::State::ONCE_PER_ATTRIBUTE;
    if (checkForGLErrors) state.checkGLErrors("start of Geometry::drawImplementation()");
    
    bool useFastPath = areFastPathsUsed();
    // useFastPath = false;

    bool usingVertexBufferObjects = _useVertexBufferObjects && state.isVertexBufferObjectSupported();
    bool handleVertexAttributes = !_vertexAttribList.empty();

    osg::ArrayDispatchers& arrayDispatchers = state.getArrayDispatchers();

    arrayDispatchers.reset();
    arrayDispatchers.setUseVertexAttribAlias(useFastPath && state.getUseVertexAttributeAliasing());
    arrayDispatchers.setUseGLBeginEndAdapter(!useFastPath);

    arrayDispatchers.activateNormalArray(_normalData.binding, _normalData.array.get(), _normalData.indices.get());
    arrayDispatchers.activateColorArray(_colorData.binding, _colorData.array.get(), _colorData.indices.get());
    arrayDispatchers.activateSecondaryColorArray(_secondaryColorData.binding, _secondaryColorData.array.get(), _secondaryColorData.indices.get());
    arrayDispatchers.activateFogCoordArray(_fogCoordData.binding, _fogCoordData.array.get(), _fogCoordData.indices.get());

    if (handleVertexAttributes)
    {
        for(unsigned int unit=0;unit<_vertexAttribList.size();++unit)
        {
            arrayDispatchers.activateVertexAttribArray(_vertexAttribList[unit].binding, unit, _vertexAttribList[unit].array.get(), _vertexAttribList[unit].indices.get());
        }
    }

    // dispatch any attributes that are bound overall
    arrayDispatchers.dispatch(BIND_OVERALL,0);

    state.lazyDisablingOfVertexAttributes();

    if (useFastPath)
    {
        // set up arrays
        if( _vertexData.array.valid() )
            state.setVertexPointer(_vertexData.array.get());

        if (_normalData.binding==BIND_PER_VERTEX && _normalData.array.valid())
            state.setNormalPointer(_normalData.array.get());

        if (_colorData.binding==BIND_PER_VERTEX && _colorData.array.valid())
            state.setColorPointer(_colorData.array.get());

        if (_secondaryColorData.binding==BIND_PER_VERTEX && _secondaryColorData.array.valid())
            state.setSecondaryColorPointer(_secondaryColorData.array.get());

        if (_fogCoordData.binding==BIND_PER_VERTEX && _fogCoordData.array.valid())
            state.setFogCoordPointer(_fogCoordData.array.get());

        for(unsigned int unit=0;unit<_texCoordList.size();++unit)
        {
            const osg::Array* array = _texCoordList[unit].array.get();
            if (array) state.setTexCoordPointer(unit,array);
        }

        if( handleVertexAttributes )
        {
            for(unsigned int index = 0; index < _vertexAttribList.size(); ++index )
            {
                const osg::Array* array = _vertexAttribList[index].array.get();
                const AttributeBinding ab = _vertexAttribList[index].binding;
                if( ab == BIND_PER_VERTEX && array )
                {
                    state.setVertexAttribPointer( index, array, _vertexAttribList[index].normalize );
                }
            }
        }
    }
    else
    {
        for(unsigned int unit=0;unit<_texCoordList.size();++unit)
        {
            arrayDispatchers.activateTexCoordArray(BIND_PER_VERTEX, unit, _texCoordList[unit].array.get(), _texCoordList[unit].indices.get());
        }

        arrayDispatchers.activateVertexArray(BIND_PER_VERTEX, _vertexData.array.get(), _vertexData.indices.get());
    }

    state.applyDisablingOfVertexAttributes();

    bool bindPerPrimitiveSetActive = arrayDispatchers.active(BIND_PER_PRIMITIVE_SET);
    bool bindPerPrimitiveActive = arrayDispatchers.active(BIND_PER_PRIMITIVE);

    unsigned int primitiveNum = 0;

    if (checkForGLErrors) state.checkGLErrors("Geometry::drawImplementation() after vertex arrays setup.");


    //OE_INFO << LC << "Rendering " << _drawList.size() << " tris" << std::endl;

    // this will enable the amrgeometry's stateset (and activate the Program)
    state.pushStateSet( this->getStateSet() );

    // foreach triangle:
    //    apply its stateset (settings its uniforms)
    //    select the appropriate pattern (only one for now)
    //    render the pattern EBO
    for( AMRTriangleList::const_iterator i = _drawList.begin(); i != _drawList.end(); ++i )
    {
        state.apply( i->get()->_stateSet.get() );

        // render the pattern (a collection of primitive sets)
        for( Pattern::const_iterator p = _pattern.begin(); p != _pattern.end(); ++p )
        {
            p->get()->draw( state, true );
        }
    }

    // unbind the buffer objects.
    state.unbindVertexBufferObject();
    state.unbindElementBufferObject();

    // undo the program.
    state.popStateSet();
}

void
AMRGeometry::initShaders()
{
    // initialize the shader program.
    _program = new osg::Program();
    _program->setName( "AMRGeometry" );
    
    std::string vertShaderSource =
        //std::string(fnormal_source) + 
        //std::string(directionalLight_source) + 
        //std::string(xyz_to_lat_lon_height_source) +
        std::string(vert_shader_source);

    osg::Shader* vertexShader = new osg::Shader( osg::Shader::VERTEX, vertShaderSource );
    vertexShader->setName( "AMR Vert Shader" );
    _program->addShader( vertexShader );

    osg::Shader* fragmentShader = new osg::Shader( osg::Shader::FRAGMENT, frag_shader_source );
    fragmentShader->setName( "AMR Frag Shader" );
    _program->addShader( fragmentShader );

    // hopefully this is sufficient
    this->getOrCreateStateSet()->setAttributeAndModes( _program.get(), osg::StateAttribute::ON );
}

static void
toBarycentric( const osg::Vec3f& p1, const osg::Vec3f& p2, const osg::Vec3f& p3, const osg::Vec3f& in, osg::Vec3f& out )
{
    //from: http://forums.cgsociety.org/archive/index.php/t-275372.html
    osg::Vec3f 
        v1 = in - p1,
        v2 = in - p2, 
        v3 = in - p3;

    double 
        area1 = 0.5 * (v2 ^ v3).length(),
        area2 = 0.5 * (v1 ^ v3).length(),
        area3 = 0.5 * (v1 ^ v2).length();

    double fullArea = area1 + area2 + area3;

    out.set( area1/fullArea, area2/fullArea, area3/fullArea );
}

#define ROWS 8
//#define ROWS 1

void
AMRGeometry::initPatterns()
{
    this->setUseVertexBufferObjects( true );

    //todo- build triangle AMR pattern(s) in barycentric coordinate space.
    osg::Vec3Array* v = new osg::Vec3Array();
    this->setVertexArray( v );
 
    // build a right-triangle pattern. (0,0) is the lower-left (90d),
    // (0,1) is the lower right (45d) and (1,0) is the upper-left (45d)
    {
        osg::Vec3f p1(0,0,0), p2(0,1,0), p3(1,0,0);

        for( int r=ROWS-1; r >=0; --r )
        {
            int cols = ROWS-r;
            OE_NOTICE << "ROW " << r << std::endl;
            for( int c=0; c<cols; ++c )
            {
                osg::Vec3f ec( (float)c/(float)(ROWS-1), (float)r/(float)(ROWS-1), 0 );
                osg::Vec3f bc;
                toBarycentric( p1, p2, p3, ec, bc );
                v->push_back( osg::Vec3f( bc.x(), bc.y(), bc.z() ) );

                OE_NOTICE << "   " << std::fixed 
                    << ec.x() << "," << ec.y() << "," << ec.z() << " ==> "
                    << bc.x() << "," << bc.y() << "," << bc.z() << std::endl;
            }
        }

        unsigned short off = 0;
        unsigned short rowptr = off;

        for( int r=1; r<ROWS; ++r )
        {
            OE_NOTICE << "ROW " << r << std::endl;
            rowptr += r;
            osg::DrawElementsUShort* e = new osg::DrawElementsUShort( GL_TRIANGLE_STRIP );
            e->setElementBufferObject( this->getOrCreateElementBufferObject() );

            for( int c=0; c<=r; ++c )
            {
                e->push_back( rowptr + c );
                OE_NOTICE << " " << rowptr+c << ",";
                
                if ( c < r ) {
                    e->push_back( rowptr + c - r );
                    OE_NOTICE << rowptr+c-r << ",";
                }
            }
            OE_NOTICE << std::endl;
            _pattern.push_back( e );

            OE_INFO << LC << "Added element to pattern, size = " << e->size() << std::endl;
        }
    }
}

// --------------------------------------------------------------------------

AMRTriangle::AMRTriangle(const osg::Vec3d& p0, const osg::Vec3d& p1, const osg::Vec3d& p2,
                         const osg::Vec3d& n0, const osg::Vec3d& n1, const osg::Vec3d& n2)
                         //const osg::Vec2f& t0, const osg::Vec2f& t1, const osg::Vec2f& t2)
{
    _stateSet = new osg::StateSet();

    // the triangle points:
    _stateSet->addUniform( new osg::Uniform( "p0", p0 ) );
    _stateSet->addUniform( new osg::Uniform( "p1", p1 ) );
    _stateSet->addUniform( new osg::Uniform( "p2", p2 ) );

    // the normals:
    _stateSet->addUniform( new osg::Uniform( "n0", p0 ) );
    _stateSet->addUniform( new osg::Uniform( "n1", p1 ) );
    _stateSet->addUniform( new osg::Uniform( "n2", p2 ) );

    // the texcoords (unit 0)
    // maybe put this in an array, eventually
    //_stateSet->addUniform( new osg::Uniform( "t0", t0 ) );
    //_stateSet->addUniform( new osg::Uniform( "t1", t1 ) );
    //_stateSet->addUniform( new osg::Uniform( "t2", t2 ) );

    //todo: add displacement texture
    //todo: add color texture
}
