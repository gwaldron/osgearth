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
#include "TileDrawable"
#include "MPTexture"

#include <osg/Version>
#include <osgUtil/MeshOptimizers>
#include <iterator>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

using namespace osg;
using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[TileDrawable] "


TileDrawable::TileDrawable(const TileKey&        key,
                           const RenderBindings& bindings,
                           osg::Geometry*        geometry,
                           osg::Geometry*        proxygeometry) :
osg::Drawable( ),
_key         ( key ),
_bindings    ( bindings ),
_geom        ( geometry ),
_proxyGeom   ( proxygeometry ),
_minmax      ( 0, 0 ),
_drawPatch   ( false )
{
    setUseVertexBufferObjects( true );
    setUseDisplayList( false );

    _supportsGLSL = Registry::capabilities().supportsGLSL();

    // establish uniform name IDs.
    _uidUniformNameID           = osg::Uniform::getNameID( "oe_layer_uid" );
    _orderUniformNameID         = osg::Uniform::getNameID( "oe_layer_order" );
    _opacityUniformNameID       = osg::Uniform::getNameID( "oe_layer_opacity" );
    _texMatrixUniformNameID     = osg::Uniform::getNameID( "oe_layer_texMatrix" );

    _textureImageUnit = SamplerBinding::findUsage(bindings, SamplerBinding::COLOR)->unit();
}

void
TileDrawable::drawPrimitivesImplementation(osg::RenderInfo& renderInfo) const
{
    if ( _drawPatch )
        drawPatches( renderInfo );
    else
        drawSurface( renderInfo );
}


void
TileDrawable::drawPatches(osg::RenderInfo& renderInfo) const
{
    if ( _geom->getNumPrimitiveSets() < 1 )
        return;

    osg::State& state = *renderInfo.getState(); 
    
    const osg::DrawElementsUShort* de = static_cast<osg::DrawElementsUShort*>(_geom->getPrimitiveSet(0));
    osg::GLBufferObject* ebo = de->getOrCreateGLBufferObject(state.getContextID());
    state.bindElementBufferObject(ebo);
    if (ebo)
        glDrawElements(GL_PATCHES, de->size(), GL_UNSIGNED_SHORT, (const GLvoid *)(ebo->getOffset(de->getBufferIndex())));
    else
        glDrawElements(GL_PATCHES, de->size(), GL_UNSIGNED_SHORT, &de->front());
}


void
TileDrawable::drawSurface(osg::RenderInfo& renderInfo) const
{
    unsigned layersDrawn = 0;

    osg::State& state = *renderInfo.getState();    

    // access the GL extensions interface for the current GC:
    const osg::Program::PerContextProgram* pcp = 0L;

#if OSG_MIN_VERSION_REQUIRED(3,3,3)
	osg::ref_ptr<osg::GLExtensions> ext;
#else
    osg::ref_ptr<osg::GL2Extensions> ext;
#endif
    unsigned contextID;

    if (_supportsGLSL)
    {
        contextID = state.getContextID();
#if OSG_MIN_VERSION_REQUIRED(3,3,3)
		ext = osg::GLExtensions::Get(contextID, true);
#else   
		ext = osg::GL2Extensions::Get( contextID, true );
#endif
        pcp = state.getLastAppliedProgramObject();
    }

    // cannot store these in the object since there could be multiple GCs (and multiple
    // PerContextPrograms) at large
    GLint opacityLocation       = -1;
    GLint uidLocation           = -1;
    GLint orderLocation         = -1;
    GLint texMatrixLocation     = -1;

    // The PCP can change (especially in a VirtualProgram environment). So we do need to
    // requery the uni locations each time unfortunately. TODO: explore optimizations.
    if ( pcp )
    {
        opacityLocation      = pcp->getUniformLocation( _opacityUniformNameID );
        uidLocation          = pcp->getUniformLocation( _uidUniformNameID );
        orderLocation        = pcp->getUniformLocation( _orderUniformNameID );
        texMatrixLocation    = pcp->getUniformLocation( _texMatrixUniformNameID );
    }

    float prevOpacity = -1.0f;
    if ( _mptex.valid() && !_mptex->getPasses().empty() )
    {
        float prevOpacity = -1.0f;

        state.setActiveTextureUnit( _textureImageUnit );

        // in FFP mode, we need to enable the GL mode for texturing:
        if ( !pcp )
            state.applyMode(GL_TEXTURE_2D, true);

        for(MPTexture::Passes::const_iterator p = _mptex->getPasses().begin();
            p != _mptex->getPasses().end();
            ++p)
        {
            const MPTexture::Pass& pass = *p;

            if ( pass._layer->getVisible() && pass._layer->getOpacity() > 0.1 )
            {
                // Apply the texture.
                const osg::StateAttribute* lastTex = state.getLastAppliedTextureAttribute(_textureImageUnit, osg::StateAttribute::TEXTURE);
                if ( lastTex != pass._texture.get() )
                    pass._texture->apply( state );
            
                // Apply the texture matrix.
                ext->glUniformMatrix4fv( texMatrixLocation, 1, GL_FALSE, pass._matrix.ptr() );
            
                // Order uniform (TODO: evaluate whether we still need this)
                if ( orderLocation >= 0 )
                {
                    ext->glUniform1i( orderLocation, (GLint)layersDrawn );
                }

                // assign the layer UID:
                if ( uidLocation >= 0 )
                {
                    ext->glUniform1i( uidLocation, (GLint)pass._layer->getUID() );
                }

                // apply opacity:
                if ( opacityLocation >= 0 )
                {
                    float opacity = pass._layer->getOpacity();
                    if ( opacity != prevOpacity )
                    {
                        ext->glUniform1f( opacityLocation, (GLfloat)opacity );
                        prevOpacity = opacity;
                    }
                }

                _geom->getPrimitiveSet(0)->draw(state, true);

                ++layersDrawn;
            }
        }
    }

    // No mptex or no layers in the mptex? Draw simple.
    if ( layersDrawn == 0 )
    {
        if ( opacityLocation >= 0 )
            ext->glUniform1f( opacityLocation, (GLfloat)1.0f );

        if ( uidLocation >= 0 )
            ext->glUniform1i( uidLocation, (GLint)-1 );

        if ( orderLocation >= 0 )
            ext->glUniform1i( orderLocation, (GLint)0 );
        
        _geom->getPrimitiveSet(0)->draw(state, true);
    }

}

#if OSG_VERSION_GREATER_OR_EQUAL(3,3,2)
#    define COMPUTE_BOUND computeBoundingBox
#    define GET_BOUNDING_BOX getBoundingBox
#else
#    define COMPUTE_BOUND computeBound
#    define GET_BOUNDING_BOX getBound
#endif

#if OSG_VERSION_GREATER_OR_EQUAL(3,1,8)
#   define GET_ARRAY(a) (a)
#else
#   define GET_ARRAY(a) (a).array
#endif

osg::BoundingBox
TileDrawable:: COMPUTE_BOUND() const
{
    osg::BoundingBox bbox = _geom->COMPUTE_BOUND();

    // Replace the min/max Z with our computes extrema.
    // Offset the zmin to account for cuvature.
    bbox.zMin() = bbox.zMin() + _minmax[0];
    bbox.zMax() = _minmax[1];

    OE_DEBUG << LC << "zmin/max = " << bbox.zMin() << "/" << bbox.zMax() << "; minmax = " << _minmax[0] << "/" << _minmax[1] << "\n";

    return bbox;
}

void    
TileDrawable::setElevationExtrema(const osg::Vec2f& minmax)
{
    _minmax = minmax;
    dirtyBound();
}

const osg::BoundingBox&
TileDrawable::getBox() const
{
    return GET_BOUNDING_BOX();
}

osg::BoundingBox
TileDrawable::computeBox() const
{
    return COMPUTE_BOUND();
}

void 
TileDrawable::releaseGLObjects(osg::State* state) const
{
    osg::Drawable::releaseGLObjects( state );

    if ( _geom.valid() )
    {
        _geom->releaseGLObjects( state );
    }
}


void
TileDrawable::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::Drawable::resizeGLObjectBuffers( maxSize );

    if ( _geom.valid() )
    {
        _geom->resizeGLObjectBuffers( maxSize );
    }

    if ( _pcd.size() < maxSize )
    {
        _pcd.resize(maxSize);
    }
}


void 
TileDrawable::compileGLObjects(osg::RenderInfo& renderInfo) const
{
    osg::Drawable::compileGLObjects( renderInfo );

    if ( _geom.valid() )
    {
        _geom->compileGLObjects( renderInfo );
    }

    // unbind the BufferObjects
    //extensions->glBindBuffer(GL_ARRAY_BUFFER_ARB,0);
    //extensions->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
}

void
TileDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
    State& state = *renderInfo.getState();
    bool checkForGLErrors = state.getCheckForGLErrors() == osg::State::ONCE_PER_ATTRIBUTE;
    if ( checkForGLErrors ) state.checkGLErrors("start of TileDrawable::drawImplementation()");

#if OSG_MIN_VERSION_REQUIRED(3,3,1)
    _geom->drawVertexArraysImplementation( renderInfo );
#else
    drawVertexArraysImplementation( renderInfo );
#endif

    drawPrimitivesImplementation( renderInfo );

    if ( checkForGLErrors ) state.checkGLErrors("end of TileDrawable::drawImplementation()");
    
    // unbind the VBO's if any are used.
    state.unbindVertexBufferObject();
    state.unbindElementBufferObject();
}


#if OSG_MIN_VERSION_REQUIRED(3,1,8)
void
TileDrawable::drawVertexArraysImplementation(osg::RenderInfo& renderInfo) const
{
    if ( !_geom.valid() )
        return;

    State& state = *renderInfo.getState();

    bool handleVertexAttributes = !_geom->getVertexAttribArrayList().empty();

    ArrayDispatchers& arrayDispatchers = state.getArrayDispatchers();

    arrayDispatchers.reset();
    arrayDispatchers.setUseVertexAttribAlias(state.getUseVertexAttributeAliasing());

    arrayDispatchers.activateNormalArray(_geom->getNormalArray());
    arrayDispatchers.activateColorArray(_geom->getColorArray());
    arrayDispatchers.activateSecondaryColorArray(_geom->getSecondaryColorArray());
    arrayDispatchers.activateFogCoordArray(_geom->getFogCoordArray());

    if (handleVertexAttributes)
    {
        for(unsigned int unit=0;unit<_geom->getVertexAttribArrayList().size();++unit)
        {
            arrayDispatchers.activateVertexAttribArray(unit, _geom->getVertexAttribArray(unit));
        }
    }

    // dispatch any attributes that are bound overall
    arrayDispatchers.dispatch(osg::Array::BIND_OVERALL,0);

    state.lazyDisablingOfVertexAttributes();

    // set up arrays
    if( _geom->getVertexArray() )
        state.setVertexPointer(_geom->getVertexArray());

    if (_geom->getNormalArray() && _geom->getNormalArray()->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setNormalPointer(_geom->getNormalArray());

    if (_geom->getColorArray() && _geom->getColorArray()->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setColorPointer(_geom->getColorArray());

    if (_geom->getSecondaryColorArray() && _geom->getSecondaryColorArray()->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setSecondaryColorPointer(_geom->getSecondaryColorArray());

    if (_geom->getFogCoordArray() && _geom->getFogCoordArray()->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setFogCoordPointer(_geom->getFogCoordArray());

    for(unsigned int unit=0;unit<_geom->getTexCoordArrayList().size();++unit)
    {
        const Array* array = _geom->getTexCoordArray(unit);
        if (array)
        {
            state.setTexCoordPointer(unit,array);
        }
    }

    if ( handleVertexAttributes )
    {
        for(unsigned int index = 0; index < _geom->getVertexAttribArrayList().size(); ++index)
        {
            const Array* array = _geom->getVertexAttribArray(index);
            if (array && array->getBinding()==osg::Array::BIND_PER_VERTEX)
            {
                if (array->getPreserveDataType())
                {
                    GLenum dataType = array->getDataType();
                    if (dataType==GL_FLOAT) state.setVertexAttribPointer( index, array );
                    else if (dataType==GL_DOUBLE) state.setVertexAttribLPointer( index, array );
                    else state.setVertexAttribIPointer( index, array );
                }
                else
                {
                    state.setVertexAttribPointer( index, array );
                }
            }
        }
    }

    state.applyDisablingOfVertexAttributes();
}

#else

void
TileDrawable::drawVertexArraysImplementation(osg::RenderInfo& renderInfo) const
{
    State& state = *renderInfo.getState();
    bool handleVertexAttributes = !_vertexAttribList.empty();

    ArrayDispatchers& arrayDispatchers = state.getArrayDispatchers();

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
            const Array* array = _texCoordList[unit].array.get();
            if (array) state.setTexCoordPointer(unit,array);
        }

        if( handleVertexAttributes )
        {
            for(unsigned int index = 0; index < _vertexAttribList.size(); ++index )
            {
                const Array* array = _vertexAttribList[index].array.get();
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
}

#endif