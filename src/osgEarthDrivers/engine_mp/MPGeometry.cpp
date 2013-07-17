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
#include "MPGeometry"

#include <osg/Version>

using namespace osg;
using namespace osgEarth_engine_mp;
using namespace osgEarth;

#define LC "[MPGeometry] "


//----------------------------------------------------------------------------

MPGeometry::MPGeometry(const Map* map, int imageUnit) : 
osg::Geometry    ( ), 
_map             ( map, Map::IMAGE_LAYERS ),
_imageUnit       ( imageUnit )
{
    _opacityUniform = new osg::Uniform( osg::Uniform::FLOAT, "oe_layer_opacity" );
    _opacityUniform->set( 1.0f );

    _layerUIDUniform = new osg::Uniform( osg::Uniform::INT, "oe_layer_uid" );
    _layerUIDUniform->set( 0 );

    _layerOrderUniform = new osg::Uniform( osg::Uniform::INT, "oe_layer_order" );
    _layerOrderUniform->set( 0 );

    _texMatParentUniform = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "oe_layer_parent_matrix");

    _imageUnitParent = _imageUnit + 1; // temp
}


void
MPGeometry::renderPrimitiveSets(osg::State& state,
                                bool        usingVBOs) const
{
    // check the map frame to see if it's up to date
    if ( _map.needsSync() )
    {
        // this lock protects a MapFrame sync when we have multiple DRAW threads.
        Threading::ScopedMutexLock exclusive( _mapSyncMutex );

        if ( _map.needsSync() && _map.sync() ) // always double check
        {
            // This should only happen is the layer ordering changes;
            // If layers are added or removed, the Tile gets rebuilt and
            // the point is moot.
            std::vector<Layer> reordered;
            const ImageLayerVector& layers = _map.imageLayers();
            reordered.reserve( layers.size() );
            for( ImageLayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i )
            {
                std::vector<Layer>::iterator j = std::find( _layers.begin(), _layers.end(), i->get()->getUID() );
                if ( j != _layers.end() )
                    reordered.push_back( *j );
            }
            _layers.swap( reordered );
        }
    }

    unsigned layersDrawn = 0;


    osg::ref_ptr<osg::GL2Extensions> ext = osg::GL2Extensions::Get( state.getContextID(), true );
    const osg::Program::PerContextProgram* pcp = state.getLastAppliedProgramObject();

    GLint opacityLocation;
    GLint uidLocation;
    GLint orderLocation;
    GLint texMatParentLocation;

    // yes, it's possible that the PCP is not set up yet.
    // TODO: can we optimize this so we don't need to get uni locations every time?
    if ( pcp )
    {
        opacityLocation      = pcp->getUniformLocation( _opacityUniform->getNameID() );
        uidLocation          = pcp->getUniformLocation( _layerUIDUniform->getNameID() );
        orderLocation        = pcp->getUniformLocation( _layerOrderUniform->getNameID() );
        texMatParentLocation = pcp->getUniformLocation( _texMatParentUniform->getNameID() );
    }

    // activate the tile coordinate set - same for all layers
    state.setTexCoordPointer( _imageUnit+1, _tileCoords.get() );

    if ( _layers.size() > 0 )
    {
        float prev_opacity        = -1.0f;
        float prev_alphaThreshold = -1.0f;

        // first bind any shared layers
        // TODO: optimize by pre-storing shared indexes
        for(unsigned i=0; i<_layers.size(); ++i)
        {
            const Layer& layer = _layers[i];

            // a "shared" layer binds to a secondary texture unit so that other layers
            // can see it and use it.
            if ( layer._imageLayer->isShared() )
            {
                int sharedUnit = layer._imageLayer->shareImageUnit().get();
                {
                    state.setActiveTextureUnit( sharedUnit );
                    state.setTexCoordPointer( sharedUnit, layer._texCoords.get() );
                    // bind the texture for this layer to the active share unit.
                    layer._tex->apply( state );

                    // no texture LOD blending for shared layers for now. maybe later.
                }
            }
        }

        // track the active image unit.
        int activeImageUnit = -1;

        // interate over all the image layers
        for(unsigned i=0; i<_layers.size(); ++i)
        {
            const Layer& layer = _layers[i];

            if ( layer._imageLayer->getVisible() )
            {       
                // activate the visible unit if necessary:
                if ( activeImageUnit != _imageUnit )
                {
                    state.setActiveTextureUnit( _imageUnit );
                    activeImageUnit = _imageUnit;
                }

                // bind the texture for this layer:
                layer._tex->apply( state );

                // if we're using a parent texture for blending, activate that now
                if ( layer._texParent.valid() )
                {
                    state.setActiveTextureUnit( _imageUnitParent );
                    activeImageUnit = _imageUnitParent;
                    layer._texParent->apply( state );
                }

                // bind the texture coordinates for this layer.
                // TODO: can probably optimize this by sharing or using texture matrixes.
                // State::setTexCoordPointer does some redundant work under the hood.
                state.setTexCoordPointer( _imageUnit, layer._texCoords.get() );

                // apply uniform values:
                if ( pcp )
                {
                    // apply opacity:
                    float opacity = layer._imageLayer->getOpacity();
                    if ( opacity != prev_opacity )
                    {
                        _opacityUniform->set( opacity );
                        _opacityUniform->apply( ext, opacityLocation );
                        prev_opacity = opacity;
                    }

                    // assign the layer UID:
                    _layerUIDUniform->set( layer._layerID );
                    _layerUIDUniform->apply( ext, uidLocation );

                    // assign the layer order:
                    _layerOrderUniform->set( (int)layersDrawn );
                    _layerOrderUniform->apply( ext, orderLocation );

                    // assign the parent texture matrix
                    if ( layer._texParent.valid() )
                    {
                        _texMatParentUniform->set( layer._texMatParent );
                        _texMatParentUniform->apply( ext, texMatParentLocation );
                    }
                }

                // draw the primitive sets.
                for(unsigned int primitiveSetNum=0; primitiveSetNum!=_primitives.size(); ++primitiveSetNum)
                {
                    const osg::PrimitiveSet* primitiveset = _primitives[primitiveSetNum].get();
                    primitiveset->draw(state, usingVBOs);
                }

                ++layersDrawn;
            }
        }

        // prevent texture leakage
        glBindTexture( GL_TEXTURE_2D, 0 );
    }

    // if we didn't draw anything, draw the raw tiles anyway with no texture.
    if ( layersDrawn == 0 )
    {
        _opacityUniform->set( 1.0f );
        _opacityUniform->apply( ext, opacityLocation );

        _layerUIDUniform->set( (int)-1 ); // indicates a non-textured layer
        _layerUIDUniform->apply( ext, uidLocation );

        _layerOrderUniform->set( (int)0 );
        _layerOrderUniform->apply( ext, orderLocation );

        // draw the primitives themselves.
        for(unsigned int primitiveSetNum=0; primitiveSetNum!=_primitives.size(); ++primitiveSetNum)
        {
            const osg::PrimitiveSet* primitiveset = _primitives[primitiveSetNum].get();
            primitiveset->draw(state, usingVBOs);
        }
    }
}


void 
MPGeometry::releaseGLObjects(osg::State* state) const
{
    osg::Geometry::releaseGLObjects( state );

    for(unsigned i=0; i<_layers.size(); ++i)
    {
        const Layer& layer = _layers[i];

        //Moved to TileModel since that's where the texture is created. Releasing it
        // here could break texture sharing.
        //if ( layer._tex.valid() )
        //    layer._tex->releaseGLObjects( state );

        // Check the refcount since texcoords can be cached/shared.
        if ( layer._texCoords.valid() && layer._texCoords->referenceCount() == 1 )
            layer._texCoords->releaseGLObjects( state );
    }
}


void 
MPGeometry::compileGLObjects( osg::RenderInfo& renderInfo ) const
{
    osg::Geometry::compileGLObjects( renderInfo );

    for(unsigned i=0; i<_layers.size(); ++i)
    {
        const Layer& layer = _layers[i];
        if ( layer._tex.valid() )
            layer._tex->apply( *renderInfo.getState() );
    }
}


void 
MPGeometry::drawImplementation(osg::RenderInfo& renderInfo) const
{
    osg::State& state = *renderInfo.getState();

    bool usingVertexBufferObjects = _useVertexBufferObjects && state.isVertexBufferObjectSupported();
    bool handleVertexAttributes = !_vertexAttribList.empty();

    osg::ArrayDispatchers& arrayDispatchers = state.getArrayDispatchers();

    arrayDispatchers.reset();
    arrayDispatchers.setUseVertexAttribAlias(state.getUseVertexAttributeAliasing());


    //Remove 
#if OSG_VERSION_LESS_THAN(3,1,8)
    arrayDispatchers.setUseGLBeginEndAdapter(false);
#endif


#if OSG_MIN_VERSION_REQUIRED(3,1,8)
    arrayDispatchers.activateNormalArray(_normalArray.get());
    arrayDispatchers.activateColorArray(_colorArray.get());
    arrayDispatchers.activateSecondaryColorArray(_secondaryColorArray.get());
    arrayDispatchers.activateFogCoordArray(_fogCoordArray.get());
#else
    arrayDispatchers.activateNormalArray(_normalData.binding, _normalData.array.get(), _normalData.indices.get());
    arrayDispatchers.activateColorArray(_colorData.binding, _colorData.array.get(), _colorData.indices.get());
    arrayDispatchers.activateSecondaryColorArray(_secondaryColorData.binding, _secondaryColorData.array.get(), _secondaryColorData.indices.get());
    arrayDispatchers.activateFogCoordArray(_fogCoordData.binding, _fogCoordData.array.get(), _fogCoordData.indices.get());
#endif
    

    if (handleVertexAttributes)
    {
        for(unsigned int unit=0;unit<_vertexAttribList.size();++unit)
        {
#if OSG_MIN_VERSION_REQUIRED(3,1,8)
            arrayDispatchers.activateVertexAttribArray(unit, _vertexAttribList[unit].get());
#else
            arrayDispatchers.activateVertexAttribArray(_vertexAttribList[unit].binding, unit, _vertexAttribList[unit].array.get(), _vertexAttribList[unit].indices.get());
#endif             
        }
    }

    // dispatch any attributes that are bound overall
    arrayDispatchers.dispatch(BIND_OVERALL,0);
    state.lazyDisablingOfVertexAttributes();


    // set up arrays
#if OSG_MIN_VERSION_REQUIRED( 3, 1, 8 )
    if( _vertexArray.valid() )
        state.setVertexPointer(_vertexArray.get());

    if (_normalArray.valid() && _normalArray->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setNormalPointer(_normalArray.get());

    if (_colorArray.valid() && _colorArray->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setColorPointer(_colorArray.get());

    if (_secondaryColorArray.valid() && _secondaryColorArray->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setSecondaryColorPointer(_secondaryColorArray.get());

    if (_fogCoordArray.valid() && _fogCoordArray->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setFogCoordPointer(_fogCoordArray.get());
#else
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
#endif    
        
    for(unsigned int unit=0;unit<_texCoordList.size();++unit)
    {
#if OSG_MIN_VERSION_REQUIRED( 3, 1, 8)
        const Array* array = _texCoordList[unit].get();
        if (array)
        {
            state.setTexCoordPointer(unit,array);
        }
#else
        const osg::Array* array = _texCoordList[unit].array.get();
        if (array) state.setTexCoordPointer(unit,array);
#endif        
    }

    if( handleVertexAttributes )
    {
        for(unsigned int index = 0; index < _vertexAttribList.size(); ++index )
        {
#if OSG_MIN_VERSION_REQUIRED( 3, 1, 8)
            const Array* array = _vertexAttribList[index].get();
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
#else            
            const osg::Array* array = _vertexAttribList[index].array.get();
            const AttributeBinding ab = _vertexAttribList[index].binding;
            if( ab == BIND_PER_VERTEX && array )
            {
                state.setVertexAttribPointer( index, array, _vertexAttribList[index].normalize );
            }
#endif
        }
    }

    state.applyDisablingOfVertexAttributes();

    // draw the multipass geometry.
    renderPrimitiveSets(state, usingVertexBufferObjects);

    // unbind the VBO's if any are used.
    state.unbindVertexBufferObject();
    state.unbindElementBufferObject();
}
