/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "MPGeometry"

#include <osg/Version>
#include <osgUtil/MeshOptimizers>
#include <iterator>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

#include <osgUtil/IncrementalCompileOperation>
#include <osg/Version>

using namespace osg;
using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;

#define LC "[MPGeometry] "


MPGeometry::MPGeometry() :
osg::Geometry(),
_frame(0L),
_uidUniformNameID(0),
_birthTimeUniformNameID(0u),
_orderUniformNameID(0u),
_opacityUniformNameID(0u),
_texMatParentUniformNameID(0u),
_tileKeyUniformNameID(0u),
_minRangeUniformNameID(0u),
_maxRangeUniformNameID(0u),
_imageUnit(0),
_imageUnitParent(0),
_elevUnit(0),
_supportsGLSL(false)
{
}

MPGeometry::MPGeometry(const MPGeometry& rhs, const osg::CopyOp& cop) :
osg::Geometry(rhs, cop),
_frame(rhs._frame),
_uidUniformNameID(rhs._uidUniformNameID),
_birthTimeUniformNameID(rhs._birthTimeUniformNameID),
_orderUniformNameID(rhs._orderUniformNameID),
_opacityUniformNameID(rhs._opacityUniformNameID),
_texMatParentUniformNameID(rhs._texMatParentUniformNameID),
_tileKeyUniformNameID(rhs._tileKeyUniformNameID),
_minRangeUniformNameID(rhs._minRangeUniformNameID),
_maxRangeUniformNameID(rhs._maxRangeUniformNameID),
_imageUnit(rhs._imageUnit),
_imageUnitParent(rhs._imageUnitParent),
_elevUnit(rhs._elevUnit),
_supportsGLSL(rhs._supportsGLSL)
{
}


MPGeometry::MPGeometry(const TileKey& key, const MapFrame& frame, int imageUnit) : 
osg::Geometry    ( ),
_frame           ( frame ),
_imageUnit       ( imageUnit ),
_uidUniformNameID(0),
_birthTimeUniformNameID(0u),
_orderUniformNameID(0u),
_opacityUniformNameID(0u),
_texMatParentUniformNameID(0u),
_tileKeyUniformNameID(0u),
_minRangeUniformNameID(0u),
_maxRangeUniformNameID(0u),
_imageUnitParent(0),
_elevUnit(0),
_supportsGLSL(false)
{
    _supportsGLSL = Registry::capabilities().supportsGLSL();
    
    // Encode the tile key in a uniform. Note! The X and Y components are presented
    // modulo 2^16 form so they don't overrun single-precision space.
    unsigned tw, th;
    key.getProfile()->getNumTiles(key.getLOD(), tw, th);

    const double m = pow(2.0, 16.0);

    double x = (double)key.getTileX();
    double y = (double)(th - key.getTileY()-1);

    _tileKeyValue.set(
        (float)fmod(x, m),
        (float)fmod(y, m),
        (float)key.getLOD(),
        -1.0f);

    _imageUnitParent = _imageUnit + 1; // temp

    _elevUnit = _imageUnit + 2; // temp

    // establish uniform name IDs.
    _tileKeyUniformNameID      = osg::Uniform::getNameID( "oe_tile_key" );
    _birthTimeUniformNameID    = osg::Uniform::getNameID( "oe_tile_birthtime" );
    _uidUniformNameID          = osg::Uniform::getNameID( "oe_layer_uid" );
    _orderUniformNameID        = osg::Uniform::getNameID( "oe_layer_order" );
    _opacityUniformNameID      = osg::Uniform::getNameID( "oe_layer_opacity" );
    _texMatParentUniformNameID = osg::Uniform::getNameID( "oe_layer_parent_texmat" );
    _minRangeUniformNameID     = osg::Uniform::getNameID( "oe_layer_minRange" );
    _maxRangeUniformNameID     = osg::Uniform::getNameID( "oe_layer_maxRange" );

    // we will set these later (in TileModelCompiler)
    this->setUseDisplayList(false);
    this->setUseVertexBufferObjects(true);
}


void
MPGeometry::renderPrimitiveSets(osg::State& state,
                                bool        renderColor,
                                bool        usingVBOs) const
{
    // check the map frame to see if it's up to date
    if ( _frame.needsSync() )
    {
        // this lock protects a MapFrame sync when we have multiple DRAW threads.
        Threading::ScopedMutexLock exclusive( _frameSyncMutex );

        if ( _frame.needsSync() && _frame.sync() ) // always double check
        {
            // This should only happen is the layer ordering changes;
            // If layers are added or removed, the Tile gets rebuilt and
            // the point is moot.
            ImageLayerVector layers;
            _frame.getLayers(layers);

            std::vector<Layer> reordered;
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

    // access the GL extensions interface for the current GC:
    const osg::Program::PerContextProgram* pcp = 0L;

	osg::ref_ptr<osg::GLExtensions> ext;
    unsigned contextID;

    if (_supportsGLSL)
    {
        contextID = state.getContextID();
		ext = osg::GLExtensions::Get(contextID, true);
        pcp = state.getLastAppliedProgramObject();
    }

    // cannot store these in the object since there could be multiple GCs (and multiple
    // PerContextPrograms) at large
    GLint tileKeyLocation       = -1;
    GLint birthTimeLocation     = -1;
    GLint opacityLocation       = -1;
    GLint uidLocation           = -1;
    GLint orderLocation         = -1;
    GLint texMatParentLocation  = -1;
    GLint minRangeLocation      = -1;
    GLint maxRangeLocation      = -1;

    // The PCP can change (especially in a VirtualProgram environment). So we do need to
    // requery the uni locations each time unfortunately. TODO: explore optimizations.
    if ( pcp )
    {
        tileKeyLocation      = pcp->getUniformLocation( _tileKeyUniformNameID );
        birthTimeLocation    = pcp->getUniformLocation( _birthTimeUniformNameID );
        opacityLocation      = pcp->getUniformLocation( _opacityUniformNameID );
        uidLocation          = pcp->getUniformLocation( _uidUniformNameID );
        orderLocation        = pcp->getUniformLocation( _orderUniformNameID );
        texMatParentLocation = pcp->getUniformLocation( _texMatParentUniformNameID );
        minRangeLocation = pcp->getUniformLocation( _minRangeUniformNameID );
        maxRangeLocation = pcp->getUniformLocation( _maxRangeUniformNameID );
    }
    
    // apply the tilekey uniform once.
    if ( tileKeyLocation >= 0 )
    {
        ext->glUniform4fv( tileKeyLocation, 1, _tileKeyValue.ptr() );
    }

    // set the "birth time" - i.e. the time this tile last entered the scene in the current GC.
    if ( birthTimeLocation >= 0 )
    {
        PerContextData& pcd = _pcd[contextID];
        if ( pcd.birthTime < 0.0f )
        {
            const osg::FrameStamp* stamp = state.getFrameStamp();
            if ( stamp )
            {
                pcd.birthTime = stamp->getReferenceTime();
            }
        }
        ext->glUniform1f( birthTimeLocation, pcd.birthTime );
    }

    // activate the tile coordinate set - same for all layers
    if ( renderColor && _layers.size() > 0 )
    {
        state.setTexCoordPointer( _imageUnit+1, _tileCoords.get() );
    }

#if !( defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE) || defined(OSG_GL3_AVAILABLE) )
    if ( renderColor )
    {
        // emit a default terrain color since we're not binding a color array:
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    }
#endif    

    // track the active image unit.
    int activeImageUnit = -1;

    // remember whether we applied a parent texture.
    bool usedTexParent = false;

    if ( _layers.size() > 0 )
    {
        float prev_opacity        = -1.0f;

        // first bind any shared layers. We still have to do this even if we are
        // in !renderColor mode b/c these textures could be used by vertex shaders
        // to alter the geometry.
        int sharedLayers = 0;
        if ( pcp )
        {
            for(unsigned i=0; i<_layers.size(); ++i)
            {
                const Layer& layer = _layers[i];

                // a "shared" layer binds to a secondary texture unit so that other layers
                // can see it and use it.
                if ( layer._imageLayer->isShared() )
                {
                    ++sharedLayers;
                    int sharedUnit = layer._imageLayer->shareImageUnit().get();
                    {
                        state.setActiveTextureUnit( sharedUnit );

                        state.setTexCoordPointer( sharedUnit, layer._texCoords.get() );
                        // bind the texture for this layer to the active share unit.
                        layer._tex->apply( state );

                        // Shared layers need a texture matrix since the terrain engine doesn't
                        // provide a "current texture coordinate set" uniform (i.e. oe_layer_texc)
                        GLint texMatLocation = 0;
                        texMatLocation = pcp->getUniformLocation( layer._texMatUniformID );
                        if ( texMatLocation >= 0 )
                        {
                            ext->glUniformMatrix4fv( texMatLocation, 1, GL_FALSE, layer._texMat.ptr() );
                        }
                    }
                }
            }
        }
        if (renderColor)
        {
            // find the first opaque layer, top-down, and start there:
            unsigned first = 0;
            for(first = _layers.size()-1; first > 0; --first)
            {
                const Layer& layer = _layers[first];
                if (layer._opaque &&
                    //Color filters can modify the opacity
                    layer._imageLayer->getColorFilters().empty() &&
                    layer._imageLayer->getVisible() &&
                    layer._imageLayer->getOpacity() >= 1.0f)
                {
                    break;
                }
            }

            // interate over all the image layers
            for(unsigned i=first; i<_layers.size(); ++i)
            {
                const Layer& layer = _layers[i];

                if ( layer._imageLayer->getVisible() && layer._imageLayer->getOpacity() > 0.0f )
                {       
                    // activate the visible unit if necessary:
                    if ( activeImageUnit != _imageUnit )
                    {
                        state.setActiveTextureUnit( _imageUnit );
                        activeImageUnit = _imageUnit;
                    }

                    // bind the texture for this layer:
                    layer._tex->apply( state );

                    // in FFP mode, we need to enable the GL mode for texturing:
                    if ( !pcp ) //!_supportsGLSL)
                    {
                        state.applyMode(GL_TEXTURE_2D, true);
                    }

                    // if we're using a parent texture for blending, activate that now
                    if ( texMatParentLocation >= 0 && layer._texParent.valid() )
                    {
                        state.setActiveTextureUnit( _imageUnitParent );
                        activeImageUnit = _imageUnitParent;
                        layer._texParent->apply( state );
                        usedTexParent = true;
                    }

                    // bind the texture coordinates for this layer.
                    // TODO: can probably optimize this by sharing or using texture matrixes.
                    // State::setTexCoordPointer does some redundant work under the hood.
                    state.setTexCoordPointer( _imageUnit, layer._texCoords.get() );

                    // apply uniform values:
                    if ( pcp )
                    {
                        // apply opacity:
                        if ( opacityLocation >= 0 )
                        {
                            float opacity = layer._imageLayer->getOpacity();
                            if ( opacity != prev_opacity )
                            {
                                ext->glUniform1f( opacityLocation, (GLfloat)opacity );
                                prev_opacity = opacity;
                            }
                        }

                        // assign the layer UID:
                        if ( uidLocation >= 0 )
                        {
                            ext->glUniform1i( uidLocation, (GLint)layer._layerID );
                        }

                        // assign the layer order:
                        if ( orderLocation >= 0 )
                        {
                            ext->glUniform1i( orderLocation, (GLint)layersDrawn );
                        }

                        // assign the parent texture matrix
                        if ( texMatParentLocation >= 0 && layer._texParent.valid() )
                        {
                            ext->glUniformMatrix4fv( texMatParentLocation, 1, GL_FALSE, layer._texMatParent.ptr() );
                        }

                        // assign the min range
                        if ( minRangeLocation >= 0 )
                        {
                            ext->glUniform1f( minRangeLocation, layer._imageLayer->options().minVisibleRange().get() );
                        }

                        // assign the max range
                        if ( maxRangeLocation >= 0 )
                        {
                            ext->glUniform1f( maxRangeLocation, layer._imageLayer->options().maxVisibleRange().get() );
                        }
                    }

                    // draw the primitive sets.
                    for(unsigned int primitiveSetNum=0; primitiveSetNum!=_primitives.size(); ++primitiveSetNum)
                    {
                        const osg::PrimitiveSet* primitiveset = _primitives[primitiveSetNum].get();
                        if ( primitiveset )
                        {
                            primitiveset->draw(state, usingVBOs);
                        }
                        else
                        {
                            OE_WARN << LC << "Strange, MPGeometry had a 0L primset" << std::endl;
                        }
                    }

                    ++layersDrawn;
                }
            }
        }
    }

    // if we didn't draw anything, draw the raw tiles anyway with no texture.
    if ( layersDrawn == 0 )
    {
        if ( pcp )
        {
            if ( opacityLocation >= 0 )
                ext->glUniform1f( opacityLocation, (GLfloat)1.0f );
            if ( uidLocation >= 0 )
                ext->glUniform1i( uidLocation, (GLint)-1 );
            if ( orderLocation >= 0 )
                ext->glUniform1i( orderLocation, (GLint)0 );
        }

        // draw the primitives themselves.
        for(unsigned int primitiveSetNum=0; primitiveSetNum!=_primitives.size(); ++primitiveSetNum)
        {
            const osg::PrimitiveSet* primitiveset = _primitives[primitiveSetNum].get();
            primitiveset->draw(state, usingVBOs);
        }
    }

    else // at least one textured layer was drawn:
    {
        // prevent texture leakage
        // TODO: find a way to remove this to speed things up
        if ( renderColor )
        {
            glBindTexture( GL_TEXTURE_2D, 0 );

            // if a parent texture was applied, need to disable both.
            if ( usedTexParent )
            {
                state.setActiveTextureUnit(
                    activeImageUnit != _imageUnitParent ? _imageUnitParent :
                    _imageUnit );

                glBindTexture( GL_TEXTURE_2D, 0);
            }
        }
    }
}

#if OSG_VERSION_GREATER_OR_EQUAL(3,3,2)
#    define COMPUTE_BOUND computeBoundingBox
#else
#    define COMPUTE_BOUND computeBound
#endif

#if OSG_VERSION_GREATER_OR_EQUAL(3,1,8)
#   define GET_ARRAY(a) (a)
#else
#   define GET_ARRAY(a) (a).array
#endif

osg::BoundingBox
MPGeometry:: COMPUTE_BOUND() const
{
    osg::BoundingBox bbox = osg::Geometry:: COMPUTE_BOUND ();
    {
        // update the uniform.
        Threading::ScopedMutexLock exclusive(_frameSyncMutex);
        _tileKeyValue.w() = bbox.radius();

        // create the patch triangles index if necessary.
        if ( getNumPrimitiveSets() > 0 && getPrimitiveSet(0)->getMode() == GL_PATCHES )
        {
            _patchTriangles = osg::clone( getPrimitiveSet(0), osg::CopyOp::SHALLOW_COPY );
            _patchTriangles->setMode( GL_TRIANGLES );
        }
    }
    return bbox;
}

void
MPGeometry::validate()
{
    unsigned numVerts = getVertexArray()->getNumElements();

    for(unsigned i=0; i < _primitives.size(); ++i)
    {
        osg::DrawElements* de = static_cast<osg::DrawElements*>(_primitives[i].get());
        if ( de->getMode() != GL_TRIANGLES )
        {
            OE_WARN << LC << "Invalid primitive set - not GL_TRIANGLES" << std::endl;
            _primitives.clear();
        }

        else if ( de->getNumIndices() % 3 != 0 )
        {
            OE_WARN << LC << "Invalid primitive set - wrong number of indices" << std::endl;
            //_primitives.clear();
            osg::DrawElementsUShort* deus = static_cast<osg::DrawElementsUShort*>(de);
            int extra = de->getNumIndices() % 3;
            deus->resize(de->getNumIndices() - extra);
            OE_WARN << LC << "   ..removed " << extra << " indices" << std::endl;
            //return;
        }
        else
        {
            for( unsigned j=0; j<de->getNumIndices(); ++j ) 
            {
                unsigned index = de->index(j);
                if ( index >= numVerts )
                {
                    OE_WARN << LC << "Invalid primitive set - index out of bounds" << std::endl;
                    _primitives.clear();
                    return;
                }
            }
        }
    }
}


void 
MPGeometry::releaseGLObjects(osg::State* state) const
{
    osg::Geometry::releaseGLObjects( state );

    // Note: don't release the textures here; instead we release them in the
    // TileModel where they were created. -gw
}


void
MPGeometry::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::Geometry::resizeGLObjectBuffers( maxSize );

    for(unsigned i=0; i<_layers.size(); ++i)
    {
        const Layer& layer = _layers[i];
        if ( layer._tex.valid() )
            layer._tex->resizeGLObjectBuffers( maxSize );
    }

    if ( _pcd.size() < maxSize )
    {
        _pcd.resize(maxSize);
    }
}


void 
MPGeometry::compileGLObjects( osg::RenderInfo& renderInfo ) const
{
    State& state = *renderInfo.getState();
    
    // compile the image textures:
    for(unsigned i=0; i<_layers.size(); ++i)
    {
        const Layer& layer = _layers[i];
        if ( layer._tex.valid() )
            layer._tex->apply( state );
    }

    // compile the elevation texture:
    if ( _elevTex.valid() )
    {
        _elevTex->apply( state );
    }

    osg::Geometry::compileGLObjects( renderInfo );
}

#if OSG_MIN_VERSION_REQUIRED(3,5,6)

osg::VertexArrayState*
#if OSG_MIN_VERSION_REQUIRED(3,5,9)
MPGeometry::createVertexArrayStateImplementation(osg::RenderInfo& renderInfo) const
{
    osg::VertexArrayState* vas = osg::Geometry::createVertexArrayStateImplementation(renderInfo);
#else
MPGeometry::createVertexArrayState(osg::RenderInfo& renderInfo) const
{
    osg::VertexArrayState* vas = osg::Geometry::createVertexArrayState(renderInfo);
#endif
    // make sure we have array dispatchers for the multipass coords
    vas->assignTexCoordArrayDispatcher(_texCoordList.size() + 2);

    return vas;
}
#endif


void 
MPGeometry::drawImplementation(osg::RenderInfo& renderInfo) const
{
    // See if this is a pre-render depth-only camera. If so we can skip all the layers
    // and just render the primitive sets.
    osg::Camera* camera = renderInfo.getCurrentCamera();
    bool renderColor =
        (camera->getRenderOrder() != osg::Camera::PRE_RENDER) ||
        ((camera->getClearMask() & GL_COLOR_BUFFER_BIT) != 0L);

    osg::State& state = *renderInfo.getState();

    bool hasVertexAttributes = !_vertexAttribList.empty();

#if OSG_VERSION_LESS_THAN(3,5,6)
    osg::ArrayDispatchers& dispatchers = state.getArrayDispatchers();
#else
    osg::AttributeDispatchers& dispatchers = state.getAttributeDispatchers();
#endif

    dispatchers.reset();
    dispatchers.setUseVertexAttribAlias(state.getUseVertexAttributeAliasing());
    dispatchers.activateNormalArray(_normalArray.get());   

    if (hasVertexAttributes)
    {
        for(unsigned int unit=0;unit<_vertexAttribList.size();++unit)
        {
            dispatchers.activateVertexAttribArray(unit, _vertexAttribList[unit].get());
        }
    }

    // dispatch any attributes that are bound overall
#if OSG_VERSION_LESS_THAN(3,5,6)
    dispatchers.dispatch(BIND_OVERALL,0);
#else
    dispatchers.dispatch(0);
#endif
    state.lazyDisablingOfVertexAttributes();


    // set up arrays
    if( _vertexArray.valid() )
        state.setVertexPointer(_vertexArray.get());

    if (_normalArray.valid() && _normalArray->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setNormalPointer(_normalArray.get());

    if( hasVertexAttributes )
    {
        for(unsigned int index = 0; index < _vertexAttribList.size(); ++index )
        {
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
        }
    }

    state.applyDisablingOfVertexAttributes();

    // draw the multipass geometry.
    renderPrimitiveSets(state, renderColor, true);

    // unbind the VBO's if any are used.
#if OSG_MIN_VERSION_REQUIRED(3,5,6)
    if (!state.useVertexArrayObject(_useVertexArrayObject) || state.getCurrentVertexArrayState()->getRequiresSetArrays())
#endif
    {
        state.unbindVertexBufferObject();
        state.unbindElementBufferObject();
    }
}

void
MPGeometry::accept(osg::PrimitiveIndexFunctor& functor) const
{
    osg::Geometry::accept(functor);

    if ( getNumPrimitiveSets() > 0 && getPrimitiveSet(0)->getMode() == GL_PATCHES && _patchTriangles.valid() )
    {
        _patchTriangles->accept( functor );
    }
}

void
MPGeometry::accept(osg::PrimitiveFunctor& functor) const
{
    osg::Geometry::accept(functor);

    if ( getNumPrimitiveSets() > 0 && getPrimitiveSet(0)->getMode() == GL_PATCHES && _patchTriangles.valid() )
    {
        _patchTriangles->accept( functor );
    }
}
