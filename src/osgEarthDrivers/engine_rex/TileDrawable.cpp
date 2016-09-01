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
#include <osgEarth/ImageUtils>

using namespace osg;
using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[TileDrawable] "


static Threading::Mutex _profMutex;
static unsigned s_frame = 0;
static unsigned s_draws = 0;
static unsigned s_functors = 0;


TileDrawable::TileDrawable(const TileKey&        key,
                           const RenderBindings& bindings,
                           osg::Geometry*        geometry,
                           int                   tileSize,
                           int                   skirtSize) :
osg::Drawable( ),
_key         ( key ),
_bindings    ( bindings ),
_geom        ( geometry ),
_tileSize    ( tileSize ),
_drawPatch   ( false ),
_skirtSize   ( skirtSize )
{
    this->setDataVariance( DYNAMIC );

    if (_geom.valid())
        _geom->setDataVariance( DYNAMIC );

    this->setName( key.str() );

    setUseVertexBufferObjects( true );
    setUseDisplayList( false );

    _supportsGLSL = Registry::capabilities().supportsGLSL();

    // establish uniform name IDs.
    _uidUniformNameID             = osg::Uniform::getNameID( "oe_layer_uid" );
    _orderUniformNameID           = osg::Uniform::getNameID( "oe_layer_order" );
    _opacityUniformNameID         = osg::Uniform::getNameID( "oe_layer_opacity" );
    _texMatrixUniformNameID       = osg::Uniform::getNameID( "oe_layer_texMatrix" );
    _texMatrixParentUniformNameID = osg::Uniform::getNameID( "oe_layer_texParentMatrix" );
    _texParentExistsUniformNameID = osg::Uniform::getNameID( "oe_layer_texParentExists" );
    _minRangeUniformNameID        = osg::Uniform::getNameID( "oe_layer_minRange" );
    _maxRangeUniformNameID        = osg::Uniform::getNameID( "oe_layer_maxRange" );

    _textureImageUnit       = SamplerBinding::findUsage(bindings, SamplerBinding::COLOR)->unit();
    _textureParentImageUnit = SamplerBinding::findUsage(bindings, SamplerBinding::COLOR_PARENT)->unit();
    
    int tileSize2 = tileSize*tileSize;
    _heightCache = new float[ tileSize2 ];
    for(int i=0; i<tileSize2; ++i)
        _heightCache[i] = 0.0f;    
}

TileDrawable::~TileDrawable()
{
    delete [] _heightCache;
}

void
TileDrawable::drawPrimitivesImplementation(osg::RenderInfo& renderInfo) const
{
    if ( _drawPatch )
    {
        drawPatches( renderInfo );
    }
    else
    {
        const osg::Camera* camera = renderInfo.getCurrentCamera();

        bool renderColor =
            //(camera->getRenderOrder() != osg::Camera::PRE_RENDER) ||
            ((camera->getClearMask() & GL_COLOR_BUFFER_BIT) != 0L);

        drawSurface( renderInfo, renderColor );
    }
}


void
TileDrawable::drawPatches(osg::RenderInfo& renderInfo) const
{
    if  (!_geom.valid() || _geom->getNumPrimitiveSets() < 1 )
        return;

    osg::State& state = *renderInfo.getState(); 
    
    const osg::DrawElementsUShort* de = static_cast<osg::DrawElementsUShort*>(_geom->getPrimitiveSet(0));
    osg::GLBufferObject* ebo = de->getOrCreateGLBufferObject(state.getContextID());
    state.bindElementBufferObject(ebo);
    if (ebo)
        glDrawElements(GL_PATCHES, de->size()-_skirtSize, GL_UNSIGNED_SHORT, (const GLvoid *)(ebo->getOffset(de->getBufferIndex())));
    else
        glDrawElements(GL_PATCHES, de->size()-_skirtSize, GL_UNSIGNED_SHORT, &de->front());
}


void
TileDrawable::drawSurface(osg::RenderInfo& renderInfo, bool renderColor) const
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

    // safely latch
    if ( !_geom.valid() || _geom->getNumPrimitiveSets() < 1 )
        return;

    // cannot store these in the object since there could be multiple GCs (and multiple
    // PerContextPrograms) at large
    GLint opacityLocation            = -1;
    GLint uidLocation                = -1;
    GLint orderLocation              = -1;
    GLint texMatrixLocation          = -1;
    GLint texMatrixParentLocation    = -1;
    GLint texParentExistsLocation    = -1;
    GLint minRangeLocation           = -1;
    GLint maxRangeLocation           = -1;

    // The PCP can change (especially in a VirtualProgram environment). So we do need to
    // requery the uni locations each time unfortunately. TODO: explore optimizations.
    if ( pcp )
    {
        opacityLocation             = pcp->getUniformLocation( _opacityUniformNameID );
        uidLocation                 = pcp->getUniformLocation( _uidUniformNameID );
        orderLocation               = pcp->getUniformLocation( _orderUniformNameID );
        texMatrixLocation           = pcp->getUniformLocation( _texMatrixUniformNameID );
        texMatrixParentLocation     = pcp->getUniformLocation( _texMatrixParentUniformNameID );
        texParentExistsLocation     = pcp->getUniformLocation( _texParentExistsUniformNameID );
        minRangeLocation            = pcp->getUniformLocation( _minRangeUniformNameID );
        maxRangeLocation            = pcp->getUniformLocation( _maxRangeUniformNameID );
    }

    float prevOpacity = -1.0f;
    if ( renderColor && _mptex.valid() && !_mptex->getPasses().empty() )
    {
        float prevOpacity = -1.0f;

        // in FFP mode, we need to enable the GL mode for texturing:
        if ( !pcp )
            state.applyMode(GL_TEXTURE_2D, true);

        optional<bool> texParentExists_lastValue;

        for(MPTexture::Passes::const_iterator p = _mptex->getPasses().begin();
            p != _mptex->getPasses().end();
            ++p)
        {
            const MPTexture::Pass& pass = *p;

            if ( pass._layer->getVisible() && pass._layer->getOpacity() > 0.1 )
            {
                // Apply the texture.
                state.setActiveTextureUnit( _textureImageUnit );
                const osg::StateAttribute* lastTex = state.getLastAppliedTextureAttribute(_textureImageUnit, osg::StateAttribute::TEXTURE);
                if ( lastTex != pass._texture.get() )
                    pass._texture->apply( state );

                // Apply the texture matrix.
                ext->glUniformMatrix4fv( texMatrixLocation, 1, GL_FALSE, pass._textureMatrix.ptr() );

                bool texParentExists = pass._parentTexture.valid();
                if ( texParentExists )
                {
                    // Apply the parent texture.
                    state.setActiveTextureUnit( _textureParentImageUnit );
                    const osg::StateAttribute* lastTex = state.getLastAppliedTextureAttribute(_textureParentImageUnit, osg::StateAttribute::TEXTURE);
                    if ( lastTex != pass._parentTexture.get() )
                        pass._parentTexture->apply( state );

                    // Apply the parent texture matrix.
                    ext->glUniformMatrix4fv( texMatrixParentLocation, 1, GL_FALSE, pass._parentTextureMatrix.ptr() );
                }

                if ( !texParentExists_lastValue.isSetTo(texParentExists) )
                {
                    texParentExists_lastValue = texParentExists;
                    ext->glUniform1f( texParentExistsLocation, texParentExists? 1.0f : 0.0f );
                }

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

                // Apply the min/max range
                float minRange = pass._layer->getImageLayerOptions().minVisibleRange().getOrUse(0.0);
                float maxRange = pass._layer->getImageLayerOptions().maxVisibleRange().getOrUse(-1.0);
                ext->glUniform1f( minRangeLocation, minRange );
                ext->glUniform1f( maxRangeLocation, maxRange );

                for (unsigned i=0; i < _geom->getNumPrimitiveSets(); i++)
                    _geom->getPrimitiveSet(i)->draw(state, true);

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

        if ( renderColor )
        {
            for (unsigned i=0; i < _geom->getNumPrimitiveSets(); i++)
            {
                _geom->getPrimitiveSet(i)->draw(state, true);
            }
        }
        else
        {
            // draw the surface w/o the skirt:
            const osg::DrawElementsUShort* de = static_cast<osg::DrawElementsUShort*>(_geom->getPrimitiveSet(0));
            osg::GLBufferObject* ebo = de->getOrCreateGLBufferObject(state.getContextID());
            state.bindElementBufferObject(ebo);
            glDrawElements(GL_TRIANGLES, de->size()-_skirtSize, GL_UNSIGNED_SHORT, (const GLvoid *)(ebo->getOffset(de->getBufferIndex())));
        
            // draw the remaining primsets normally
            for (unsigned i=1; i < _geom->getNumPrimitiveSets(); i++)
            {
                _geom->getPrimitiveSet(i)->draw(state, true);
            }
        }
    }

}

void
TileDrawable::setElevationRaster(const osg::Image*   image,
                                 const osg::Matrixf& scaleBias)
{
    _elevationRaster = image;
    _elevationScaleBias = scaleBias;

    if (osg::equivalent(0.0f, _elevationScaleBias(0,0)) ||
        osg::equivalent(0.0f, _elevationScaleBias(1,1)))
    {
        OE_WARN << "("<<_key.str()<<") precision error\n";
    }

    if ( _elevationRaster.valid() )
    {
        const osg::Vec3Array& verts   = *static_cast<osg::Vec3Array*>(_geom->getVertexArray());
        const osg::Vec3Array& normals = *static_cast<osg::Vec3Array*>(_geom->getNormalArray());

        //OE_INFO << LC << _key.str() << " - rebuilding height cache" << std::endl;

        ImageUtils::PixelReader elevation(_elevationRaster.get());
        elevation.setBilinear(true);

        float
            scaleU = _elevationScaleBias(0,0),
            scaleV = _elevationScaleBias(1,1),
            biasU  = _elevationScaleBias(3,0),
            biasV  = _elevationScaleBias(3,1);

        if ( osg::equivalent(scaleU, 0.0f) || osg::equivalent(scaleV, 0.0f) )
        {
            OE_WARN << LC << "Precision loss in tile " << _key.str() << "\n";
        }
    
        for(int t=0; t<_tileSize; ++t)
        {
            float v = (float)t / (float)(_tileSize-1);
            v = v*scaleV + biasV;

            for(int s=0; s<_tileSize; ++s)
            {
                float u = (float)s / (float)(_tileSize-1);
                u = u*scaleU + biasU;
                _heightCache[t*_tileSize+s] = elevation(u, v).r();
            }
        }
    }

    dirtyBound();
}

const osg::Image*
TileDrawable::getElevationRaster() const
{
    return _elevationRaster.get();
}

const osg::Matrixf&
TileDrawable::getElevationMatrix() const
{
    return _elevationScaleBias;
}

// Functor supplies triangles to things like IntersectionVisitor, ComputeBoundsVisitor, etc.
void
TileDrawable::accept(osg::PrimitiveFunctor& f) const
{
    const osg::Vec3Array& verts   = *static_cast<osg::Vec3Array*>(_geom->getVertexArray());
    const osg::Vec3Array& normals = *static_cast<osg::Vec3Array*>(_geom->getNormalArray());
        
#if 1 // triangles (OSG-stats-friendly)

    //TODO: improve by caching the entire Vec3f, not just the height.

    f.begin(GL_TRIANGLES);
    for(int t=0; t<_tileSize-1; ++t)
    {
        for(int s=0; s<_tileSize-1; ++s)
        {
            int i00 = t*_tileSize + s;
            int i10 = i00 + 1;
            int i01 = i00 + _tileSize;
            int i11 = i01 + 1;
            
            osg::Vec3d v01 = verts[i01] + normals[i01] * _heightCache[i01];
            osg::Vec3d v10 = verts[i10] + normals[i10] * _heightCache[i10];

            f.vertex( verts[i00] + normals[i00] * _heightCache[i00] );
            f.vertex( v01 );
            f.vertex( v10 );
            
            f.vertex( v10 );
            f.vertex( v01 );
            f.vertex( verts[i11] + normals[i11] * _heightCache[i11] );
        }
    }
    f.end();

#else
    // triangle-strips (faster? but not stats-friendly; will cause the OSG stats
    // to report _tileSize-1 primitive sets per TileDrawable even though there
    // is only one.

    for(int t=0; t<_tileSize-1; ++t)
    {
        f.begin( GL_TRIANGLE_STRIP );

        for(int s=0; s<_tileSize; ++s)
        {
            int i = t*_tileSize + s;
            f.vertex( verts[i] + normals[i] * _heightCache[i] );

            i += _tileSize;
            f.vertex( verts[i] + normals[i] * _heightCache[i] );
        }

        f.end();
    }

#endif
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


#if OSG_VERSION_GREATER_OR_EQUAL(3,1,8)
#   define GET_ARRAY(a) (a)
#else
#   define GET_ARRAY(a) (a).array
#endif


void
TileDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
    State& state = *renderInfo.getState();
    //bool checkForGLErrors = state.getCheckForGLErrors() == osg::State::ONCE_PER_ATTRIBUTE;
    //if ( checkForGLErrors ) state.checkGLErrors("start of TileDrawable::drawImplementation()");

#if 0
    const osg::FrameStamp* fs = state.getFrameStamp();
    if ( fs )
    {
        Threading::ScopedMutexLock lock(_profMutex);
        if ( s_frame != fs->getFrameNumber() )
        {
            OE_NOTICE << "Frame " << s_frame << " : draws = " << s_draws << ", functors = " << s_functors << std::endl;
            s_draws = 0;
            s_functors = 0;
            s_frame = fs->getFrameNumber();
        }
        s_draws++;
    }
#endif


#if OSG_MIN_VERSION_REQUIRED(3,3,1)
    _geom->drawVertexArraysImplementation( renderInfo );
#else
    drawVertexArraysImplementation( renderInfo );
#endif

    drawPrimitivesImplementation( renderInfo );

    //if ( checkForGLErrors ) state.checkGLErrors("end of TileDrawable::drawImplementation()");
    
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
    //arrayDispatchers.activateColorArray(_geom->getColorArray());
    //arrayDispatchers.activateSecondaryColorArray(_geom->getSecondaryColorArray());
    //arrayDispatchers.activateFogCoordArray(_geom->getFogCoordArray());

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

    //if (_geom->getColorArray() && _geom->getColorArray()->getBinding()==osg::Array::BIND_PER_VERTEX)
    //    state.setColorPointer(_geom->getColorArray());

    //if (_geom->getSecondaryColorArray() && _geom->getSecondaryColorArray()->getBinding()==osg::Array::BIND_PER_VERTEX)
    //    state.setSecondaryColorPointer(_geom->getSecondaryColorArray());

    //if (_geom->getFogCoordArray() && _geom->getFogCoordArray()->getBinding()==osg::Array::BIND_PER_VERTEX)
    //    state.setFogCoordPointer(_geom->getFogCoordArray());

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
    bool handleVertexAttributes = !_geom->getVertexAttribArrayList().empty();
    //bool handleVertexAttributes = !_vertexAttribList.empty();

    ArrayDispatchers& arrayDispatchers = state.getArrayDispatchers();

    arrayDispatchers.reset();
    arrayDispatchers.setUseVertexAttribAlias(state.getUseVertexAttributeAliasing());
    arrayDispatchers.setUseGLBeginEndAdapter(false);

    arrayDispatchers.activateNormalArray(_geom->getNormalBinding(), _geom->getNormalArray(), _geom->getNormalIndices());
    arrayDispatchers.activateColorArray(_geom->getColorBinding(), _geom->getColorArray(), _geom->getColorIndices());
    arrayDispatchers.activateSecondaryColorArray(_geom->getSecondaryColorBinding(), _geom->getSecondaryColorArray(), _geom->getSecondaryColorIndices());
    arrayDispatchers.activateFogCoordArray(_geom->getFogCoordBinding(), _geom->getFogCoordArray(), _geom->getFogCoordIndices());

    if (handleVertexAttributes)
    {
        for(unsigned int unit=0;unit < _geom->getVertexAttribArrayList().size();++unit)
        {
            const osg::Geometry::ArrayData& val = _geom->getVertexAttribArrayList()[unit];
            arrayDispatchers.activateVertexAttribArray(val.binding, unit, val.array.get(), val.indices.get());
        }
    }

    // dispatch any attributes that are bound overall
    arrayDispatchers.dispatch(_geom->BIND_OVERALL, 0);

    state.lazyDisablingOfVertexAttributes();

    // set up arrays
    if( _geom->getVertexArray() )
        state.setVertexPointer(_geom->getVertexArray()); //_vertexData.array.get());

    if (_geom->getNormalBinding()==_geom->BIND_PER_VERTEX && _geom->getNormalArray())
        state.setNormalPointer(_geom->getNormalArray());

    if (_geom->getColorBinding()==_geom->BIND_PER_VERTEX && _geom->getColorArray())
        state.setColorPointer(_geom->getColorArray());

    if (_geom->getSecondaryColorBinding()==_geom->BIND_PER_VERTEX && _geom->getSecondaryColorArray())
        state.setSecondaryColorPointer(_geom->getSecondaryColorArray());

    if (_geom->getFogCoordBinding()==_geom->BIND_PER_VERTEX && _geom->getFogCoordArray())
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
            const osg::Array* array = _geom->getVertexAttribArray(index);
            if ( array && _geom->getVertexAttribBinding(index) == _geom->BIND_PER_VERTEX )
            {
                if (array->getPreserveDataType())
                {
                    GLenum dataType = array->getDataType();
                    if (dataType==GL_FLOAT) state.setVertexAttribPointer( index, array, GL_FALSE );
                    else if (dataType==GL_DOUBLE) state.setVertexAttribLPointer( index, array );
                    else state.setVertexAttribIPointer( index, array );
                }
                else
                {
                    state.setVertexAttribPointer( index, array, GL_FALSE );
                }
            }
        }
    }

    state.applyDisablingOfVertexAttributes();
}

#endif
