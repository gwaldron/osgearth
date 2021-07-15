/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include "LayerDrawable"
#include "TerrainRenderData"
#include <osgEarth/Metrics>
#include <sstream>

using namespace osgEarth::REX;

#undef  LC
#define LC "[LayerDrawable] "


LayerDrawable::LayerDrawable() :
_renderType(Layer::RENDERTYPE_TERRAIN_SURFACE),
_drawOrder(0),
_layer(0L),
_visibleLayer(0L),
_imageLayer(0L),
_patchLayer(0L),
_clearOsgState(false),
_draw(true)
{
    setDataVariance(DYNAMIC);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
    _tiles.reserve(128);
}

LayerDrawable::~LayerDrawable()
{
    // Drawable's DTOR will release GL objects on any attached stateset;
    // we don't want that because our Layer stateset is shared and re-usable.
    // So detach it before OSG has a chance to do so.
    setStateSet(nullptr);
}

namespace
{
    // Hack State so we can dirty the texture attrs without dirtying the other 
    // attributes (as dirtyAllAttributes() would do.
    struct StateEx : public osg::State
    {
        void dirtyAllTextureAttributes()
        {
            // dirtyAllTextureAttributes. (Don't call state->dirtyAllAttributes because that
            // will mess up positional state attributes like light sources)
            for (TextureAttributeMapList::iterator tamItr = _textureAttributeMapList.begin();
                tamItr != _textureAttributeMapList.end();
                ++tamItr)
            {
                osg::State::AttributeMap& attributeMap = *tamItr;
                for (osg::State::AttributeMap::iterator aitr = attributeMap.begin();
                    aitr != attributeMap.end();
                    ++aitr)
                {
                    osg::State::AttributeStack& as = aitr->second;
                    as.last_applied_attribute = 0;
                    as.changed = true;
                }
            }
        }
    };
}

void
LayerDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    OE_PROFILING_ZONE;
    //char buf[64];
    //sprintf(buf, "%.36s (%zd tiles)", _layer ? _layer->getName().c_str() : "unknown layer", _tiles.size());
    //OE_PROFILING_ZONE_TEXT(buf);

    if (_patchLayer && _patchLayer->getRenderer())
    {
        TileBatch batch(_drawState.get());
        batch._tiles.reserve(_tiles.size());
        for (auto& tile : _tiles)
            batch._tiles.push_back(&tile);

        _patchLayer->getRenderer()->draw(ri, batch);
    }
    else
    {
        ProgramState& pps = _drawState->getProgramState(ri);

        if (pps._layerUidUL >= 0)
        {
            osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
            GLint uid = _layer ? (GLint)_layer->getUID() : (GLint)-1;
            ext->glUniform1i(pps._layerUidUL, uid);
        }

        for (auto& tile : _tiles)
        {
            tile.apply(ri, _drawState.get());
            tile.draw(ri);
        }
    }

    // If set, dirty all OSG state to prevent any leakage - this is sometimes
    // necessary when doing custom OpenGL within a Drawable.
    if (_clearOsgState)
    {
        // Dirty the texture attributes so OSG can properly reset them
        // NOTE: cannot call state.dirtyAllAttributes, because that would invalidate
        // positional state like light sources!
        reinterpret_cast<StateEx*>(ri.getState())->dirtyAllTextureAttributes();

        // NOTE: this is a NOOP in OSG 3.5.x, but not in 3.4.x ... Later we will need to
        // revisit whether to call disableAllVertexArrays() in 3.5.x instead.
        ri.getState()->dirtyAllVertexArrays();
        
        // unbind local buffers when finished.
        osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

        // Not necessary if using VAOs?
        ext->glBindBuffer(GL_ARRAY_BUFFER_ARB,0);
        ext->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB,0);

        // gw: no need to do this, in fact it will cause positional attributes
        // (light clip planes and lights) to immediately be reapplied under the
        // current MVM, which will by definition be wrong!)
        //ri.getState()->apply();
    }
}

void LayerDrawable::accept(osg::PrimitiveFunctor& functor) const
{
    for (auto& tile : _tiles)
        tile.accept(functor);
}

void LayerDrawable::accept(osg::PrimitiveIndexFunctor& functor) const
{
    for (auto& tile : _tiles)
        tile.accept(functor);
}
