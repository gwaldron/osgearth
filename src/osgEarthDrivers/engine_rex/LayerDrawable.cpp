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
    setStateSet(0L);
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
    OE_PROFILING_ZONE_TEXT(osgEarth::Stringify() << (_layer ? _layer->getName() : "unknown layer") << " (" << _tiles.size() << " tiles)");
    //OE_INFO << LC << (_layer ? _layer->getName() : "[empty]") << " tiles=" << _tiles.size() << std::endl;

    // Get this context's state values:
    PerContextDrawState& ds = _drawState->getPCDS(ri.getContextID());

    ds.refresh(ri, _drawState->_bindings);

    if (ds._layerUidUL >= 0)
    {
        GLint uid = _layer ? (GLint)_layer->getUID() : (GLint)-1;
        ds._ext->glUniform1i(ds._layerUidUL, uid);
    }
    else
    {
        // This just means that the fragment shader for this layer doesn't use oe_layer_uid
    }
    osg::ref_ptr<osg::Referenced> layerData;

    if (_patchLayer && _patchLayer->getDrawCallback())
    {
        _patchLayer->getDrawCallback()->preDraw(ri, layerData);
    }

    for (DrawTileCommands::const_iterator tile = _tiles.begin(); tile != _tiles.end(); ++tile)
    {
        tile->draw(ri, *_drawState, layerData.get());
    }

    if (_patchLayer && _patchLayer->getDrawCallback())
    {
        _patchLayer->getDrawCallback()->postDraw(ri, layerData.get());
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
        ds._ext->glBindBuffer(GL_ARRAY_BUFFER_ARB,0);
        ds._ext->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB,0);

        // gw: no need to do this, in fact it will cause positional attributes
        // (light clip planes and lights) to immediately be reapplied under the
        // current MVM, which will by definition be wrong!)
        //ri.getState()->apply();
    }
}
