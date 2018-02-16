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
#include "LayerDrawable"

using namespace osgEarth::Drivers::RexTerrainEngine;

#undef  LC
#define LC "[LayerDrawable] "


LayerDrawable::LayerDrawable() :
_renderType(Layer::RENDERTYPE_TERRAIN_SURFACE),
_order(0),
_layer(0L),
_clearOsgState(false),
_draw(true)
{
    setDataVariance(DYNAMIC);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
}

void
LayerDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    //OE_INFO << LC << (_layer ? _layer->getName() : "[empty]") << " tiles=" << _tiles.size() << std::endl;

    // Get this context's state values:
    PerContextDrawState& ds = _drawState->getPCDS(ri.getContextID());

    ds.refresh(ri, _drawState->_bindings);

    if (_layer)
    {
        if (ds._layerUidUL >= 0)
            ds._ext->glUniform1i(ds._layerUidUL,      (GLint)_layer->getUID());
        if (ds._layerOpacityUL >= 0 && _visibleLayer)
            ds._ext->glUniform1f(ds._layerOpacityUL,  (GLfloat)_visibleLayer->getOpacity());
        if (ds._layerMinRangeUL >= 0 && _imageLayer)
            ds._ext->glUniform1f(ds._layerMinRangeUL, (GLfloat)_imageLayer->getMinVisibleRange());
        if (ds._layerMaxRangeUL >= 0 && _imageLayer)
            ds._ext->glUniform1f(ds._layerMaxRangeUL, (GLfloat)_imageLayer->getMaxVisibleRange());
    }
    else
    {
        if (ds._layerUidUL >= 0)
            ds._ext->glUniform1i(ds._layerUidUL,      (GLint)-1);
        if (ds._layerOpacityUL >= 0)
            ds._ext->glUniform1f(ds._layerOpacityUL,  (GLfloat)1.0f);
        if (ds._layerMinRangeUL >= 0)
            ds._ext->glUniform1f(ds._layerMinRangeUL, (GLfloat)0.0f);
        if (ds._layerMaxRangeUL >= 0)
            ds._ext->glUniform1f(ds._layerMaxRangeUL, (GLfloat)FLT_MAX);
    }

    for (DrawTileCommands::const_iterator tile = _tiles.begin(); tile != _tiles.end(); ++tile)
    {
        tile->draw(ri, *_drawState, 0L);
    }

    // If set, dirty all OSG state to prevent any leakage - this is sometimes
    // necessary when doing custom OpenGL within a Drawable.
    if (_clearOsgState)
    {
        // Necessary because tile->draw() applies texture attributes without informing the State:
        ri.getState()->dirtyAllAttributes();

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
