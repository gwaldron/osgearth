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
#ifndef OSGEARTH_REX_TERRAIN_LAYER_DRAWABLE_H
#define OSGEARTH_REX_TERRAIN_LAYER_DRAWABLE_H 1

#include "DrawTileCommand"
#include "DrawState"

#include <osgEarth/ImageLayer>
#include <vector>

using namespace osgEarth;

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    class TerrainRenderData;

    /**
     * Drawable for single "Layer" i.e. rendering pass. 
     * It is important that LayerDrawables be rendered in the order in which
     * they appear. Since all LayerDrawables share a common bounds, this 
     * should happen automatically, but let's keep an eye out for trouble.
     */
    class LayerDrawable : public osg::Drawable
    {
    public:
        LayerDrawable();

        // The (sorted) list of tiles to render for this layer
        DrawTileCommands  _tiles;

        // Determines whether to use the default surface shader program
        Layer::RenderType _renderType;

        // Pointer back to the actual Map layer, if there is one
        const Layer* _layer;

        // If _layer is a VisibleLayer, this will be set as well, otherwise NULL
        const VisibleLayer* _visibleLayer;

        // If _layer is an ImageLayer, this will be set as well, otherwise NULL
        const ImageLayer* _imageLayer;

        // Layer render order, which is pushed into a Uniform at render time.
        // This value is assigned at cull time by RexTerrainEngineNode.
        int _drawOrder;

        // The last layer to render will have this flag set, which will
        // prompt the render to dirty the osg::State to prevent corruption.
        // This flag is set at cull time by RexTerrainEngineNode.
        bool _clearOsgState;

        // Reference the terrain-wide state
        osg::ref_ptr<DrawState> _drawState;

        // Whether to render this layer.
        bool _draw;
        

    public: // osg::Drawable
        
        void drawImplementation(osg::RenderInfo& ri) const;

        // All LayerDrawables share the common terrain bounds.
        osg::BoundingSphere computeBound() const { return _drawState->_bs; }
        osg::BoundingBox computeBoundingBox() const { return _drawState->_box; }

    protected:
        // overriden to prevent OSG from releasing GL objects on an attached stateset.
        virtual ~LayerDrawable();
    };


    // Straight list of LayerDrawables.
    typedef std::vector< osg::ref_ptr<LayerDrawable> > LayerDrawableList;
    typedef std::map<UID, osg::ref_ptr<LayerDrawable> > LayerDrawableMap;

} } } // namespace 

#endif // OSGEARTH_REX_TERRAIN_LAYER_DRAWABLE_H
