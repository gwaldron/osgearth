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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef OSGEARTH_PATCH_LAYER_H
#define OSGEARTH_PATCH_LAYER_H 1

#include <osgEarth/VisibleLayer>
#include <osg/RenderInfo>
#include <osg/Texture>

namespace osgEarth
{
    //! Options that govern a PatchLayer
    class /*header only*/ PatchLayerOptions : public VisibleLayerOptions
    {
    public:
        PatchLayerOptions(const ConfigOptions& co = ConfigOptions()) : VisibleLayerOptions(co) { }
    };

    /**
     * PatchLayer is a layer that can render terrain tiles using either
     * a geometry shader patch (GL_PATCHES) or a custom draw callback.
     */
    class OSGEARTH_EXPORT PatchLayer : public VisibleLayer
    {
    public:
        META_Layer(osgEarth, PatchLayer, PatchLayerOptions, patch);

        struct TileData : public osg::Referenced
        {
            osg::ref_ptr<osg::Texture> _texture;
        };

        struct DrawContext
        {
            const TileKey* key;
            float          range;
            osg::Texture*  colorTexture;
            osg::Texture*  elevationTexture;
            osg::Texture*  normalTexture;
            osg::Texture*  coverageTexture;
            DrawContext() : range(0.0f), colorTexture(0), elevationTexture(0), normalTexture(0), coverageTexture(0) { }
        };

        /**
         * Callback that the terrain engine will call for custom tile rendering.
         */
        struct DrawCallback : public osg::Referenced
        {
            virtual void draw(osg::RenderInfo& ri, const DrawContext& di, osg::Referenced* data) =0;
        };

        /**
         * Callback that the terrain engine will call to decide whether to 
         * render a tile in this layer.
         */
        struct AcceptCallback : public osg::Referenced
        {
            /** Whether to accept the entire layer. Innvoked during Cull. */
            virtual bool acceptLayer(osg::NodeVisitor& nv, const osg::Camera* camera) const { return true; }

            /** Whether to accept a specific tile. Invoked during Cull. */
            virtual bool acceptKey(const TileKey& key) const { return true; }
        };


    public:
        PatchLayer();

        /**
         * Draw callback to use to render tiles in this layer
         */
        void setDrawCallback(DrawCallback* value) { _drawCallback = value; }
        DrawCallback* getDrawCallback() const { return _drawCallback.get(); }

        /**
         * Terrain LOD at which to render tiles in this patch layer
         */
        void setAcceptCallback(AcceptCallback* value) { _acceptCallback = value; }
        AcceptCallback* getAcceptCallback() const { return _acceptCallback.get(); }


        TileData* createTileData(const TileKey& key) { return 0L; }

    protected:

        //! Subclass constructor
        PatchLayer(PatchLayerOptions* options);

        /** dtor */
        virtual ~PatchLayer() { }

        // post-ctor initialization, chain to subclasses.
        virtual void init();

    private:
        osg::ref_ptr<DrawCallback> _drawCallback;
        osg::ref_ptr<AcceptCallback> _acceptCallback;
    };

    typedef std::vector< osg::ref_ptr<PatchLayer> > PatchLayerVector;

} // namespace osgEarth

#endif // OSGEARTH_PATCH_LAYER_H
