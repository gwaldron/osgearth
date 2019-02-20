/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#ifndef OSGEARTH_SPLAT_GROUND_COVER_LAYER_H
#define OSGEARTH_SPLAT_GROUND_COVER_LAYER_H

#include "Export"
#include "Coverage"
#include "Zone"
#include <osgEarth/PatchLayer>
#include <osgEarth/LayerListener>
#include <osgEarth/LandCoverLayer>

namespace osgEarth { namespace Features {
    class FeatureSource;
    class FeatureSourceLayer;
} }

namespace osgEarth { namespace Splat
{
    using namespace osgEarth;
    using namespace osgEarth::Features;

    //! Serializable configuration for a GroundCoverLayer.
    class OSGEARTHSPLAT_EXPORT GroundCoverLayerOptions : public PatchLayerOptions
    {
    public:
        GroundCoverLayerOptions(const ConfigOptions& co = ConfigOptions()) :
            PatchLayerOptions(co),
            _lod(13u),
            _castShadows(false)
        {
            fromConfig(_conf);
        }

        //! Required layer containing land cover data
        optional<std::string>& landCoverLayer() { return _landCoverLayerName; }
        const optional<std::string>& landCoverLayer() const { return _landCoverLayerName; }        
        
        //! Optional name of a map layer to use for masking ground cover
        optional<std::string>& maskLayer() { return _maskLayerName; }
        const optional<std::string>& maskLayer() const { return _maskLayerName; }

        //! Geographic zones
        std::vector<ZoneOptions>& zones() { return _zones; }
        const std::vector<ZoneOptions>& zones() const { return _zones; }

        //! Terrain LOD at which this layer should render
        optional<unsigned>& lod() { return _lod; }
        const optional<unsigned>& lod() const { return _lod; }

        //! Whether objects in this layer should cast shadows.
        optional<bool>& castShadows() { return _castShadows; }
        const optional<bool>& castShadows() const { return _castShadows; }

    public:
        virtual Config getConfig() const;
    protected:
        virtual void mergeConfig(const Config& conf) {
            VisibleLayerOptions::mergeConfig(conf);
            fromConfig(conf);
        }
        void fromConfig(const Config& conf);

    private:
        optional<std::string> _landCoverLayerName;
        optional<std::string> _maskLayerName;
        ZoneOptionsVector _zones;
        optional<unsigned> _lod;
        optional<bool> _castShadows;
    };


    //! Layer that renders billboards on the ground using the GPU,
    //! like trees, grass, rocks, etc.
    class OSGEARTHSPLAT_EXPORT GroundCoverLayer : public PatchLayer
    {
    public:
        META_Layer(osgEarthSplat, GroundCoverLayer, GroundCoverLayerOptions, splat_groundcover);

        //! Construct a layer with default configuration
        GroundCoverLayer();
        
        //! Construct a layer with configuration options
        GroundCoverLayer(const GroundCoverLayerOptions& options);

        //! Layer containing required coverage data
        void setLandCoverLayer(LandCoverLayer* landCoverLayer);

        //! Layer containing the land cover dictionary.
        void setLandCoverDictionary(LandCoverDictionary* landCoverDict);

        //! Masking layer (optional)
        void setMaskLayer(ImageLayer* layer);

        //! LOD at which to draw ground cover
        unsigned getLOD() const;

        //! Geogrphic zones; at least one is required
        Zones& zones() { return _zones; }
        const Zones& zones() const { return _zones; }

    protected:

        //! Override post-ctor init
        virtual void init();

        //! Override layer open
        virtual const Status& open();

    public:

        //! Called when this layer is added to the map
        virtual void addedToMap(const Map* map);
        virtual void removedFromMap(const Map* map);
        virtual void setTerrainResources(TerrainResources*);

        virtual void resizeGLObjectBuffers(unsigned maxSize);
        virtual void releaseGLObjects(osg::State* state) const;

    protected:
        virtual ~GroundCoverLayer() { }

        osg::observer_ptr<LandCoverDictionary> _landCoverDict;
        osg::observer_ptr<LandCoverLayer> _landCoverLayer;
        osg::observer_ptr<ImageLayer> _maskLayer;

        LayerListener<GroundCoverLayer, LandCoverDictionary> _landCoverDictListener;
        LayerListener<GroundCoverLayer, LandCoverLayer> _landCoverListener;
        LayerListener<GroundCoverLayer, ImageLayer> _maskLayerListener;

        TextureImageUnitReservation _groundCoverTexBinding;
        TextureImageUnitReservation _noiseBinding;

        Zones _zones;
        bool _zonesConfigured;

        void buildStateSets();

        struct LayerAcceptor : public PatchLayer::AcceptCallback
        {
            GroundCoverLayer* _layer;
            LayerAcceptor(GroundCoverLayer* layer) : _layer(layer) { }
            bool acceptLayer(osg::NodeVisitor& nv, const osg::Camera* camera) const;
            bool acceptKey(const TileKey& key) const;
        };
        friend struct LayerAcceptor;

        struct ZoneSelector : public Layer::TraversalCallback
        {
            GroundCoverLayer* _layer;
            ZoneSelector(GroundCoverLayer* layer) : _layer(layer) { }
            void operator()(osg::Node*, osg::NodeVisitor*) const;
        };
        friend struct ZoneSelector;
    };

} } // namespace osgEarth::Splat

#endif // OSGEARTH_SPLAT_GROUND_COVER_LAYER_H
