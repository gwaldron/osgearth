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
#ifndef OSGEARTH_SPLAT_SPLAT_LAYER_H
#define OSGEARTH_SPLAT_SPLAT_LAYER_H

#include "Export"
#include "Coverage"
#include "Zone"
#include <osgEarth/VisibleLayer>
#include <osgEarth/LayerListener>
#include <osgEarth/LandCoverLayer>

namespace osgEarth {
    namespace Features {
        class FeatureSource;
        class FeatureSourceLayer;
    }
}
namespace osgEarth { namespace Splat
{
    using namespace osgEarth;
    using namespace osgEarth::Features;
    
    //! Options to configure a splat layer.
    class OSGEARTHSPLAT_EXPORT SplatLayerOptions : public VisibleLayerOptions
    {
    public:
        SplatLayerOptions(const ConfigOptions& co = ConfigOptions()) :
            VisibleLayerOptions(co)
        {
            fromConfig(_conf);
        }

        //! Required layer containing land cover data
        optional<std::string>& landCoverLayer() { return _landCoverLayerName; }
        const optional<std::string>& landCoverLayer() const { return _landCoverLayerName; }

        //! Splatting zones (one required, more optional)
        std::vector<ZoneOptions>& zones() { return _zones; }
        const std::vector<ZoneOptions>& zones() const { return _zones; }

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
        std::vector<ZoneOptions> _zones;
    };


    //! Layer that renders geotypical textures on the terrain based on
    //! classification data ("texture splatting").
    class OSGEARTHSPLAT_EXPORT SplatLayer : public VisibleLayer
    {
    public:
        META_Layer(osgEarthSplat, SplatLayer, SplatLayerOptions, splat_imagery);

        //! Construct a splat layer with default configuration
        SplatLayer();
        
        //! Construct a splat layer with configuration options
        SplatLayer(const SplatLayerOptions& options);

        //! Layer containing required coverage data
        void setLandCoverLayer(LandCoverLayer* landCoverLayer);

        //! Layer containing the land cover dictionary.
        void setLandCoverDictionary(LandCoverDictionary* landCoverDict);

        //! Splatting zones
        Zones& zones() { return _zones; }
        const Zones& zones() const { return _zones; }

    protected:

        //! Override post-ctor init
        virtual void init();

    public:

        //! Called when this layer is added to the map
        virtual void addedToMap(const Map* map);
        virtual void removedFromMap(const Map* map);
        virtual void setTerrainResources(TerrainResources* res);

    public:

        virtual void resizeGLObjectBuffers(unsigned maxSize);
        virtual void releaseGLObjects(osg::State* state) const;

    protected:
        virtual ~SplatLayer() { }

        osg::observer_ptr<LandCoverDictionary> _landCoverDict;
        osg::observer_ptr<LandCoverLayer> _landCoverLayer;

        LayerListener<SplatLayer, LandCoverDictionary> _landCoverDictListener;
        LayerListener<SplatLayer, LandCoverLayer> _landCoverListener;

        TextureImageUnitReservation _splatBinding;
        TextureImageUnitReservation _lutBinding;
        TextureImageUnitReservation _noiseBinding;

        Zones _zones;
        bool _zonesConfigured;
        bool _editMode;
        bool _gpuNoise;

        void buildStateSets();

        void aaa();

        struct ZoneSelector : public Layer::TraversalCallback
        {
            SplatLayer* _layer;
            ZoneSelector(SplatLayer* layer) : _layer(layer) { }
            void operator()(osg::Node*, osg::NodeVisitor*) const;
        };
        friend struct ZoneSelector;
    };

} } // namespace osgEarth::Splat

#endif // OSGEARTH_SPLAT_SPLAT_LAYER_FACTORY_H
