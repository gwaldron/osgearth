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
#ifndef OSGEARTH_UTIL_MULTI_ELEVATION_LAYER
#define OSGEARTH_UTIL_MULTI_ELEVATION_LAYER 1

#include <osgEarthUtil/Common>
#include <osgEarth/ElevationLayer>

namespace osgEarth { namespace Util
{
    using namespace osgEarth;

    /**
     * Combines multiple elevation layers into one.
     */
    class OSGEARTHUTIL_EXPORT MultiElevationLayerOptions : public ElevationLayerOptions
    {
    public:
        // constructor
        MultiElevationLayerOptions(const ConfigOptions& co = ConfigOptions());
        
        std::vector<ConfigOptions>& layers() { return _layers; }
        const std::vector<ConfigOptions>& layers() const { return _layers; }

    public:
        virtual Config getConfig() const;

    protected:
        virtual void mergeConfig(const Config& conf) {
            ElevationLayerOptions::mergeConfig(conf);
            fromConfig(conf);
        }

        void fromConfig(const Config& conf);
        
    private:
        std::vector<ConfigOptions> _layers;
    };


    /**
     * Elevation layer that...
     */
    class OSGEARTHUTIL_EXPORT MultiElevationLayer : public ElevationLayer
    {
    public:
        META_Layer(osgEarth, MultiElevationLayer, MultiElevationLayerOptions, multi_elevation);

        //! Create a blank layer to be configurated through options().
        MultiElevationLayer();

        //! Create a layer with initial options.
        MultiElevationLayer(const MultiElevationLayerOptions& options);

    public: // ElevationLayer

        // override to generate custom heightfield
        virtual void createImplementation(
            const TileKey& key,
            osg::ref_ptr<osg::HeightField>& out_heightField,
            osg::ref_ptr<NormalMap>& out_normalMap,
            ProgressCallback* progress);

    protected: // Layer

        // opens the layer and returns the status
        virtual const Status& open();

        virtual void init();

        virtual void addedToMap(const class Map*);

        virtual void removedFromMap(const class Map*);

    protected:

        virtual ~MultiElevationLayer();

        ElevationLayerVector _layers;
    };

} } // namespace osgEarth::Util

#endif // OSGEARTH_UTIL_FRACTAL_ELEVATION_LAYER
