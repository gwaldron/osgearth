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
#ifndef OSGEARTH_LAND_COVER_LAYER
#define OSGEARTH_LAND_COVER_LAYER 1

#include <osgEarth/ImageLayer>
#include <osgEarth/LandCover>

namespace osgEarth
{
    /**
     * Serializable configuration for a LandCoverLayer.
     */
    class OSGEARTH_EXPORT LandCoverLayerOptions : public ImageLayerOptions
    {
    public:
        LandCoverLayerOptions(const ConfigOptions& co = ConfigOptions());
        
        /**
         * Images layer from which to read source coverage data.
         */
        std::vector<LandCoverCoverageLayerOptions>& coverages() { return _coverages; }
        const std::vector<LandCoverCoverageLayerOptions>& coverages() const { return _coverages; }

        /**
         * Amount by which to warp texture coordinates of coverage data.
         * Try 0.01 as a starting point.
         */
        optional<float>& warpFactor() { return _warp; }
        const optional<float>& warpFactor() const { return _warp; }

        /**
         * LOD at which to calculate the noise function for warping.
         */
        optional<unsigned>& noiseLOD() { return _noiseLOD; }
        const optional<unsigned>& noiseLOD() const { return _noiseLOD; }

    public:
        virtual Config getConfig() const;

    protected:
        void fromConfig(const Config& conf);
        virtual void mergeConfig(const Config& conf) {
            ImageLayerOptions::mergeConfig(conf);
            fromConfig(conf);
        }

        optional<float> _warp;
        optional<unsigned> _noiseLOD;
        std::vector<LandCoverCoverageLayerOptions> _coverages;
    };

    
    /**
     * Layer that provides land cover raster data, in which each texel 
     * contains a land cover code as defined in the LandCoverDictionary.
     * This appears in a Map as a shared, non-visible Layer.
     */
    class OSGEARTH_EXPORT LandCoverLayer : public ImageLayer
    {
    public:
        META_Layer(osgEarth, LandCoverLayer, LandCoverLayerOptions, land_cover);

        /** Construct an emptry land cover layer. Use options() to configure */
        LandCoverLayer();

        /** Construct a land cover layer from options. */
        LandCoverLayer(const LandCoverLayerOptions& options);

        //! Given a land cover tile, which you can generate by calling
        //! createImage, get the land cover class at the given parametric
        //! coordinates [0..1].
        const LandCoverClass* getClassByUV(const GeoImage& tile, double u, double v) const;

    protected: // Layer

        virtual void init();

        virtual void addedToMap(const class Map*);

        virtual GeoImage createImageImplementation(const TileKey& key, ProgressCallback*);

    protected: // TerrainLayer

        virtual TileSource* createTileSource();

        osg::ref_ptr<LandCoverDictionary> _lcDictionary;

        struct MetaImageComponent {
            MetaImageComponent() : pixel(0L) { }
            osg::ref_ptr<osg::Image> image;
            osg::Matrix scaleBias;
            ImageUtils::PixelReader pixel;
        };
        typedef std::map<TileKey, MetaImageComponent> MetaImage;
        bool readMetaImage(MetaImage&, const TileKey&, double, double, osg::Vec4f& output, ProgressCallback*);
    };

} // namespace osgEarth

#endif // OSGEARTH_LAND_COVER_LAYER
