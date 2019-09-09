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

#ifndef OSGEARTH_MASK_LAYER_H
#define OSGEARTH_MASK_LAYER_H 1

#include <osgEarth/Common>
#include <osgEarth/Layer>
#include <osgEarth/Config>
#include <osgEarth/MaskSource>
#include <osg/Node>

namespace osgEarth
{
    class Map;

    /**
     * Configuration options for a MaskLayer.
     */
    class OSGEARTH_EXPORT MaskLayerOptions : public LayerOptions
    {
    public:
        MaskLayerOptions(const ConfigOptions& options =ConfigOptions());

        /** dtor */
        virtual ~MaskLayerOptions() { }

        /** Minimum of detail for which this layer should apply. */
        optional<unsigned>& minLevel() { return _minLevel; }
        const optional<unsigned>& minLevel() const { return _minLevel; }

    public:
        virtual Config getConfig() const;
        virtual void mergeConfig( const Config& conf );

    private:
        void fromConfig( const Config& conf );
        void setDefaults();

        optional<MaskSourceOptions> _driver;
        optional<unsigned>          _minLevel;
    };


    /**
     * A MaskLayer is a specialized layer used to mask out a part of the terrain.
     * Typically you would use this if you had a pre-built 3D terrain model for
     * an inset area.
     */
    class OSGEARTH_EXPORT MaskLayer : public Layer
    {
    public:
        META_Layer(osgEarth, MaskLayer, MaskLayerOptions, mask);

        /**
         * Minimum terrain LOD at which masking should occur
         */
        unsigned getMinLevel() const { return options().minLevel().get(); }

        /** 
         * Gets the geometric boundary polygon representing the area of the
         * terrain to mask out.
         */
        virtual osg::Vec3dArray* getOrCreateMaskBoundary(
            float                   heightScale,
            const SpatialReference* srs,
            ProgressCallback*       progress) { return 0L; }

    protected:

        /** Create from subclass. */
        MaskLayer(MaskLayerOptions* =0L);

        /** dtor */
        virtual ~MaskLayer() { }
    };

    typedef std::vector< osg::ref_ptr<MaskLayer> > MaskLayerVector;
}

#endif // OSGEARTH_MASK_LAYER_H

