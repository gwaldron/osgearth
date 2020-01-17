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
#ifndef OSGEARTHANNOTATION_GARS_GRATICLE
#define OSGEARTHANNOTATION_GARS_GRATICLE

#include <osgEarth/VisibleLayer>
#include <osgEarthUtil/Common>
#include <osgEarthSymbology/Style>

namespace osgEarth { namespace Util
{
    using namespace osgEarth;
    using namespace osgEarth::Symbology;

    /**
     * Configuration options for the geodetic graticule.
     */
    class GARSGraticuleOptions : public VisibleLayerOptions
    {
    public:
        GARSGraticuleOptions(const ConfigOptions& conf =ConfigOptions()) : VisibleLayerOptions(conf) {
            fromConfig(_conf);
        }
        
    public:
        //! Style for grid zone designator geometry and text
        optional<Style>& style() { return _style; }
        const optional<Style>& style() const { return _style; }

    public:
        virtual Config getConfig() const {
            Config conf = VisibleLayerOptions::getConfig();
            conf.set("style", _style);
            return conf;
        }

    protected:
        optional<Style> _style;

        virtual void mergeConfig(const Config& conf) {
            VisibleLayerOptions::mergeConfig(conf);
            fromConfig(conf);
        }

        void fromConfig(const Config& conf) {
            conf.get("style", _style);
        }
    };

    /**
     * GARS (Global Area Reference System) Graticuler map layer.
     * http://earth-info.nga.mil/GandG/coordsys/grids/gars.html
     */
    class OSGEARTHUTIL_EXPORT GARSGraticule : public VisibleLayer
    {
    public:
        META_Layer(osgEarthUtil, GARSGraticule, GARSGraticuleOptions, gars_graticule);

        //! Construct a default GARS graticule
        GARSGraticule();

        //! Construct a graticule with custom options
        GARSGraticule(const GARSGraticuleOptions& options);

        //! Call to refresh after setting an option
        void dirty();

    public: // Layer

        virtual void addedToMap(const Map* map);

        virtual void removedFromMap(const Map* map);
        
        virtual osg::Node* getNode() const;

        virtual void init();

    protected:

        /** dtor */
        virtual ~GARSGraticule() { }        

    private:

        void rebuild();
        void build30MinCells();

        UID _uid;
        osg::ref_ptr<const Profile> _profile;
        osg::ref_ptr<osg::Group> _root;
    };  
} } // namespace osgEarth::Util

#endif // OSGEARTHANNOTATION_GARS_GRATICLE
