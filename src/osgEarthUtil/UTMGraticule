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
#ifndef OSGEARTHUTIL_UTM_GRATICLE
#define OSGEARTHUTIL_UTM_GRATICLE

#include <osgEarthUtil/Common>
#include <osgEarth/MapNode>
#include <osgEarth/MapNodeObserver>
#include <osgEarth/ModelLayer>
#include <osgEarthSymbology/Style>
#include <osgEarthFeatures/Feature>
#include <osg/ClipPlane>
#include <vector>

namespace osgEarth { namespace Util
{
    using namespace osgEarth;
    using namespace osgEarth::Features;
    using namespace osgEarth::Symbology;

    /**
     * Configuration options for the geodetic graticule.
     */
    class UTMGraticuleOptions : public VisibleLayerOptions
    {
    public:
        UTMGraticuleOptions(const ConfigOptions& conf =ConfigOptions()) : VisibleLayerOptions(conf) {
            fromConfig(_conf);
        }
        
    public:

        /** Style for grid zone designator geometry and text */
        optional<Style>& gzdStyle() { return _gzdStyle; }
        const optional<Style>& gzdStyle() const { return _gzdStyle; }

        /** Text scale factor (default = 1) */
        optional<float>& textScale() { return _textScale; }
        const optional<float>& textScale() const { return _textScale; }

    public:
        virtual Config getConfig() const {
            Config conf = VisibleLayerOptions::getConfig();
            conf.set("gzd_style", _gzdStyle);
            conf.set("text_scale", _textScale);
            return conf;
        }

        virtual void fromConfig(const Config& conf) {
            conf.get("gzd_style", _gzdStyle);
            conf.get("text_scale", _textScale);
        }

    protected:
        optional<Style> _gzdStyle;
        optional<float> _textScale;

        void mergeConfig(const Config& conf) {
            VisibleLayerOptions::mergeConfig(conf);
            fromConfig(conf);
        }
    };


    /**
     * UTM data (used by the UTM Graticule and the MGRS Graticule).
     */
    class OSGEARTHUTIL_EXPORT UTMData
    {
    public:
        typedef std::map<std::string, GeoExtent> SectorTable;

    public:
        UTMData() { }

        void rebuild(const Profile* profile);

        osg::Group* buildGZDTile(const std::string& name, const GeoExtent& extent, const Style& style, const FeatureProfile* featureProfile, const Map* map);

        SectorTable& sectorTable() { return _gzd; }
        const SectorTable& sectorTable() const { return _gzd; }

    private:
        SectorTable _gzd;
    };


    /**
     * Implements a UTM map graticule. 
     * This only works for geocentric maps.
     */
    class OSGEARTHUTIL_EXPORT UTMGraticule : public VisibleLayer
    {
    public:
        META_Layer(osgEarthUtil, UTMGraticule, UTMGraticuleOptions, utm_graticule);

        /**
         * Construct a new UTM graticule with default settings.
         */
        UTMGraticule();

        /**
         * Constructs a new graticule for use with the specified map.
         *
         * @param options
         *      Optional "options" that configure the graticule. Defaults will be used
         *      if you don't specify this.
         */
        UTMGraticule(const UTMGraticuleOptions& options);

        /**
         * If you change any of the options, call this to refresh the display
         * to refelct the new settings.
         */
        void dirty();

    public: // Layer

        virtual void addedToMap(const Map* map);

        virtual void removedFromMap(const Map* map);
        
        virtual osg::Node* getNode() const;

        virtual void init();

    protected:

        /** dtor */
        virtual ~UTMGraticule() { }

    private:

        void setUpDefaultStyles();

        void rebuild();

        UID _uid;

        osg::ref_ptr<const Profile> _profile;

        osg::ref_ptr<FeatureProfile> _featureProfile;

        osg::ref_ptr<osg::ClipPlane> _clipPlane;

        osg::ref_ptr<osg::Group> _root;

        osg::observer_ptr<const Map> _map;

        UTMData _utmData;
    };

} } // namespace osgEarth::Util

#endif // OSGEARTHUTIL_UTM_GRATICLE
