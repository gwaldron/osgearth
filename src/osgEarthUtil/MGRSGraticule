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
#ifndef OSGEARTHUTIL_MGRS_GRATICLE
#define OSGEARTHUTIL_MGRS_GRATICLE

#include <osgEarthUtil/Common>
#include <osgEarth/VisibleLayer>
#include <osgEarthSymbology/StyleSheet>
#include <osgEarthFeatures/Feature>
#include <osg/ClipPlane>


namespace osgEarth { namespace Util
{
    using namespace osgEarth;
    using namespace osgEarth::Features;
    using namespace osgEarth::Symbology;

    /**
     * Configuration options for the geodetic graticule.
     */
    class OSGEARTHUTIL_EXPORT MGRSGraticuleOptions : public VisibleLayerOptions
    {
    public:
        //! Construct an MGRS options structure
        MGRSGraticuleOptions(const ConfigOptions& conf =ConfigOptions());

        //! Location of the SQID feature data
        //! This is a prebuilt binary file that you should distribute 
        //! with the application - typically names "mgrs_sqid.bin"
        optional<URI>& sqidData() { return _sqidURI; }
        const optional<URI>& sqidData() const { return _sqidURI; }

        //! Styles for each display level. Each level should be names in the
        //! following manner:
        //! The top-level grid zone designator is called "gzd".
        //! Each consecutive level is a power of 10, starting with "100000"
        //! (for the SQID 100km grid) and going down from there (10000, 1000, etc.
        //! down to level 1).
        //! You can set only certain levels if you wish in order to override
        //! the defaults.
        osg::ref_ptr<StyleSheet>& styleSheet() { return _styleSheet; }
        const osg::ref_ptr<StyleSheet>& styleSheet() const { return _styleSheet; }

        //! Whether to apply default styles when you do not expressly
        //! set them for all levels. Defaults to true.
        optional<bool>& useDefaultStyles() { return _useDefaultStyles; }
        const optional<bool>& useDefaultStyles() const { return _useDefaultStyles; }

    public:
        virtual Config getConfig() const;

    protected:
        virtual void mergeConfig(const Config& conf) {
            VisibleLayerOptions::mergeConfig(conf);
            fromConfig(conf);
        }

        void fromConfig(const Config& conf);

        optional<URI> _sqidURI;
        osg::ref_ptr<StyleSheet> _styleSheet;
        optional<bool> _useDefaultStyles;
    };


    /**
     * Implements a MGRS (Military Grid Reference System) map graticule. 
     * 
     * NOTE: So far, this only works for geocentric maps.
     * TODO: Add projected support; add text label support
     */
    class OSGEARTHUTIL_EXPORT MGRSGraticule : public VisibleLayer
    {
    public:
        META_Layer(osgEarthUtil, MGRSGraticule, MGRSGraticuleOptions, mgrs_graticule);

        //! Constructs a default graticule
        MGRSGraticule();

        /**
         * Constructs a new graticule for use with the specified map.
         *
         * @param options
         *      Optional "options" that configure the graticule. Defaults will be used
         *      if you don't specify this.
         */
        MGRSGraticule(const MGRSGraticuleOptions& options);

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
        virtual ~MGRSGraticule() { }


    private:
        
        void setUpDefaultStyles();

        void rebuild();

        osg::ref_ptr<const Profile> _profile;

        osg::ref_ptr<FeatureProfile> _featureProfile;

        osg::ref_ptr<osg::Group> _root;

        osg::observer_ptr<const Map> _map;

        void loadGZDFeatures(const SpatialReference* srs, FeatureList& output) const;

    };

} } // namespace osgEarth::Util

#endif // OSGEARTHUTIL_UTM_GRATICLE
