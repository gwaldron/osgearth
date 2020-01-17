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

#ifndef OSGEARTH_FEATURES_SESSION_H
#define OSGEARTH_FEATURES_SESSION_H 1

#include <osgEarthFeatures/Common>

#include <osgEarth/MapInfo>
#include <osgEarth/Map>
#include <osgEarth/URI>

namespace osgEarth {
    class StateSetCache;
    namespace Symbology {
        class ResourceCache;
        class StyleSheet;
    }
    namespace Features {
        class FeatureSource;
        class ScriptEngine;
    }
}
namespace osgEarth { namespace Features
{
    using namespace osgEarth;
    using namespace osgEarth::Symbology;

    /**
     * Session is a state object that exists throughout the life of one or more related
     * feature compilations.
     *
     * A Session holds shared, re-usable data elements that can be accessed through a 
     * FilterContext.
     *
     * Whereas a FilterContext exists thoughout the life of a single compilation, a Session
     * exists one level above this and governs any number of "related" compilations
     * (e.g., the compilation of many grid cells comprising a single feature layer).
     */
    class OSGEARTHFEATURES_EXPORT Session : public osg::Object
    {
    public:
        META_Object(osgEarthFeatures, Session);

        //! New session
        Session(const Map* map);
        
        //! New session
        Session(const Map* map, StyleSheet* styles);
        
        //! New session
        Session(const Map* map, StyleSheet* styles, FeatureSource* source, const osgDB::Options* dbOptions);

        virtual ~Session();

        /**
         * URI Context for relative path resolution.
         */
        void setURIContext( const URIContext& value ) { _uriContext = value; }
        const URIContext& uriContext() const { return _uriContext; }

        /** Underlying map. Check it for null before using! */
        osg::ref_ptr<const Map> getMap() const;

        /** Map information */
        const MapInfo& getMapInfo() const { return _mapInfo; }

        /** SRS of the map */
        const SpatialReference* getMapSRS() const;

        /** The style sheet governing this session. */
        void setStyles( StyleSheet* value );
        StyleSheet* styles() const { return _styles.get(); }

        /**
         * Set the feature source to draw upon in this Session. 
         * Do no call this after the Session is already in use.
         */
        void setFeatureSource(FeatureSource*);
        
        /** Gets the current feature source */
        FeatureSource* getFeatureSource() const;

        /** The I/O options for operations within this session */
        const osgDB::Options* getDBOptions() const;

        /** Shared resource cache (optional) */
        void setResourceCache(ResourceCache* cache);
        ResourceCache* getResourceCache();

        /** Optional name for this session */
        void setName(const std::string& name) { _name = name; }
        const std::string& getName() const { return _name; }

    public:
        /**
         * The cache for optimizing stateset sharing within a session
         */
        StateSetCache* getStateSetCache();

    public:
      ScriptEngine* getScriptEngine() const;

    private:
        void init();
        void initScriptEngine();

        URIContext                         _uriContext;         
        osg::observer_ptr<const Map>       _map;
        MapInfo                            _mapInfo;
        osg::ref_ptr<StyleSheet>           _styles;
        osg::ref_ptr<const osgDB::Options> _dbOptions;
        osg::ref_ptr<ScriptEngine>         _styleScriptEngine;
        osg::ref_ptr<FeatureSource>        _featureSource;
        osg::ref_ptr<StateSetCache>        _stateSetCache;
        osg::ref_ptr<ResourceCache>        _resourceCache;
        std::string                        _name;

        // hidden - support for META_Object
        Session();
        Session(const Session& rhs, const osg::CopyOp& op = osg::CopyOp::SHALLOW_COPY);
    };

} }

#endif // OSGEARTH_FEATURES_SESSION_H
