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
#ifndef OSGEARTHFEATURES_FILTER_CONTEXT_H
#define OSGEARTHFEATURES_FILTER_CONTEXT_H 1

#include <osgEarthFeatures/Common>
#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/Session>

#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/ResourceCache>

#include <osgEarth/GeoData>
#include <osgEarth/ShaderUtils>

#include <osg/Matrix>
#include <list>
#include <vector>

namespace osgEarth { namespace Features
{
    using namespace osgEarth;
    using namespace osgEarth::Symbology;

    //class Session;
    class FeatureProfile;
    class FeatureIndexBuilder;

    /**
     * Context within which a chain of filters is executed.
     */
    class OSGEARTHFEATURES_EXPORT FilterContext
    {
    public:
        FilterContext();

        FilterContext(
            Session*              session,
            const FeatureProfile* profile       =0L,
            const GeoExtent&      workingExtent =GeoExtent::INVALID,
            FeatureIndexBuilder*  index         =0L);

        FilterContext( const FilterContext& rhs );

        ~FilterContext();
        
        /**
         * Assigns a resource cache to use. One is created automatically if you
         * don't assign one.
         */
        void setResourceCache( ResourceCache* value ) { _resourceCache = value; }

        /**
         * Sets the profile (SRS etc) of the feature data
         */
        void setProfile( const FeatureProfile* profile );

        /**
         * Sets the output SRS for feature processing. This is optional.
         * If you do not set this, the output SRS will be that of the
         * session's Map.
         */
        void setOutputSRS(const SpatialReference* srs) { _outputSRS = srs; }
        const SpatialReference* getOutputSRS() const;


    public: // properties

        /**
         * Whether this context contains complete georeferencing information.
         */
        bool isGeoreferenced() const;

        /**
         * Access to the Session under which this filter context operates
         */
        Session* getSession();
        const Session* getSession() const;

        /**
         * The spatial profile of the feature data in this context.
         */
        const FeatureProfile* profile() const { return _profile.get(); }

        /**
         * The spatial extent of the working cell
         */
        optional<GeoExtent>& extent() { return _extent; }
        const optional<GeoExtent>& extent() const { return _extent; }

        /**
         * The feature index
         */
        FeatureIndexBuilder* featureIndex() { return _index; }
        const FeatureIndexBuilder* featureIndex() const { return _index; }

        /**
         * Whether this context has a non-identity reference frame
         */
        bool hasReferenceFrame() const { return !_referenceFrame.isIdentity(); }

        /**
         * Converts from world coordiantes into local coordinates 
         */
        osg::Vec3d toLocal( const osg::Vec3d& world ) const { return world * _referenceFrame; }

        /**
         * Converts a Geometry from world coords to local coords
         */
        void toLocal( Geometry* geom ) const;

        /**
         * Converts from local (reference frame) coordinates into world coordinates
         */
        osg::Vec3d toWorld( const osg::Vec3d& local ) const { return local * _inverseReferenceFrame; }

        /**
         * Converts a Geometry from local coords to world coords
         */
        void toWorld( Geometry* geom ) const;

        /**
         * Converts a context-local point to a map coordinate.
         */
        osg::Vec3d toMap( const osg::Vec3d& local ) const;

        /**
         * Converts a map coordinate to a context-local point.
         */
        osg::Vec3d fromMap( const osg::Vec3d& map ) const;

        /**
         * Multiplying by the reference frame takes a point from world coords into local coords.
         */
        const osg::Matrixd& referenceFrame() const { return _referenceFrame; }

        /**
         * Multiplying by the inverse reference frame takes a point from local coords into world coords.
         */
        const osg::Matrix& inverseReferenceFrame() const { return _inverseReferenceFrame; }

        /**
         * Sets the reference frame (and its inverse)
         */
        void setReferenceFrame( const osg::Matrixd& in ) {
            _referenceFrame = in;
            _inverseReferenceFrame.invert( _referenceFrame ); // = osg::Matrixd::inverse( _referenceFrame );
        }      

        /**
         * Accesses the shared resource cache where filters can store data.
         */
        ResourceCache* resourceCache();

        /**
         * Shader policy. Unset by default, but code using this context can expressly
         * set it to affect shader generation. Typical use case it to set the policy
         * to "inherit" to inhibit shader generation if you have already generated
         * shaders yourself.
         */
        optional<ShaderPolicy>& shaderPolicy() { return _shaderPolicy; }
        const optional<ShaderPolicy>& shaderPolicy() const { return _shaderPolicy; }

        /**
         * Dump as a string
         */
        std::string toString() const;

        /**
         * Gets the DB Options associated with the context's session
         */
        const osgDB::Options* getDBOptions() const;

        /**
         * Gets a "history" string for debugging purposes
         */
        std::string getHistory() const;

        void pushHistory(const std::string& value) { _history.push_back(value); }

    protected:

        osg::ref_ptr<Session>              _session;
        osg::ref_ptr<const FeatureProfile> _profile;
        bool                               _isGeocentric;
        optional<GeoExtent>                _extent;
        osg::Matrixd                       _referenceFrame;
        osg::Matrixd                       _inverseReferenceFrame;
        osg::ref_ptr<ResourceCache>        _resourceCache;
        FeatureIndexBuilder*               _index;
        optional<ShaderPolicy>             _shaderPolicy;
        std::vector<std::string>           _history;
        osg::ref_ptr<const SpatialReference> _outputSRS;
    };

} } // namespace osgEarth::Features

#endif // OSGEARTHFEATURES_FILTER_CONTEXT_H

