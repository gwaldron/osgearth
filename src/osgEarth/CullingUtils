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

#ifndef OSGEARTH_CULLING_UTILS_H
#define OSGEARTH_CULLING_UTILS_H 1

#include <osgEarth/Common>
#include <osgEarth/optional>
#include <osgEarth/SpatialReference>
#include <osgEarth/Horizon>
#include <osgEarth/GeoTransform>

#include <osg/NodeCallback>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osg/MatrixTransform>
#include <osg/Vec3d>
#include <osg/Vec3>
#include <osg/ClipPlane>
#include <osgUtil/CullVisitor>

namespace osgEarth
{    
    /**
     * A customized CCC that works correctly under an RTT camera and also with 
     * an orthographic projection matrix.
     */
    class SuperClusterCullingCallback : public osg::ClusterCullingCallback
    {
    public:
        bool cull(osg::NodeVisitor* nv, osg::Drawable* , osg::State*) const;
    };


    /**
     * Utility functions for creating cluster cullers
     */
    class OSGEARTH_EXPORT ClusterCullingFactory
    {
    public:
        /**
         * Creates a cluster culling callback based on the data in a node graph.
         * NOTE! Never put a CCC somewhere where it will be under a transform. They
         * only work in absolute world space.
         */
        static osg::NodeCallback* create( osg::Node* node, const osg::Vec3d& ecefControlPoint );

        /**
         * Same as above, but uses another method to compute the parameters. There should only
         * be one, but we need to investigate to see which is the better algorithm. Keeping this
         * for now since it works with the feature setup..
         */
        static osg::NodeCallback* create2( osg::Node* node, const osg::Vec3d& ecefControlPoint );

        /**
         * Creates a cluster culling callback and installs it on the node. If the node is
         * a transform, it will create a group above the transform and install the callback
         * on that group instead.
         */
        static osg::Node* createAndInstall( osg::Node* node, const osg::Vec3d& ecefControlPoint );

        /**
         * Creates a cluster culling callback with the standard parameters.
         */
        static osg::NodeCallback* create(const osg::Vec3& controlPoint, const osg::Vec3& normal, float deviation, float radius);        

        /**
         * Creates a cluster culling callback based on an extent (for geocentric tiles).
         */
        static osg::NodeCallback* create(const class GeoExtent& extent);
    };

    /**
     * Simple occlusion culling callback that does a ray interseciton between the eyepoint
     * and a A GeoTransform node.
     */
    struct OSGEARTH_EXPORT OcclusionCullingCallback : public osg::NodeCallback
    {
        OcclusionCullingCallback(GeoTransform* xform);

        /** Maximum eye altitude at which to perform the occlusion culling test */
        double getMaxAltitude() const;
        void setMaxAltitude( double value);

        /**
        * Gets the maximum number of ms that the OcclusionCullingCallback can run on each frame.
        */
        static double getMaxFrameTime();

        /**
        * Sets the maximum number of ms that the OcclusionCullingCallback can run on each frame.
        * @param ms
        *     The maximum number of milliseconds to run the OcclusionCullingCallback on each frame. 
        */
        static void setMaxFrameTime( double ms );

    public: //NodeCallback
        
        void operator()(osg::Node* node, osg::NodeVisitor* nv);

    private:

        osg::observer_ptr<GeoTransform> _xform;
        osg::Vec3d _prevEye;
        bool _visible;
        double _maxAltitude;
        static double _maxFrameTime;
    };

    /**
     * Culling utilities.
     */
    struct OSGEARTH_EXPORT Culling
    {
        static osgUtil::CullVisitor* asCullVisitor(osg::NodeVisitor* nv);
        static osgUtil::CullVisitor* asCullVisitor(osg::NodeVisitor& nv) { return asCullVisitor(&nv); }
    };

    /**
     * Node Visitor that proxies the CullVisitor but uses a separate
     * frustum for bounds-culling.
     */
    class ProxyCullVisitor : public osg::NodeVisitor, public osg::CullStack
    {
    private:
        osgUtil::CullVisitor* _cv;
        osg::Polytope         _proxyFrustum;
        osg::Polytope         _proxyProjFrustum;
        osg::Matrix           _proxyModelViewMatrix;
        osg::Matrix           _proxyProjMatrix;

    public:
        ProxyCullVisitor( osgUtil::CullVisitor* cv, const osg::Matrix& proj, const osg::Matrix& view );

        // access to the underlying cull visitor.
        osgUtil::CullVisitor* getCullVisitor() { return _cv; }

        bool isCulledByProxyFrustum(osg::Node& node);
        bool isCulledByProxyFrustum(const osg::BoundingBox& bbox);

    public: // proxy functions:
        osg::Vec3 getEyePoint() const;
        osg::Vec3 getViewPoint() const;
        float getDistanceToEyePoint(const osg::Vec3& pos, bool useLODScale) const;
        float getDistanceFromEyePoint(const osg::Vec3& pos, bool useLODScale) const;
        float getDistanceToViewPoint(const osg::Vec3& pos, bool useLODScale) const;

    protected:
        
        osgUtil::CullVisitor::value_type distance(const osg::Vec3& coord,const osg::Matrix& matrix);

        void handle_cull_callbacks_and_traverse(osg::Node& node);

        void apply(osg::Node& node);
        void apply(osg::Transform& xform);
        void apply(osg::Drawable& drawable);
        void apply(osg::LOD& lod);
    };


    /**
     * Horizon culling in a shader program.
     */
    class OSGEARTH_EXPORT HorizonCullingProgram
    {
    public:
        static void install( osg::StateSet* stateset );
        static void remove ( osg::StateSet* stateset );
    };


    /**
     * Group that lets you adjust the LOD scale on its children.
     */
    class OSGEARTH_EXPORT LODScaleGroup : public osg::Group
    {
    public:
        LODScaleGroup();

        /**
         * Factor by which to multiply the camera's LOD scale.
         */
        void setLODScaleFactor( float value ) { _scaleFactor = value; }
        float getLODScaleFactor() const { return _scaleFactor; }

    public: // osg::Group
        virtual void traverse(osg::NodeVisitor& nv);

    protected:
        virtual ~LODScaleGroup() { }

    private:
        float _scaleFactor;
    };


    
    /**
     * Cull callback that sets a clip plane at the visible horizon
     * of the Ellipsoid.
     */
    class OSGEARTH_EXPORT ClipToGeocentricHorizon : public osg::NodeCallback
    {
    public:
        ClipToGeocentricHorizon(
            const osgEarth::SpatialReference* srs,
            osg::ClipPlane*                   clipPlane);
        
        void operator()(osg::Node* node, osg::NodeVisitor* nv);

    protected:
        osg::ref_ptr<Horizon>             _horizon;
        osg::observer_ptr<osg::ClipPlane> _clipPlane;
    };


    struct OSGEARTH_EXPORT CullDebugger
    {
        Config dumpRenderBin(osgUtil::RenderBin* bin) const;
    };


    /**
     * Cull callback for cameras that sets the oe_ViewportSize uniform.
     */
    class OSGEARTH_EXPORT InstallViewportSizeUniform : public osg::NodeCallback
    {
    public:
        META_Object(osgEarth, InstallViewportSizeUniform);
        
        void operator()(osg::Node* node, osg::NodeVisitor* nv);

        InstallViewportSizeUniform() { }

        InstallViewportSizeUniform(const InstallViewportSizeUniform& rhs, const osg::CopyOp& copy) :
            osg::NodeCallback(rhs, copy) { }
    };

    /**
     * Cull callback that limits visibility to an altitude range (HAE)
     */
    class OSGEARTH_EXPORT AltitudeCullCallback : public osg::NodeCallback
    {
    public:
        optional<float>& maxAltitude() { return _maxAltitude; }
        osg::ref_ptr<const SpatialReference>& srs() { return _srs; }
        
        void operator()(osg::Node* node, osg::NodeVisitor* nv);

    private:
        optional<float> _maxAltitude;
        osg::ref_ptr<const SpatialReference> _srs;
    };
}

#endif // OSGEARTH_CULLING_UTILS_H
