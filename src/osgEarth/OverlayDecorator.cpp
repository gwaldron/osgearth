/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/OverlayDecorator>
#include <osgEarth/DrapingTechnique>
#include <osgEarth/MapInfo>
#include <osgEarth/NodeUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>

#include <osg/AutoTransform>
#include <osg/ComputeBoundsVisitor>
#include <osg/ShapeDrawable>
#include <osgShadow/ConvexPolyhedron>
#include <osgUtil/LineSegmentIntersector>

#include <iomanip>
#include <stack>

#define LC "[OverlayDecorator] "

//#define OE_TEST if (_dumpRequested) OE_INFO << std::setprecision(9)
#define OE_TEST OE_NULL


using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    struct ComputeVisibleBounds : public OverlayDecorator::InternalNodeVisitor
    {
        ComputeVisibleBounds(osg::Polytope& tope, osg::Matrix& local2tope)
        {
            setTraversalMode( TRAVERSE_ACTIVE_CHILDREN );
            _matrixStack.push(local2tope);
            _tope = tope;
        }

        void apply(osg::Geode& node)
        {
            const osg::BoundingSphere& bs = node.getBound();
            osg::Vec3 p = bs.center() * _matrixStack.top();
            if ( _tope.contains(p) )
            {
                _bs.expandBy( osg::BoundingSphere(p, bs.radius()) );
            }
        }

        void apply(osg::Transform& xform)
        {
            osg::Matrix m( _matrixStack.top() );
            xform.computeLocalToWorldMatrix(m, this);
            _matrixStack.push( m );

            traverse(xform);

            _matrixStack.pop();
        }

        std::stack<osg::Matrix>   _matrixStack;
        osg::Polytope             _tope;
        osg::BoundingSphere       _bs;
    };

    void setFar(osg::Matrix& m, double newFar)
    {
        if ( osg::equivalent(m(0,3),0.0) && osg::equivalent(m(1,3),0.0) && osg::equivalent(m(2,3),0.0) )
        {
            double l,r,b,t,n,f;
            m.getOrtho(l,r,b,t,n,f);
            m.makeOrtho(l,r,b,t,n,newFar);
        }
        else
        {
            double v,a,n,f;
            m.getPerspective(v,a,n,f);
            m.makePerspective(v,a,n,newFar);
        }
    }

    void clampToNearFar(osg::Matrix& m, double newNear, double newFar)
    {
        if ( osg::equivalent(m(0,3),0.0) && osg::equivalent(m(1,3),0.0) && osg::equivalent(m(2,3),0.0) )
        {
            double l,r,b,t,n,f;
            m.getOrtho(l,r,b,t,n,f);
            m.makeOrtho(l,r,b,t, std::max(n, newNear), std::min(f,newFar));
        }
        else
        {
            double v,a,n,f;
            m.getPerspective(v,a,n,f);
            m.makePerspective(v,a, std::max(n, newNear), std::min(f, newFar));
        }
    }


    /**
     * Interects a finite ray with a sphere of radius R. The ray is defined
     * by the X and Y components (in projection aka "clip" space). The function
     * uses this information to build a ray from Z=-1 to Z=1. 
     *
     * Places the intersection point(s) in the output vector.
     */
    bool
    intersectClipRayWithSphere(double                   clipx, 
                               double                   clipy, 
                               const osg::Matrix&       clipToWorld, 
                               double                   R,
                               double&                  inout_maxDist2)
    {
        double dist2 = 0.0;

        osg::Vec3d p0 = osg::Vec3d(clipx, clipy, -1.0) * clipToWorld; // near plane
        osg::Vec3d p1 = osg::Vec3d(clipx, clipy,  1.0) * clipToWorld; // far plane

        // http://stackoverflow.com/questions/6533856/ray-sphere-intersection

        osg::Vec3d d = p1-p0;

        double A = d * d;
        double B = 2.0 * (d * p0);
        double C = (p0 * p0) - R*R;

        // now solve the quadratic A + B*t + C*t^2 = 0.
        double D = B*B - 4.0*A*C;
        if ( D >= 0 )
        {
            if ( osg::equivalent(D, 0.0) )
            {
                // one root (line is tangent to sphere)
                double t = -B/(2.0*A);
                if (t >= 0.0)
                {
                    osg::Vec3d v = d*t;
                    dist2 = v.length2();
                }
            }
            else
            {
                // two roots (line passes through sphere twice)
                // find the closer of the two.
                double sqrtD = sqrt(D);
                double t0 = (-B + sqrtD)/(2.0*A);
                double t1 = (-B - sqrtD)/(2.0*A);

                if ( t0 >= 0.0 && t1 >= 0.0 )
                {
                    osg::Vec3d v = d*std::min(t0,t1);
                    dist2 = v.length2();
                }
                else if ( t0 >= 0.0 )
                {
                    osg::Vec3d v = d*t0;
                    dist2 = v.length2();
                }
                else if ( t1 >= 0.0 )
                {
                    osg::Vec3d v = d*t1;
                    dist2 = v.length2();
                }
            }
        }

        if ( dist2 > inout_maxDist2 )
        {
            inout_maxDist2 = dist2;
            return true;
        }
        else
        {
            // either no intersection, or the distance was not the max.
            return false;
        }
    }

    /**
     * Same as above, but intersects the ray with a static 2D plane
     * (for use in projected map mode)
     */
    void
    intersectClipRayWithPlane(double                   clipx, 
                              double                   clipy, 
                              const osg::Matrix&       clipToWorld,
                              double&                  inout_maxDist2)
    {
        osg::Vec3d p0 = osg::Vec3d(clipx, clipy, -1.0) * clipToWorld; // near plane
        osg::Vec3d p1 = osg::Vec3d(clipx, clipy,  1.0) * clipToWorld; // far plane

        // zero-level plane is hard-coded here
        osg::Vec3d planePoint(0,0,0);
        osg::Vec3d planeNormal(0,0,1);

        osg::Vec3d L = p1-p0;
        L.normalize();

        double denom = L * planeNormal;
        if ( !osg::equivalent(denom, 0.0) )
        {
            double d = ((planePoint - p0) * planeNormal) / denom;
            if ( d > 0.0 )
                inout_maxDist2 = (L*d).length2();
        }
    }


    /**
     * Takes a set of world verts and finds their X-Y bounding box in the 
     * plane of the camera represented by the specified view matrix. Also
     * calculates the maximum distance from the eyepoint to a vertex (3D).
     */
    void
    getExtentInSilhouette(const osg::Matrix& viewMatrix,
                          const osg::Vec3d& eye,
                          std::vector<osg::Vec3d>& verts,
                          double& xmin, double& ymin,
                          double& xmax, double& ymax,
                          double& maxDistance)
    {
        xmin  = DBL_MAX, ymin = DBL_MAX, xmax = -DBL_MAX, ymax = -DBL_MAX;
        double maxDist2 = 0.0;

        for( std::vector<osg::Vec3d>::iterator i = verts.begin(); i != verts.end(); ++i )
        {
            osg::Vec3d d = (*i) * viewMatrix; // world to view
            if ( d.x() < xmin ) xmin = d.x();
            if ( d.x() > xmax ) xmax = d.x();
            if ( d.y() < ymin ) ymin = d.y();
            if ( d.y() > ymax ) ymax = d.y();
            
            double dist2 = ((*i)-eye).length2();
            if ( dist2 > maxDist2 )
                maxDist2 = dist2;
        }

        maxDistance = sqrt(maxDist2);
    }


    /**
     * 
     */
    void
    getNearFar(const osg::Matrix&      viewMatrix,
              std::vector<osg::Vec3d>& verts,
              double& znear,
              double& zfar)
    {
        znear = DBL_MAX, zfar = 0.0;

        for( std::vector<osg::Vec3d>::iterator i = verts.begin(); i != verts.end(); ++i )
        {
            osg::Vec3d d = (*i) * viewMatrix; // world to view
            if ( -d.z() < znear ) znear = -d.z();
            if ( -d.z() > zfar  ) zfar  = -d.z();
        }
    }
}

//---------------------------------------------------------------------------

OverlayDecorator::OverlayDecorator() :
_useShaders          ( true ),
_dumpRequested       ( false ),
_rttTraversalMask    ( ~0 ),
_maxHorizonDistance  ( DBL_MAX ),
_totalOverlayChildren( 0 )
{
    //nop.
}


void
OverlayDecorator::addTechnique(OverlayTechnique* technique)
{
    if ( _engine.valid() )
    {
        OE_WARN << LC <<
            "Illegal: you cannot install any more techniques once the Decorator "
            "has been installed by the terrain engine." << std::endl;

    }

    else if ( technique )
    {
        if ( technique->supported() )
        {
            _overlayGroups.push_back( new NotifierGroup<OverlayDecorator>(this) );
            _techniques.push_back( technique );
        }
        else
        {
            // stick unsupported techniques in a temporary holding cell
            // for reference management -- no harm
            _unsupportedTechniques.push_back( technique );
        }
    }
}


void
OverlayDecorator::onGroupChanged(osg::Group* group)
{
    // the group changed so we need to give the corresponding
    // technique a chance to re-establish itself based on the 
    // contents of that group.

    // update the total child count
    _totalOverlayChildren = 0;

    for( unsigned i=0; i<_techniques.size(); ++i )
    {
        //TODO: change to technique->getActive() or something
        _totalOverlayChildren += _overlayGroups[i]->getNumChildren();

        if ( _overlayGroups[i] == group )
        {
            _techniques[i]->reestablish( _engine.get() );
        }
    }
}


void
OverlayDecorator::initializePerViewData( PerViewData& pvd, osg::Camera* cam )
{
    pvd._camera = cam;
    pvd._sharedTerrainStateSet = new osg::StateSet();

    pvd._techParams.resize( _overlayGroups.size() );

    for(unsigned i=0; i<_overlayGroups.size(); ++i )
    {
        TechRTTParams& params = pvd._techParams[i];
        params._group = _overlayGroups[i].get();
        params._terrainStateSet = pvd._sharedTerrainStateSet.get(); // share it.
        params._horizonDistance = &pvd._sharedHorizonDistance;      // share it.
        params._terrainParent = this;
        params._mainCamera = cam;
    }
}


void
OverlayDecorator::setOverlayGraphTraversalMask( unsigned mask )
{
    _rttTraversalMask = mask;
}


void
OverlayDecorator::onInstall( TerrainEngineNode* engine )
{
    _engine = engine;

    // establish the earth's major axis:
    MapInfo info(engine->getMap());
    _isGeocentric = info.isGeocentric();
    _srs = info.getProfile()->getSRS();
    _ellipsoid = info.getProfile()->getSRS()->getEllipsoid();

    for(Techniques::iterator t = _techniques.begin(); t != _techniques.end(); ++t )
    {
        t->get()->onInstall( engine );
    }
}


void
OverlayDecorator::onUninstall( TerrainEngineNode* engine )
{
    for(Techniques::iterator t = _techniques.begin(); t != _techniques.end(); ++t )
    {
        t->get()->onUninstall( engine );
    }

    _engine = 0L;
}


void
OverlayDecorator::cullTerrainAndCalculateRTTParams(osgUtil::CullVisitor* cv,
                                                   PerViewData&          pvd)
{
    static int s_frame = 1;

    osg::Vec3d eye = cv->getViewPoint();

    double eyeLen;
    osg::Vec3d worldUp;

    // Radius at eyepoint (geocentric)
    double R;

    // height above sea level
    double hasl;

    // weight of the HASL value when calculating extent compensation
    double haslWeight;

    // approximate distance to the visible horizon
    double horizonDistance;

    OE_TEST << LC << "------- OD CULL ------------------------" << std::endl;

    if ( _isGeocentric )
    {
        eyeLen = eye.length();

        const SpatialReference* geoSRS = _engine->getTerrain()->getSRS();
        osg::Vec3d geodetic;
        geoSRS->transformFromWorld(eye, geodetic);

        hasl = geodetic.z();
        R = eyeLen - hasl;
        
        //Actually sample the terrain to get the height and adjust the eye position so it's a tighter fit to the real data.
        double height;
        if (_engine->getTerrain()->getHeight(geoSRS, geodetic.x(), geodetic.y(), &height)) // SpatialReference::create("epsg:4326"), osg::RadiansToDegrees( lon ), osg::RadiansToDegrees( lat ), &height))
        {
            geodetic.z() -= height;
        }
        hasl = osg::maximum( hasl, 100.0 );

        // up vector tangent to the ellipsoid under the eye.
        worldUp = _ellipsoid->computeLocalUpVector(eye.x(), eye.y(), eye.z());

        // radius of the earth under the eyepoint
        // gw: wrong. use R instead.
        double radius = eyeLen - hasl; 
        horizonDistance = sqrt( 2.0*radius*hasl + hasl*hasl );
    }
    else // projected map
    {
        hasl = eye.z();
        hasl = osg::maximum( hasl, 100.0 );
        worldUp.set( 0.0, 0.0, 1.0 );
        eyeLen = hasl * 2.0;

        // there "horizon distance" in a projected map is infinity,
        // so just simulate one.
        horizonDistance = sqrt(2.0*6356752.3142*hasl + hasl*hasl);
    }
    
    // update the shared horizon distance.
    pvd._sharedHorizonDistance = horizonDistance;

    // create a "weighting" that weights HASL against the camera's pitch.
    osg::Vec3d lookVector = cv->getLookVectorLocal();
    haslWeight = osg::absolute(worldUp * lookVector);

    // unit look-vector of the eye:
    osg::Vec3d camEye, camTo, camUp;
    const osg::Matrix& mvMatrix = *cv->getModelViewMatrix();
    mvMatrix.getLookAt( camEye, camTo, camUp, 1.0); //eyeLen);
    osg::Vec3 camLook = camTo-camEye;
    camLook.normalize();

    // Save and reset the current near/far planes before traversing the subgraph.
    // We do this because we want a projection matrix that includes ONLY the clip
    // planes from the subgraph, and not anything traversed up to this point.
    double zSavedNear = cv->getCalculatedNearPlane();
    double zSavedFar  = cv->getCalculatedFarPlane();

    cv->setCalculatedNearPlane( FLT_MAX );
    cv->setCalculatedFarPlane( -FLT_MAX );

    // cull the subgraph (i.e. the terrain) here. This doubles as the subgraph's official 
    // cull traversal and a gathering of its clip planes.
    cv->pushStateSet( pvd._sharedTerrainStateSet.get() );
    osg::Group::traverse( *cv );
    cv->popStateSet();

    // Pull a copy of the projection matrix; we will use this to calculate the optimum
    // projected texture extent
    osg::Matrixd projMatrix = *cv->getProjectionMatrix();

    // Clamp the projection matrix to the newly calculated clip planes. This prevents
    // any "leakage" from outside the subraph.
    double zNear = cv->getCalculatedNearPlane();
    double zFar  = cv->getCalculatedFarPlane();
    cv->clampProjectionMatrix( projMatrix, zNear, zFar );

    OE_TEST << LC << "Subgraph clamp: zNear = " << zNear << ", zFar = " << zFar << std::endl;

    // restore the clip planes in the cull visitor, now that we have our subgraph
    // projection matrix.
    cv->setCalculatedNearPlane( osg::minimum(zSavedNear, zNear) );
    cv->setCalculatedFarPlane( osg::maximum(zSavedFar, zFar) );

    // clamp the far plane (for RTT purposes) to the horizon distance.
    double maxFar = std::min( horizonDistance, _maxHorizonDistance );
    cv->clampProjectionMatrix( projMatrix, zNear, maxFar );

    // prepare to calculate the ideal far plane for RTT extent resolution.
    osg::Matrixd MVP = *cv->getModelViewMatrix() * projMatrix;
    osg::Matrixd inverseMVP;
    inverseMVP.invert(MVP);

    double maxDist2 = 0.0;

    // constrain the far plane.
    // intersect the top corners of the projection volume since those are the farthest.
    if ( _isGeocentric )
    {
        intersectClipRayWithSphere( -1.0, 1.0, inverseMVP, R, maxDist2 );
        intersectClipRayWithSphere(  1.0, 1.0, inverseMVP, R, maxDist2 );
    }
    else // projected
    {
        intersectClipRayWithPlane( -1.0, 1.0, inverseMVP, maxDist2 );
        if ( maxDist2 == 0.0 )
            intersectClipRayWithPlane( 1.0, 1.0, inverseMVP, maxDist2 );
        if ( maxDist2 == 0.0 )
            intersectClipRayWithPlane( 0.0, 1.0, inverseMVP, maxDist2 );
    }

    // clamp down the far plane:
    if ( maxDist2 != 0.0 )
    {
        maxFar = std::min( zNear+sqrt(maxDist2), maxFar );
    }

    // reset the projection matrix if we changed the far:
    if ( maxFar != zFar )
    {
        setFar( projMatrix, maxFar );
        MVP = *cv->getModelViewMatrix() * projMatrix;
        inverseMVP.invert(MVP);
    }

    // calculate the new RTT matrices. All techniques will share the 
    // same set. We could probably put these in the "shared" category
    // and use pointers..todo.
    osg::Matrix rttViewMatrix, rttProjMatrix;

    // for a camera that cares about geometry (like the draping technique) it's important
    // to include the geometry in the ortho-camera's Z range. But for a camera that just
    // cares about the terrain depth (like the clamping technique) we want to constrain 
    // the Ortho Z as mush as possible in order to maintain depth precision. Perhaps
    // later we can split this out and have each technique calculation its own View and
    // Proj matrix.

    // For now: our RTT camera z range will be based on this equation:
    double zspan = std::max(50000.0, hasl+25000.0);
    osg::Vec3d up = camLook;
    if ( _isGeocentric )
    {
        osg::Vec3d rttEye = eye+worldUp*zspan;
        //establish a valid up vector
        osg::Vec3d rttLook = -rttEye;
        rttLook.normalize();
        if ( fabs(rttLook * camLook) > 0.9999 )
            up.set( camUp );

        // do NOT look at (0,0,0); must look down the ellipsoid up vector.
        rttViewMatrix.makeLookAt( rttEye, rttEye-worldUp*zspan, up );
    }
    else
    {
        osg::Vec3d rttLook(0, 0, -1);
        if ( fabs(rttLook * camLook) > 0.9999 )
            up.set( camUp );

        rttViewMatrix.makeLookAt( camEye + worldUp*zspan, camEye - worldUp*zspan, up );
    }

    // Build a polyhedron for the new frustum so we can slice it.
    // TODO: do we really even need to slice it anymore? consider
    osgShadow::ConvexPolyhedron frustumPH;
    frustumPH.setToUnitFrustum(true, true);
    frustumPH.transform( inverseMVP, MVP );

    // now copy the RTT matrixes over to the techniques.
    for( unsigned t=0; t<pvd._techParams.size(); ++t )
    {
        TechRTTParams& params = pvd._techParams[t];

        // skip empty techniques
        if ( !_techniques[t]->hasData(params) )
            continue;

        // slice it to fit the overlay geometry. (this says 'visible' but it's just everything..
        // perhaps we can truly make it visible)
        osgShadow::ConvexPolyhedron visiblePH( frustumPH );

#if 0
        osg::Polytope frustumPT;
        frustumPH.getPolytope(frustumPT);
        ComputeVisibleBounds cvb(frustumPT, MVP);
        params._group->accept(cvb);
        const osg::BoundingSphere& visibleOverlayBS = cvb._bs;
        OE_WARN << "VBS radius = " << visibleOverlayBS.radius() << std::endl;
#else
        const osg::BoundingSphere& visibleOverlayBS = params._group->getBound();
#endif
        if ( visibleOverlayBS.valid() )
        {
            // form an axis-aligned polytope around the bounding sphere of the
            // overlay geometry. Use that to cut the camera frustum polytope.
            // This will minimize the coverage area and also ensure inclusion
            // of geometry that falls outside the camera frustum but inside
            // the overlay area.
            osg::Polytope visibleOverlayPT;

            osg::Vec3d tangent(0,0,1);
            if (fabs(worldUp*tangent) > 0.9999)
                tangent.set(0,1,0);

            osg::Vec3d westVec  = worldUp^tangent; westVec.normalize();
            osg::Vec3d southVec = worldUp^westVec; southVec.normalize();
            osg::Vec3d eastVec  = -westVec;
            osg::Vec3d northVec = -southVec;

            osg::Vec3d westPt  = visibleOverlayBS.center() + westVec*visibleOverlayBS.radius();
            osg::Vec3d eastPt  = visibleOverlayBS.center() + eastVec*visibleOverlayBS.radius();
            osg::Vec3d northPt = visibleOverlayBS.center() + northVec*visibleOverlayBS.radius();
            osg::Vec3d southPt = visibleOverlayBS.center() + southVec*visibleOverlayBS.radius();

            visibleOverlayPT.add(osg::Plane(-westVec,  westPt));
            visibleOverlayPT.add(osg::Plane(-eastVec,  eastPt));
            visibleOverlayPT.add(osg::Plane(-southVec, southPt));
            visibleOverlayPT.add(osg::Plane(-northVec, northPt));

            visiblePH.cut( visibleOverlayPT );
        }

        // for dumping, we want the previous fram's projection matrix
        // becasue the technique itself may have modified it.
        osg::Matrix prevProjMatrix = params._rttProjMatrix;

        // extract the verts associated with the frustum's PH:
        std::vector<osg::Vec3d> verts;
        visiblePH.getPoints( verts );

        // zero verts means the visible PH does not intersect the frustum.
        // TODO: when verts = 0 should we do something different? or use the previous
        // frame's view matrix?
        if ( verts.size() > 0 )
        {
            // calculate an orthographic RTT projection matrix based on the view-space
            // bounds of the vertex list (i.e. the extents surrounding the RTT camera 
            // that bounds all the polyherdron verts in its XY plane)
            double xmin, ymin, xmax, ymax, maxDist;
            getExtentInSilhouette(rttViewMatrix, eye, verts, xmin, ymin, xmax, ymax, maxDist);

            // make sure the ortho camera penetrates the terrain. This is a must for depth buffer sampling
            double dist = std::max(hasl*1.5, std::min(maxDist, eyeLen));

            // in ecef it can't go past the horizon though, or you get bleed thru
            if ( _isGeocentric )
                dist = std::min(dist, eyeLen);

            // Even through using xmin and xmax directly results in a tighter fit, 
            // it offsets the eyepoint from the center of the projection frustum.
            // This causes problems for the draping projection matrix optimizer, so
            // for now instead of re-doing that code we will just center the eyepoint
            // here by using the larger of xmin and xmax. -gw.
#if 1
            double x = std::max( fabs(xmin), fabs(xmax) );
            rttProjMatrix.makeOrtho(-x, x, ymin, ymax, 0.0, dist+zspan);
#else
            //Note: this was the original setup, which is technically optimal:
            rttProjMatrix.makeOrtho(xmin, xmax, ymin, ymax, 0.0, dist+zspan);
#endif

            // Clamp the view frustum's N/F to the visible geometry. This clamped
            // frustum is the one we'll send to the technique.
            double visNear, visFar;
            getNearFar( *cv->getModelViewMatrix(), verts, visNear, visFar );
            osg::Matrix clampedProjMat( projMatrix );
            clampToNearFar( clampedProjMat, visNear, visFar );
            osg::Matrix clampedMVP = *cv->getModelViewMatrix() * clampedProjMat;
            osg::Matrix inverseClampedMVP;
            inverseClampedMVP.invert(clampedMVP);
            osgShadow::ConvexPolyhedron clampedFrustumPH;
            clampedFrustumPH.setToUnitFrustum(true, true);
            clampedFrustumPH.transform( inverseClampedMVP, clampedMVP );

            // assign the matrices to the technique.
            params._rttViewMatrix.set( rttViewMatrix );
            params._rttProjMatrix.set( rttProjMatrix );
            params._eyeWorld = eye;
            params._visibleFrustumPH = clampedFrustumPH; //frustumPH;
        }

        // service a "dump" of the polyhedrons for dubugging purposes
        // (see osgearth_overlayviewer)
        if ( _dumpRequested )
        {
            static const char* fn = "convexpolyhedron.osg";

            // camera frustum:
            {
                frustumPH.dumpGeometry(0,0,0,fn);
            }
            osg::Node* camNode = osgDB::readNodeFile(fn);
            camNode->setName("camera");

            // visible overlay BEFORE cutting:
            //uncutVisiblePH.dumpGeometry(0,0,0,fn,osg::Vec4(0,1,1,1),osg::Vec4(0,1,1,.25));
            //osg::Node* overlay = osgDB::readNodeFile(fn);
            //overlay->setName("overlay");

            // visible overlay Polyherdron AFTER cuting:
            visiblePH.dumpGeometry(0,0,0,fn,osg::Vec4(1,.5,1,1),osg::Vec4(1,.5,0,.25));
            osg::Node* intersection = osgDB::readNodeFile(fn);
            intersection->setName("intersection");

            // RTT frustum:
            {
                osgShadow::ConvexPolyhedron rttPH;
                rttPH.setToUnitFrustum( true, true );
                osg::Matrixd MVP = params._rttViewMatrix * prevProjMatrix; //params._rttProjMatrix;
                osg::Matrixd inverseMVP;
                inverseMVP.invert(MVP);
                rttPH.transform( inverseMVP, MVP );
                rttPH.dumpGeometry(0,0,0,fn,osg::Vec4(1,1,0,1),osg::Vec4(1,1,0,0.25));
            }
            osg::Node* rttNode = osgDB::readNodeFile(fn);
            rttNode->setName("rtt");

            // EyePoint
            osg::Geode* dsg = new osg::Geode();
            dsg->addDrawable( new osg::ShapeDrawable(new osg::Box(osg::Vec3f(0,0,0), 10.0f)));
            osg::AutoTransform* dsgmt = new osg::AutoTransform();
            dsgmt->setPosition( osg::Vec3d(0,0,0) * osg::Matrix::inverse(*cv->getModelViewMatrix()) );
            dsgmt->setAutoScaleToScreen(true);
            dsgmt->addChild( dsg );

            osg::Group* g = new osg::Group();
            g->getOrCreateStateSet()->setAttribute(new osg::Program(), 0);
            g->addChild(camNode);
            //g->addChild(overlay);
            g->addChild(intersection);
            g->addChild(rttNode);
            g->addChild(dsgmt);

            _dump = g;
            _dumpRequested = false;
        }
    }
}

OverlayDecorator::PerViewData&
OverlayDecorator::getPerViewData(osg::Camera* key)
{
    // first check for it:
    {
        Threading::ScopedReadLock shared( _perViewDataMutex );
        PerViewDataMap::iterator i = _perViewData.find(key);
        if ( i != _perViewData.end() )
        {
            if ( !i->second._sharedTerrainStateSet.valid() )
            {
                initializePerViewData( i->second, key );
            }
            return i->second;
        }
    }

    // then exclusive lock and make/check it:
    {
        Threading::ScopedWriteLock exclusive( _perViewDataMutex );

        // double check pattern:
        PerViewDataMap::iterator i = _perViewData.find(key);
        if ( i != _perViewData.end() )
            return i->second;

        PerViewData& pvd = _perViewData[key];
        initializePerViewData(pvd, key);

        return pvd;
    }    
}


void
OverlayDecorator::traverse( osg::NodeVisitor& nv )
{
    bool defaultTraversal = true;

    // in the CULL traversal, find the per-view data associated with the 
    // cull visitor's current camera view and work with that:
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        osg::Camera* camera = cv->getCurrentCamera();

        if ( camera != 0L && (_rttTraversalMask & nv.getTraversalMask()) != 0 )
        {
            // access per-camera data to support multi-threading:
            PerViewData& pvd = getPerViewData( camera );

            // technique-specific setup prior to traversing:
            bool hasOverlayData = false;
            for(unsigned i=0; i<_techniques.size(); ++i)
            {
                if ( _techniques[i]->hasData(pvd._techParams[i]) )
                {
                    hasOverlayData = true;
                    _techniques[i]->preCullTerrain( pvd._techParams[i], cv );
                }
            }

            if ( hasOverlayData )
            {
                defaultTraversal = false;

                // shared terrain culling pass:
                cullTerrainAndCalculateRTTParams( cv, pvd );

                // prep and traverse the RTT camera(s):
                for(unsigned i=0; i<_techniques.size(); ++i)
                {
                    TechRTTParams& params = pvd._techParams[i];
                    _techniques[i]->cullOverlayGroup( params, cv );
                }
            }
        }
    }

    else
    {
        // Some other type of visitor (like update or intersection). Skip the technique
        // and traverse the geometry directly.
        for(unsigned i=0; i<_overlayGroups.size(); ++i)
        {
            _overlayGroups[i]->accept( nv );
        }
    }

    if ( defaultTraversal )
    {
        osg::Group::traverse( nv );
    }
}


double
OverlayDecorator::getMaxHorizonDistance() const
{
    return _maxHorizonDistance;
}

void
OverlayDecorator::setMaxHorizonDistance( double horizonDistance )
{
    _maxHorizonDistance = horizonDistance;
}
