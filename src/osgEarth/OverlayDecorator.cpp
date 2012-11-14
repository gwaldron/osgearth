/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

//#define GROUP_COUNT    1 // change to 2 later.
//#define GROUP_DRAPING  0
//#define GROUP_CLAMPING 1

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    /**
     * This method takes a set of verts and finds the nearest and farthest distances from
     * the points to the camera. It does this calculation in the plane defined by the
     * look vector.
     *
     * IOW, all the test points are "projected" on to the plane defined by the camera point
     * and the look (normal) vector, and then the distances from the camera point to each
     * projected point are tested in order to find the min/max extent.
     */
    void
    getMinMaxExtentInSilhouette(const osg::Vec3d& cam, const osg::Vec3d& look, 
                                std::vector<osg::Vec3d>& verts,
                                double& out_eMin, double& out_eMax )
    {
        double minSqrDist2D = DBL_MAX;
        double maxSqrDist2D = -DBL_MAX;
        osg::Plane plane( look, cam );

        for( std::vector<osg::Vec3d>::iterator i = verts.begin(); i != verts.end(); ++i )
        {
            osg::Vec3d& point = *i;

            // project the vert onto the camera plane:
            double signedDist = plane.distance( point );
            point += (-plane.getNormal() * signedDist);
            
            // then calculate the 2D distance to the camera:
            double sqrDist2D = (cam-point).length2();
            if ( sqrDist2D > maxSqrDist2D )
                maxSqrDist2D = sqrDist2D;
            if ( sqrDist2D < minSqrDist2D )
                minSqrDist2D = sqrDist2D;
        }

        out_eMin = sqrt( minSqrDist2D );
        out_eMax = sqrt( maxSqrDist2D );
    }
    
    /**
     * Same as the method above, but extracts the verts from a bounding box.
     */
    void
    getMinMaxExtentInSilhouette(const osg::Vec3d& cam, const osg::Vec3d& look, 
                                const osg::BoundingBox& bbox,
                                double& out_eMin, double& out_eMax )
    {
        std::vector<osg::Vec3d> verts(8);
        verts[0].set( bbox.xMin(), bbox.yMin(), bbox.zMin() );
        verts[1].set( bbox.xMin(), bbox.yMin(), bbox.zMax() );
        verts[2].set( bbox.xMin(), bbox.yMax(), bbox.zMin() );
        verts[3].set( bbox.xMin(), bbox.yMax(), bbox.zMax() );
        verts[4].set( bbox.xMax(), bbox.yMin(), bbox.zMin() );
        verts[5].set( bbox.xMax(), bbox.yMin(), bbox.zMax() );
        verts[6].set( bbox.xMax(), bbox.yMax(), bbox.zMin() );
        verts[7].set( bbox.xMax(), bbox.yMax(), bbox.zMax() );
        getMinMaxExtentInSilhouette( cam, look, verts, out_eMin, out_eMax );
    }
}

//---------------------------------------------------------------------------

OverlayDecorator::OverlayDecorator() :
_useShaders          ( false ),
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
        params._horizonDistance = _isGeocentric ? &pvd._sharedHorizonDistance : 0L; // share it.
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

    // the maximum extent (for projected maps only)
    if ( !_isGeocentric )
    {
        const GeoExtent& extent = info.getProfile()->getExtent();
        _maxProjectedMapExtent = osg::maximum( extent.width(), extent.height() );
    }

    //todo: need this? ... probably not anymore
    _useShaders = 
        Registry::capabilities().supportsGLSL() && 
        engine->getTextureCompositor()->usesShaderComposition();

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

    osg::Vec3 eye = cv->getEyePoint();

    double eyeLen;
    osg::Vec3d worldUp;

    // height above sea level
    double hasl;

    // weight of the HASL value when calculating extent compensation
    double haslWeight;

    // approximate distance to the visible horizon
    double horizonDistance; 

    // distance to the horizon, projected into the RTT camera's tangent plane.
    double horizonDistanceInRTTPlane;

    OE_TEST << LC << "------- OD CULL ------------------------" << std::endl;

    if ( _isGeocentric )
    {
        double lat, lon;
        _ellipsoid->convertXYZToLatLongHeight( eye.x(), eye.y(), eye.z(), lat, lon, hasl );
        
        //Actually sample the terrain to get the height and adjust the eye position so it's a tighter fit to the real data.
        double height;
        if (_engine->getTerrain()->getHeight( SpatialReference::create("epsg:4326"), osg::RadiansToDegrees( lon ), osg::RadiansToDegrees( lat ), &height))
        {
            hasl -= height;
        }
        hasl = osg::maximum( hasl, 100.0 );

        worldUp = _ellipsoid->computeLocalUpVector(eye.x(), eye.y(), eye.z());

        eyeLen = eye.length();

        // radius of the earth under the eyepoint
        double radius = eyeLen - hasl; 
        horizonDistance = sqrt( 2.0 * radius * hasl ); 

        pvd._sharedHorizonDistance = horizonDistance;
        //OE_NOTICE << LC << "Horizon distance = " << pvd._sharedHorizonDistance << std::endl;
    
        // calculate the distance to the horizon, projected into the RTT camera plane.
        // This is the maximum limit of eMax since there is no point in drawing overlay
        // data beyond the visible horizon.
        double pitchAngleOfHorizon_rad = acos( horizonDistance/eyeLen );
        horizonDistanceInRTTPlane = horizonDistance * sin( pitchAngleOfHorizon_rad );

        OE_TEST << LC << "RTT distance to horizon: " << horizonDistanceInRTTPlane << std::endl;
    }
    else // projected map
    {
        hasl = eye.z();
        hasl = osg::maximum( hasl, 100.0 );
        worldUp.set( 0.0, 0.0, 1.0 );
        eyeLen = hasl * 2.0;

        // there is no maximum horizon distance in a projected map
        horizonDistance = DBL_MAX;
        horizonDistanceInRTTPlane = DBL_MAX;

        for(unsigned t=0; t<pvd._techParams.size(); ++t)
        {
            TechRTTParams& params = pvd._techParams[t];

            //TODO: change to tech->getActive() or something
            if ( params._group->getNumChildren() > 0 )
            {
                params._rttViewMatrix = osg::Matrixd::lookAt( eye, eye-worldUp*hasl, osg::Vec3(0,1,0) );
            }
        }
    }

    // create a "weighting" that weights HASL against the camera's pitch.
    osg::Vec3d lookVector = cv->getLookVectorLocal();
    haslWeight = osg::absolute(worldUp * lookVector);

    // unit look-vector of the eye:
    osg::Vec3d from, to, up;
    const osg::Matrix& mvMatrix = *cv->getModelViewMatrix();
    mvMatrix.getLookAt( from, to, up, eyeLen);
    osg::Vec3 camLookVec = to-from;
    camLookVec.normalize();

    // unit look-vector of the RTT camera:
    osg::Vec3d rttLookVec = -worldUp;

    // the minimum and maximum extents of the overlay ortho projector:
    double eMin = 0.1;
    double eMax = DBL_MAX;

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

    if ( _isGeocentric )
    {        
        // in geocentric mode, clamp the far clip plane to the horizon.
        double maxDistance = (1.0 - haslWeight)  * horizonDistance  + haslWeight * hasl;
        maxDistance = osg::clampBelow( maxDistance, _maxHorizonDistance );
        maxDistance *= 1.5;
        if (zFar - zNear >= maxDistance)
            zFar = zNear + maxDistance;

        cv->clampProjectionMatrix( projMatrix, zNear, zFar );

        OE_TEST << LC << "Horizon clamp: zNear = " << zNear << ", zFar = " << zFar << std::endl;
    }
       
    // contruct the polyhedron representing the viewing frustum.
    osgShadow::ConvexPolyhedron frustumPH;
    //MyConvexPolyhedron frustumPH;
    frustumPH.setToUnitFrustum( true, true );
    osg::Matrixd MVP = *cv->getModelViewMatrix() * projMatrix;
    osg::Matrixd inverseMVP;
    inverseMVP.invert(MVP);
    frustumPH.transform( inverseMVP, MVP );

    // take the bounds of the overlay graph, constrained to the view frustum:
    osg::BoundingSphere visibleOverlayBS;
    osg::Polytope frustumPT;
    frustumPH.getPolytope( frustumPT );

    // now we need to calculate the visible bounds for each overlay group.
    for( unsigned t=0; t<pvd._techParams.size(); ++t )
    {
        TechRTTParams& params = pvd._techParams[t];

        //TODO: changed to technique->getActive() or something
        // skip an empty group.
        if ( params._group->getNumChildren() == 0 )
            continue;

        // time to calculate the visible polyhedron.
        // intiailize it to the view frustum.
        osgShadow::ConvexPolyhedron visiblePH( frustumPH );

        // get a bounds of the overlay graph as a whole, and convert that to a
        // bounding box. We can probably do better with a ComputeBoundsVisitor but it
        // will be slower.
        visibleOverlayBS = params._group->getBound();
        if ( visibleOverlayBS.valid() )
        {
            osg::BoundingBox visibleOverlayBBox;
            visibleOverlayBBox.expandBy( visibleOverlayBS );

            // intersect that bound with the camera frustum:
            osg::Polytope visibleOverlayPT;
            visibleOverlayPT.setToBoundingBox( visibleOverlayBBox );
            visiblePH.cut( visibleOverlayPT );
        }

        // calculate the extents for our orthographic RTT camera (clamping it to the
        // visible horizon)
        std::vector<osg::Vec3d> verts;
        visiblePH.getPoints( verts );

        if ( _isGeocentric )
        {
            // for a geocentric map, try to place the RTT camera position at an optimal point
            // that will minimize the span of the RTT texture. Take the centroid of the 
            // visible polyhedron and clamp it's distance to the eyepoint by half the horizon
            // distance.
            osg::BoundingBox box = visiblePH.computeBoundingBox();
            osg::Vec3d bc = box.center();
            osg::Vec3d eye2bc = eye - bc;
            if ( eye2bc.length() > horizonDistance )
            {
                eye2bc.normalize();
                bc = eye + eye2bc * 0.5*horizonDistance;
            }
            
            rttLookVec = -bc;
            rttLookVec.normalize();

            double new_eMax;
            getMinMaxExtentInSilhouette( bc, rttLookVec, verts, eMin, new_eMax );
            eMax = std::min( eMax, new_eMax );

            bc.normalize();
            bc *= eyeLen;


            //TODO: resolve.... first is original; second is from the depthmaptest branch.....
            params._rttViewMatrix = osg::Matrixd::lookAt( bc, osg::Vec3d(0,0,0), osg::Vec3d(0,0,1) );
            params._rttProjMatrix = osg::Matrixd::ortho( -eMax, eMax, -eMax, eMax, -eyeLen, bc.length() );
            //params._rttProjMatrix = osg::Matrixd::ortho2D( -eMax, eMax, -eMax, eMax );

            OE_TEST << LC 
                << "1/2 RTT ortho span: " << eMax << ", near=" << -eyeLen << ", far=" << bc.length() << std::endl;

            OE_TEST << LC
                << "eMax = " << eMax
                << ", bc = " << bc.x() << ", " << bc.y() << ", " << bc.z()
                << ", eye = " << eye.x() << ", " << eye.y() << ", " << eye.z()
                << ", eyeLen = " << eyeLen
                << std::endl;
        }
        else
        {
            // for a projected map, just point the RTT straight down at the camera position.
            // TODO: this could be optimized, probably.
            double new_eMax;
            getMinMaxExtentInSilhouette( from, osg::Vec3d(0,0,-1), verts, eMin, new_eMax );   
            eMax = std::min( eMax, new_eMax ); 
            params._rttProjMatrix = osg::Matrix::ortho( -eMax, eMax, -eMax, eMax, -eyeLen, eyeLen );
            // (note: the View matrix was set eariler)
        }

        if ( _dumpRequested )
        {
            static const char* fn = "convexpolyhedron.osg";

            // camera frustum:
            {
                osgShadow::ConvexPolyhedron frustumPH;
                frustumPH.setToUnitFrustum( true, true );
                osg::Matrixd MVP = *cv->getModelViewMatrix() * projMatrix;
                osg::Matrixd inverseMVP;
                inverseMVP.invert(MVP);
                frustumPH.transform( inverseMVP, MVP );
                frustumPH.dumpGeometry(0,0,0,fn);
            }
            osg::Node* camNode = osgDB::readNodeFile(fn);
            camNode->setName("camera");

            //// visible PH or overlay:
            //visiblePHBeforeCut.dumpGeometry(0,0,0,fn,osg::Vec4(0,1,1,1),osg::Vec4(0,1,1,.25));
            //osg::Node* overlay = osgDB::readNodeFile(fn);
            //overlay->setName("overlay");

            // visible overlay Polyherdron AFTER frustum intersection:
            visiblePH.dumpGeometry(0,0,0,fn,osg::Vec4(1,.5,1,1),osg::Vec4(1,.5,0,.25));
            osg::Node* intersection = osgDB::readNodeFile(fn);
            intersection->setName("intersection");

            // RTT frustum:
            {
                osgShadow::ConvexPolyhedron rttPH;
                rttPH.setToUnitFrustum( true, true );
                osg::Matrixd MVP = params._rttViewMatrix * params._rttProjMatrix;
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
    if ( true ) //if (_totalOverlayChildren > 0 )
    {
        // in the CULL traversal, find the per-view data associated with the 
        // cull visitor's current camera view and work with that:
        if ( nv.getVisitorType() == nv.CULL_VISITOR )
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>( &nv );
            osg::Camera* camera = cv->getCurrentCamera();

            if ( camera != 0L && (_rttTraversalMask & nv.getTraversalMask()) != 0 )
            {
                PerViewData& pvd = getPerViewData( camera );

                //TODO:
                // check whether we need to recalculate the RTT camera params.
                // don't do it if the main camera hasn't moved;
                // also, tell the ClampingTech not to re-snap the depth texture
                // unless something has changed (e.g. camera params, terrain bounds..?
                // what about paging..?)

                // technique-specific setup prior to traversing:
                for(unsigned i=0; i<_techniques.size(); ++i)
                {
                    _techniques[i]->preCullTerrain( pvd._techParams[i], cv );
                }

                // shared terrain culling pass:
                cullTerrainAndCalculateRTTParams( cv, pvd );

                // prep and traverse the RTT camera(s):
                for(unsigned i=0; i<_techniques.size(); ++i)
                {
                    TechRTTParams& params = pvd._techParams[i];
                    _techniques[i]->cullOverlayGroup( params, cv );
                }
            }
            else
            {
                osg::Group::traverse(nv);
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

            osg::Group::traverse( nv );
        }
    }
    else
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
