/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/CullingUtils>
#include <osgEarth/LineFunctor>
#include <osgEarth/VirtualProgram>
#include <osgEarth/DPLineSegmentIntersector>
#include <osgEarth/GeoData>
#include <osgEarth/Utils>
#include <osg/ClusterCullingCallback>
#include <osg/PrimitiveSet>
#include <osg/Geode>
#include <osg/TemplatePrimitiveFunctor>
#include <osgGA/GUIActionAdapter>
#include <osgUtil/CullVisitor>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

using namespace osgEarth;

namespace
{
    struct ComputeMaxNormalLength
    {
        void set( const osg::Vec3& normalECEF, const osg::Matrixd& local2world, float* maxNormalLen )
        {
            _normal       = normalECEF;
            _local2world  = local2world;
            _maxNormalLen = maxNormalLen;
        }

        void operator()( const osg::Vec3 &v1, bool)
        {         
            compute( v1 );
        }

        void operator()( const osg::Vec3 &v1, const osg::Vec3 &v2, bool)
        {         
            compute( v1 );
            compute( v2 );
        }

        void operator()( const osg::Vec3 &v1, const osg::Vec3 &v2, const osg::Vec3& v3, bool)
        {         
            compute( v1 );
            compute( v2 );
            compute( v3 );
        }

        void operator()( const osg::Vec3 &v1, const osg::Vec3 &v2, const osg::Vec3& v3, const osg::Vec3& v4, bool)
        {         
            compute( v1 );
            compute( v2 );
            compute( v3 );
            compute( v4 );
        }

        void compute( const osg::Vec3& v )
        {
            osg::Vec3d vworld = v * _local2world;
            double vlen = vworld.length();
            vworld.normalize();

            // the dot product of the 2 vecs is the cos of the angle between them;
            // mult that be the vector length to get the new normal length.
            float normalLen = fabs(_normal * vworld) * vlen;

            if ( normalLen < *_maxNormalLen )
                *_maxNormalLen = normalLen;
        }

        osg::Vec3 _normal;
        osg::Matrixd _local2world;
        float*    _maxNormalLen;
    };

    struct ComputeMaxRadius2
    {
        void set( const osg::Vec3& center, float* maxRadius2 )
        {
            _center = center;
            _maxRadius2 = maxRadius2;
        }

        void operator()( const osg::Vec3 &v1, bool )
        {            
            compute( v1 );
        }

        void operator()( const osg::Vec3 &v1, const osg::Vec3 &v2, bool )
        {            
            compute( v1 );
            compute( v2 );
        }

        void operator()( const osg::Vec3 &v1, const osg::Vec3 &v2, const osg::Vec3 &v3, bool )
        {            
            compute( v1 );
            compute( v2 );
            compute( v3 );
        }        

        void operator()( const osg::Vec3 &v1, const osg::Vec3 &v2, const osg::Vec3 &v3, const osg::Vec3& v4, bool )
        {            
            compute( v1 );
            compute( v2 );
            compute( v3 );
            compute( v4 );
        }

        void compute( const osg::Vec3& v )
        {
            float dist = (v - _center).length2();
            if ( dist > *_maxRadius2 )
                *_maxRadius2 = dist;
        }



        osg::Vec3 _center;
        float*    _maxRadius2;
    };

    struct ComputeVisitor : public osg::NodeVisitor
    {
        ComputeVisitor()
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), 
              _maxRadius2(0.0f),
              _maxNormalLen(0.0f),
              _pass(0u) { }

        void run( osg::Node* node, const osg::Vec3d& centerECEF )
        {
            _centerECEF = centerECEF;
            _normalECEF = _centerECEF;
            _normalECEF.normalize();
            _maxNormalLen = _centerECEF.length();

            _pass = 1;
            node->accept( *this );

            _centerECEF = _normalECEF * _maxNormalLen;

            _pass = 2;
            node->accept( *this );
        }

        void apply( osg::Geode& geode )
        {            
            if ( _pass == 1 )
            {
                osg::Matrixd local2world;
                if ( !_matrixStack.empty() )
                    local2world = _matrixStack.back();

                osg::TemplatePrimitiveFunctor<ComputeMaxNormalLength> pass1;
                pass1.set( _normalECEF, local2world, &_maxNormalLen );

                for( unsigned i=0; i<geode.getNumDrawables(); ++i )
                    geode.getDrawable(i)->accept( pass1 );
            }

            else // if ( _pass == 2 )
            {
                osg::Vec3d center = _matrixStack.empty() ? _centerECEF : _centerECEF * osg::Matrixd::inverse(_matrixStack.back());

                osg::TemplatePrimitiveFunctor<ComputeMaxRadius2> pass2;
                pass2.set( center, &_maxRadius2 );
                for( unsigned i=0; i<geode.getNumDrawables(); ++i )
                    geode.getDrawable(i)->accept( pass2 );
            }
        }

        void apply( osg::Transform& xform )
        {
            osg::Matrixd matrix;
            if (!_matrixStack.empty()) matrix = _matrixStack.back();
            xform.computeLocalToWorldMatrix( matrix, this );
            _matrixStack.push_back( matrix );
            traverse(xform);
            _matrixStack.pop_back();
        }
        
        unsigned   _pass;
        osg::Vec3d _centerECEF;
        osg::Vec3f _normalECEF;
        float      _maxNormalLen;
        float      _maxRadius2;
        std::vector<osg::Matrixd> _matrixStack;
    };

    /** 
     * Line functor that computes the minimum deviation from a world normal, based on 
     * line normals in world space.
     */
    struct ComputeMinDeviation
    {
        void set( const osg::Matrixd& local2world, const osg::Vec3& normal, double* minDeviation )
        {
            _local2world        = local2world;
            _worldControlNormal = normal;
            _minDeviation       = minDeviation;
        }

        // called for each line segment in the geometry. Compute the world-space
        // "face" normal of the line, and DOT that with the world control normal
        // to find the deviation required for that line to be visible.
        void operator()( const osg::Vec3 &v1, const osg::Vec3 &v2, bool )
        {
            // convert endpoints to world space:
            osg::Vec3d v1d = v1;
            osg::Vec3d v2d = v2;
            v1d = v1d * _local2world;
            v1d.normalize();
            v2d = v2d * _local2world;
            v2d.normalize();

            // find the world-space normal of the line segment:
            osg::Vec3d segmentNormal = (v1d+v2d)/2.0;

            // calculate its deviation from the control normal
            double deviation = (_worldControlNormal * segmentNormal);
            if ( deviation < *_minDeviation )
                *_minDeviation = deviation;
        }

        osg::Vec3d   _worldControlNormal;
        osg::Matrixd _local2world;
        double*      _minDeviation;
    };

    /**
     * Visitor that goes over a scene graph and calculates all the cluster
     * culling parameters for that graph.
     */
    struct ComputeClusterCullingParams : public osg::NodeVisitor
    {
        ComputeClusterCullingParams( const osg::Vec3d& ecefControlPoint )
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
              _minDeviation( 1.0 ),
              _maxOffset(0)
        {
            _ecefControl = ecefControlPoint;
            _ecefNormal = ecefControlPoint;
            _ecefNormal.normalize();
            _matrixStack.push_back( osg::Matrixd::identity() );
        }

        void apply( osg::Geode& geode )
        {
            LineFunctor<ComputeMinDeviation> functor;
            functor.set( _matrixStack.back(), _ecefNormal, &_minDeviation );

             for( unsigned i=0; i<geode.getNumDrawables(); ++i )
             {
                 geode.getDrawable(i)->accept( functor );
             }

             traverse(geode);
        }

        void apply( osg::Transform& xform )
        {
            osg::Matrixd local2world = _matrixStack.back();
            xform.computeLocalToWorldMatrix(local2world, this);

            _matrixStack.push_back(local2world);

            traverse(xform);

            _matrixStack.pop_back();
        }
        
        std::vector<osg::Matrixd> _matrixStack;
        double _maxOffset, _minDeviation;
        osg::Vec3d _ecefControl, _ecefNormal;
    };
}


//----------------------------------------------------------------------------


osgUtil::CullVisitor*
Culling::asCullVisitor(osg::NodeVisitor* nv)
{
    if ( !nv )
        return 0L;

    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( nv );
    if ( cv )
        return cv;

    ProxyCullVisitor* pcv = dynamic_cast<ProxyCullVisitor*>( nv );
    if ( pcv )
        return pcv->getCullVisitor();

    return 0L;
}

//------------------------------------------------------------------------

void
DoNotComputeNearFarCullCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osgUtil::CullVisitor* cv = static_cast< osgUtil::CullVisitor*>( nv );
    osg::CullSettings::ComputeNearFarMode oldMode;
    if( cv )
    {
        oldMode = cv->getComputeNearFarMode();
        cv->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    }
    traverse(node, nv);
    if( cv )
    {
        cv->setComputeNearFarMode(oldMode);
    }
}


//----------------------------------------------------------------------------

bool 
SuperClusterCullingCallback::cull(osg::NodeVisitor* nv, osg::Drawable* , osg::State*) const
{
    static int c0, c1;
    static int frame = -1;

    osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

    if (!cv) return false;

    // quick bail if cluster culling is disabled:
    if ( !(cv->getCullingMode() & osg::CullSettings::CLUSTER_CULLING) )
        return false;

    bool visible = true;

    // quick bail is the deviation is maxed out
    if ( _deviation > -1.0f )
    {
        // accept if we're within the culling radius
        osg::Vec3d eye_cp = nv->getViewPoint() - _controlPoint;
        float radius = (float)eye_cp.length();
        if ( radius >= _radius )
        {
            // handle perspective and orthographic projections differently.
            const osg::Matrixd& proj = *cv->getProjectionMatrix();
            bool isOrtho = ( proj(3,3) == 1. ) && ( proj(2,3) == 0. ) && ( proj(1,3) == 0. ) && ( proj(0,3) == 0.);

            float deviation;
            if ( isOrtho )
            {
                // For an ortho camera, use the reverse look vector instead of the eye->controlpoint
                // vector for the deviation test. Transform the local reverse-look vector (always 0,0,1)
                // into world space and dot them. (Use 3x3 since we're xforming a vector, not a point)
                osg::Vec3d revLookWorld = osg::Matrix::transform3x3( *cv->getModelViewMatrix(), osg::Vec3d(0,0,1) );
                revLookWorld.normalize();
                deviation = revLookWorld * _normal;
                visible = deviation >= _deviation;
            }

            else // isPerspective
            {
                deviation = (eye_cp * _normal)/radius;
                visible = deviation >= _deviation;
            }

            visible = deviation >= _deviation;            
        }
    }

#if 0 // debugging
    int fn = cv->getFrameStamp()->getFrameNumber();
    if ( fn != frame )
    {
        frame = fn;
        OE_INFO << "Frame: " << fn << "try = " << c0 << ", viz = " << c1 << "\n";
        c0 = c1 = 0;
    }
    else
    {
        c0 ++;
        c1 += visible ? 1 : 0;
    }
#endif

    return !visible;
}


//----------------------------------------------------------------------------


osg::NodeCallback*
ClusterCullingFactory::create( osg::Node* node, const osg::Vec3d& ecefControlPoint )
{
    SuperClusterCullingCallback* ccc = 0L;
    if ( node )
    { 
        ComputeClusterCullingParams pass( ecefControlPoint );
        node->accept( pass );

        osg::Vec3d ecefNormal = ecefControlPoint;
        ecefNormal.normalize();
        
        // adjust the calculated deviation for the eyepoint angle:
        float angle = acosf(pass._minDeviation)+osg::PI*0.5f;
        float deviation = angle < osg::PI ? cosf(angle) : -1.0f;

        ccc = new SuperClusterCullingCallback();

        // note: getBound()->radius() should really be a computed max radius..todo.
        ccc->set( ecefControlPoint, ecefNormal, deviation, node->getBound().radius() );
    }
    return ccc;
}

osg::NodeCallback*
ClusterCullingFactory::create2( osg::Node* node, const osg::Vec3d& centerECEF )
{
    // Cluster culling computer. This works in two passes.
    //
    // It starts with a control point provided by the caller. I the first pass it computes
    // a new control point that is along the same geocentric normal but at a lower Z. This 
    // corresponds to the lowest Z of a vertex with respect to that normal. This means we
    // can always use a "deviation" of 0 -- and it gets around the problem of the control
    // point being higher than a vertex and corrupting the deviation.
    //
    // In the second pass, we compute the radius based on the new control point.
    //
    // NOTE: This is based on code developed separately from the method used in create()
    // above. We need to sort out which is the better approach and consolidate!

    osg::ClusterCullingCallback* ccc = 0L;
    if ( node )
    {
        ComputeVisitor cv;
        cv.run( node, centerECEF );

        ccc = new SuperClusterCullingCallback();
        ccc->set( cv._centerECEF, cv._normalECEF, 0.0f, sqrt(cv._maxRadius2) );
    }
    return ccc;
}

osg::Node*
ClusterCullingFactory::createAndInstall( osg::Node* node, const osg::Vec3d& ecefControlPoint )
{
    osg::NodeCallback* cb = create(node, ecefControlPoint);
    if ( cb )
    {
        if ( dynamic_cast<osg::Transform*>(node) )
        {
            osg::Group* group = new osg::Group();
            group->addChild( node );
            node = group;
        }

        node->addCullCallback( cb );
    }
    return node;
}

osg::NodeCallback*
ClusterCullingFactory::create(const osg::Vec3& controlPoint,
                              const osg::Vec3& normal,
                              float deviation,
                              float radius)
{
    SuperClusterCullingCallback* ccc = new SuperClusterCullingCallback();
    ccc->set(controlPoint, normal, deviation, radius);
    return ccc;
}

osg::NodeCallback*
ClusterCullingFactory::create(const GeoExtent& extent)
{
    GeoPoint centerPoint;
    extent.getCentroid( centerPoint );

    // select the farthest corner:
    GeoPoint edgePoint;
    if ( centerPoint.y() >= 0.0 )
        edgePoint = GeoPoint(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
    else
        edgePoint = GeoPoint(extent.getSRS(), extent.xMin(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);

    // convert both to ECEF and make unit vectors:
    osg::Vec3d center, centerNormal, edge, edgeNormal;

    centerPoint.toWorld( center );
    edgePoint.toWorld( edge );

    edgeNormal = edge;
    edgeNormal.normalize();

    centerNormal = center;
    centerNormal.normalize();

    // determine the height above the center point necessary to see the edge point:
    double dp = centerNormal * edgeNormal;
    double centerLen = center.length();
    double height = (edge.length() / dp) - centerLen;

    // set the control point at that height:
    osg::Vec3d controlPoint = center + centerNormal*height;

    // cluster culling only occurs beyond the maximum radius:
    double maxRadius = (controlPoint-edge).length();

    // determine the maximum visibility angle (i.e. minimum dot product
    // of the center up vector and the vector from the control point to 
    // the edge point)
    osg::Vec3d cpToEdge = edge - controlPoint;
    cpToEdge.normalize();                
    double minDeviation = centerNormal * cpToEdge;
    
    // create and return the callback.
    return create(
        controlPoint,
        centerNormal,
        minDeviation,
        maxRadius);
}

//------------------------------------------------------------------------

CullNodeByNormal::CullNodeByNormal( const osg::Vec3d& normal )
{
    _normal = normal;
    //_normal.normalize();
}

void
CullNodeByNormal::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::Vec3d eye, center, up;
    osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

    cv->getCurrentCamera()->getViewMatrixAsLookAt(eye,center,up);

    eye.normalize();
    osg::Vec3d normal = _normal;
    normal.normalize();

    double dotProduct = eye * normal;
    if ( dotProduct > 0.0 )
    {
        traverse(node, nv);
    }
}

//------------------------------------------------------------------------

void
DisableSubgraphCulling::operator()(osg::Node* n, osg::NodeVisitor* v)
{
    osgUtil::CullVisitor* cv = Culling::asCullVisitor(v);
    cv->getCurrentCullingSet().setCullingMask( osg::CullSettings::NO_CULLING );
    traverse(n, v);
}


//------------------------------------------------------------------------

// The max frame time in ms
double OcclusionCullingCallback::_maxFrameTime = 10.0;

//OcclusionCullingCallback::OcclusionCullingCallback(const osgEarth::SpatialReference *srs, const osg::Vec3d& world, osg::Node* node):
//_srs        ( srs ),
//_world      ( world ),
//_node       ( node ),
//_visible    ( true ),
//_maxAltitude( 200000 )
//{
//    //nop
//}
//
OcclusionCullingCallback::OcclusionCullingCallback(GeoTransform* xform) :
_xform      ( xform ),
_visible    ( true ),
_maxAltitude( 200000 )
{
    //nop
}

double OcclusionCullingCallback::getMaxFrameTime()
{
    return _maxFrameTime;
}

void OcclusionCullingCallback::setMaxFrameTime( double ms )
{
    _maxFrameTime = ms;
}

double OcclusionCullingCallback::getMaxAltitude() const
{
    return _maxAltitude;
}

void OcclusionCullingCallback::setMaxAltitude( double maxAltitude )
{
    _maxAltitude = maxAltitude;    
}

void OcclusionCullingCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    if (nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {        
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

        static int frameNumber = -1;
        static double remainingTime = OcclusionCullingCallback::_maxFrameTime;
        static int numCompleted = 0;
        static int numSkipped = 0;

        if (nv->getFrameStamp()->getFrameNumber() != frameNumber)
        {
            if (numCompleted > 0 || numSkipped > 0)
            {
                OE_DEBUG << "OcclusionCullingCallback frame=" << frameNumber << " completed=" << numCompleted << " skipped=" << numSkipped << std::endl;
            }
            frameNumber = nv->getFrameStamp()->getFrameNumber();
            numCompleted = 0;
            numSkipped = 0;
            remainingTime = OcclusionCullingCallback::_maxFrameTime;
        }

        osg::Vec3d eye = cv->getViewPoint();

        if (_prevEye != eye)
        {
            if (remainingTime > 0.0)
            {
                osg::ref_ptr<GeoTransform> geo;
                if ( _xform.lock(geo) )
                {
                    double alt = 0.0;

                    osg::ref_ptr<Terrain> terrain = geo->getTerrain();
                    if ( terrain.valid() && !terrain->getSRS()->isProjected() )
                    {
                        osgEarth::GeoPoint mapPoint;
                        mapPoint.fromWorld( terrain->getSRS(), eye );
                        alt = mapPoint.z();
                    }
                    else
                    {
                        alt = eye.z();
                    }

                    //Only do the intersection if we are close enough for it to matter
                    if (alt <= _maxAltitude && terrain.valid())
                    {
                        //Compute the intersection from the eye to the world point
                        osg::Timer_t startTick = osg::Timer::instance()->tick();
                        osg::Vec3d start = eye;
                        osg::Vec3d end = osg::Vec3d(0,0,0) * geo->getMatrix();

                        // shorten the intersector by 1m to prevent flickering.
                        osg::Vec3d vec = end-start; vec.normalize();
                        end -= vec*1.0;

                        DPLineSegmentIntersector* i = new DPLineSegmentIntersector( start, end );
                        i->setIntersectionLimit( osgUtil::Intersector::LIMIT_NEAREST );
                        osgUtil::IntersectionVisitor iv;
                        iv.setIntersector( i );
                        terrain->accept( iv );
                        osgUtil::LineSegmentIntersector::Intersections& results = i->getIntersections();
                        _visible = results.empty();
                        osg::Timer_t endTick = osg::Timer::instance()->tick();
                        double elapsed = osg::Timer::instance()->delta_m( startTick, endTick );
                        remainingTime -= elapsed;
                    }
                    else
                    {
                        _visible = true;
                    }

                    numCompleted++;

                    _prevEye = eye;
                }
            }
            else
            {
                numSkipped++;
                // if we skipped some we need to request a redraw so the remianing ones get processed on the next frame.
                if ( cv->getCurrentCamera() && cv->getCurrentCamera()->getView() )
                {
                    osgGA::GUIActionAdapter* aa = dynamic_cast<osgGA::GUIActionAdapter*>(cv->getCurrentCamera()->getView());
                    if ( aa )
                    {
                        aa->requestRedraw();
                    }
                }
            }
        }

        if (_visible)
        {
            traverse(node, nv );
        }
    }
    else
    {
        traverse( node, nv );
    }
}

//------------------------------------------------------------------------

ProxyCullVisitor::ProxyCullVisitor( osgUtil::CullVisitor* cv, const osg::Matrix& proj, const osg::Matrix& view ) :
osg::NodeVisitor( osg::NodeVisitor::CULL_VISITOR, osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN ),
_cv             ( cv )
{
    // set up the initial proxy frustum for culling:
    _proxyProjFrustum.setToUnitFrustum( true, true );
    _proxyProjFrustum.transformProvidingInverse( proj );
    _proxyModelViewMatrix = view;
    _proxyFrustum.setAndTransformProvidingInverse( _proxyProjFrustum, view );

    // copy NodeVisitor values over to this visitor:
    _nodePath = _cv->getNodePath();

    this->setFrameStamp( const_cast<osg::FrameStamp*>(_cv->getFrameStamp()) );
    this->setTraversalNumber( _cv->getTraversalNumber() );
    this->setTraversalMask( _cv->getTraversalMask() );
    this->setNodeMaskOverride( _cv->getNodeMaskOverride() );
    this->setDatabaseRequestHandler( _cv->getDatabaseRequestHandler() );
    this->setImageRequestHandler( _cv->getImageRequestHandler() );
    this->setUserData( _cv->getUserData() );
    this->setComputeNearFarMode( _cv->getComputeNearFarMode() );

    this->pushViewport( _cv->getViewport() );
    this->pushProjectionMatrix( _cv->getProjectionMatrix() );
    this->pushModelViewMatrix( _cv->getModelViewMatrix(), osg::Transform::ABSOLUTE_RF );
}

osg::Vec3 
ProxyCullVisitor::getEyePoint() const { 
    return _cv->getEyePoint();
}

osg::Vec3 
ProxyCullVisitor::getViewPoint() const { 
    return _cv->getViewPoint();
}

float 
ProxyCullVisitor::getDistanceToEyePoint(const osg::Vec3& pos, bool useLODScale) const { 
    return _cv->getDistanceToEyePoint(pos, useLODScale);
}

float 
ProxyCullVisitor::getDistanceFromEyePoint(const osg::Vec3& pos, bool useLODScale) const {
    return _cv->getDistanceFromEyePoint(pos, useLODScale);
}

float 
ProxyCullVisitor::getDistanceToViewPoint(const osg::Vec3& pos, bool useLODScale) const { 
    return _cv->getDistanceToViewPoint(pos, useLODScale);
}

bool 
ProxyCullVisitor::isCulledByProxyFrustum(osg::Node& node)
{
    return node.isCullingActive() && !_proxyFrustum.contains(node.getBound());
}

bool 
ProxyCullVisitor::isCulledByProxyFrustum(const osg::BoundingBox& bbox)
{
    return !_proxyFrustum.contains(bbox);
}

osgUtil::CullVisitor::value_type 
ProxyCullVisitor::distance(const osg::Vec3& coord,const osg::Matrix& matrix)
{
    return -((osgUtil::CullVisitor::value_type)coord[0]*(osgUtil::CullVisitor::value_type)matrix(0,2)+(osgUtil::CullVisitor::value_type)coord[1]*(osgUtil::CullVisitor::value_type)matrix(1,2)+(osgUtil::CullVisitor::value_type)coord[2]*(osgUtil::CullVisitor::value_type)matrix(2,2)+matrix(3,2));
}

void 
ProxyCullVisitor::handle_cull_callbacks_and_traverse(osg::Node& node)
{
#if OSG_VERSION_GREATER_THAN(3,3,1)
    osg::Callback* callback = node.getCullCallback();
    if (callback) callback->run(&node, this);
    else traverse(node);
#else
    osg::NodeCallback* callback = node.getCullCallback();
    if (callback) (*callback)(&node,this);
    else traverse(node);
#endif
}

void 
ProxyCullVisitor::apply(osg::Node& node)
{
    //OE_INFO << "Node: " << node.className() << std::endl;

    if ( isCulledByProxyFrustum(node) )
        return;

    _cv->pushOntoNodePath( &node );

    _cv->pushCurrentMask();
    osg::StateSet* node_state = node.getStateSet();
    if (node_state) _cv->pushStateSet(node_state);
    handle_cull_callbacks_and_traverse(node);
    if (node_state) _cv->popStateSet();
    _cv->popCurrentMask();

    _cv->popFromNodePath();
}

void 
ProxyCullVisitor::apply(osg::Transform& node)
{
    //OE_INFO << "Transform!" << std::endl;

    if ( isCulledByProxyFrustum(node) )
        return;

    _cv->pushOntoNodePath( &node);

    _cv->pushCurrentMask();
    osg::StateSet* node_state = node.getStateSet();
    if (node_state) _cv->pushStateSet(node_state);

    // push the current proxy data:
    osg::Polytope savedF  = _proxyFrustum;
    osg::Matrix   savedMV = _proxyModelViewMatrix;

    // calculate the new proxy frustum:
    node.computeLocalToWorldMatrix(_proxyModelViewMatrix, this);
    _proxyFrustum.setAndTransformProvidingInverse( _proxyProjFrustum, _proxyModelViewMatrix );

    osg::ref_ptr<osg::RefMatrix> matrix = createOrReuseMatrix(*_cv->getModelViewMatrix());
    node.computeLocalToWorldMatrix(*matrix,this);
    _cv->pushModelViewMatrix(matrix.get(), node.getReferenceFrame());

    // traverse children:
    handle_cull_callbacks_and_traverse(node);

    // restore the previous proxy frustum and MVM
    _proxyFrustum         = savedF;
    _proxyModelViewMatrix = savedMV;

    _cv->popModelViewMatrix();
    if (node_state) _cv->popStateSet();
    _cv->popCurrentMask();

    _cv->popFromNodePath();
}

void 
ProxyCullVisitor::apply(osg::Geode& node)
{
    //OE_INFO << "Geode!" << std::endl;

    if ( isCulledByProxyFrustum(node) )
        return;

    _cv->pushOntoNodePath( &node );

    // push the node's state.
    osg::StateSet* node_state = node.getStateSet();
    if (node_state) _cv->pushStateSet(node_state);

    // traverse any call callbacks and traverse any children.
    handle_cull_callbacks_and_traverse(node);

    osg::RefMatrix& matrix = *_cv->getModelViewMatrix();
    for(unsigned int i=0;i<node.getNumDrawables();++i)
    {
        osg::Drawable* drawable = node.getDrawable(i);
        const osg::BoundingBox& bb = Utils::getBoundingBox(drawable);

        if( drawable->getCullCallback() )
        {
#if OSG_VERSION_GREATER_THAN(3,3,1)
            if( drawable->getCullCallback()->run(drawable, _cv) == true )
                continue;
#else
            if( drawable->getCullCallback()->cull( _cv, drawable, &_cv->getRenderInfo() ) == true )
                continue;
#endif
        }

        //else
        {
            if (node.isCullingActive() && isCulledByProxyFrustum(bb)) continue;
        }


        if ( _cv->getComputeNearFarMode() && bb.valid())
        {
            if (!_cv->updateCalculatedNearFar(matrix,*drawable,false)) continue;
        }

        // need to track how push/pops there are, so we can unravel the stack correctly.
        unsigned int numPopStateSetRequired = 0;

        // push the geoset's state on the geostate stack.
        osg::StateSet* stateset = drawable->getStateSet();
        if (stateset)
        {
            ++numPopStateSetRequired;
            _cv->pushStateSet(stateset);
        }

        osg::CullingSet& cs = _cv->getCurrentCullingSet();
        if (!cs.getStateFrustumList().empty())
        {
            osg::CullingSet::StateFrustumList& sfl = cs.getStateFrustumList();
            for(osg::CullingSet::StateFrustumList::iterator itr = sfl.begin();
                itr != sfl.end();
                ++itr)
            {
                if (itr->second.contains(bb))
                {
                    ++numPopStateSetRequired;
                    _cv->pushStateSet(itr->first.get());
                }
            }
        }

        float depth = bb.valid() ? distance(bb.center(),matrix) : 0.0f;

        if (osg::isNaN(depth))
        {
            for (osg::NodePath::const_iterator i = getNodePath().begin(); i != getNodePath().end(); ++i)
            {
                OSG_DEBUG << "        \"" << (*i)->getName() << "\"" << std::endl;
            }
        }
        else
        {
            _cv->addDrawableAndDepth(drawable,&matrix,depth);
        }

        for(unsigned int i=0;i< numPopStateSetRequired; ++i)
        {
            _cv->popStateSet();
        }

    }

    // pop the node's state off the geostate stack.
    if (node_state) _cv->popStateSet();

    _cv->popFromNodePath();
}

//-------------------------------------------------------------------------

namespace
{
    const char* horizon_vs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform mat4 osg_ViewMatrix; \n"
        "varying float oe_horizon_alpha; \n"
        "void oe_horizon_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    const float scale     = 0.001; \n"                 // scale factor keeps dots&crosses in SP range
        "    const float radiusMax = 6371000.0 * scale; \n"
        "    vec3  originVIEW = (osg_ViewMatrix * vec4(0,0,0,1)).xyz * scale; \n"
        "    vec3  x1 = vec3(0,0,0) - originVIEW; \n"              // vector from origin -> camera
        "    vec3  x2 = (VertexVIEW.xyz * scale) - originVIEW; \n" // vector from origin -> vertex
        "    vec3  v  = x2-x1; \n"
        "    float vlen = length(v); \n"
        "    float t = -dot(x1,v)/(vlen*vlen); \n"
        "    bool visible = false; \n"
        "    if ( t > 1.0 || t < 0.0 ) { \n"
        "        oe_horizon_alpha = 1.0; \n"
        "    } \n"
        "    else { \n"
        "        float d = length(cross(x1,x2)) / vlen; \n"
        "        oe_horizon_alpha = d >= radiusMax ? 1.0 : 0.0; \n"
        "    } \n"
        "} \n";

    const char* horizon_fs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "varying float oe_horizon_alpha; \n"
        "void oe_horizon_fragment(inout vec4 color) \n"
        "{ \n"
        "    color.a *= oe_horizon_alpha; \n"
        "} \n";
}


void
HorizonCullingProgram::install(osg::StateSet* stateset)
{
    if ( stateset )
    {
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setFunction( "oe_horizon_vertex",   horizon_vs, ShaderComp::LOCATION_VERTEX_VIEW );
        vp->setFunction( "oe_horizon_fragment", horizon_fs, ShaderComp::LOCATION_FRAGMENT_COLORING );
    }
}

void
HorizonCullingProgram::remove(osg::StateSet* stateset)
{
    if ( stateset )
    {
        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( vp )
        {
            vp->removeShader( "oe_horizon_vertex" );
            vp->removeShader( "oe_horizon_fragment" );
        }
    }
}

//----------------------------------------------------------------

LODScaleGroup::LODScaleGroup() :
_scaleFactor( 1.0f )
{
    //nop
}

void
LODScaleGroup::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        osg::CullStack* cs = dynamic_cast<osg::CullStack*>( &nv );
        if ( cs )
        {
            float lodscale = cs->getLODScale();
            cs->setLODScale( lodscale * _scaleFactor );
            std::for_each( _children.begin(), _children.end(), osg::NodeAcceptOp(nv));
            cs->setLODScale( lodscale );
            return;
        }
    }

    osg::Group::traverse( nv );
}

//------------------------------------------------------------------

ClipToGeocentricHorizon::ClipToGeocentricHorizon(const osgEarth::SpatialReference* srs,
                                                 osg::ClipPlane*                   clipPlane)
{
    if ( srs )
    {
        _horizon = new Horizon();
        _horizon->setEllipsoid( *srs->getEllipsoid() );
    }

    _clipPlane = clipPlane;
}

void
ClipToGeocentricHorizon::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::ref_ptr<osg::ClipPlane> clipPlane;
    if ( _clipPlane.lock(clipPlane) )
    {
        osg::ref_ptr<Horizon> horizon = Horizon::get(*nv);
        if ( !horizon.valid() ) 
        {
            horizon = new Horizon(*_horizon.get());
            horizon->setEye( nv->getViewPoint() );
        }

        osg::Plane horizonPlane;
        horizon->getPlane( horizonPlane );

        _clipPlane->setClipPlane( horizonPlane );
    }
    traverse(node, nv);
}

//......................................................................

namespace
{
    Config dumpStateGraph(osgUtil::StateGraph* sg)
    {
        Config conf("StateGraph");

        Config leaves("Leaves");
        for(osgUtil::StateGraph::LeafList::const_iterator i = sg->_leaves.begin(); i != sg->_leaves.end(); ++i)
        {
            Config leaf("Leaf");
            leaf.add("Name", i->get()->getDrawable()->getName());
            leaf.add("Depth", i->get()->_depth);
            leaves.add( leaf );
        }
        if ( !leaves.empty() )
            conf.add(leaves);

        Config kids("Children");
        for(osgUtil::StateGraph::ChildList::const_iterator i = sg->_children.begin(); i != sg->_children.end(); ++i)
        {
            kids.add( dumpStateGraph(i->second.get()) );
        }
        if ( !kids.children().empty() )
            conf.add(kids);

        return conf;
    }
}

Config
CullDebugger::dumpRenderBin(osgUtil::RenderBin* bin) const
{
    Config conf("RenderBin");
    if ( !bin->getName().empty() )
        conf.set("Name", bin->getName());
    conf.set("BinNum", bin->getBinNum());
    
    Config sg("StateGraphList");
    sg.add("NumChildren", bin->getStateGraphList().size());

    for(osgUtil::RenderBin::StateGraphList::const_iterator i = bin->getStateGraphList().begin(); i != bin->getStateGraphList().end(); ++i)
    {
        sg.add( dumpStateGraph(*i) );
    }

    if ( !sg.children().empty() )
        conf.add(sg);

    Config rb("_children");
    for(osgUtil::RenderBin::RenderBinList::const_iterator i = bin->getRenderBinList().begin(); i != bin->getRenderBinList().end(); ++i)
    {
        osgUtil::RenderBin* childBin = i->second.get();
        rb.add( dumpRenderBin(childBin) );
    }
    if ( !rb.children().empty() )
        conf.add(rb);

    return conf;
}
