/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarth/FindNode>
#include <osgEarth/Registry>
#include <osgEarth/Utils>

using namespace osgEarth::Util;
using namespace osgEarth;

namespace
{
    struct CustomProjClamper : public osg::CullSettings::ClampProjectionMatrixCallback
    {
        double _minNear, _maxFar, _nearFarRatio;

        CustomProjClamper() : _minNear( -DBL_MAX ), _maxFar( DBL_MAX ), _nearFarRatio( 0.0005 ) { }

        // NOTE: this code is just copied from CullVisitor. I could not find a way to simply 
        // call into it from a custom callback..
        template<class matrix_type, class value_type>
        bool _clampProjectionMatrix(matrix_type& projection, double& znear, double& zfar, value_type nearFarRatio) const
        {
            double epsilon = 1e-6;
            if (zfar<znear-epsilon)
            {
                OSG_INFO<<"_clampProjectionMatrix not applied, invalid depth range, znear = "<<znear<<"  zfar = "<<zfar<<std::endl;
                return false;
            }
            
            if (zfar<znear+epsilon)
            {
                // znear and zfar are too close together and could cause divide by zero problems
                // late on in the clamping code, so move the znear and zfar apart.
                double average = (znear+zfar)*0.5;
                znear = average-epsilon;
                zfar = average+epsilon;
                // OSG_INFO << "_clampProjectionMatrix widening znear and zfar to "<<znear<<" "<<zfar<<std::endl;
            }

            if (fabs(projection(0,3))<epsilon  && fabs(projection(1,3))<epsilon  && fabs(projection(2,3))<epsilon )
            {
                // OSG_INFO << "Orthographic matrix before clamping"<<projection<<std::endl;

                value_type delta_span = (zfar-znear)*0.02;
                if (delta_span<1.0) delta_span = 1.0;
                value_type desired_znear = znear - delta_span;
                value_type desired_zfar = zfar + delta_span;

                // assign the clamped values back to the computed values.
                znear = desired_znear;
                zfar = desired_zfar;

                projection(2,2)=-2.0f/(desired_zfar-desired_znear);
                projection(3,2)=-(desired_zfar+desired_znear)/(desired_zfar-desired_znear);

                // OSG_INFO << "Orthographic matrix after clamping "<<projection<<std::endl;
            }
            else
            {

                // OSG_INFO << "Persepective matrix before clamping"<<projection<<std::endl;

                //std::cout << "_computed_znear"<<_computed_znear<<std::endl;
                //std::cout << "_computed_zfar"<<_computed_zfar<<std::endl;

                value_type zfarPushRatio = 1.02;
                value_type znearPullRatio = 0.98;

                //znearPullRatio = 0.99; 

                value_type desired_znear = znear * znearPullRatio;
                value_type desired_zfar = zfar * zfarPushRatio;

                // near plane clamping.
                double min_near_plane = zfar*nearFarRatio;

                //// GW: changed this to enforce the NF ratio.
                if (desired_znear<min_near_plane) desired_znear=min_near_plane;
                //if (desired_znear > min_near_plane) desired_znear=min_near_plane;

                if ( desired_znear < 1.0 )
                    desired_znear = 1.0;

#if 0
                OE_INFO << std::fixed
                    << "nfr=" << nearFarRatio << ", znear=" << znear << ", zfar=" << zfar
                    << ", dznear=" << desired_znear << ", dzfar=" << desired_zfar
                    << std::endl;
#endif

                // assign the clamped values back to the computed values.
                znear = desired_znear;
                zfar = desired_zfar;


                value_type trans_near_plane = (-desired_znear*projection(2,2)+projection(3,2))/(-desired_znear*projection(2,3)+projection(3,3));
                value_type trans_far_plane = (-desired_zfar*projection(2,2)+projection(3,2))/(-desired_zfar*projection(2,3)+projection(3,3));

                value_type ratio = fabs(2.0/(trans_near_plane-trans_far_plane));
                value_type center = -(trans_near_plane+trans_far_plane)/2.0;

                projection.postMult(osg::Matrix(1.0f,0.0f,0.0f,0.0f,
                                                0.0f,1.0f,0.0f,0.0f,
                                                0.0f,0.0f,ratio,0.0f,
                                                0.0f,0.0f,center*ratio,1.0f));

                // OSG_INFO << "Persepective matrix after clamping"<<projection<<std::endl;
            }
            return true;
        }


        bool clampProjectionMatrixImplementation(osg::Matrixf& projection, double& znear, double& zfar) const
        {
            double n = std::max( znear, _minNear );
            double f = std::min( zfar, _maxFar );
            bool r = _clampProjectionMatrix( projection, n, f, _nearFarRatio );
            if ( r ) {
                znear = n;
                zfar = f;
            }
            return r;
        }

        bool clampProjectionMatrixImplementation(osg::Matrixd& projection, double& znear, double& zfar) const
        {
            double n = std::max( znear, _minNear );
            double f = std::min( zfar, _maxFar );
            bool r = _clampProjectionMatrix( projection, n, f, _nearFarRatio );
            if ( r ) {
                znear = n;
                zfar = f;
            }
            return r;
        }
    };
}

//--------------------------------------------------------------------------

AutoClipPlaneCullCallback::AutoClipPlaneCullCallback( Map* map ) :
_map                 ( map ),
_active              ( false ),
_minNearFarRatio     ( 0.00001 ),
_maxNearFarRatio     ( 0.0005 ),
_haeThreshold        ( 250.0 ),
_rp                  ( -1 ),
_rp2                 ( -1 ),
_autoFarPlaneClamping( true )
{
    if ( map )
    {
        if ( map->isGeocentric() )
        {
            // Select the minimal radius..
            const osg::EllipsoidModel* em = map->getProfile()->getSRS()->getEllipsoid();
            _rp = std::min( em->getRadiusEquator(), em->getRadiusPolar() );
            _rp2 = _rp*_rp;
            _active = true;
        }
        else
        {
            // deactivate for a projected map
            _active = false;
        }
    }
    else
    {
        const osg::EllipsoidModel* em = Registry::instance()->getGlobalGeodeticProfile()->getSRS()->getEllipsoid();
        _rp = std::min( em->getRadiusEquator(), em->getRadiusPolar() );
        _rp2 = _rp*_rp;
        _active = true;
    }
}

#if 0

void
AutoClipPlaneCullCallback::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    if ( !_active )
        return;

    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( nv );
    if ( !cv )
        return;

    osg::Camera* cam = cv->getCurrentCamera();

    cam->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );

    osg::Vec3d eye, center, up;
    cam->getViewMatrixAsLookAt( eye, center, up );

    double d = eye.length();
    double d2 = d*d;

    if ( d2 > _rp2 )
    {
        double fovy, ar, znear, zfar, finalZfar;
        cam->getProjectionMatrixAsPerspective( fovy, ar, znear, finalZfar );

        // far clip at the horizon:
        zfar = sqrt( d2 - _rp2 );

        if (_autoFarPlaneClamping)
        {
            finalZfar = zfar;
        }

        double nfr = _minNearFarRatio + _maxNearFarRatio * ((d-_rp)/d);
        znear = osg::clampAbove( zfar * nfr, 1.0 );

        cam->setProjectionMatrixAsPerspective( fovy, ar, znear, finalZfar );

        //OE_NOTICE << fixed
        //    << "near=" << znear << ", far=" << zfar << std::endl;
    }

    traverse(node, nv);
}

#else

void
AutoClipPlaneCullCallback::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    if ( _active )
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( nv );
        if ( cv )
        {
            osg::Camera* cam = cv->getCurrentCamera();
            osg::ref_ptr<osg::CullSettings::ClampProjectionMatrixCallback>& clamper = _clampers.get(cam);
            if ( !clamper.valid() )
            {
                clamper = new CustomProjClamper();
                cam->setClampProjectionMatrixCallback( clamper.get() );
            }
            else
            {
                CustomProjClamper* c = static_cast<CustomProjClamper*>(clamper.get());

                osg::Vec3d eye, center, up;
                cam->getViewMatrixAsLookAt( eye, center, up );

                // clamp the far clipping plane to the approximate horizon distance
                if ( _autoFarPlaneClamping )
                {
                    double d = eye.length();
                    c->_maxFar = sqrt( d*d - _rp2 );
                }
                else
                {
                    c->_maxFar = DBL_MAX;
                }

                // get the height-above-ellipsoid. If we need to be more accurate, we can use 
                // ElevationQuery in the future..
                osg::Vec3d loc;
                if ( _map.valid() )
                {
                    _map->worldPointToMapPoint( eye, loc );
                }
                else
                {
                    static osg::EllipsoidModel em;
                    em.convertXYZToLatLongHeight( eye.x(), eye.y(), eye.z(), loc.y(), loc.x(), loc.z() );
                }
                
                double hae = loc.z();

                // ramp a new near/far ratio based on the HAE.
                c->_nearFarRatio = Utils::remap( hae, 0.0, _haeThreshold, _minNearFarRatio, _maxNearFarRatio );
            }

#if 0
            {
                double n, f, a, v;
                cv->getProjectionMatrix()->getPerspective(v, a, n, f);
                OE_INFO << std::fixed << "near = " << n << ", far = " << f << ", ratio = " << n/f << std::endl;
            }
#endif
        }
    }
    traverse( node, nv );
}

#endif