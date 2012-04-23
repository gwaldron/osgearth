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
#include <osgEarth/CullingUtils>
#include <osgEarth/LineFunctor>
#include <osg/ClusterCullingCallback>
#include <osg/PrimitiveSet>
#include <osg/Geode>
#include <osgUtil/CullVisitor>

using namespace osgEarth;

namespace
{
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
              _minDeviation( 1.0 )
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
        double _maxOffset, _maxRadius, _minDeviation;
        osg::Vec3d _ecefControl, _ecefNormal;
    };
}


//----------------------------------------------------------------------------


bool 
SuperClusterCullingCallback::cull(osg::NodeVisitor* nv, osg::Drawable* , osg::State*) const
{
    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);

    // quick bail if cluster culling is disabled:
    if ( cv && !(cv->getCullingMode() & osg::CullSettings::CLUSTER_CULLING) )
        return false;

    // quick bail is the deviation is maxed out
    if ( _deviation <= -1.0f )
        return false;

    // accept if we're within the culling radius
    osg::Vec3d eye_cp = nv->getViewPoint() - _controlPoint;
    float radius = (float)eye_cp.length();
    if (radius < _radius)
        return false;

    // handle perspective and orthographic projections differently.
    const osg::Matrixd& proj = *cv->getProjectionMatrix();
    bool isOrtho = ( proj(3,3) == 1. ) && ( proj(2,3) == 0. ) && ( proj(1,3) == 0. ) && ( proj(0,3) == 0.);

    if ( isOrtho )
    {
        // For an ortho camera, use the reverse look vector instead of the eye->controlpoint
        // vector for the deviation test. Transform the local reverse-look vector (always 0,0,1)
        // into world space and dot them. (Use 3x3 since we're xforming a vector, not a point)
        osg::Vec3d revLookWorld = osg::Matrix::transform3x3( *cv->getModelViewMatrix(), osg::Vec3d(0,0,1) );
        revLookWorld.normalize();
        float deviation = revLookWorld * _normal;
        return deviation < _deviation;
    }

    else // isPerspective
    {
        float deviation = (eye_cp * _normal)/radius;
        return deviation < _deviation;
    }
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


//------------------------------------------------------------------------

CullNodeByHorizon::CullNodeByHorizon( const osg::Vec3d& world, const osg::EllipsoidModel* model ) :
_world(world),
_r(model->getRadiusPolar()),
_r2(model->getRadiusPolar() * model->getRadiusPolar())
{
    //nop
}

void
CullNodeByHorizon::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    if ( nv )
    {
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>( nv );

        // get the viewpoint. It will be relative to the current reference location (world).
        osg::Matrix l2w = osg::computeLocalToWorld( nv->getNodePath(), true );
        osg::Vec3d vp  = cv->getViewPoint() * l2w;

        // same quadrant:
        if ( vp * _world >= 0.0 )
        {
            double d2 = vp.length2();
            double horiz2 = d2 - _r2;
            double dist2 = (_world-vp).length2();
            if ( dist2 < horiz2 )
            {
                traverse(node, nv);
            }
        }

        // different quadrants:
        else
        {
            // there's a horizon between them; now see if the thing is visible.
            // find the triangle formed by the viewpoint, the target point, and 
            // the center of the earth.
            double a = (_world-vp).length();
            double b = _world.length();
            double c = vp.length();

            // Heron's formula for triangle area:
            double s = 0.5*(a+b+c);
            double area = 0.25*sqrt( s*(s-a)*(s-b)*(s-c) );

            // Get the triangle's height:
            double h = (2*area)/a;

            if ( h >= _r )
            {
                traverse(node, nv);
            }
        }
    }
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
    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>( nv );

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