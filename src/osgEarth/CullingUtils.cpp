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
    
    /**
     * A customized CCC that works correctly under an RTT camera. The built-in one
     * called getEyePoint() instead of getViewPoint() and therefore isn't compatible
     * with osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT mode.
     *
     * Note: this is fixed in newer versions of OSG, but let's keep it around -gw
     */
    struct MyClusterCullingCallback : public osg::ClusterCullingCallback
    {
        bool cull(osg::NodeVisitor* nv, osg::Drawable* , osg::State*) const
        {
            osg::CullSettings* cs = dynamic_cast<osg::CullSettings*>(nv);
            if (cs && !(cs->getCullingMode() & osg::CullSettings::CLUSTER_CULLING))
            {
                return false;
            }

            if (_deviation<=-1.0f)
            {
                return false;
            }

            osg::Vec3d eye_cp = nv->getViewPoint() - _controlPoint;
            float radius = (float)eye_cp.length();

            if (radius<_radius)
            {
                return false;
            }

            float deviation = (eye_cp * _normal)/radius;

            return deviation < _deviation;
        }
    };
}

//----------------------------------------------------------------------------

osg::NodeCallback*
ClusterCullingFactory::create( osg::Node* node, const osg::Vec3d& ecefControlPoint )
{
    MyClusterCullingCallback* ccc = 0L;
    if ( node )
    { 
        ComputeClusterCullingParams pass( ecefControlPoint );
        node->accept( pass );

        osg::Vec3d ecefNormal = ecefControlPoint;
        ecefNormal.normalize();
        
        // adjust the calculated deviation for the eyepoint angle:
        float angle = acosf(pass._minDeviation)+osg::PI*0.5f;
        float deviation = angle < osg::PI ? cosf(angle) : -1.0f;

        ccc = new MyClusterCullingCallback();

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
