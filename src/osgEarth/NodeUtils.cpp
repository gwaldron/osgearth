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

#include <osgEarth/NodeUtils>
#include <osg/TemplatePrimitiveFunctor>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/CullSettings>
#include <vector>

using namespace osgEarth;

//------------------------------------------------------------------------

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
              _maxRadius2(0.0f) { }

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
     * A customized CCC that works correctly under an RTT camera. The built-in one
     * called getEyePoint() instead of getViewPoint() and therefore isn't compatible
     * with osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT mode.
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

            osg::Vec3 eye_cp = nv->getViewPoint() - _controlPoint;
            float radius = eye_cp.length();

            if (radius<_radius)
            {
                return false;
            }

            float deviation = (eye_cp * _normal)/radius;

            return deviation < _deviation;
        }
    };
}

//------------------------------------------------------------------------

osg::ClusterCullingCallback*
ClusterCullerFactory::create( osg::Node* node, const osg::Vec3d& centerECEF )
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

    osg::ClusterCullingCallback* ccc = 0L;
    if ( node )
    {
        ComputeVisitor cv;
        cv.run( node, centerECEF );

        ccc = new MyClusterCullingCallback(); //osg::ClusterCullingCallback();
        ccc->set( cv._centerECEF, cv._normalECEF, 0.0f, sqrt(cv._maxRadius2) );
    }
    return ccc;
}

//------------------------------------------------------------------------

RemoveEmptyGroupsVisitor::RemoveEmptyGroupsVisitor() :
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
{
    //nop
}

void
RemoveEmptyGroupsVisitor::apply( osg::Group& group )
{
    bool removed = true;
    while( removed )
    {
        removed = false;
        for( unsigned i = 0; i < group.getNumChildren(); ++i )
        {
            osg::Group* child = group.getChild(i)->asGroup();
            if ( child )
            {
                if (child->className() == "Group"         &&
                    child->getStateSet() == 0L            &&
                    child->getCullCallback() == 0L        &&
                    child->getUpdateCallback() == 0L      &&
                    child->getUserData() == 0L            &&
                    child->getName().empty()              &&
                    child->getDescriptions().size() == 0 )
                {
                    for( unsigned j = 0; j < child->getNumChildren(); ++j )
                    {
                        group.addChild( child->getChild( j ) );
                    }

                    group.removeChild( i-- );
                    removed = true;
                }                
            }
        }
    }

    traverse(group);
}

//------------------------------------------------------------------------

PrimitiveSetTypeCounter::PrimitiveSetTypeCounter() :
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
_point  ( 0 ),
_line   ( 0 ),
_polygon( 0 )
{
    //nop
}

void
PrimitiveSetTypeCounter::apply(osg::Geode& geode)
{
    const osg::Geode::DrawableList& drawables = geode.getDrawableList();
    for( osg::Geode::DrawableList::const_iterator i = drawables.begin(); i != drawables.end(); ++i )
    {
        osg::Geometry* g = i->get()->asGeometry();
        if ( g )
        {
            const osg::Geometry::PrimitiveSetList& primSets = g->getPrimitiveSetList();
            for( osg::Geometry::PrimitiveSetList::const_iterator j = primSets.begin(); j != primSets.end(); ++j )
            {
                switch( j->get()->getMode() )
                {
                case GL_POINTS:
                    _point++;
                    break;
                case GL_LINES:
                case GL_LINE_LOOP:
                case GL_LINE_STRIP:
                    _line++;
                    break;
                default:
                    _polygon++;
                    break;
                }
            }
        }
    }
}
