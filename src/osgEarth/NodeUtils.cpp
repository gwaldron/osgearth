/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/CullSettings>
#include <osg/KdTree>
#include <osg/TriangleFunctor>
#include <vector>
#include <string>

using namespace osgEarth;


//------------------------------------------------------------------------

PagedLODWithNodeOperations::PagedLODWithNodeOperations( RefNodeOperationVector* postMergeOps ) :
_postMergeOps( postMergeOps )
{
    //nop
}


void
PagedLODWithNodeOperations::runPostMerge( osg::Node* node )
{
    if ( _postMergeOps.valid() )
    {
        for( NodeOperationVector::iterator i = _postMergeOps->begin(); i != _postMergeOps->end(); ++i )
        {
            i->get()->operator()( node );
        }
    }
}


bool 
PagedLODWithNodeOperations::addChild( osg::Node* child )
{
    bool ok = false;
    if ( child )
    {
        ok = osg::PagedLOD::addChild( child );
        if ( ok )
            runPostMerge( child );
    }
    return ok;
}


bool 
PagedLODWithNodeOperations::insertChild( unsigned index, Node* child )
{
    bool ok = false;
    if ( child )
    {
        ok = osg::PagedLOD::insertChild( index, child );
        if ( ok )
            runPostMerge( child );
    }
    return ok;
}


bool 
PagedLODWithNodeOperations::replaceChild( Node* origChild, Node* newChild )
{
    bool ok = false;
    if ( origChild && newChild )
    {
        ok = osg::PagedLOD::replaceChild( origChild, newChild );
        if ( ok )
            runPostMerge( newChild );
    }
    return ok;
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
                if (std::string(child->className()).compare("Group") == 0 &&
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


namespace
{
    struct TriangleIntersection
    {
        TriangleIntersection(unsigned int index, const osg::Vec3d& normal, double r1, const osg::Vec3* v1, double r2, const osg::Vec3* v2, double r3, const osg::Vec3* v3):
            _index(index),
            _normal(normal),
            _r1(r1),
            _v1(v1),
            _r2(r2),
            _v2(v2),
            _r3(r3),
            _v3(v3) {}

        unsigned int         _index;
        const osg::Vec3      _normal;
        double               _r1;
        const osg::Vec3*     _v1;
        double               _r2;
        const osg::Vec3*     _v2;
        double               _r3;
        const osg::Vec3*     _v3;

    protected:

        TriangleIntersection& operator = (const TriangleIntersection&) { return *this; }
    };


    typedef std::multimap<double,TriangleIntersection> TriangleIntersections;


    struct DoublePrecisionTriangleIntersector
    {
        osg::Vec3d   _s;
        osg::Vec3d   _d;
        double       _length;

        int         _index;
        double       _ratio;
        bool        _hit;
        bool        _limitOneIntersection;

        TriangleIntersections _intersections;

        DoublePrecisionTriangleIntersector()
        {
            _length = 0.0;
            _index = 0;
            _ratio = 0.0;
            _hit = false;
            _limitOneIntersection = false;
        }

        void set(const osg::Vec3d& start, osg::Vec3d& end, double ratio=FLT_MAX)
        {
            _hit=false;
            _index = 0;
            _ratio = ratio;

            _s = start;
            _d = end - start;
            _length = _d.length();
            _d /= _length;
        }

        inline void operator () (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3, bool treatVertexDataAsTemporary)
        {
            ++_index;

            if (_limitOneIntersection && _hit) return;

            if (v1==v2 || v2==v3 || v1==v3) return;

            osg::Vec3d v1d(v1), v2d(v2), v3d(v3);

            osg::Vec3d v12 = v2d-v1d;
            osg::Vec3d n12 = v12^_d;
            double ds12 = (_s-v1d)*n12;
            double d312 = (v3d-v1d)*n12;
            if (d312>=0.0)
            {
                if (ds12<0.0) return;
                if (ds12>d312) return;
            }
            else                     // d312 < 0
            {
                if (ds12>0.0) return;
                if (ds12<d312) return;
            }

            osg::Vec3d v23 = v3d-v2d;
            osg::Vec3d n23 = v23^_d;
            double ds23 = (_s-v2d)*n23;
            double d123 = (v1d-v2d)*n23;
            if (d123>=0.0)
            {
                if (ds23<0.0) return;
                if (ds23>d123) return;
            }
            else                     // d123 < 0
            {
                if (ds23>0.0) return;
                if (ds23<d123) return;
            }

            osg::Vec3d v31 = v1d-v3d;
            osg::Vec3d n31 = v31^_d;
            double ds31 = (_s-v3d)*n31;
            double d231 = (v2d-v3d)*n31;
            if (d231>=0.0)
            {
                if (ds31<0.0) return;
                if (ds31>d231) return;
            }
            else                     // d231 < 0
            {
                if (ds31>0.0) return;
                if (ds31<d231) return;
            }


            double r3;
            if (osg::equivalent(ds12,0.0)) r3=0.0;
            else if (!osg::equivalent(d312,0.0)) r3 = ds12/d312;
            else return; // the triangle and the line must be parallel intersection.

            double r1;
            if (osg::equivalent(ds23,0.0)) r1=0.0;
            else if (!osg::equivalent(d123,0.0)) r1 = ds23/d123;
            else return; // the triangle and the line must be parallel intersection.

            double r2;
            if (osg::equivalent(ds31,0.0)) r2=0.0;
            else if (!osg::equivalent(d231,0.0)) r2 = ds31/d231;
            else return; // the triangle and the line must be parallel intersection.

            double total_r = (r1+r2+r3);
            if (!osg::equivalent(total_r,1.0))
            {
                if (osg::equivalent(total_r,0.0)) return; // the triangle and the line must be parallel intersection.
                double inv_total_r = 1.0/total_r;
                r1 *= inv_total_r;
                r2 *= inv_total_r;
                r3 *= inv_total_r;
            }

            osg::Vec3d in = v1d*r1+v2d*r2+v3d*r3;
            if (!in.valid())
            {
                //OSG_WARN<<"Warning:: Picked up error in TriangleIntersect"<<std::endl;
                //OSG_WARN<<"   ("<<v1<<",\t"<<v2<<",\t"<<v3<<")"<<std::endl;
                //OSG_WARN<<"   ("<<r1<<",\t"<<r2<<",\t"<<r3<<")"<<std::endl;
                return;
            }

            float d = (in-_s)*_d;

            if (d<0.0) return;
            if (d>_length) return;

            osg::Vec3d normal = v12^v23;
            normal.normalize();

            float r = d/_length;


            if (treatVertexDataAsTemporary)
            {
                _intersections.insert(std::pair<const double,TriangleIntersection>(r,TriangleIntersection(_index-1,normal,r1,0,r2,0,r3,0)));
            }
            else
            {
                _intersections.insert(std::pair<const double,TriangleIntersection>(r,TriangleIntersection(_index-1,normal,r1,&v1,r2,&v2,r3,&v3)));
            }
            _hit = true;

        }

    };
}

//----------------------------------------------------------------------------

#undef  LC
#define LC "[ObserverGroup] "

ObserverGroup::ObserverGroup()
{
    ADJUST_EVENT_TRAV_COUNT(this, 1);
}

void
ObserverGroup::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == nv.EVENT_VISITOR )
    {
        // check for orphans:
        for(osg::NodeList::iterator itr = _children.begin(); itr != _children.end(); ++itr )
        {
            if ( (*itr)->referenceCount() == 1 )
            {
                // found one, queue an update traversal so we can safely delete it.
                // (it's probably safe to just delete it here, but anyway)
                if ( _orphans.insert(itr->get()).second == true )
                {
                    ADJUST_UPDATE_TRAV_COUNT( this, 1 );
                }
            }
        }
    }

    else if ( nv.getVisitorType() == nv.UPDATE_VISITOR && _orphans.size() > 0 )
    {
        // delete orphans:
        for( std::set<osg::Node*>::iterator i = _orphans.begin(); i != _orphans.end(); ++i )
        {
            this->removeChild( *i );
            ADJUST_UPDATE_TRAV_COUNT( this, -1 );
        }

        OE_DEBUG << LC << _orphans.size() << " orphaned children removed." << std::endl;
        _orphans.clear();
    }

    osg::Group::traverse( nv );
}
