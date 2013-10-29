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
#include <osgEarthUtil/HTM>
#include <osgEarth/CullingUtils>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarthAnnotation/LabelNode>
#include <osg/Geometry>
#include <osgText/Text>

#define LC "[HTMGroup] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

//-----------------------------------------------------------------------

bool
HTMNode::PolytopeDP::contains(const osg::Vec3d& p) const
{
    for( PlaneList::const_iterator i = _planeList.begin(); i != _planeList.end(); ++i )
    {
        if ( i->distance(p) < 0 )
            return false;
    }
    return true;
}

bool
HTMNode::PolytopeDP::containsAnyOf(const std::vector<osg::Vec3d>& points) const
{
    for( PlaneList::const_iterator i = _planeList.begin(); i != _planeList.end(); ++i )
    {
        if ( i->intersect(points) < 0 )
            return false;
    }
    return true;
}

//-----------------------------------------------------------------------

void
HTMNode::Triangle::set(const osg::Vec3d& v0, const osg::Vec3d& v1, const osg::Vec3d& v2)
{
    _v.resize(4);

    _v[0] = v0;
    _v[1] = v1;
    _v[2] = v2;
    _v[3].set(0,0,0);

    // assume verts are CCW.
    osg::Vec3d n0 = _v[0] ^ _v[1]; n0.normalize();
    osg::Vec3d n1 = _v[1] ^ _v[2]; n1.normalize();
    osg::Vec3d n2 = _v[2] ^ _v[0]; n2.normalize();

    // assemble the bounding polytope.
    _tope.add( osg::Plane(n0, _v[3]) );
    _tope.add( osg::Plane(n1, _v[3]) );
    _tope.add( osg::Plane(n2, _v[3]) );
}

void
HTMNode::Triangle::getMidpoints(osg::Vec3d* w) const
{
    w[0] = (_v[0]+_v[1]); w[0].normalize();
    w[1] = (_v[1]+_v[2]); w[1].normalize();
    w[2] = (_v[2]+_v[0]); w[2].normalize();
}

//-----------------------------------------------------------------------

HTMNode::HTMNode(HTMGroup*         root,
                 const osg::Vec3d& v0, 
                 const osg::Vec3d& v1, 
                 const osg::Vec3d& v2)
{
    this->setCullingActive(false);

    _root = root;
    _tri.set( v0, v1, v2 );
    _dataCount = 0;

    // init the bounding sphere
    _bs.expandBy( _tri._v[0] * 6380000);
    _bs.expandBy( _tri._v[1] * 6380000);
    _bs.expandBy( _tri._v[2] * 6380000);

    if ( true )
    {
        _debugGeode = new osg::Geode();
        osg::Geometry* g = new osg::Geometry();
        osg::Vec3Array* v = new osg::Vec3Array();
        v->push_back( v0 * 6372000 );
        v->push_back( v1 * 6372000 );
        v->push_back( v2 * 6372000 );
        g->setVertexArray( v );
        osg::Vec4Array* c = new osg::Vec4Array();
        c->push_back( osg::Vec4(1,1,0,1) );
        g->setColorArray( c );
        g->setColorBinding( osg::Geometry::BIND_OVERALL );
        g->addPrimitiveSet( new osg::DrawArrays(GL_LINE_LOOP, 0, 3) );
        g->getOrCreateStateSet()->setMode(GL_LIGHTING, 0);
        _debugGeode->addDrawable( g );
        _debugGeode->getOrCreateStateSet()->setRenderBinDetails(INT_MAX, "DepthSortedBin");
        _debugGeode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, 0);
    }

    {
        osgText::Text* text = new osgText::Text();
        text->setText("Hi.");
        text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
        text->setCharacterSize(48.0f);
        text->setAutoRotateToScreen(true);
        text->setPosition( ((v0+v1+v2)/(v0+v1+v2).length()) * 6400000 );
        text->setDataVariance(osg::Object::DYNAMIC);
        text->setFont( Registry::instance()->getDefaultFont() );
        text->setStateSet(new osg::StateSet());
        osg::Geode* geode = new osg::Geode();
        geode->addDrawable(text);
        geode->getOrCreateStateSet()->setRenderBinDetails(INT_MAX, "DepthSortedBin");
        geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, 0);
        geode->setCullingActive( false );
        _clusterNode = geode;
    }
}

HTMNode*
HTMNode::insert(osg::Node* node)
{
    HTMNode* leaf = this;

    dirtyBound();

    _data.push_back( node );

    if (_data.size() >= _root->getSplitThreshold() &&
        getNumChildren() == 0 )
    {
        split();
    }

    if ( _children.size() > 0 )
    {
        const osg::Vec3d& p = node->getBound().center();

        for(unsigned i=0; i<_children.size(); ++i)
        {
            HTMNode* child = dynamic_cast<HTMNode*>(_children[i].get());
            if ( child && child->contains(p) )
            {
                leaf = child->insert(node);
                break;
            }
        }
    }

    _dataCount++;

    dynamic_cast<osgText::Text*>(dynamic_cast<osg::Geode*>(_clusterNode.get())->getDrawable(0))
        ->setText( Stringify() << _dataCount );

    return leaf;
}

bool
HTMNode::remove(osg::Node* node)
{
    NodeList::iterator i = std::find( _data.begin(), _data.end(), node );
    if ( i != _data.end() )
    {
        dirtyBound();

        _data.erase( i );
        _dataCount--;

        bool found = false;
        for(unsigned i=0; i<_children.size() && !found; ++i)
        {
            HTMNode* child = dynamic_cast<HTMNode*>(_children[i].get());
            if ( child )
            {
                found = child->remove( node );
            }
        }

        return found;
    }
    else
    {
        return false;
    }
}

HTMNode*
HTMNode::findLeaf(osg::Node* node)
{
    HTMNode* leaf = 0L;

    NodeList::iterator i = std::find( _data.begin(), _data.end(), node );
    if ( i != _data.end() )
    {
        leaf = this;
        for(unsigned i=0; i<_children.size(); ++i)
        {
            HTMNode* child = dynamic_cast<HTMNode*>(_children[i].get());
            HTMNode* leaf2 = child->findLeaf( node );
            if ( leaf2 )
            {
                leaf = leaf2;
                break;
            }
        }
    }

    return leaf;
}

bool
HTMNode::refresh(osg::Node* node)
{
    const osg::Vec3d& p = node->getBound().center();

    if ( contains(p) )
    {
        // already in this node; if there are children, re-insert
        // into the new child.
        if ( _children.size() > 0 )
        {
            for(unsigned i=0; i<_children.size(); ++i)
            {
                HTMNode* child = dynamic_cast<HTMNode*>(_children[i].get());
                if ( child && child->contains(p) )
                {
                    child->insert(node);
                    break;
                }
            }
        }
        return true; // done.
    }
    else
    {
        std::remove( _data.begin(), _data.end(), node );
        _dataCount--;

        HTMNode* parentNode = dynamic_cast<HTMNode*>( getParent(0) );
        if ( parentNode )
        {
            return parentNode->refresh( node );
        }
        HTMGroup* parentGroup = dynamic_cast<HTMGroup*>( getParent(0) );
        if ( parentGroup )
        {
            return parentGroup->addChild( node );
        }

        // should never get here
        OE_WARN << LC << "trouble." << std::endl;
        return false;
    }
}

void
HTMNode::split()
{
    OE_DEBUG << LC << "Splitting htmid:" << getName() << std::endl;

    // find the midpoints of each side of the triangle
    osg::Vec3d w[3];
    _tri.getMidpoints( w );

    // split into four children, each wound CCW
    HTMNode* c[4];
    c[0] = new HTMNode(_root, _tri._v[0], w[0], w[2]);
    c[1] = new HTMNode(_root, _tri._v[1], w[1], w[0]);
    c[2] = new HTMNode(_root, _tri._v[2], w[2], w[1]);
    c[3] = new HTMNode(_root, w[0], w[1], w[2]);

    // distibute the data amongst the children
    for(NodeList::iterator i = _data.begin(); i != _data.end(); ++i)
    {
        osg::Node* node = i->get();
        const osg::BoundingSphere& bs = node->getBound();
        
        osg::Vec3d p = bs.center();
        p.normalize(); // need?

        for(unsigned j=0; j<4; ++j)
        {
            if ( c[j]->contains(p) )
            {
                c[j]->insert( node );
                break;
            }
        }
    }

    // add the node children
    for(unsigned i=0; i<4; ++i)
    {
        c[i]->setName( Stringify() << getName() << i );
        osg::Group::addChild( c[i] );

        OE_DEBUG << LC << "  htmid " << c[i]->getName() << " size = " << c[i]->dataCount() << std::endl;
    }
}

void
HTMNode::merge()
{
    dirtyBound();

    OE_INFO << LC << "Merging htmid:" << getName() << std::endl;
    //todo
}

bool
HTMNode::entirelyWithin(const osg::Polytope& tope) const
{
    const osg::Polytope::PlaneList& planes = tope.getPlaneList();
    for(unsigned i=0; i<3; ++i)
    {
        osg::Vec3d v = _tri._v[i] * 6372000;
        for( osg::Polytope::PlaneList::const_iterator plane = planes.begin(); plane != planes.end(); ++plane )
        {
            if ( plane->distance(v) < 0.0 )
                return false;
        }
    }
    return true;
}

bool
HTMNode::intersects(const osg::Polytope& tope) const
{
    const osg::Polytope::PlaneList& planes = tope.getPlaneList();

    for( osg::Polytope::PlaneList::const_iterator plane = planes.begin(); plane != planes.end(); ++plane )
    {
        unsigned pointsVisibleToPlane = 0;

        for(unsigned i=0; i<3; ++i)
        {
            osg::Vec3d v;
            
            v = _tri._v[i] * 6000000;
            if ( plane->distance(v) >= 0.0 )
                ++pointsVisibleToPlane;

            v = _tri._v[i] * 12000000;
            if ( plane->distance(v) >= 0.0 )
                ++pointsVisibleToPlane;
        }

        // if none of the points are visible to any one plane, intersection fails.
        if ( pointsVisibleToPlane == 0 )
            return false;
    }

    return true;
}

void
HTMNode::traverse(osg::NodeVisitor& nv)
{
    bool accepted   = true;
    bool inRange    = true;
    bool cull       = nv.getVisitorType()   == nv.CULL_VISITOR;
    bool activeOnly = nv.getTraversalMode() == nv.TRAVERSE_ACTIVE_CHILDREN;

    osg::Polytope* frustum = 0L;

    if ( activeOnly )
    {
        // first make sure this node is in range.
        //float alt = nv.getViewPoint().length() - 6300000;
        //inRange = alt <= _bs.radius()*5.0f;
        
        //float d = nv.getDistanceToViewPoint(_bs.center(),true);
        //inRange = d <= _bs.radius()*5.0f;

        // if this isn't a leaf node (i.e. has child nodes), check whether we
        // can "trivially accept" them all.
        if ( !isLeaf() && inRange )
        {
            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
            if ( cv )
            {
                frustum = &cv->getCurrentCullingSet().getFrustum();
                if ( entirelyWithin(*frustum) )
                {
                    OE_DEBUG << LC << getName() << ": trivially accepted. yay!" << std::endl;
                }
                else
                {
                    accepted = false;
                }
            }
        }
    }

    if ( accepted )
    {
        // should we draw a clustering node instead of the data?
        if ( _root->getCluster() && (!isLeaf() || !inRange) )
        {
            _clusterNode->accept(nv);
        }

        // draw the data itself?
        else if ( inRange )
        {
            for(NodeList::iterator i = _data.begin(); i != _data.end(); ++i)
            {
                i->get()->accept( nv );
            }
        }

        // draw a debugging node?
        if ( _debugGeode.valid() )
        {
            _debugGeode->accept( nv );
        }
    }
    else
    {
        // traverse the children.
        if ( frustum )
        {
            // A frustum is available, so cull against it
            for(unsigned i=0; i<_children.size(); ++i)
            {
                HTMNode* child = static_cast<HTMNode*>(_children[i].get());
                //if ( frustum->contains(child->getBound()) )
                if ( child->intersects(*frustum) )
                {
                    child->accept( nv );
                }
            }
        }
        else
        {
            // no frustum, just traverse them all as normal
            osg::Group::traverse( nv );
        }
    }
}

osg::BoundingSphere
HTMNode::computeBound() const
{
    return _bs;
}


//-----------------------------------------------------------------------

HTMGroup::HTMGroup() :
_dataCount     ( 0 ),
_splitThreshold( 48 ),
_mergeThreshold( 48 ),
_debug         ( false ),
_cluster       ( false )
{
    // hopefully prevent the OSG optimizer from altering this graph:
    setDataVariance( osg::Object::DYNAMIC );

    // assemble the base manifold of 8 triangles.
    osg::Vec3d v0( 0, 0, 1);      // lat= 90  long=  0
    osg::Vec3d v1( 1, 0, 0);      // lat=  0  long=  0
    osg::Vec3d v2( 0, 1, 0);      // lat=  0  long= 90
    osg::Vec3d v3(-1, 0, 0);      // lat=  0  long=180
    osg::Vec3d v4( 0,-1, 0);      // lat=  0  long=-90
    osg::Vec3d v5( 0, 0,-1);      // lat=-90  long=  0

    // CCW triangles.
    osg::Group::addChild( new HTMNode(this, v0, v1, v2) );
    osg::Group::addChild( new HTMNode(this, v0, v2, v3) );
    osg::Group::addChild( new HTMNode(this, v0, v3, v4) );
    osg::Group::addChild( new HTMNode(this, v0, v4, v1) );
    osg::Group::addChild( new HTMNode(this, v5, v1, v4) );
    osg::Group::addChild( new HTMNode(this, v5, v4, v3) );
    osg::Group::addChild( new HTMNode(this, v5, v3, v2) );
    osg::Group::addChild( new HTMNode(this, v5, v2, v1) );

    // HTMIDs.
    for(unsigned i=0; i<8; ++i)
    {
        getChild(i)->setName( Stringify() << i );
    }
}

bool
HTMGroup::insert(osg::Node* node)
{
    osg::Vec3d p = node->getBound().center();
    p.normalize();

    bool inserted = false;

    for(unsigned i=0; i<8; ++i)
    {
        HTMNode* child = static_cast<HTMNode*>(_children[i].get());
        if ( child->contains(p) )
        {
            child->insert(node);
            inserted = true;
            //_dataCount++;
            break;
        }
    }

    return inserted;
}

bool
HTMGroup::remove(osg::Node* node)
{
    bool found = false;

    for(unsigned i=0; i<8 && !found; ++i)
    {
        HTMNode* child = static_cast<HTMNode*>(_children[i].get());
        found = child->remove( node );
    }

    return found;
}

bool
HTMGroup::refresh(osg::Node* node)
{
    HTMNode* leaf = 0L;

    for(unsigned i=0; i<8 && !leaf; ++i)
    {
        HTMNode* child = static_cast<HTMNode*>(_children[i].get());
        leaf = child->findLeaf( node );
        if ( leaf )
        {
            leaf->refresh( node );
            break;
        }
    }

    return leaf != 0L;
}

void 
HTMGroup::traverse(osg::NodeVisitor& nv)
{
    osg::Group::traverse(nv);
}

bool 
HTMGroup::addChild(osg::Node* child)
{
    return insert( child );
}

bool 
HTMGroup::insertChild(unsigned index, osg::Node* child)
{
    return insert( child );
}

bool 
HTMGroup::removeChildren(unsigned pos, unsigned numChildrenToRemove)
{
    OE_WARN << LC << "removeChildren() not implemented for HTM" << std::endl;
    return false;
}

bool 
HTMGroup::replaceChild(osg::Node* origChild, osg::Node* newChild)
{
    OE_WARN << LC << "replaceChild() not implemented for HTM" << std::endl;
    return false;
}

bool 
HTMGroup::setChild(unsigned index, osg::Node* node)
{
    OE_WARN << LC << "setChild() not implemented for HTM" << std::endl;
    return false;
}
