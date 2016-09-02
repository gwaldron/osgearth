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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
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

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

//-----------------------------------------------------------------------

#undef  LC
#define LC "[HTMGroup] "

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

HTMNode::HTMNode(HTMSettings& settings,
                 const osg::Vec3d& v0, const osg::Vec3d& v1, const osg::Vec3d& v2) :
_settings(settings)
{
    _isLeaf = true;
    _tri.set( v0, v1, v2 );
}

void
HTMNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
#if 0
        if ( _isLeaf )
        {
            if (_settings._debugFrame != nv.getFrameStamp()->getFrameNumber())
            {
                OE_NOTICE << "Frame " << _settings._debugFrame << ": " << _settings._debugCount << std::endl;
                _settings._debugCount = 0;
                _settings._debugFrame = nv.getFrameStamp()->getFrameNumber();
            }
            _settings._debugCount += getNumChildren();
        }
#endif

        const osg::BoundingSphere& bs = getBound();

        if ( nv.getDistanceToViewPoint(bs.center(), true) <= (bs.radius() + _settings._maxLeafRange) )
        {
            osg::Group::traverse( nv );
        }
    }
    else
    {
        osg::Group::traverse( nv );
    }
}

void
HTMNode::insert(osg::Node* node)
{
    if ( _isLeaf )
    {
        if ((getNumChildren() < _settings._maxLeaves) ||
            (getBound().radius() < _settings._maxLeafRange))
        {
            addChild( node );
        }

        else
        {
            split();
            insert( node );
        }
    }

    else
    {
        const osg::Vec3d& p = node->getBound().center();

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
    c[0] = new HTMNode(_settings, _tri._v[0], w[0], w[2]);
    c[1] = new HTMNode(_settings, _tri._v[1], w[1], w[0]);
    c[2] = new HTMNode(_settings, _tri._v[2], w[2], w[1]);
    c[3] = new HTMNode(_settings, w[0], w[1], w[2]);
    
    // distibute the data amongst the children
    for(osg::NodeList::iterator i = _children.begin(); i != _children.end(); ++i)
    {
        osg::Node* node = i->get();        
        const osg::Vec3d& p = node->getBound().center();

        for(unsigned j=0; j<4; ++j)
        {
            if ( c[j]->contains(p) )
            {
                c[j]->insert( node );
                break;
            }
        }
    }

    // remove the leaves from this node
    osg::Group::removeChildren(0, getNumChildren());

    // add the new subnodes to this node.
    for(unsigned i=0; i<4; ++i)
    {
        c[i]->setName( Stringify() << getName() << i );
        osg::Group::addChild( c[i] );
    }

    _isLeaf = false;
}

//-----------------------------------------------------------------------

HTMGroup::HTMGroup()
{
    _settings._maxLeaves = 16;
    _settings._maxLeafRange = 50000.0f;

    // hopefully prevent the OSG optimizer from altering this graph:
    setDataVariance( osg::Object::DYNAMIC );

    reinitialize();
}

void
HTMGroup::setMaxLeaves(unsigned maxLeaves)
{
    _settings._maxLeaves = maxLeaves;

    reinitialize();
}

void
HTMGroup::setMaxLeafRange(float range)
{
    _settings._maxLeafRange = range;

    reinitialize();
}

void
HTMGroup::reinitialize()
{
    _children.clear();

    double rx = 1.0;
    double ry = 1.0;
    double rz = 1.0;

    // assemble the base manifold of 8 triangles.
    osg::Vec3d v0( 0,   0,   rz);     // lat= 90  long=  0
    osg::Vec3d v1( rx,  0,   0);      // lat=  0  long=  0
    osg::Vec3d v2( 0,   ry,  0);      // lat=  0  long= 90
    osg::Vec3d v3(-rx,  0,   0);      // lat=  0  long=180
    osg::Vec3d v4( 0,  -ry,  0);      // lat=  0  long=-90
    osg::Vec3d v5( 0,   0,  -rz);     // lat=-90  long=  0

    // CCW triangles.
    osg::Group::addChild( new HTMNode(_settings, v0, v1, v2) );
    osg::Group::addChild( new HTMNode(_settings, v0, v2, v3) );
    osg::Group::addChild( new HTMNode(_settings, v0, v3, v4) );
    osg::Group::addChild( new HTMNode(_settings, v0, v4, v1) );
    osg::Group::addChild( new HTMNode(_settings, v5, v1, v4) );
    osg::Group::addChild( new HTMNode(_settings, v5, v4, v3) );
    osg::Group::addChild( new HTMNode(_settings, v5, v3, v2) );
    osg::Group::addChild( new HTMNode(_settings, v5, v2, v1) );
}

bool
HTMGroup::insert(osg::Node* node)
{
    osg::Vec3d p = node->getBound().center();
    p.normalize(); // need?

    bool inserted = false;

    for(unsigned i=0; i<_children.size(); ++i)
    {
        HTMNode* child = static_cast<HTMNode*>(_children[i].get());
        if ( child->contains(p) )
        {
            child->insert(node);
            inserted = true;
            break;
        }
    }

    return inserted;
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
