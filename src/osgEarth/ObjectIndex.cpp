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

#include <osgEarth/ObjectIndex>
#include <osgEarth/Registry>
#include <osg/NodeVisitor>
#include <osg/Uniform>

using namespace osgEarth;

#define LC "[ObjectIndex] "

//#undef OE_DEBUG
//#define OE_DEBUG OE_NOTICE

// Object IDs under this reserved
#define STARTING_OBJECT_ID 10

ObjectIndex::ObjectIndex() :
_idGen( STARTING_OBJECT_ID )
{
    _attribLocation    = osg::Drawable::SECONDARY_COLORS;
    _attribUniformName = "oe_index_objectid";

}

void
ObjectIndex::setAtrribLocation(int value)
{
    if ( _index.size() == 0 )
    {
        _attribLocation = value;
    } 
    else
    {
        OE_WARN << LC << "Illgeal: Cannot change the attrib location once index is in use.\n";
    }
}

void
ObjectIndex::setAttribUniformName(const std::string& name)
{
    if ( _index.size() == 0 )
    {
        _attribUniformName = name;
    }
    else
    {
        OE_WARN << LC << "Illgeal: Cannot change the attrib uniform name once index is in use.\n";
    }
}

ObjectID
ObjectIndex::insert(osg::Referenced* object)
{
    Threading::ScopedMutexLock excl( _mutex );
    return insertImpl( object );
}

ObjectID
ObjectIndex::insertImpl(osg::Referenced* object)
{
    ReverseIndexMap::iterator i = _reverseIndex.find( object );
    if ( i != _reverseIndex.end() )
    {
        return i->second;
    }

    // not found; need to new ObjectID.
    ObjectID id = ++_idGen; // atomic
    _index[id] = object;
    _reverseIndex[object] = id;

    OE_DEBUG << "Insert " << id << "; size = " << _index.size() << "\n";
    return id;
}

osg::Referenced*
ObjectIndex::getImpl(ObjectID id) const
{
    // assume the mutex is locked
    IndexMap::const_iterator i = _index.find(id);
    return i != _index.end() ? i->second.get() : 0L;
}

void
ObjectIndex::remove(ObjectID id)
{
    Threading::ScopedMutexLock excl(_mutex);
    removeImpl(id);
}

void
ObjectIndex::removeImpl(ObjectID id)
{
    // internal - assume mutex is locked
    IndexMap::iterator i = _index.find(id);
    if ( i != _index.end() )
    {
        _reverseIndex.erase( i->second.get() );
        _index.erase( i );
         OE_DEBUG << "Remove " << id << "; size = " << _index.size() << "\n";
    }
}

ObjectID
ObjectIndex::tagDrawable(osg::Drawable* drawable, osg::Referenced* object)
{
    Threading::ScopedMutexLock lock(_mutex);
    ObjectID oid = insertImpl(object);
    tagDrawable(drawable, oid);
    return oid;
}

void
ObjectIndex::tagDrawable(osg::Drawable* drawable, ObjectID id) const
{
    if ( drawable == 0L )
        return;

    osg::Geometry* geom = drawable->asGeometry();
    if ( !geom )
        return;

    // add a new integer attributer to store the feautre ID per vertex.
    osg::UIntArray* ids = new osg::UIntArray();
    ids->setPreserveDataType(true);
    geom->setVertexAttribArray    (_attribLocation, ids);
    geom->setVertexAttribBinding  (_attribLocation, osg::Geometry::BIND_PER_VERTEX);
    geom->setVertexAttribNormalize(_attribLocation, false);

    // The tag is actually FeatureID + 1, to preserve "0" as an "empty" value.
    // TODO: use a ObjectID generator and mapping instead.
    ids->assign( geom->getVertexArray()->getNumElements(), (unsigned)id );
}

namespace
{
    struct FindAndTagDrawables : public osg::NodeVisitor
    {
        FindAndTagDrawables(const ObjectIndex* index, ObjectID id) : _index(index), _id(id)
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
        }

        void apply(osg::Geode& geode)
        {
            for(unsigned i=0; i<geode.getNumDrawables(); ++i)
            {
                _index->tagDrawable( geode.getDrawable(i), _id );
            }
            traverse( geode );
        }

        const ObjectIndex* _index;
        ObjectID           _id;
    };
}

ObjectID
ObjectIndex::tagAllDrawables(osg::Node* node, osg::Referenced* object)
{
    Threading::ScopedMutexLock lock(_mutex);
    ObjectID oid = insertImpl(object);
    tagAllDrawables(node, oid);
    return oid;
}

void
ObjectIndex::tagAllDrawables(osg::Node* node, ObjectID id) const
{
    if ( node )
    {
        FindAndTagDrawables visitor(this, id);
        node->accept( visitor );
    }
}

ObjectID
ObjectIndex::tagNode(osg::Node* node, osg::Referenced* object)
{
    Threading::ScopedMutexLock lock(_mutex);
    ObjectID oid = insertImpl(object);
    tagNode(node, oid);
    return oid;
}

void
ObjectIndex::tagNode(osg::Node* node, ObjectID id) const
{
    if ( node )
    {
        osg::StateSet* stateSet = node->getOrCreateStateSet();
        stateSet->addUniform( new osg::Uniform(_attribUniformName.c_str(), (unsigned)id) );
    }
}
