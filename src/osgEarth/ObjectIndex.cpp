/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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

#include "ObjectIndex"
#include "Capabilities"
#include "StringUtils"
#include "VirtualProgram"

#include <osg/Geometry>

using namespace osgEarth;

#define LC "[ObjectIndex] "

//#undef OE_DEBUG
//#define OE_DEBUG OE_NOTICE

// Object IDs under this reserved
#define STARTING_OBJECT_ID 10

namespace
{
    const char* indexVertexInit = R"(
        #pragma vp_function oe_index_readObjectID, vertex_model, first

        uniform uint oe_index_objectid_uniform; // override objectid if > 0
        in uint      oe_index_objectid_attr;    // Vertex attribute containing the object ID.
        uint         oe_index_objectid;         // Stage global containing the Object ID.

        void oe_index_readObjectID(inout vec4 vertex)
        {
            if ( oe_index_objectid_uniform > 0u )
                oe_index_objectid = oe_index_objectid_uniform;
            else if ( oe_index_objectid_attr > 0u )
                oe_index_objectid = oe_index_objectid_attr;
            else
                oe_index_objectid = 0u;
        }
)";
}

ObjectIndex::ObjectIndex() :
_idGen( STARTING_OBJECT_ID ),
_mutex("ObjectIndex(OE)")
{
    _attribName     = "oe_index_objectid_attr";
    _attribLocation = osg::Drawable::SECONDARY_COLORS;
    _oidUniformName = "oe_index_objectid_uniform";

    // set up the shader package.
    std::string source = Stringify()
        << "#version " << std::to_string(Capabilities::get().getGLSLVersionInt()) << "\n"
        << indexVertexInit;
    _shaders.add("ObjectIndex.vert.glsl", source);
}

bool
ObjectIndex::loadShaders(VirtualProgram* vp) const
{
    if ( vp )
    {
        _shaders.loadAll( vp );
        vp->addBindAttribLocation( getObjectIDAttribName(), getObjectIDAttribLocation() );
    }
    return vp != 0L;
}

void
ObjectIndex::setObjectIDAtrribLocation(int value)
{
    if ( _index.size() == 0 )
    {
        _attribLocation = value;
    } 
    else
    {
        OE_WARN << LC << "Illegal: Cannot change the attrib location once index is in use.\n";
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
    // internal: assume mutex is locked
    ObjectID id = ++_idGen;
    _index[id] = object;
    OE_DEBUG << LC << "Insert " << id << "; size = " << _index.size() << "\n";
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
    _index.erase( id );
    OE_DEBUG << "Remove " << id << "; size = " << _index.size() << "\n";

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
    ObjectIDArray* ids = new ObjectIDArray();
    ids->setBinding(osg::Array::BIND_PER_VERTEX);
    ids->setNormalize(false);
    geom->setVertexAttribArray(_attribLocation, ids);
    ids->setPreserveDataType(true);

    ids->assign( geom->getVertexArray()->getNumElements(), id );
}

ObjectID
ObjectIndex::tagRange(osg::Drawable* drawable, osg::Referenced* object, unsigned int start, unsigned int count)
{
    Threading::ScopedMutexLock lock(_mutex);
    ObjectID oid = insertImpl(object);
    tagRange(drawable, oid, start, count);
    return oid;
}

void
ObjectIndex::tagRange(osg::Drawable* drawable, ObjectID id, unsigned int start, unsigned int count) const
{
    if (drawable == 0L)
        return;

    osg::Geometry* geom = drawable->asGeometry();
    if (!geom)
        return;

    ObjectIDArray* ids = dynamic_cast<ObjectIDArray*>(geom->getVertexAttribArray(_attribLocation));
    if (!ids)
    {
        // add a new integer attributer to store the feautre ID per vertex.
        ids = new ObjectIDArray();
        ids->setBinding(osg::Array::BIND_PER_VERTEX);
        ids->setNormalize(false);
        geom->setVertexAttribArray(_attribLocation, ids);
        ids->setPreserveDataType(true);
    }

    for (unsigned int i = 0; i < count; ++i)
    {
        (*ids)[start + i] = id;
    }

    ids->dirty();
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

        void apply(osg::Drawable& drawable)
        {
            _index->tagDrawable(&drawable, _id);
            traverse(drawable);
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
        stateSet->addUniform( new osg::Uniform(_oidUniformName.c_str(), id) );
    }
}

bool
ObjectIndex::getObjectIDs(const osg::Drawable* drawable, std::set<ObjectID>& output) const
{
    if (!drawable) return false;
    
    const osg::Geometry* geometry = drawable->asGeometry();
    if (!geometry) return false;

    const ObjectIDArray* oids = dynamic_cast<const ObjectIDArray*>(geometry->getVertexAttribArray(_attribLocation));
    if ( !oids ) return false;
    if (oids->empty()) return false;

    for (ObjectIDArray::const_iterator i = oids->begin(); i != oids->end(); ++i)
        output.insert( *i );

    return true;
}

bool
ObjectIndex::getObjectID(osg::Node* node, ObjectID& output) const
{
    if (!node) return false;

    osg::StateSet* stateSet = node->getStateSet();
    if (!stateSet) return false;

    osg::Uniform* uniform = stateSet->getUniform(_oidUniformName.c_str());
    if ( !uniform ) return false;

    uniform->get(output);
    return true;
}

bool
ObjectIndex::updateObjectIDs(osg::Drawable* drawable,
                             std::unordered_map<ObjectID, ObjectID>& oldNewMap,
                             osg::Referenced* object)
{
    // in a drawable, replaces each OIDs in map.first with the corresponding OID in map.second
    if (!drawable) return false;

    osg::Geometry* geometry = drawable->asGeometry();
    if (!geometry) return false;

    ObjectIDArray* oids = dynamic_cast<ObjectIDArray*>(geometry->getVertexAttribArray(_attribLocation));
    if ( !oids ) return false;
    if (oids->empty()) return false;
    
    for (ObjectIDArray::iterator i = oids->begin(); i != oids->end(); ++i)
    {
        ObjectID newoid;
        std::unordered_map<ObjectID, ObjectID>::iterator k = oldNewMap.find(*i);
        if (k != oldNewMap.end()) {
            newoid = k->second;
        }
        else {
            newoid = insert(object);
            oldNewMap[*i] = newoid;
        }
        *i = newoid;
    }

    oids->dirty();

    return true;
}

bool
ObjectIndex::updateObjectID(osg::Node* node,
                            std::unordered_map<ObjectID, ObjectID>& oldNewMap,
                            osg::Referenced* object)
{
    if (!node) return false;

    osg::StateSet* stateSet = node->getStateSet();
    if (!stateSet) return false;

    osg::Uniform* uniform = stateSet->getUniform(_oidUniformName.c_str());
    if ( !uniform ) return false;

    ObjectID oldoid;
    uniform->get(oldoid);

    ObjectID newoid;
    std::unordered_map<ObjectID, ObjectID>::iterator k = oldNewMap.find(oldoid);
    if (k != oldNewMap.end()) {
        newoid = k->second;
    }
    else {
        newoid = insert(object);
        oldNewMap[oldoid] = newoid;
    }

    uniform->set(newoid);

    return true;
}
