/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarth/Registry>
#include <osgEarth/NodeUtils>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;

// for testing:
//#undef  OE_DEBUG
//#define OE_DEBUG OE_INFO

namespace
{
    // nifty template to iterate over a map's keys
    template<typename T>
    struct KeyIter : public std::iterator<std::input_iterator_tag, typename T::value_type>
    {
		typename T::const_iterator it;
        KeyIter(typename T::const_iterator i) : it(i) { }
		KeyIter &operator++() {	++it; return *this;	}
		KeyIter operator++(int) { KeyIter t = *this; ++*this; return t; }

		typename T::key_type const *operator->() const { return &(it->first); }
		typename T::key_type const &operator*() const {	return it->first; }
		friend bool operator==(KeyIter const &a, KeyIter const &b) { return a.it == b.it; }
		friend bool operator!=(KeyIter const &a, KeyIter const &b) { return a.it != b.it; }
    };
}

//-----------------------------------------------------------------------------


FeatureSourceIndexOptions::FeatureSourceIndexOptions(const Config& conf) :
_enabled      ( true ),
_embedFeatures( false )
{
    conf.get( "enabled",        _enabled );
    conf.get( "embed_features", _embedFeatures );
}

Config
FeatureSourceIndexOptions::getConfig() const
{
    Config conf("feature_indexing");
    conf.set( "enabled",        _enabled );
    conf.set( "embed_features", _embedFeatures );
    return conf;
}

//-----------------------------------------------------------------------------

#undef  LC
#define LC "[FeatureSourceIndexNode] "

FeatureSourceIndexNode::FeatureSourceIndexNode()
{
    //nop
}

FeatureSourceIndexNode::FeatureSourceIndexNode(const FeatureSourceIndexNode& rhs, const osg::CopyOp& copy) :
osg::Group(rhs, copy)
{
    _index = rhs._index.get();
    _fids  = rhs._fids;
}

FeatureSourceIndexNode::FeatureSourceIndexNode(FeatureSourceIndex* index) :
_index( index )
{
    //nop
}

FeatureSourceIndexNode::~FeatureSourceIndexNode()
{
    if ( _index.valid() )
    {
        // must copy and clear the original list first to dereference the RefIDPair instances.
        std::set<FeatureID> fidsToRemove;
        fidsToRemove.insert(KeyIter<FIDMap>(_fids.begin()), KeyIter<FIDMap>(_fids.end()));
        _fids.clear();

        OE_DEBUG << LC << "Removing " << fidsToRemove.size() << " fids\n";
        _index->removeFIDs( fidsToRemove.begin(), fidsToRemove.end() );
    }
}

ObjectID
FeatureSourceIndexNode::tagDrawable(osg::Drawable* drawable, Feature* feature)
{
    if ( !feature || !_index.valid() ) return OSGEARTH_OBJECTID_EMPTY;
    RefIDPair* r = _index->tagDrawable( drawable, feature );
    if ( r ) _fids[ feature->getFID() ] = r;
    return r ? r->_oid : OSGEARTH_OBJECTID_EMPTY;
}

ObjectID
FeatureSourceIndexNode::tagAllDrawables(osg::Node* node, Feature* feature)
{
    if ( !feature || !_index.valid() ) return OSGEARTH_OBJECTID_EMPTY;
    RefIDPair* r = _index->tagAllDrawables( node, feature );
    if ( r ) _fids[ feature->getFID() ] = r;
    return r ? r->_oid : OSGEARTH_OBJECTID_EMPTY;
}

ObjectID
FeatureSourceIndexNode::tagNode(osg::Node* node, Feature* feature)
{
    if ( !feature || !_index.valid() ) return OSGEARTH_OBJECTID_EMPTY;
    RefIDPair* r = _index->tagNode( node, feature );
    if ( r ) _fids[ feature->getFID() ] = r;
    return r ? r->_oid : OSGEARTH_OBJECTID_EMPTY;
}

bool
FeatureSourceIndexNode::getAllFIDs(std::vector<FeatureID>& output) const
{
    KeyIter<FIDMap> start( _fids.begin() );
    KeyIter<FIDMap> end  ( _fids.end() );
    for(KeyIter<FIDMap> i = start; i != end; ++i )
    {
        output.push_back( *i );
    }

    return true;
}

void
FeatureSourceIndexNode::setFIDMap(const FeatureSourceIndexNode::FIDMap& fids)
{
    _fids = fids;
    //todo
}

namespace
{
    /** Visitor that finds FeatureSourceIndexNodes, assigns their index, and 
     *  reconstitutes their object ID maps. */
    struct Reconstitute : public osg::NodeVisitor
    {
        FeatureSourceIndex* _index;
        std::map<ObjectID,ObjectID> _oldToNew;

        Reconstitute(FeatureSourceIndex* index) :
            _index(index)
        {
            setNodeMaskOverride(~0);
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
        }

        void apply(osg::Node& node)
        {
            FeatureSourceIndexNode* indexNode = dynamic_cast<FeatureSourceIndexNode*>(&node);
            if (indexNode)
            {
                //OE_INFO << LC << "Reconstituting index...\n";
                indexNode->setIndex(_index);
                indexNode->reIndex(_oldToNew);
            }
            traverse(node);
        }
    };

    /** Visitor that re-indexes objects after deserialization. */
    struct ReIndex : public osg::NodeVisitor
    {
        FeatureSourceIndexNode*        _indexNode;
        FeatureSourceIndexNode::FIDMap _newFIDMap;
        std::map<ObjectID,ObjectID>&   _oldToNew;

        ReIndex(FeatureSourceIndexNode* indexNode, std::map<ObjectID,ObjectID>& oldToNew) :
            _indexNode(indexNode), _oldToNew(oldToNew)
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
        }

        void apply(osg::Node& node)
        {
            _indexNode->reIndexNode(&node, _oldToNew, _newFIDMap);
            traverse(node);
        }

        void apply(osg::Geode& geode)
        {
            _indexNode->reIndexNode(&geode, _oldToNew, _newFIDMap);
            for (unsigned i = 0; i < geode.getNumDrawables(); ++i)
            {
                _indexNode->reIndexDrawable(geode.getDrawable(i), _oldToNew, _newFIDMap);
            }
            traverse(geode);
        }
    };
}

void
FeatureSourceIndexNode::reconstitute(osg::Node* graph, FeatureSourceIndex* index)
{
    if ( !graph || !index )
    {
        OE_WARN << LC << "INTERNAL ERROR cannot call reconsitute with null graph or null index\n";
        return;
    }

    Reconstitute visitor(index);
    graph->accept(visitor);
}

void
FeatureSourceIndexNode::reIndex(std::map<ObjectID,ObjectID>& oidmappings)
{
    ReIndex visitor(this, oidmappings);
    this->accept(visitor);
    _fids = visitor._newFIDMap;
    //OE_INFO << LC << "Reindexed " << _fids.size() << " mappings\n";
}

void
FeatureSourceIndexNode::reIndexDrawable(osg::Drawable* drawable, std::map<ObjectID,ObjectID>& oldNew, FIDMap& newFIDMap)
{
    if ( !drawable || !_index.valid() ) return;

    _index->update(drawable, oldNew, _fids, newFIDMap);
}

void
FeatureSourceIndexNode::reIndexNode(osg::Node* node, std::map<ObjectID,ObjectID>& oldNew, FIDMap& newFIDMap)
{
    if (!node || !_index.valid()) return;

    _index->update(node, oldNew, _fids, newFIDMap);
}

FeatureSourceIndexNode* FeatureSourceIndexNode::get(osg::Node* graph)
{
    return graph ? osgEarth::findTopMostNodeOfType<FeatureSourceIndexNode>(graph) : 0L;
}

//-----------------------------------------------------------------------------

#undef  LC
#define LC "[FeatureSourceIndex Serializer] "

// OSG SERIALIZER for FeatureSourceIndexNode
#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

namespace osgEarth { namespace Serializers { namespace FeatureSourceIndexNodeClass
{
    using namespace osgEarth::Features;

    bool checkFIDMap(const FeatureSourceIndexNode& node)
    {
        return !node.getFIDMap().empty();
    }

    bool writeFIDMap(osgDB::OutputStream& os, const FeatureSourceIndexNode& node)
    {
        const FeatureSourceIndexNode::FIDMap& fids = node.getFIDMap();

        os.writeSize(fids.size());
        os << os.BEGIN_BRACKET << std::endl;
        {
            for (FeatureSourceIndexNode::FIDMap::const_iterator i = fids.begin(); i != fids.end(); ++i)
            {
                const RefIDPair* idPair = i->second.get();
                os << idPair->_fid << idPair->_oid;
            }
        }
        os << os.END_BRACKET << std::endl;

        return true;
    }

    bool readFIDMap(osgDB::InputStream& is, FeatureSourceIndexNode& node)
    {
        FeatureSourceIndexNode::FIDMap fids;
        FeatureID fid;
        ObjectID oid;

        unsigned size = is.readSize();
        is >> is.BEGIN_BRACKET;
        {
            for (unsigned i=0; i<size; ++i)
            {
                is >> fid >> oid;
                fids[fid] = new RefIDPair(fid, oid);
            }
        }
        is >> is.END_BRACKET;
        node.setFIDMap(fids);

        return true;
    }

    REGISTER_OBJECT_WRAPPER(
        FeatureSourceIndexNode,
        new osgEarth::Features::FeatureSourceIndexNode,
        osgEarth::Features::FeatureSourceIndexNode,
        "osg::Object osg::Node osg::Group osgEarth::Features::FeatureSourceIndexNode")
    {
        ADD_USER_SERIALIZER(FIDMap);
    }

} } }

//-----------------------------------------------------------------------------

#undef  LC
#define LC "[FeatureSourceIndex] "

FeatureSourceIndex::FeatureSourceIndex(FeatureSource* featureSource, 
                                       ObjectIndex*   index,
                                       const FeatureSourceIndexOptions& options) :
_featureSource  ( featureSource ), 
_masterIndex    ( index ),
_options        ( options )
{
    _embed = 
        _options.embedFeatures() == true ||
        featureSource == 0L ||
        featureSource->supportsGetFeature() == false;
}

FeatureSourceIndex::~FeatureSourceIndex()
{
    if ( _masterIndex.valid() && !_oids.empty() )
    {
        // remove all OIDs from the master index.
        _masterIndex->remove( KeyIter<OIDMap>(_oids.begin()), KeyIter<OIDMap>(_oids.end()) );
    }

    _oids.clear();
    _fids.clear();
    _embeddedFeatures.clear();
}

RefIDPair*
FeatureSourceIndex::tagDrawable(osg::Drawable* drawable, Feature* feature)
{
    if ( !feature ) return 0L;

    Threading::ScopedMutexLock lock(_mutex);
    
    RefIDPair* p = 0L;
    FeatureID fid = feature->getFID();

    FIDMap::const_iterator f = _fids.find( fid );
    if ( f != _fids.end() )
    {
        ObjectID oid = f->second->_oid;
        _masterIndex->tagDrawable( drawable, oid );
        p = f->second.get();
    }
    else
    {
        ObjectID oid = _masterIndex->tagDrawable( drawable, this );
        p = new RefIDPair( fid, oid );
        _fids[fid] = p;
        _oids[oid] = fid;
    
        if ( _embed )
        {
            _embeddedFeatures[fid] = feature;
        }
    }

    return p;
}

RefIDPair*
FeatureSourceIndex::tagAllDrawables(osg::Node* node, Feature* feature)
{
    if ( !feature ) return 0L;

    Threading::ScopedMutexLock lock(_mutex);
    
    RefIDPair* p = 0L;
    FeatureID fid = feature->getFID();

    FIDMap::const_iterator f = _fids.find( fid );
    if ( f != _fids.end() )
    {
        ObjectID oid = f->second->_oid;
        _masterIndex->tagAllDrawables( node, oid );
        p = f->second.get();
    }
    else
    {
        ObjectID oid = _masterIndex->tagAllDrawables( node, this );
        p = new RefIDPair( fid, oid );
        _fids[fid] = p;
        _oids[oid] = fid;
    
        if ( _embed )
        {
            _embeddedFeatures[fid] = feature;
        }
    }

    return p;
}

RefIDPair*
FeatureSourceIndex::tagNode(osg::Node* node, Feature* feature)
{
    if ( !feature ) return 0L;

    Threading::ScopedMutexLock lock(_mutex);
    
    RefIDPair* p = 0L;
    FeatureID fid = feature->getFID();
    ObjectID oid;

    FIDMap::const_iterator f = _fids.find( fid );
    if ( f != _fids.end() )
    {
        oid = f->second->_oid;
        _masterIndex->tagNode( node, oid );
        p = f->second.get();
    }
    else
    {
        oid = _masterIndex->tagNode( node, this );
        p = new RefIDPair( fid, oid );
        _fids[fid] = p;
        _oids[oid] = fid;
    
        if ( _embed )
        {
            _embeddedFeatures[fid] = feature;
        }
    }

    OE_DEBUG << LC << "Tagging feature ID = " << fid << " => " << oid << " (" << feature->getString("name") << ")\n";

    return p;
}

Feature*
FeatureSourceIndex::getFeature(ObjectID oid) const
{
    Feature* feature = 0L;
    Threading::ScopedMutexLock lock(_mutex);
    OIDMap::const_iterator i = _oids.find( oid );
    if ( i != _oids.end() )
    {
        FeatureID fid = i->second;

        if ( _embed )
        {
            FeatureMap::const_iterator j = _embeddedFeatures.find( fid );
            feature = j != _embeddedFeatures.end() ? j->second.get() : 0L;
        }
        else if ( _featureSource.valid() && _featureSource->supportsGetFeature() )
        {
            feature = _featureSource->getFeature( fid );
        }
    }
    return feature;
}

ObjectID
FeatureSourceIndex::getObjectID(FeatureID fid) const
{
    Threading::ScopedMutexLock lock(_mutex);
    FIDMap::const_iterator i = _fids.find(fid);
    if ( i != _fids.end() )
        return i->second->_oid;
    else
        return OSGEARTH_OBJECTID_EMPTY;
}

// When Feature index data is deserialized, the old serialized ObjectIDs are 
// no longer valid. This method will re-install the mappings in the master index
// and write new local mappings with new ObjectIDs.
void
FeatureSourceIndex::update(osg::Drawable* drawable, std::map<ObjectID,ObjectID>& oldToNew, const FIDMap& oldFIDMap, FIDMap& newFIDMap)
{
    unsigned count = 0;
    if (_masterIndex->updateObjectIDs(drawable, oldToNew, this))
    {
        for (std::map<ObjectID, ObjectID>::const_iterator i = oldToNew.begin(); i != oldToNew.end(); ++i)
        {
            const ObjectID& oldoid = i->first;
            const ObjectID& newoid = i->second;

            for (FIDMap::const_iterator j = oldFIDMap.begin(); j != oldFIDMap.end(); ++j)
            {
                const RefIDPair* rip = j->second.get();
                if (rip && rip->_oid == oldoid)
                {
                    RefIDPair* newrip = new RefIDPair(rip->_fid, newoid);
                    _oids[newoid] = rip->_fid;
                    _fids[rip->_fid] = newrip;
                    newFIDMap[rip->_fid] = newrip;
                    ++count;
                }
            }
        }
    }
}

// When Feature index data is deserialized, the old serialized ObjectIDs are 
// no longer valid. This method will re-install the mappings in the master index
// and write new local mappings with new ObjectIDs.
void
FeatureSourceIndex::update(osg::Node* node, std::map<ObjectID,ObjectID>& oldToNew, const FIDMap& oldFIDMap, FIDMap& newFIDMap)
{
    unsigned count = 0;
    if (_masterIndex->updateObjectID(node, oldToNew, this))
    {
        for (std::map<ObjectID, ObjectID>::const_iterator i = oldToNew.begin(); i != oldToNew.end(); ++i)
        {
            const ObjectID& oldoid = i->first;
            const ObjectID& newoid = i->second;

            for (FIDMap::const_iterator j = oldFIDMap.begin(); j != oldFIDMap.end(); ++j)
            {
                const RefIDPair* rip = j->second.get();
                if (rip && rip->_oid == oldoid)
                {
                    RefIDPair* newrip = new RefIDPair(rip->_fid, newoid);
                    _oids[newoid] = rip->_fid;
                    _fids[rip->_fid] = newrip;
                    newFIDMap[rip->_fid] = newrip;
                    ++count;
                }
            }
        }
    }
}
