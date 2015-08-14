/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
    struct KeyIter : public T::iterator
    {
        KeyIter() : T::iterator() { }
        KeyIter(typename T::iterator i) : T::iterator(i) { }
        typename T::key_type* operator->() { return (typename T::key_type* const)&(T::iterator::operator->()->first); }
        typename T::key_type operator*() { return T::iterator::operator*().first; }
    };

    template<typename T>
    struct ConstKeyIter : public T::const_iterator
    {
        ConstKeyIter() : T::const_iterator() { }
        ConstKeyIter(typename T::const_iterator i) : T::const_iterator(i) { }
        typename T::key_type* operator->() { return (typename T::key_type* const)&(T::const_iterator::operator->()->first); }
        typename T::key_type operator*() { return T::const_iterator::operator*().first; }
    };
}

//-----------------------------------------------------------------------------


FeatureSourceIndexOptions::FeatureSourceIndexOptions(const Config& conf) :
_enabled      ( true ),
_embedFeatures( false )
{
    conf.getIfSet( "enabled",        _enabled );
    conf.getIfSet( "embed_features", _embedFeatures );
}

Config
FeatureSourceIndexOptions::getConfig() const
{
    Config conf("feature_indexing");
    conf.addIfSet( "enabled",        _enabled );
    conf.addIfSet( "embed_features", _embedFeatures );
    return conf;
}

//-----------------------------------------------------------------------------

#undef  LC
#define LC "[FeatureSourceIndexNode] "

FeatureSourceIndexNode::FeatureSourceIndexNode(FeatureSourceIndex* index) :
_index( index )
{
    if ( !index )
    {
        OE_WARN << LC << "INTERNAL ERROR: created a feature source index node with a NULL index.\n";
    }
}

FeatureSourceIndexNode::~FeatureSourceIndexNode()
{
    if ( _index.valid() )
    {
        // must copy and clear the original list first to dereference the RefIDPair instances.
        std::set<FeatureID> fidsToRemove;
        fidsToRemove.insert( KeyIter<FIDMap>(_fids.begin()), KeyIter<FIDMap>(_fids.end()) );
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
    ConstKeyIter<FIDMap> start( _fids.begin() );
    ConstKeyIter<FIDMap> end  ( _fids.end() );
    for(ConstKeyIter<FIDMap> i = start; i != end; ++i )
    {
        output.push_back( *i );
    }

    return true;
}

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
