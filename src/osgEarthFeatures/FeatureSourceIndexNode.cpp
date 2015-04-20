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
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarth/Registry>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;

#define LC "[FeatureSourceIndexNode] "

// for testing:
//#undef  OE_DEBUG
//#define OE_DEBUG OE_INFO

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

FeatureSourceIndexNode::FeatureSourceIndexNode(FeatureSource*                   featureSource, 
                                               ObjectIndex*                     index,
                                               const FeatureSourceIndexOptions& options) :
_featureSource  ( featureSource ), 
_masterIndex    ( index ),
_options        ( options )
{
    //nop
}

FeatureSourceIndexNode::~FeatureSourceIndexNode()
{
    if ( _masterIndex.valid() && !_oids.empty() )
    {
        _masterIndex->remove( _oids.begin(), _oids.end() );
        _oids.clear();
    }
}

ObjectID
FeatureSourceIndexNode::tagDrawable(osg::Drawable* drawable, Feature* feature)
{
    ObjectID oid = _masterIndex->tagDrawable(drawable, feature);
    _oids.insert( oid );
    return oid;
}

ObjectID
FeatureSourceIndexNode::tagAllDrawables(osg::Node* node, Feature* feature)
{
    ObjectID oid = _masterIndex->tagAllDrawables(node, feature);
    _oids.insert( oid );
    return oid;
}

ObjectID
FeatureSourceIndexNode::tagNode(osg::Node* node, Feature* feature)
{
    ObjectID oid = _masterIndex->tagNode(node, feature);
    _oids.insert( oid );
    return oid;
}

bool
FeatureSourceIndexNode::getAllFIDs(std::vector<FeatureID>& output) const
{
    if ( !_masterIndex.valid() )
        return false;

    for(std::set<ObjectID>::const_iterator oid = _oids.begin(); oid != _oids.end(); ++oid)
    {
        Feature* feature = _masterIndex->get<Feature>( *oid );
        if ( feature )
            output.push_back( feature->getFID() );
    }

    return true;
}

bool
FeatureSourceIndexNode::getFID(osg::Drawable* drawable, int vertIndex, FeatureID& output) const
{
    if ( drawable == 0L || vertIndex < 0 || !_masterIndex.valid() )
        return false;

    osg::Geometry* geom = drawable->asGeometry();
    if ( geom == 0L )
        return false;

    int slot = Registry::objectIndex()->getAttribLocation();
    osg::UIntArray* ids = dynamic_cast<osg::UIntArray*>( geom->getVertexAttribArray(slot) );
    if ( ids == 0L )
        return false;
    
    ObjectID oid = (*ids)[vertIndex];

    Feature* feature = _masterIndex->get<Feature>( oid );
    if ( feature )
        output = feature->getFID();

    return feature != 0L;
}
