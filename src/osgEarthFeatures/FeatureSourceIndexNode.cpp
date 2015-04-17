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
#include <osgEarth/ImageUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osg/MatrixTransform>
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

FeatureSourceIndexNode::Collect::Collect(FeatureIDDrawSetMap& index, int idAttrArraySlot) :
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
_index          ( index ),
_psets          ( 0 ),
_idAttrArraySlot( idAttrArraySlot )
{
    _index.clear();
}

void
FeatureSourceIndexNode::Collect::apply( osg::Node& node )
{
    RefFeatureID* fid = dynamic_cast<RefFeatureID*>( node.getUserData() );
    if ( fid )
    {
        FeatureDrawSet& drawSet = _index[*fid];
        drawSet.nodes().push_back( &node );
    }
    traverse(node);
}

void
FeatureSourceIndexNode::Collect::apply( osg::Geode& geode )
{
    RefFeatureID* fid = dynamic_cast<RefFeatureID*>( geode.getUserData() );
    if ( fid )
    {
        FeatureDrawSet& drawSet = _index[*fid];
        drawSet.nodes().push_back( &geode );
    }
    else
    {
        for( unsigned i = 0; i < geode.getNumDrawables(); ++i )
        {
            osg::Geometry* geom = dynamic_cast<osg::Geometry*>( geode.getDrawable(i) );
            if ( geom )
            {
                osg::IntArray* ids = dynamic_cast<osg::IntArray*>(geom->getVertexAttribArray(_idAttrArraySlot));

                osg::Geometry::PrimitiveSetList& psets = geom->getPrimitiveSetList();
                for( unsigned p = 0; p < psets.size(); ++p )
                {
                    osg::PrimitiveSet* pset = psets[p];

                    // first check for user data:
                    RefFeatureID* fid = dynamic_cast<RefFeatureID*>( pset->getUserData() );
                    if ( fid )
                    {
                        FeatureDrawSet& drawSet = _index[*fid];
                        drawSet.getOrCreateSlice(geom).push_back(pset);
                        _psets++;
                    }

                    // failing that, check for attribution:
                    else if ( ids )
                    {
                        osg::DrawElements* de = pset->getDrawElements();
                        if ( de && de->getNumIndices() > 0 )
                        {
                            std::set<FeatureID> fidsVisitedInThisPSet;

                            for(unsigned i = 0; i < de->getNumIndices(); ++i)
                            {
                                int vi = de->getElement(i);
                                if ( vi < (int)ids->getNumElements() )
                                {
                                    FeatureID fid( (*ids)[vi] );

                                    if ( fidsVisitedInThisPSet.find(fid) == fidsVisitedInThisPSet.end() )
                                    {
                                        FeatureDrawSet& drawSet = _index[fid];
                                        drawSet.getOrCreateSlice(geom).push_back(pset);
                                        fidsVisitedInThisPSet.insert(fid);
                                        _psets++;
                                    }
                                }
                            }
                        }
                        else
                        {
                            OE_WARN << LC << "TODO: try DrawArrays\n";
                        }
                    }
                }
            }
        }
    }

    // NO traverse.
}

//-----------------------------------------------------------------------------

const int FeatureSourceIndexNode::IndexAttrLocation = osg::Drawable::SECONDARY_COLORS;


FeatureSourceIndexNode::FeatureSourceIndexNode(FeatureSource*                   featureSource, 
                                               ObjectIndex*                     index,
                                               const FeatureSourceIndexOptions& options) :
_featureSource  ( featureSource ), 
_masterIndex    ( index ),
_options        ( options ),
_idAttrArraySlot( IndexAttrLocation )
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

// Rebuilds the feature index based on all the tagged primitive sets found in a graph
void
FeatureSourceIndexNode::reindex()
{
    _drawSets.clear();

    Collect c(_drawSets, _idAttrArraySlot);
    this->accept( c );

    OE_DEBUG << LC << "Reindexed; draw sets = " << _drawSets.size() << std::endl;
}

// Tags the vertex array with the specified FeatureID.
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
    output.reserve( _drawSets.size() );
    output.clear();
    for(FeatureIDDrawSetMap::const_iterator i = _drawSets.begin(); i != _drawSets.end(); ++i )
    {
        output.push_back( i->first );
    }
    return true;
}

bool
FeatureSourceIndexNode::getFID(osg::Drawable* drawable, int vertIndex, FeatureID& output) const
{
    if ( drawable == 0L || vertIndex < 0 )
        return false;

    osg::Geometry* geom = drawable->asGeometry();
    if ( geom == 0L )
        return false;

    osg::IntArray* ids = dynamic_cast<osg::IntArray*>( geom->getVertexAttribArray(_idAttrArraySlot) );
    if ( ids == 0L )
        return false;
    
    output = (*ids)[vertIndex];
    return true;
}

FeatureDrawSet&
FeatureSourceIndexNode::getDrawSet(const FeatureID& fid )
{
    static FeatureDrawSet s_empty;

    FeatureIDDrawSetMap::iterator i = _drawSets.find(fid);
    return i != _drawSets.end() ? i->second : s_empty;
}


bool
FeatureSourceIndexNode::getFeature(const FeatureID& fid, const Feature*& output) const
{
    if ( _options.embedFeatures() == true )
    {
        FeatureMap::const_iterator f = _features.find(fid);

        if(f != _features.end())
        {
            output = f->second.get();
            return output != 0L;
        }
    }
    else if ( _featureSource.valid() && _featureSource->supportsGetFeature() )
    {
        output = _featureSource->getFeature( fid );
        return output != 0L;
    }

    return false;
}
