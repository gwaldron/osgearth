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
_embedFeatures( false )
{
    conf.getIfSet( "embed_features", _embedFeatures );
}

Config
FeatureSourceIndexOptions::getConfig() const
{
    Config conf("feature_indexing");
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
                                if ( vi < ids->getNumElements() )
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
                                               const FeatureSourceIndexOptions& options) :
_featureSource  ( featureSource ), 
_options        ( options ),
_idAttrArraySlot( IndexAttrLocation )
{
    //nop
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
void
FeatureSourceIndexNode::tagGeometry(osg::Drawable* drawable, Feature* feature) const
{
    if ( drawable == 0L )
        return;

    osg::Geometry* geom = drawable->asGeometry();
    if ( !geom )
        return;

    // add a new integer attributer to store the feautre ID per vertex.
    osg::IntArray* ids = new osg::IntArray();
    ids->setPreserveDataType(true);
    geom->setVertexAttribArray    (_idAttrArraySlot, ids);
    geom->setVertexAttribBinding  (_idAttrArraySlot, osg::Geometry::BIND_PER_VERTEX);
    geom->setVertexAttribNormalize(_idAttrArraySlot, false);

    ids->assign( geom->getVertexArray()->getNumElements(), (int)feature->getFID() );

    // optionally save the actual feature object in the index.
    if ( _options.embedFeatures() == true )
    {
        _features[feature->getFID()] = feature;
    }
}

void
FeatureSourceIndexNode::tagNode( osg::Node* node, Feature* feature ) const
{
    node->setUserData( new RefFeatureID(feature->getFID()) );

    if ( _options.embedFeatures() == true )
    {
        _features[feature->getFID()] = feature;
    }
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
FeatureSourceIndexNode::getFID(osg::PrimitiveSet* primSet, FeatureID& output) const
{
    const RefFeatureID* fid = dynamic_cast<const RefFeatureID*>( primSet->getUserData() );
    if ( fid )
    {
        output = *fid;
        return true;
    }

    OE_DEBUG << LC << "getFID failed b/c the primSet was not tagged with a RefFeatureID" << std::endl;
    return false;
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

#if 0
bool
FeatureSourceIndexNode::getFID(osg::Drawable* drawable, int primIndex, FeatureID& output) const
{
    if ( drawable == 0L || primIndex < 0 )
        return false;

    for( FeatureIDDrawSetMap::const_iterator i = _drawSets.begin(); i != _drawSets.end(); ++i )
    {
        const FeatureDrawSet& drawSet = i->second;
        FeatureDrawSet::DrawableSlices::const_iterator d = drawSet.slice(drawable);
        if ( d != drawSet.slices().end() )
        {
            const osg::Geometry* geom = drawable->asGeometry();
            if ( geom )
            {
                const osg::Geometry::PrimitiveSetList& geomPrimSets = geom->getPrimitiveSetList();

                unsigned encounteredPrims = 0;
                for( osg::Geometry::PrimitiveSetList::const_iterator p = geomPrimSets.begin(); p != geomPrimSets.end(); ++p )
                {
                    const osg::PrimitiveSet* pset = p->get();
                    unsigned numPrims = pset->getNumPrimitives();
                    encounteredPrims += numPrims;

                    if ( encounteredPrims > (unsigned)primIndex )
                    {
                        // check for a feature ID in the userdata first:
                        const RefFeatureID* fid = dynamic_cast<const RefFeatureID*>( pset->getUserData() );
                        if ( fid )
                        {
                            output = *fid;
                            return true;
                        }

                        // failing that, check for vertex attribution:
                        else
                        {
                            const osg::IntArray* ids = dynamic_cast<const osg::IntArray*>(geom->getVertexAttribArray(_idAttrArraySlot));
                            if ( ids && ids->getNumElements() > 0 )
                            {
                                output = *ids->begin(); // first one.
                                return true;
                            }

                            else
                            {
                                OE_DEBUG << LC << "INTERNAL: found primset, but it's not tagged with a FID" << std::endl;
                                return false;
                            }
                        }
                    }
                }
            }
        }
    }

    // see if we have a node in the path
    for( osg::Node* node = drawable->getParent(0); node != 0L; node = (node->getNumParents()>0?node->getParent(0):0L) )
    {
        RefFeatureID* fid = dynamic_cast<RefFeatureID*>( node->getUserData() );
        if ( fid )
        {
            output = *fid;
            return true;
        }
    }

    return false;
}
#endif



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
    else if ( _featureSource.valid() )
    {
        output = _featureSource->getFeature( fid );
        return output != 0L;
    }

    return false;
}
