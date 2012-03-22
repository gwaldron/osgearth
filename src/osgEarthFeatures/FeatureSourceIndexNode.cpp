/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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

osg::Node*
FeatureSourceIndex::FeatureDrawSet::createCopy()
{
    osg::Group* group = new osg::Group();

    for( NodeVector::iterator n = _nodes.begin(); n != _nodes.end(); ++n )
    {
        osg::Node* node = *n;
        osg::Node* nodeCopy = osg::clone(node, osg::CopyOp::SHALLOW_COPY);
        osg::Matrix local2world = osg::computeLocalToWorld( node->getParentalNodePaths()[0] );
        if ( !local2world.isIdentity() )
        {
            osg::MatrixTransform* xform = new osg::MatrixTransform(local2world);
            xform->addChild( nodeCopy );
            group->addChild( xform );
        }
        else
        {
            group->addChild( nodeCopy );
        }
    }

    osg::Geode* geode = 0L;
    for( PrimitiveSetGroups::iterator p = _primSetGroups.begin(); p != _primSetGroups.end(); ++p )
    {
        osg::Drawable* d = p->first;
        const PrimitiveSetList& psets = p->second;
        if ( psets.size() > 0 )
        {        
            osg::Geometry* featureGeom = d->asGeometry();

            if ( !geode )
            {
                geode = new osg::Geode();
            }

            osg::Geometry* copiedGeom = new osg::Geometry( *featureGeom, osg::CopyOp::SHALLOW_COPY );
            copiedGeom->setPrimitiveSetList( psets );

            geode->addDrawable( copiedGeom );

            // include a matrix transform if necessary:
            osg::Matrix local2world = osg::computeLocalToWorld( featureGeom->getParent(0)->getParentalNodePaths()[0] );
            if ( !local2world.isIdentity() )
            {
                osg::MatrixTransform* xform = new osg::MatrixTransform(local2world);
                xform->addChild( geode );
                group->addChild( xform );
            }
        }
    }

    return group;
}

//-----------------------------------------------------------------------------

FeatureSourceIndexNode::Collect::Collect( FeatureIDDrawSetMap& index ) :
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
_index          ( index ),
_psets          ( 0 )
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
        drawSet._nodes.push_back( &node );
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
        drawSet._nodes.push_back( &geode );
    }
    else
    {
        for( unsigned i = 0; i < geode.getNumDrawables(); ++i )
        {
            osg::Geometry* geom = dynamic_cast<osg::Geometry*>( geode.getDrawable(i) );
            if ( geom )
            {
                osg::Geometry::PrimitiveSetList& psets = geom->getPrimitiveSetList();
                for( unsigned p = 0; p < psets.size(); ++p )
                {
                    osg::PrimitiveSet* pset = psets[p];
                    RefFeatureID* fid = dynamic_cast<RefFeatureID*>( pset->getUserData() );
                    if ( fid )
                    {
                        FeatureDrawSet& drawSet = _index[*fid];
                        drawSet._primSetGroups[geom].push_back( pset );
                        _psets++;
                    }
                }
            }
        }
    }

    // NO traverse.
}

//-----------------------------------------------------------------------------

FeatureSourceIndexNode::FeatureSourceIndexNode(FeatureSource* featureSource) : 
_featureSource( featureSource )
{
    //nop
}


// Rebuilds the feature index based on all the tagged primitive sets found in a graph
void
FeatureSourceIndexNode::reindex()
{
    _drawSets.clear();

    Collect c(_drawSets);
    this->accept( c );

    OE_DEBUG << LC << "Reindexed; draw sets = " << _drawSets.size() << std::endl;
}


// Tags all the primitive sets in a Drawable with the specified FeatureID
void
FeatureSourceIndexNode::tagPrimitiveSets(osg::Drawable* drawable, FeatureID fid) const
{
    if ( drawable == 0L )
        return;

    osg::Geometry* geom = drawable->asGeometry();
    if ( !geom )
        return;

    RefFeatureID* rfid = 0L;

    PrimitiveSetList& plist = geom->getPrimitiveSetList();
    for( PrimitiveSetList::iterator p = plist.begin(); p != plist.end(); ++p )
    {
        if ( !rfid )
            rfid = new RefFeatureID(fid);

        p->get()->setUserData( rfid );
    }
}


void
FeatureSourceIndexNode::tagNode( osg::Node* node, FeatureID fid ) const
{
    node->setUserData( new RefFeatureID(fid) );
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
FeatureSourceIndexNode::getFID(osg::Drawable* drawable, int primIndex, FeatureID& output) const
{
    if ( drawable == 0L || primIndex < 0 )
        return false;

    for( FeatureIDDrawSetMap::const_iterator i = _drawSets.begin(); i != _drawSets.end(); ++i )
    {
        const FeatureDrawSet& drawSet = i->second;
        PrimitiveSetGroups::const_iterator d = drawSet._primSetGroups.find(drawable);
        if ( d != drawSet._primSetGroups.end() )
        {
            const osg::Geometry* geom = drawable->asGeometry();
            if ( geom )
            {
                const PrimitiveSetList& geomPrimSets = geom->getPrimitiveSetList();

                unsigned encounteredPrims = 0;
                for( PrimitiveSetList::const_iterator p = geomPrimSets.begin(); p != geomPrimSets.end(); ++p )
                {
                    const osg::PrimitiveSet* pset = p->get();
                    unsigned numPrims = pset->getNumPrimitives();
                    encounteredPrims += numPrims;

                    if ( encounteredPrims > (unsigned)primIndex )
                    {
                        const RefFeatureID* fid = dynamic_cast<const RefFeatureID*>( pset->getUserData() );
                        if ( fid )
                        {
                            output = *fid;
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



FeatureSourceIndexNode::FeatureDrawSet&
FeatureSourceIndexNode::getDrawSet(const FeatureID& fid )
{
    static FeatureDrawSet s_empty;

    FeatureIDDrawSetMap::iterator i = _drawSets.find(fid);
    return i != _drawSets.end() ? i->second : s_empty;
}
