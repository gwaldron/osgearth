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

#include <osgEarthUtil/FeatureDraggerTool>
#include <osgViewer/View>

#define LC "[FeatureDraggerTool] "

using namespace osgEarth;
using namespace osgEarth::Util;

//-----------------------------------------------------------------------

#if 0
namespace
{
    // Extracts the geometry from a draw set, and re-centers it around the world
    // centroid and puts it in the output node. out_local2world is a matrix that
    // will place it exactly where it was originally in world coordinates.
    void extractLocalizedCopy(FeatureSourceIndexNode::FeatureDrawSet& ds,
                              osg::ref_ptr<osg::Node>&                out_node,
                              osg::Matrixd&                           out_local2world)
    {
        std::vector<osg::Vec3d> centroids;

        osg::Group* group = new osg::Group();

#if 0 // figure out nodes later. we'll need to discard transforms here, or collect geodes or something.
        for( NodeVector::iterator n = ds._nodes.begin(); n != ds._nodes.end(); ++n )
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
#endif

        // a geode that will hold the cloned drawables
        osg::Geode* geode = 0L;

        for( PrimitiveSetGroups::iterator p = _primSetGroups.begin(); p != _primSetGroups.end(); ++p )
        {
            osg::Drawable* d = p->first;
            const PrimitiveSetList& psets = p->second;
            if ( psets.size() > 0 )
            {
                osg::Geometry* featureGeom = d->asGeometry();

                if ( !geode )
                    geode = new osg::Geode();

                // make a shallow copy - i.e., don't copy the actual buffer objects.
                osg::Geometry* copiedGeom = new osg::Geometry( *featureGeom, osg::CopyOp::SHALLOW_COPY );

                // replace the primset list with the subset found in the draw set.
                copiedGeom->setPrimitiveSetList( psets );

                geode->addDrawable( copiedGeom );


                //TODO: LEFT OFF HERE...

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
}
#endif

//-----------------------------------------------------------------------

FeatureDraggerTool::FeatureDraggerTool(MapNode*          mapNode,
                                       FeatureQueryTool* queryTool ) :
_mapNode( mapNode )
{
    if ( queryTool )
    {
        // install this object as a query callback so we will receive messages.
        queryTool->addCallback( this );
    }
}


void 
FeatureDraggerTool::onHit( FeatureSourceIndexNode* index, FeatureID fid, const EventArgs& args )
{
    // delete an existing dragger first.
    if ( _dragger.valid() )
    {
        _dragger->getParent(0)->removeChild(_dragger.get());
        _dragger = 0L;
    }

    // extract the "selected" feature model from the scene graph.
    FeatureSourceIndex::FeatureDrawSet& drawSet = index->getDrawSet( fid );
    if ( !drawSet.empty() )
    {
        osg::Node* node = 0L; //TODO...drawSet->createCopy();
        if ( node )
        {
            _dragger = new Dragger( _mapNode );
            _dragger->addChild( node );
        }
    }
}


void 
FeatureDraggerTool::onMiss( const EventArgs& args )
{
    //TODO
}


bool
FeatureDraggerTool::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    bool handled = false;

    if ( _dragger.valid() )
    {
        if ( ea.getEventType() == ea.MOVE )
        {
            //todo            
        }
        else if ( ea.getEventType() == ea.RELEASE )
        {
            handled = false; // let others release as well
        }
    }

    return handled;
}
