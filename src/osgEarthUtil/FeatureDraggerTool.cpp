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
#include <osgEarth/ECEF>
#include <osgViewer/View>

#define LC "[FeatureDraggerTool] "

#define OE_TEST OE_INFO

using namespace osgEarth;
using namespace osgEarth::Util;

//-----------------------------------------------------------------------

namespace
{
    // walks a node path, accumulating the state and storing it in the target.
    osg::StateSet* accumulateStateSet( osg::StateSet* target, const osg::NodePath& path )
    {
        osg::StateSet* s = new osg::StateSet();
        for( osg::NodePath::const_iterator i = path.begin(); i != path.end(); ++i )
        {
            if ( (*i)->getStateSet() )
                s->merge( *(*i)->getStateSet() );
        }

        s->merge( *target );
        return s;
    }


    // Extracts the geometry from a draw set, and re-centers it around the world
    // centroid and puts it in the output node. out_local2world is a matrix that
    // will place it exactly where it was originally in world coordinates.
    void extractLocalizedCopy(FeatureSourceIndexNode::FeatureDrawSet& ds,
                              const osg::Vec3d&                       anchorPointWorld,
                              const SpatialReference*                 mapSRS,
                              osg::ref_ptr<osg::Node>&                out_node,
                              osg::Matrixd&                           out_local2world)
    {
        osg::Group* group = 0L;

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

        // stores the reference frame of each cloned drawable.
        std::vector<osg::Matrixd> local2worlds;

        // clone each of the drawables and place the clones under the new geode.
        for( FeatureSourceIndex::PrimitiveSetGroups::iterator p = ds._primSetGroups.begin(); p != ds._primSetGroups.end(); ++p )
        {
            osg::Drawable* d = p->first;
            const FeatureSourceIndex::PrimitiveSetList& psets = p->second;
            if ( psets.size() > 0 )
            {
                osg::Geometry* featureGeom = d->asGeometry();
                osg::NodePath parentNodePath = featureGeom->getParent(0)->getParentalNodePaths()[0];

                if ( !geode )
                {
                    geode = new osg::Geode();
                }

                // make a shallow copy; we want to share arrays for now.
                osg::Geometry* clonedGeom = new osg::Geometry( *featureGeom, osg::CopyOp::SHALLOW_COPY );
                clonedGeom->setStateSet(accumulateStateSet( clonedGeom->getOrCreateStateSet(), parentNodePath ));

                // replace the primset list with the subset found in the draw set.
                clonedGeom->setPrimitiveSetList( psets );

                // add it to the geode.
                geode->addDrawable( clonedGeom );

                // store the local2world transform for this drawable:
                local2worlds.push_back( osg::computeLocalToWorld(parentNodePath) );
            }
        }

        // next, find a center point.
        if ( geode )
        {
            // use the first l2w as the anchor point. In all likelihood a single feature will
            // exist under a single localizer anyway.
            const osg::Matrixd& local2world = local2worlds[0];

            // figure out the centroid of the selected feature, and then shift the verts
            // to be relative to this new anchor point. We do this by transforming them into
            // work coords and then back into local coords.
            osg::Matrixd local2worldAtAnchorPoint = ECEF::createLocalToWorld( anchorPointWorld );

            // take the inverse, as this will be the new localizer.
            osg::Matrixd world2localAtAnchorPoint;
            world2localAtAnchorPoint.invert( local2worldAtAnchorPoint );

            // for each drawable, shift its verticies so they are relative to the new centroid
            // instead of being relative to the original reference frame.
            for( unsigned i=0; i<geode->getNumDrawables(); ++i )
            {
                osg::Geometry*     geom        = geode->getDrawable(i)->asGeometry();
                const osg::Matrix& local2world = local2worlds[i];

                osg::Vec3Array* newVerts = static_cast<osg::Vec3Array*>(
                    osg::clone( geom->getVertexArray(), osg::CopyOp::DEEP_COPY_ARRAYS ) );

                for( osg::Vec3Array::iterator v = newVerts->begin(); v != newVerts->end(); ++v )
                {
                    osg::Vec3d vert         = *v;                           // single->double precision first
                    osg::Vec3d vertWorld    = vert * local2world;           // bring out to world coords
                    osg::Vec3d vertNewLocal = vertWorld * world2localAtAnchorPoint;  // back into the new local coords.
                    *v = vertNewLocal;
                }

                geom->setVertexArray( newVerts );
            }

            out_node        = geode;
            out_local2world = local2worldAtAnchorPoint;
        }
    }


    struct CustomQueryPredicate : public FeatureQueryTool::InputPredicate
    {
        bool accept( const osgGA::GUIEventAdapter& ea )
        {
            return
                ea.getEventType() == ea.PUSH &&
                (ea.getModKeyMask() & ea.MODKEY_SHIFT) != 0;
        }
    };
}

//-----------------------------------------------------------------------

FeatureDraggerTool::FeatureDraggerTool(MapNode*          mapNode,
                                       FeatureQueryTool* queryTool ) :
_mapNode( mapNode )
{
    if ( queryTool )
    {
        // install this object as a query callback so we will receive messages.
        queryTool->addCallback( this );

        // a custom input predicate so we can be dragging
        queryTool->setInputPredicate( new CustomQueryPredicate );
    }
}


void 
FeatureDraggerTool::onHit( FeatureSourceIndexNode* index, FeatureID fid, const EventArgs& args )
{
    //OE_TEST << LC << "onHit" << std::endl;

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
        // grab the point under the mouse and use this as the anchor.
        osg::Vec3d mouseWorld;
        _mapNode->getTerrain()->getWorldCoordsUnderMouse(args._aa->asView(), args._ea->getX(), args._ea->getY(), mouseWorld);

        // extract the "hit" feature from its draw set into a new draggable node.
        osg::ref_ptr<osg::Node> node;
        osg::Matrixd            local2world;
        extractLocalizedCopy( drawSet, mouseWorld, _mapNode->getMapSRS(), node, local2world );

        if ( node.valid() )
        {
            _dragger = new osg::MatrixTransform( local2world );
            _dragger->addChild( node.get() );

            // stick it somewhere.
            _mapNode->addChild( _dragger.get() );

            //OE_TEST << LC << "Added dragger!" << std::endl;

            //store the HAT so we can apply it as a vertical offset while dragging......
            GeoPoint mapPoint;
            mapPoint.fromWorld( _mapNode->getMapSRS(), local2world.getTrans() );

            double hae;
            _mapNode->getTerrain()->getHeight( mapPoint.x(), mapPoint.y(), 0L, &hae );

            _verticalOffset = mapPoint.z() - hae;

            _lastMouseWorld = mouseWorld;
        }
    }
}


void 
FeatureDraggerTool::onMiss( const EventArgs& args )
{
    //OE_TEST << LC << "onMiss" << std::endl;

    if ( _dragger.valid() )
    {
        _dragger->getParent(0)->removeChild( _dragger.get() );
        _dragger = 0L;
    }
}


bool
FeatureDraggerTool::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    bool handled = false;

    if ( _dragger.valid() )
    {
        if ( ea.getEventType() == ea.DRAG )
        {
            //OE_TEST << LC << "DRAG" << std::endl;

            // get the position under the mouse, and use that as the anchor point for dragging.
            osg::Vec3d mouseWorld;
            if (_mapNode->getTerrain()->getWorldCoordsUnderMouse(aa.asView(), ea.getX(), ea.getY(), mouseWorld) )
            {
                GeoPoint mouseMap;
                _mapNode->getMap()->worldPointToMapPoint(mouseWorld, mouseMap);
                mouseMap.z() += _verticalOffset;

                osg::Matrixd local2world;
                _mapNode->getMapSRS()->createLocalToWorld(mouseMap.vec3d(), local2world);
                _dragger->setMatrix( local2world );
                aa.requestRedraw();
                return true;
            }
        }
        else if ( ea.getEventType() == ea.RELEASE )
        {
            //OE_TEST << LC << "RELEASE" << std::endl;
            if ( _dragger.valid() )
            {
                _dragger->getParent(0)->removeChild(_dragger.get());
                _dragger = 0L;
                aa.requestRedraw();
            }
            handled = false; // let others release as well
        }
    }

    return handled;
}
