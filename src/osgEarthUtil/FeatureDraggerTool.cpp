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
#include <osgEarth/Registry>
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


    struct CustomQueryPredicate : public FeatureQueryTool::InputPredicate
    {
        bool accept( const osgGA::GUIEventAdapter& ea )
        {
            return
                ea.getEventType() == ea.PUSH &&
                (ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) != 0 &&
                (ea.getModKeyMask() & ea.MODKEY_SHIFT)      != 0;
        }
    };


    struct ColorReplacer : public osg::NodeVisitor 
    {
        osg::ref_ptr<osg::Vec4Array> _colors;
        ColorReplacer(const osg::Vec4f& color) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
        {
            _colors = new osg::Vec4Array(1);
            (*_colors.get())[0] = color;
        }

        void apply(osg::Geode& geode) 
        {
            for( unsigned i=0; i<geode.getNumDrawables(); ++i )
            {
                osg::Geometry* g = geode.getDrawable(i)->asGeometry();
                if ( g )
                {
                    g->setColorArray( _colors );
                    g->setColorBinding( osg::Geometry::BIND_OVERALL );
                }
            }
            traverse( geode );
        };
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

    // cancel any existing drag first, just to be safe
    cancel();

    // extract the "selected" feature model from the scene graph.
    _drawSet = index->getDrawSet( fid );
    if ( !_drawSet.empty() )
    {
        // grab the point on the ground under the hit point and use that as the anchor.
        osg::Vec3d anchorWorld;
        anchorWorld = args._worldPoint;

        // calculate the vertical offset of the mouse's hit point from the ground
        GeoPoint hitMap;
        hitMap.fromWorld( _mapNode->getMapSRS(), args._worldPoint );
        double hae;
        _verticalOffset = 0.0;
        if (_mapNode->getTerrain()->getHeight( hitMap.x(), hitMap.y(), 0L, &hae ))
            _verticalOffset = hitMap.z() - hae;

        // NOTE: the above technique works, but has the annoying effect of "snapping" the
        // building to the mouse/terrain point on the first movement of the mouse...

        // extract the "hit" feature from its draw set into a new draggable node.
        osg::ref_ptr<osg::Node> node;

        // create a copy of the drawset's geometry for dragging:
        osg::Node* dragModel = _drawSet.createCopy();
        if ( dragModel )
        {
            // set up the dragged model's appearance:
            configureDragger( dragModel );

            // create a SECOND copy of the drawset's geometry that will act as the "ghost" model -- 
            // it will sit in its original position to "remind" the user of where the drag started.
            _ghostModel = _drawSet.createCopy();
            configureGhost( _ghostModel.get() );
            _mapNode->addChild( _ghostModel.get() );

            // create a transform that moves the feature from world coords to the local coordinate
            // system around the mouse. This will allow us to move the feature without messing around
            // with its relatively-positioned verts.
            GeoPoint anchorMap;
            anchorMap.fromWorld( _mapNode->getMapSRS(), anchorWorld );

            osg::Matrixd world2local_anchor;
            anchorMap.createWorldToLocal( world2local_anchor );

            osg::MatrixTransform* world2local_xform = new osg::MatrixTransform(world2local_anchor);
            world2local_xform->addChild( dragModel );

            // next, create the positioner matrix that goes from the local coordinates to mouse
            // world coords. This is the matrix that will change as the user drags the mouse.
            // It just starts out as the inverse of the matrix we just created.
            osg::Matrixd local2world_anchor;
            local2world_anchor.invert( world2local_anchor );

            _dragXform = new osg::MatrixTransform( local2world_anchor );
            _dragXform->addChild( world2local_xform );

            // stick it somewhere.
            _mapNode->addChild( _dragXform.get() );

            // hide the original draw set.
            _drawSet.setVisible( false );
        }
    }
}


void 
FeatureDraggerTool::onMiss( const EventArgs& args )
{
    cancel();
}


bool
FeatureDraggerTool::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    bool handled = false;

    if ( _dragXform.valid() )
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
                _dragXform->setMatrix( local2world );
                aa.requestRedraw();
                return true;
            }
        }

        else if ( ea.getEventType() == ea.RELEASE )
        {
            cancel();
            aa.requestRedraw();
            handled = true;
        }
            
        else if ( ea.getEventType() != ea.FRAME )
        {
            // capture and supress further events if drag is in progress.
            handled = true;
        }
    }

    return handled;
}


void
FeatureDraggerTool::cancel()
{
    if ( _dragXform.valid() )
    {
        if ( _dragXform->getNumParents() > 0 )
            _dragXform->getParent(0)->removeChild(_dragXform.get());
        _dragXform = 0L;
    }

    if ( _ghostModel.valid() )
    {
        if ( _ghostModel->getNumParents() > 0 )
            _ghostModel->getParent(0)->removeChild(_ghostModel.get());
        _ghostModel = 0L;
    }

    _drawSet.setVisible( true );
    _drawSet.clear();
}


osg::Node*
FeatureDraggerTool::configureDragger( osg::Node* node ) const
{
    return node;
}


osg::Node*
FeatureDraggerTool::configureGhost( osg::Node* node ) const
{
    osg::StateSet* s = node->getOrCreateStateSet();

    // note: everything here must be OVERRIDE so we override the default state of the ghost model.

    s->setRenderBinDetails( 10, "DepthSortedBin", osg::StateSet::USE_RENDERBIN_DETAILS );
    s->setMode( GL_BLEND,    osg::StateAttribute::ON  | osg::StateAttribute::OVERRIDE );
    s->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );

    // turn off texturing:
    for( int ii = 0; ii < Registry::instance()->getCapabilities().getMaxFFPTextureUnits(); ++ii )
    {
        s->setTextureMode( ii, GL_TEXTURE_2D, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
        s->setTextureMode( ii, GL_TEXTURE_3D, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
        //sset->setTextureMode( ii, GL_TEXTURE_RECTANGLE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
        //sset->setTextureMode( ii, GL_TEXTURE_CUBE_MAP, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
    }

    // make it a nice transparent color
    // careful, the ghost is a "shallow copy" of the original, so don't go modifying any buffer objects!
    // (replacing them is OK though.)
    node->accept( ColorReplacer(osg::Vec4f(0.5f, 0.5f, 1.0f, 0.35f)) );

    return node;
}
