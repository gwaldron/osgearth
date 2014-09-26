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

#include <osgEarthUtil/FeatureManipTool>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/AnnotationEditing>
#include <osgEarth/ECEF>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/GeoMath>
#include <osgViewer/View>
#include <osg/Depth>

#define LC "[FeatureManipTool] "

#define OE_TEST OE_NULL

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

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


    // input predicate for the query tool that specifies what action
    // should trigger a drag.
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


    // visits a scene graph and replaces the color array on each Geometry with
    // a specified overall color.
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

    
    // Dragger callback to simply hooks back into the DraggerTool.
    struct DraggerCallback : public Dragger::PositionChangedCallback
    {
        DraggerCallback( FeatureManipTool* tool, bool isVertical=false ) : _tool(tool), _isVertical(isVertical) { }

        void onPositionChanged(const Dragger* sender, const osgEarth::GeoPoint& pos)
        {
            _tool->syncToDraggers(_isVertical);
        }

        FeatureManipTool* _tool;
        bool _isVertical;
    };

    // updates the verts in a subgraph based on a pair of re-positioning transforms.
    struct VertexMover : public osg::NodeVisitor
    {
        VertexMover(const osg::Matrixd& l2w_orig, const osg::Matrix&  w2l_new)
            : _local2world0(l2w_orig), _world2local1(w2l_new), osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) { }

        void apply(osg::Geode& geode)
        {
            for( unsigned i=0; i<geode.getNumDrawables(); ++i )
            {
                osg::Geometry* g = geode.getDrawable(i)->asGeometry();
                if ( g )
                {
                    osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>( g->getVertexArray() );
                    for( osg::Vec3Array::iterator i = verts->begin(); i != verts->end(); ++i )
                    {
                        osg::Vec3d vert3d = *i;
                        osg::Vec3d vertWorld0 = vert3d * _local2world0;
                        osg::Vec3d vertLocal1 = vertWorld0 * _world2local1;
                        *i = vertLocal1;
                    }
                }
            }
            traverse( geode );
        }

        osg::Matrix _local2world0, _world2local1;
    };
}

//-----------------------------------------------------------------------

FeatureManipTool::FeatureManipTool(MapNode* mapNode, bool verticalEnabled) :
FeatureQueryTool( mapNode ),
_verticalEnabled(verticalEnabled),
_verticalDraggerOffset(0.0)
{
    // install this object as a query callback so we will receive messages.
    this->addCallback( this );

    // a custom input predicate that triggers manip mode
    this->setInputPredicate( new CustomQueryPredicate );
}


void 
FeatureManipTool::onHit( FeatureSourceIndexNode* index, FeatureID fid, const EventArgs& args )
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

        // extract the "hit" feature from its draw set into a new draggable node.
        osg::ref_ptr<osg::Node> node;

        // create a copy of the drawset's geometry for dragging:
        osg::Node* manipModel = _drawSet.createCopy();
        if ( manipModel )
        {
            _workGroup = new osg::Group();
            _mapNode->addChild( _workGroup.get() );

            anchorWorld = manipModel->getBound().center();
            
            // calculate the vertical offset of the anchor point from the ground
            GeoPoint anchorMapCenter;
            anchorMapCenter.fromWorld( _mapNode->getMapSRS(), anchorWorld );
            _verticalOffset = anchorMapCenter.z();

            GeoPoint anchorMap(anchorMapCenter);
            anchorMap.z() = 0;
            anchorMap.altitudeMode() = ALTMODE_RELATIVE;
            anchorMap.transformZ( ALTMODE_ABSOLUTE, _mapNode->getTerrain() );

            _verticalOffset = anchorMapCenter.z() - anchorMap.z();

            anchorMap.toWorld( anchorWorld );

            // set up the dragged model's appearance:
            configureManip( manipModel );

            // create a SECOND copy of the drawset's geometry that will act as the "ghost" model -- 
            // it will sit in its original position to "remind" the user of where the drag started.
            _ghostModel = _drawSet.createCopy();
            configureGhost( _ghostModel.get() );

            // create a transform that moves the feature from world coords to the local coordinate
            // system around the mouse. This will allow us to move the feature without messing around
            // with its relatively-positioned verts.
            osg::Matrixd world2local_anchor;
            anchorMapCenter.createWorldToLocal( world2local_anchor );

            osg::MatrixTransform* world2local_xform = new osg::MatrixTransform(world2local_anchor);
            world2local_xform->addChild( manipModel );

            // next, create the positioner matrix that goes from the local coordinates to mouse
            // world coords. This is the matrix that will change as the user drags the mouse.
            // It just starts out as the inverse of the matrix we just created.
            osg::Matrixd local2world_anchor;
            local2world_anchor.invert( world2local_anchor );

            _manipModel = new osg::MatrixTransform( local2world_anchor );
            _manipModel->addChild( world2local_xform );

            // hide the original draw set.
            _drawSet.setVisible( false );

            // make a circle annotation that we will use to edit the feature's position and rotation:
            Style circleStyle;
            circleStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Yellow, 0.25);
            circleStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::White;
            const osg::BoundingSphere& bs = manipModel->getBound();
            _circle = new CircleNode( getMapNode(), anchorMap, Distance(bs.radius()*1.5), circleStyle );
            _circle->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS,0,1,false) );

            _circleEditor = new CircleNodeEditor( _circle.get() );
            _circleEditor->getPositionDragger()->addPositionChangedCallback( new DraggerCallback(this) );
            _circleEditor->getRadiusDragger()->addPositionChangedCallback( new DraggerCallback(this) );
            _circleEditor->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS,0,1,false) );

            // add an additional dragger for vertical dragging
            if (_verticalEnabled)
            {
                _verticalDragger  = new SphereDragger( getMapNode() );
                _verticalDragger->setDefaultDragMode(Dragger::DRAGMODE_VERTICAL);
                _verticalDragger->setColor(osg::Vec4f(0.0f, 1.0f, 1.0f, 1.0f));
                _verticalDragger->setPickColor(osg::Vec4f(1.0f, 0.0f, 1.0f, 1.0f));

                _verticalDraggerOffset = bs.radius() * 1.1;
                GeoPoint verticalDraggerPos(anchorMapCenter);
                verticalDraggerPos.z() += _verticalDraggerOffset;
                _verticalDragger->setPosition(verticalDraggerPos, false);
                _verticalDragger->setVerticalMinimum(_verticalDraggerOffset);

                _verticalDragger->addPositionChangedCallback( new DraggerCallback(this, true) );
            }

            // micro-manage the render order to get things just right:
            _workGroup->getOrCreateStateSet()->setRenderBinDetails( 15, "TraversalOrderBin" );
            _workGroup->addChild( _circle.get() );
            _workGroup->addChild( _manipModel.get() );
            _workGroup->addChild( _ghostModel.get() );
            _workGroup->addChild( _circleEditor.get() );

            if (_verticalDragger.valid())
                _workGroup->addChild( _verticalDragger.get() );
        }
    }
}


void 
FeatureManipTool::onMiss( const EventArgs& args )
{
    cancel();
}


bool 
FeatureManipTool::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    bool handled = FeatureQueryTool::handle( ea, aa );
    if ( !handled )
    {
        if ( _workGroup.valid() && ea.getEventType() == ea.KEYDOWN && ea.getKey() == ea.KEY_C )
        {
            commit();
            handled = true;
        }
    }
    return handled;
}


void
FeatureManipTool::setPosition( const GeoPoint& pos )
{
    if ( _circleEditor.valid() )
    {
        _circleEditor->setPosition( pos );
        syncToDraggers();
    }
}


void
FeatureManipTool::setRotation( const Angle& rot )
{
    if ( _circleEditor.valid() )
    {
        _circleEditor->setBearing( rot );
        syncToDraggers();
    }
}


void
FeatureManipTool::syncToDraggers(bool wasVertical)
{
    // position the feature based on the circle annotation's draggers:
    GeoPoint pos = _circleEditor->getPositionDragger()->getPosition();
    GeoPoint rad = _circleEditor->getRadiusDragger()->getPosition();

    // if the vertical dragger was moved, update the vertical offset
    if (wasVertical)
      _verticalOffset = _verticalDragger->getPosition().z() - _verticalDraggerOffset - pos.z();

    GeoPoint vPos(pos);
    vPos.z() += _verticalOffset;

    // update the vertical draggers (horizontal) position 
    if (_verticalEnabled)
    {
      GeoPoint vdPos(vPos);
      vdPos.z() += _verticalDraggerOffset;
      _verticalDragger->setPosition(vdPos, false);
    }
    
    pos.makeGeographic();
    rad.makeGeographic();

    double bearing = GeoMath::bearing( 
        osg::DegreesToRadians(pos.y()), osg::DegreesToRadians(pos.x()),
        osg::DegreesToRadians(rad.y()), osg::DegreesToRadians(rad.x()) );

    osg::Matrixd local2world;
    vPos.createLocalToWorld( local2world );

    // rotate the feature:
    osg::Quat rot( osg::PI_2-bearing, osg::Vec3d(0,0,1) );
    local2world.preMultRotate( rot );

    // move the feature:
    _manipModel->setMatrix(local2world);

    // only show the ghost when the ghost and dragger are sufficiently separated.
    _ghostModel->setNodeMask( _manipModel->getBound().intersects( _ghostModel->getBound() ) ? 0 : ~0 );
}


void
FeatureManipTool::cancel()
{
    if ( _workGroup.valid() )
    {
        if ( _workGroup->getNumParents() > 0 )
            _workGroup->getParent(0)->removeChild(_workGroup.get());
        _workGroup = 0L;
    }

    _manipModel   = 0L;
    _ghostModel   = 0L;
    _circle       = 0L;
    _circleEditor = 0L;

    _drawSet.setVisible( true );
    _drawSet.clear();
}


void
FeatureManipTool::commit()
{
    if ( _workGroup.valid() )
    {
        // extract the manipulation matricies:
        osg::MatrixTransform* xform1 = _manipModel.get();
        osg::MatrixTransform* xform2 = dynamic_cast<osg::MatrixTransform*>( _manipModel->getChild(0) );

        const osg::Matrixd& world2local_anchor = xform2->getMatrix();
        const osg::Matrix&  local2world_move   = xform1->getMatrix();

        // go through the draw set and update the verts based on the new location
        for( FeatureDrawSet::DrawableSlices::iterator s = _drawSet.slices().begin(); s != _drawSet.slices().end(); ++s )
        {
            const FeatureDrawSet::DrawableSlice& slice = *s;
            
            // collection a set of indicies in the slice:
            std::set<unsigned> indexSet;
            _drawSet.collectPrimitiveIndexSet( slice, indexSet );

            // need the inverse of our original L2W so we can reposition the verts:
            osg::Matrixd world2local;
            world2local.invert( slice.local2world );

            // recalculate the verts in their local reference frame:
            osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>( slice.drawable->asGeometry()->getVertexArray() );
            for( std::set<unsigned>::iterator i = indexSet.begin(); i != indexSet.end(); ++i )
            {
                osg::Vec3d vert3d = (*verts)[*i];
                osg::Vec3d vertWorld = (((vert3d * slice.local2world) * world2local_anchor) * local2world_move);
                osg::Vec3d vertLocalNew = vertWorld * world2local;
                (*verts)[*i] = vertLocalNew;
            }
            verts->dirty();
        }
    }

    cancel();
}


osg::Node*
FeatureManipTool::configureManip( osg::Node* node ) const
{
    //nop
    return node;
}


osg::Node*
FeatureManipTool::configureGhost( osg::Node* node ) const
{
    osg::StateSet* s = node->getOrCreateStateSet();

    // note: everything here must be OVERRIDE so we override the default state of the ghost model.

    //s->setRenderBinDetails( 10, "DepthSortedBin", osg::StateSet::USE_RENDERBIN_DETAILS );
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
    ColorReplacer replacer(osg::Vec4f(0.5f, 0.5f, 1.0f, 0.35f));
    node->accept( replacer );

    return node;
}
