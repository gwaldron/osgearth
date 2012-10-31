/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

#include <osgEarthAnnotation/LocalizedNode>
#include <osgEarthAnnotation/Decluttering>
#include <osgEarth/CullingUtils>
#include <osgEarth/MapNode>
#include <osg/AutoTransform>
#include <osg/MatrixTransform>

#define LC "[LocalizedNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;


LocalizedNode::LocalizedNode(MapNode*        mapNode,
                             const GeoPoint& position,
                             bool            is2D ) :
PositionedAnnotationNode( mapNode ),
_horizonCulling         ( false ),
_autoTransform          ( is2D ),
_scale                  ( 1.0f, 1.0f, 1.0f )
{
    init( mapNode, position );
}

LocalizedNode::LocalizedNode(MapNode* mapNode, const Config& conf) :
PositionedAnnotationNode(mapNode, conf),
_horizonCulling         ( false ),
_autoTransform          ( false ),
_scale                  ( 1.0f, 1.0f, 1.0f )
{
    init( mapNode, GeoPoint::INVALID );
}


void
LocalizedNode::init( MapNode* mapNode, const GeoPoint& position )
{
    if ( _autoTransform )
    {
        osg::AutoTransform* at = new osg::AutoTransform();
        at->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
        at->setAutoScaleToScreen( true );
        at->setCullingActive( false ); // just for the first pass
        _xform = at;
    }
    else
    {
        _xform = new osg::MatrixTransform();

        this->getOrCreateStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
    }
    
    //this->getOrCreateStateSet()->setMode( GL_BLEND, 1 );

    setHorizonCulling( true );

    _draper = new DrapeableNode( mapNode, false );
    _draper->addChild( _xform.get() );
    
    setPosition( position );
}

void
LocalizedNode::setMapNode( MapNode* mapNode )
{
    if ( getMapNode() != mapNode )
    {
        PositionedAnnotationNode::setMapNode( mapNode );

        // The horizon culler depends on the map node, so reinitialize it:
        if ( _horizonCulling )
        {
            setHorizonCulling( false );
            setHorizonCulling( true );
        }

        // re-apply the position since the map has changed
        setPosition( getPosition() );
    }
}

void
LocalizedNode::traverse( osg::NodeVisitor& nv )
{
    if ( _autoTransform && nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
       _xform->setCullingActive( true );
    }
    
    AnnotationNode::traverse( nv );
}

bool
LocalizedNode::setPosition( const GeoPoint& pos )
{
    if ( getMapNode() )
    {
        // first transform the point to the map's SRS:
        const SpatialReference* mapSRS = getMapNode()->getMapSRS();
        GeoPoint mapPos = mapSRS ? pos.transform(mapSRS) : pos;
        if ( !mapPos.isValid() )
            return false;

        _mapPosition = mapPos;
    }
    else
    {
        _mapPosition = pos;
    }

    // make sure the node is set up for auto-z-update if necessary:
    configureForAltitudeMode( _mapPosition.altitudeMode() );

    // update the node.
    if ( !updateTransforms( _mapPosition ) )
        return false;

    return true;
}

void
LocalizedNode::setScale( const osg::Vec3f& scale )
{
    _scale = scale;
    updateTransforms( getPosition() );
}

bool
LocalizedNode::updateTransforms( const GeoPoint& p, osg::Node* patch )
{
    if ( p.isValid() )
    {
        GeoPoint absPos(p);
        if ( !makeAbsolute(absPos, patch) )
            return false;

        OE_DEBUG << LC << "Update transforms for position: " << absPos.x() << ", " << absPos.y() << ", " << absPos.z()
            << std::endl;

        osg::Matrixd local2world;
        absPos.createLocalToWorld( local2world );
        
        // apply the local offsets
        local2world.preMult( osg::Matrix::translate(_localOffset) );

        if ( _autoTransform )
        {
            static_cast<osg::AutoTransform*>(_xform.get())->setPosition( local2world.getTrans() );
            static_cast<osg::AutoTransform*>(_xform.get())->setScale( _scale );
            static_cast<osg::AutoTransform*>(_xform.get())->setRotation( _localRotation );
        }
        else
        {
            static_cast<osg::MatrixTransform*>(_xform.get())->setMatrix( 
                osg::Matrix::scale(_scale) * 
                osg::Matrix::rotate(_localRotation) *
                local2world  );
        }

        
        CullNodeByHorizon* culler = dynamic_cast<CullNodeByHorizon*>(_xform->getCullCallback());
        if ( culler )
            culler->_world = local2world.getTrans();
    }
    else
    {
        osg::Vec3d absPos = p.vec3d() + _localOffset;

        if ( _autoTransform )
        {
            static_cast<osg::AutoTransform*>(_xform.get())->setPosition( absPos );
            static_cast<osg::AutoTransform*>(_xform.get())->setScale( _scale );
            static_cast<osg::AutoTransform*>(_xform.get())->setRotation( _localRotation );
        }
        else
        {
            static_cast<osg::MatrixTransform*>(_xform.get())->setMatrix(
                osg::Matrix::scale(_scale) * 
                osg::Matrix::rotate(_localRotation) *
                osg::Matrix::translate(absPos) );
        }
    }
    

    dirtyBound();

    return true;
}

GeoPoint
LocalizedNode::getPosition() const
{
    return _mapPosition;
}

void
LocalizedNode::setLocalOffset( const osg::Vec3d& offset )
{
    _localOffset = offset;
    setPosition( _mapPosition );
}

const osg::Vec3d&
LocalizedNode::getLocalOffset() const
{
    return _localOffset;
}

void
LocalizedNode::setLocalRotation( const osg::Quat& rotation )
{
    _localRotation = rotation;
    setPosition( _mapPosition );
}

const osg::Quat&
LocalizedNode::getLocalRotation() const
{
    return _localRotation;
}

void
LocalizedNode::setHorizonCulling( bool value )
{
    if ( _horizonCulling != value )
    {
        _horizonCulling = value;

        if ( _horizonCulling && getMapNode() && getMapNode()->isGeocentric() )
        {
            osg::Vec3d world;
            if ( _autoTransform )
                world = static_cast<osg::AutoTransform*>(_xform.get())->getPosition();
            else
                world = static_cast<osg::MatrixTransform*>(_xform.get())->getMatrix().getTrans();

            _xform->setCullCallback( new CullNodeByHorizon(world, getMapNode()->getMapSRS()->getEllipsoid()) );
        }
        else
        {
            _xform->removeCullCallback( _xform->getCullCallback() );
        }
    }
}

void
LocalizedNode::reclamp( const TileKey& key, osg::Node* tile, const Terrain* terrain )
{
    // first verify that the control position intersects the tile:
    if ( key.getExtent().contains( _mapPosition.x(), _mapPosition.y() ) )
    {
        updateTransforms(_mapPosition, tile);
    }
}
