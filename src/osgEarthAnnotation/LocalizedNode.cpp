/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarth/ClampableNode>
#include <osgEarth/DrapeableNode>
#include <osgEarth/CullingUtils>
#include <osgEarth/MapNode>
#include <osg/MatrixTransform>

#define LC "[LocalizedNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;


LocalizedNode::LocalizedNode(MapNode*        mapNode,
                             const GeoPoint& position) :
PositionedAnnotationNode( mapNode ),
_initComplete           ( false ),
_horizonCulling         ( true ),
_scale                  ( 1.0f, 1.0f, 1.0f ),
_mapPosition            ( position )
{
    init();
}

LocalizedNode::LocalizedNode(MapNode* mapNode, const Config& conf) :
PositionedAnnotationNode( mapNode, conf ),
_initComplete           ( false ),
_horizonCulling         ( true ),
_scale                  ( 1.0f, 1.0f, 1.0f )
{
    init();
}


void
LocalizedNode::init()
{
    this->getOrCreateStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
}


namespace
{
    static Threading::Mutex s_initCompleteMutex;
}

osg::BoundingSphere
LocalizedNode::computeBound() const
{
    if ( !_initComplete )
    {
        // perform initialization that cannot happen in the CTOR
        // (due to possible virtual function calls)
        Threading::ScopedMutexLock lock(s_initCompleteMutex);
        if ( !_initComplete )
        {
            const_cast<LocalizedNode*>(this)->_initComplete = true;
            const_cast<LocalizedNode*>(this)->setHorizonCulling( _horizonCulling );
            const_cast<LocalizedNode*>(this)->setPosition      ( _mapPosition );
        }
    }
    return PositionedAnnotationNode::computeBound();
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
        setPosition( _mapPosition );
    }
}


bool
LocalizedNode::setPosition( const GeoPoint& pos )
{
    if ( _initComplete )
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
        return updateTransform( _mapPosition );
    }
    else
    {
        _mapPosition = pos;
        return true;
    }
}

bool
LocalizedNode::updateTransform( const GeoPoint& p, osg::Node* patch )
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

        getTransform()->setMatrix( 
            osg::Matrix::scale (_scale)         * 
            osg::Matrix::rotate(_localRotation) *
            local2world  );
    }
    else
    {
        osg::Vec3d absPos = p.vec3d() + _localOffset;

        getTransform()->setMatrix(
            osg::Matrix::scale    (_scale)         * 
            osg::Matrix::rotate   (_localRotation) *
            osg::Matrix::translate(absPos) );
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
LocalizedNode::setScale( const osg::Vec3f& scale )
{
    _scale = scale;
    updateTransform( _mapPosition );
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
    _horizonCulling = value;

    if ( _initComplete && getMapNode() && getMapNode()->isGeocentric() )
    {
        if ( _horizonCulling && !_cullByHorizonCB.valid() )
        {
            _cullByHorizonCB = new CullNodeByHorizon(
                getTransform(),
                getMapNode()->getMapSRS()->getEllipsoid() );

            addCullCallback( _cullByHorizonCB.get() );
        }

        else if ( !_horizonCulling && _cullByHorizonCB.valid() )
        {
            removeCullCallback( _cullByHorizonCB.get() );
            _cullByHorizonCB = 0L;
        }
    }
}

void
LocalizedNode::reclamp( const TileKey& key, osg::Node* tile, const Terrain* terrain )
{
    // first verify that the control position intersects the tile:
    if ( key.getExtent().contains( _mapPosition.x(), _mapPosition.y() ) )
    {
        updateTransform(_mapPosition, tile);
    }
}


osg::Node*
LocalizedNode::applyAltitudePolicy(osg::Node* node, const Style& style)
{
    AnnotationUtils::AltitudePolicy ap;
    AnnotationUtils::getAltitudePolicy( style, ap );

    // Draped (projected) geometry
    if ( ap.draping )
    {
        DrapeableNode* drapable = new DrapeableNode( getMapNode() );
        drapable->addChild( node );
        node = drapable;
    }

    // gw - not sure whether is makes sense to support this for LocalizedNode
    // GPU-clamped geometry
    else if ( ap.gpuClamping )
    {
        ClampableNode* clampable = new ClampableNode( getMapNode() );
        clampable->addChild( node );
        node = clampable;

        const RenderSymbol* render = style.get<RenderSymbol>();
        if ( render && render->depthOffset().isSet() )
        {
            clampable->setDepthOffsetOptions( *render->depthOffset() );
        }
    }

    // scenegraph-clamped geometry
    else if ( ap.sceneClamping )
    {
        // save for later when we need to reclamp the mesh on the CPU
        _altitude = style.get<AltitudeSymbol>();

        // activate the terrain callback:
        setCPUAutoClamping( true );
    }

    return node;
}
