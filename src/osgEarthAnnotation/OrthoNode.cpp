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

#include <osgEarthAnnotation/OrthoNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/Decluttering>
#include <osgEarthSymbology/Color>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Utils>
#include <osgEarth/MapNode>
#include <osgText/Text>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/IntersectionVisitor>

using namespace osgEarth;
using namespace osgEarth::Annotation;


OrthoNode::OrthoNode(MapNode*          mapNode,
                     const osg::Vec3d& pos ) :

PositionedAnnotationNode( mapNode ),
_mapSRS                 ( mapNode ? mapNode->getMapSRS() : 0L ),
_horizonCulling         ( false )
{
    init();
    if ( _mapSRS.valid() )
    {
        setHorizonCulling( true );
    }
    setPosition( pos );
}

OrthoNode::OrthoNode(const SpatialReference* mapSRS,
                     const osg::Vec3d&       pos) :

_mapSRS        ( mapSRS ),
_horizonCulling( false )
{
    init();
    if ( _mapSRS.valid() )
    {
        setHorizonCulling( true );
    }
    setPosition( pos );
}

OrthoNode::OrthoNode() :
_mapSRS        ( 0L ),
_horizonCulling( false )
{
    init();
}

void
OrthoNode::init()
{
    _autoxform = new AnnotationUtils::OrthoNodeAutoTransform();
    _autoxform->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
    _autoxform->setAutoScaleToScreen( true );
    _autoxform->setCullingActive( false ); // for the first pass
    this->addChild( _autoxform );

    _matxform = new osg::MatrixTransform();
    this->addChild( _matxform );

    this->setSingleChildOn( 0 );

    _attachPoint = new osg::Group();
    _autoxform->addChild( _attachPoint );
    _matxform->addChild( _attachPoint );

    this->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
}

void
OrthoNode::traverse( osg::NodeVisitor& nv )
{
    osgUtil::CullVisitor* cv = 0L;

    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        cv = static_cast<osgUtil::CullVisitor*>( &nv );

        // make sure that we're NOT using the AutoTransform if this node is in the decluttering bin;
        // the decluttering bin automatically manages screen space transformation.
        bool declutter = cv->getCurrentRenderBin()->getName() == OSGEARTH_DECLUTTER_BIN;
        if ( declutter && getValue(0) == 1 )
        {
            this->setSingleChildOn( 1 );
        }
        else if ( !declutter && getValue(0) == 0 )
        {
            this->setSingleChildOn( 0 );
        }

        // If decluttering is enabled, update the auto-transform but not its children.
        // This is necessary to support picking/selection. An optimization would be to
        // disable this pass when picking is not in use
        if ( declutter )
            static_cast<AnnotationUtils::OrthoNodeAutoTransform*>(_autoxform)->acceptCullNoTraverse( cv );

        // turn off small feature culling
        cv->setSmallFeatureCullingPixelSize(0.0f);

        AnnotationNode::traverse( nv );

        this->setCullingActive( true );
    }
    
    // For an intersection visitor, ALWAYS traverse the autoxform instead of the 
    // matrix transform. The matrix transform is only used in combination with the 
    // decluttering engine, so it cannot properly support picking of decluttered
    // objects
    else if ( 
        nv.getVisitorType() == osg::NodeVisitor::NODE_VISITOR &&
        dynamic_cast<osgUtil::IntersectionVisitor*>( &nv ) )
    {
        _autoxform->accept( nv );
    }

    else
    {
        AnnotationNode::traverse( nv );
    }
}

bool
OrthoNode::setPosition( const osg::Vec3d& pos )
{
    return setPosition( pos, 0L );
}

bool
OrthoNode::setPosition( const osg::Vec3d& pos, const SpatialReference* posSRS )
{
    // first transform the point to the map's SRS:
    osg::Vec3d mapPos = pos;
    if ( posSRS && _mapSRS.valid() && !posSRS->transform(pos, _mapSRS.get(), mapPos) )
        return false;

    // clamp if necessary:
    if ( _autoclamp )
        clamp( mapPos );

    CullNodeByHorizon* culler = dynamic_cast<CullNodeByHorizon*>(this->getCullCallback());

    // update the transform:
    if ( !_mapSRS.valid() )
    {
        _autoxform->setPosition( mapPos + _localOffset );
        _matxform->setMatrix( osg::Matrix::translate(mapPos + _localOffset) );
    }
    else
    {
        osg::Matrixd local2world;
        _mapSRS->createLocal2World( mapPos, local2world );
        local2world.preMult( osg::Matrix::translate(_localOffset) );
        _autoxform->setPosition( local2world.getTrans() );
        _matxform->setMatrix( local2world );
        
        if ( culler )
            culler->_world = local2world.getTrans();
    }

    _mapPosition = mapPos;

    return true;
}

osg::Vec3d
OrthoNode::getPosition() const
{
    return _mapPosition;
    //return getPosition( 0L );
}

osg::Vec3d
OrthoNode::getPosition( const SpatialReference* srs ) const
{
    if ( _mapSRS.valid() && srs && !_mapSRS->isEquivalentTo( srs ) )
    {
        osg::Vec3d output;
        _mapSRS->transform( _mapPosition, srs, output );
        return output;
    }
    else
    {
        return _mapPosition;
    }

#if 0
    osg::Vec3d world = _position; //_autoxform->getPosition();
    if ( _mapSRS.valid() )
    {
        osg::Vec3d output = world;
        if ( _mapSRS->isGeographic() )
            _mapSRS->transformFromECEF( world, output );
    
        if ( srs )
            _mapSRS->transform( output, srs, output );

        return output;
    }
    else
    {
        return world;
    }
#endif
}

void
OrthoNode::setLocalOffset( const osg::Vec3d& offset )
{
    _localOffset = offset;
    setPosition( _mapPosition );
}

const osg::Vec3d&
OrthoNode::getLocalOffset() const
{
    return _localOffset;
}

void
OrthoNode::setHorizonCulling( bool value )
{
    if ( _horizonCulling != value && _mapSRS.valid() )
    {
        _horizonCulling = value;

        if ( _horizonCulling )
        {
            osg::Vec3d world = _autoxform->getPosition();
            this->setCullCallback( new CullNodeByHorizon(world, _mapSRS->getEllipsoid()) );
        }
        else
        {
            this->removeCullCallback( this->getCullCallback() );
        }
    }
}

void
OrthoNode::reclamp( const TileKey& key, osg::Node* tile, const Terrain* terrain )
{
    // first verify that the label position intersects the tile:
    osg::Vec3d mapPos = getPosition();
    if ( !key.getExtent().contains( mapPos.x(), mapPos.y() ) )
        return;

    // if clamping is on, this will automatically work
    setPosition(mapPos);
}

