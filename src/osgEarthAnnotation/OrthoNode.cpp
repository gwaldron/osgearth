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

#include <osgEarthAnnotation/OrthoNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationSettings>
#include <osgEarthSymbology/Color>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/CullingUtils>
#include <osgEarth/MapNode>
#include <osgText/Text>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/IntersectionVisitor>
#include <osg/OcclusionQueryNode>
#include <osg/Point>
#include <osg/Depth>

#define LC "[OrthoNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;

//------------------------------------------------------------------------

namespace
{
    struct OrthoOQNode : public osg::OcclusionQueryNode
    {
        OrthoOQNode( const std::string& name ) 
        {
            setName( name );
            setVisibilityThreshold(1);
            setDebugDisplay(true);
            setCullingActive(false);
        }

        virtual osg::BoundingSphere computeBound() const
        {
            {
                // Need to make this routine thread-safe. Typically called by the update
                //   Visitor, or just after the update traversal, but could be called by
                //   an application thread or by a non-osgViewer application.
                Threading::ScopedMutexLock lock( _computeBoundMutex );

                // This is the logical place to put this code, but the method is const. Cast
                //   away constness to compute the bounding box and modify the query geometry.
                OrthoOQNode* nonConstThis = const_cast<OrthoOQNode*>( this );

                osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array(1);
                (*v)[0].set( _xform->getMatrix().getTrans() );

                osg::Geometry* geom = static_cast< osg::Geometry* >( nonConstThis->_queryGeode->getDrawable( 0 ) );
                geom->setVertexArray( v.get() );
                geom->getPrimitiveSetList().clear();
                geom->addPrimitiveSet( new osg::DrawArrays(GL_POINTS,0,1) );
                nonConstThis->getQueryStateSet()->setAttributeAndModes(new osg::Point(15), 1);
                nonConstThis->getQueryStateSet()->setBinNumber(INT_MAX);

                geom = static_cast< osg::Geometry* >( nonConstThis->_debugGeode->getDrawable( 0 ) );
                geom->setVertexArray( v.get() );
                geom->getPrimitiveSetList().clear();
                geom->addPrimitiveSet( new osg::DrawArrays(GL_POINTS,0,1) );
                nonConstThis->getDebugStateSet()->setAttributeAndModes(new osg::Point(15), 1);
                osg::Depth* d = new osg::Depth( osg::Depth::LEQUAL, 0.f, 1.f, false );
                nonConstThis->getDebugStateSet()->setAttributeAndModes( d, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
                (*dynamic_cast<osg::Vec4Array*>(geom->getColorArray()))[0].set(1,0,0,1);
            }

            return Group::computeBound();
        }

        osg::MatrixTransform* _xform;
    };
}

//------------------------------------------------------------------------


OrthoNode::OrthoNode(MapNode*        mapNode,
                     const GeoPoint& position ) :

PositionedAnnotationNode( mapNode ),
_horizonCulling         ( false ),
_occlusionCulling       ( false )
{
    init();
    setHorizonCulling( true );
    setPosition( position );
}


OrthoNode::OrthoNode() :
_horizonCulling  ( false ),
_occlusionCulling( false )
{
    init();
    setHorizonCulling( true );
}

//#define TRY_OQ 1
#undef TRY_OQ

void
OrthoNode::init()
{
    _switch = new osg::Switch();

    // install it, but deactivate it until we can get it to work.
#ifdef TRY_OQ
    OrthoOQNode* oq = new OrthoOQNode("");
    oq->setQueriesEnabled(true);
    _oq = oq;
    _oq->addChild( _switch );
    this->addChild( _oq );
#else
    this->addChild( _switch );
#endif

    //_oq->addChild( _switch );
    //this->addChild( _oq );

    _autoxform = new AnnotationUtils::OrthoNodeAutoTransform();
    _autoxform->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
    _autoxform->setAutoScaleToScreen( true );
    _autoxform->setCullingActive( false ); // for the first pass
    _switch->addChild( _autoxform );

    _matxform = new osg::MatrixTransform();
    _switch->addChild( _matxform );

#ifdef TRY_OQ
    oq->_xform = _matxform;
#endif

    _switch->setSingleChildOn( 0 );

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
        cv = Culling::asCullVisitor(nv);

        // make sure that we're NOT using the AutoTransform if this node is in the decluttering bin;
        // the decluttering bin automatically manages screen space transformation.
        bool declutter = cv->getCurrentRenderBin()->getName() == OSGEARTH_DECLUTTER_BIN;
        if ( declutter && _switch->getValue(0) == 1 )
        {
            _switch->setSingleChildOn( 1 );
        }
        else if ( !declutter && _switch->getValue(0) == 0 )
        {
            _switch->setSingleChildOn( 0 );
        }

        // If decluttering is enabled, update the auto-transform but not its children.
        // This is necessary to support picking/selection. An optimization would be to
        // disable this pass when picking is not in use
        if ( declutter )
        {
            static_cast<AnnotationUtils::OrthoNodeAutoTransform*>(_autoxform)->acceptCullNoTraverse( cv );
        }

        // turn off small feature culling
        cv->setSmallFeatureCullingPixelSize(0.0f);

        AnnotationNode::traverse( nv );

        if ( this->getCullingActive() == false )
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
        if ( static_cast<AnnotationUtils::OrthoNodeAutoTransform*>(_autoxform)->okToIntersect() )
        {
            _autoxform->accept( nv );
        }
    }

    else
    {
        AnnotationNode::traverse( nv );
    }
}

osg::BoundingSphere
OrthoNode::computeBound() const
{
    return osg::BoundingSphere(_matxform->getMatrix().getTrans(), 1000.0);
}

void
OrthoNode::setMapNode( MapNode* mapNode )
{
    MapNode* oldMapNode = getMapNode();
    if ( oldMapNode != mapNode )
    {
        PositionedAnnotationNode::setMapNode( mapNode );

        // the occlusion culler depends on the mapnode, so re-initialize it:
        if ( _occlusionCulling )
        {
            setOcclusionCulling( false );
            setOcclusionCulling( true );
        }

        // same goes for the horizon culler:
        if ( _horizonCulling || (oldMapNode == 0L && mapNode->isGeocentric()) )
        {
            setHorizonCulling( false );
            setHorizonCulling( true );
        }

        // re-apply the position since the map has changed
        setPosition( getPosition() );
    }
}

bool
OrthoNode::setPosition( const GeoPoint& position )
{
    MapNode* mapNode = getMapNode();
    if ( mapNode )
    {
        // first transform the point to the map's SRS:
        const SpatialReference* mapSRS = mapNode->getMapSRS();
        GeoPoint mapPos = mapSRS ? position.transform(mapSRS) : position;
        if ( !mapPos.isValid() )
            return false;

        _mapPosition = mapPos;
    }
    else
    {
        _mapPosition = position;
    }

    // make sure the node is set up for auto-z-update if necessary:
    configureForAltitudeMode( _mapPosition.altitudeMode() );

    // and update the node.
    if ( !updateTransforms(_mapPosition) )
        return false;

    return true;
}

void
OrthoNode::applyStyle(const Style& style)
{
    // check for decluttering.
    const TextSymbol* text = style.get<TextSymbol>();
    if ( text && text->declutter().isSet() )
    {
        Decluttering::setEnabled( this->getOrCreateStateSet(), (text->declutter() == true) );
        //if ( text->declutter() == true )
        //{
        //    this->getOrCreateStateSet()->setRenderBinDetails(
        //        0,
        //        OSGEARTH_DECLUTTER_BIN );
        //}
        //else
        //{
        //    this->getOrCreateStateSet()->setRenderBinToInherit();
        //}
    }


    // check for occlusion culling
    if ( text && text->occlusionCull().isSet() )
    {
        setOcclusionCulling( *text->occlusionCull() );

        if (text->occlusionCullAltitude().isSet())
        {
            setOcclusionCullingMaxAltitude( *text->occlusionCullAltitude() );
        }
    }

    const IconSymbol* icon = style.get<IconSymbol>();
    if ( icon && icon->declutter().isSet() )
    {
        Decluttering::setEnabled( this->getOrCreateStateSet(), (icon->declutter() == true) );
        //if ( icon->declutter() == true )
        //{
        //    this->getOrCreateStateSet()->setRenderBinDetails(
        //        0,
        //        OSGEARTH_DECLUTTER_BIN );
        //}
        //else
        //{
        //    this->getOrCreateStateSet()->setRenderBinToInherit();
        //}
    }

    // check for occlusion culling
    if ( icon && icon->occlusionCull().isSet() )
    {
        this->setOcclusionCulling( *icon->occlusionCull() );

        if (icon->occlusionCullAltitude().isSet())
        {
            setOcclusionCullingMaxAltitude( *icon->occlusionCullAltitude() );
        }
    }

    // up the chain
    PositionedAnnotationNode::applyStyle( style );
}

bool
OrthoNode::updateTransforms( const GeoPoint& p, osg::Node* patch )
{
    if ( getMapNode() )
    {
        //OE_NOTICE << "updateTransforms" << std::endl;
        // make sure the point is absolute to terrain
        GeoPoint absPos(p);
        if ( !makeAbsolute(absPos, patch) )
            return false;

        osg::Matrixd local2world;
        if ( !absPos.createLocalToWorld(local2world) )
            return false;

        // apply the local tangent plane offset:
        local2world.preMult( osg::Matrix::translate(_localOffset) );

        // update the xforms:
        _autoxform->setPosition( local2world.getTrans() );
        _matxform->setMatrix( local2world );
        
        osg::Vec3d world = local2world.getTrans();
        if (_horizonCuller.valid())
        {
            _horizonCuller->_world = world;
        }

        if (_occlusionCuller.valid())
        {                                
            _occlusionCuller->setWorld( adjustOcclusionCullingPoint( world ));
        } 
    }
    else
    {
        osg::Vec3d absPos = p.vec3d() + _localOffset;
        _autoxform->setPosition( absPos );
        _matxform->setMatrix( osg::Matrix::translate(absPos) );
    }

    dirtyBound();
    return true;
}

GeoPoint
OrthoNode::getPosition() const
{
    return _mapPosition;
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

bool
OrthoNode::getHorizonCulling() const
{
    return _horizonCulling;
}

void
OrthoNode::setHorizonCulling( bool value )
{
    if ( _horizonCulling != value )
    {
        _horizonCulling = value;

        if ( _horizonCulling && getMapNode() && getMapNode()->isGeocentric() )
        {
            osg::Vec3d world = _autoxform->getPosition();

            _horizonCuller = new CullNodeByHorizon(world, getMapNode()->getMapSRS()->getEllipsoid());
            addCullCallback( _horizonCuller.get()  );
        }
        else
        {
            if (_horizonCuller.valid())
            {
                removeCullCallback( _horizonCuller.get() );
                _horizonCuller = 0;
            }
        }
    }
}

osg::Vec3d
OrthoNode::adjustOcclusionCullingPoint( const osg::Vec3d& world )
{
    // Adjust the height by a little bit "up", we can't have the occlusion point sitting right on the ground
    if ( getMapNode() )
    {
        const osg::EllipsoidModel* em = getMapNode()->getMapSRS()->getEllipsoid();
        osg::Vec3d up = em ? em->computeLocalUpVector( world.x(), world.y(), world.z() ) : osg::Vec3d(0,0,1);
        osg::Vec3d adjust = up * AnnotationSettings::getOcclusionCullingHeightAdjustment();        
        return world + adjust;
    }
    else
    {
        return world;
    }
}

bool
OrthoNode::getOcclusionCulling() const
{
    return _occlusionCulling;
}

void
OrthoNode::setOcclusionCulling( bool value )
{
    if (_occlusionCulling != value)
    {
        _occlusionCulling = value;

        if ( _occlusionCulling && getMapNode() )
        {
            osg::Vec3d world = _autoxform->getPosition();
            _occlusionCuller = new OcclusionCullingCallback( getMapNode()->getMapSRS(),  adjustOcclusionCullingPoint(world), getMapNode()->getTerrainEngine() );
            _occlusionCuller->setMaxAltitude( getOcclusionCullingMaxAltitude() );
            addCullCallback( _occlusionCuller.get()  );
        }
        else
        {
            if (_occlusionCulling)
            {
                if (_occlusionCuller.valid())
                {
                    removeCullCallback( _occlusionCuller.get() );
                    _occlusionCuller = 0;
                }
            }
        }
    }
}

double
OrthoNode::getOcclusionCullingMaxAltitude() const
{
    if (_occlusionCullingMaxAltitude.isSet())
    {
        return *_occlusionCullingMaxAltitude;
    }
    return AnnotationSettings::getOcclusionCullingMaxAltitude();
}

void OrthoNode::setOcclusionCullingMaxAltitude( double occlusionCullingMaxAltitude )
{
    _occlusionCullingMaxAltitude = occlusionCullingMaxAltitude;
    if ( _occlusionCuller.valid() )
    {
        _occlusionCuller->setMaxAltitude( getOcclusionCullingMaxAltitude() );         
    }
}

void
OrthoNode::reclamp( const TileKey& key, osg::Node* tile, const Terrain* terrain )
{
    // first verify that the label position intersects the tile:
    if ( key.getExtent().contains( _mapPosition.x(), _mapPosition.y() ) )
    {
        updateTransforms( _mapPosition, tile );
    }
}
