/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
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
#include <osg/Switch>

#define LC "[OrthoNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;


OrthoNode::OrthoNode(MapNode*        mapNode,
                     const GeoPoint& position ) :

PositionedAnnotationNode( mapNode ),
_occlusionCulling       ( false ),
_horizonCullingRequested( true )
{
    init();
    setPosition( position );
}


OrthoNode::OrthoNode() :
_occlusionCulling       ( false ),
_horizonCullingRequested( true )
{
    init();
}


void
OrthoNode::init()
{
    _switch = new osg::Switch();
    this->addChild( _switch );

    _autoxform = new AnnotationUtils::OrthoNodeAutoTransform();
    _autoxform->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
    _autoxform->setAutoScaleToScreen( true );
    _autoxform->setCullingActive( false ); // for the first pass
    _switch->addChild( _autoxform );

    _matxform = new osg::MatrixTransform();
    _switch->addChild( _matxform );
    _switch->setSingleChildOn( 0 );

    _attachPoint = new osg::Group();
    _autoxform->addChild( _attachPoint );
    _matxform->addChild( _attachPoint );

    this->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );

    // Callback to cull ortho nodes that are not visible over the geocentric horizon
    _horizonCuller = new HorizonCullCallback();
    setHorizonCulling( _horizonCullingRequested );

    _attachPoint->addCullCallback( _horizonCuller.get() );
}

osg::BoundingSphere
OrthoNode::computeBound() const
{
    osg::BoundingSphere bs = PositionedAnnotationNode::computeBound();
    //OE_NOTICE << "BOUND RADIUS = " << bs.radius() << "\n";
    return bs;
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
        //if ( !declutter )
        //{
        //    static_cast<AnnotationUtils::OrthoNodeAutoTransform*>(_autoxform)->accept( nv, false );
        //}

        // turn off small feature culling
        // (note: pretty sure this does nothing here -gw)
        cv->setSmallFeatureCullingPixelSize(-1.0f);

        AnnotationNode::traverse( nv );

        if ( _autoxform->getCullingActive() == false )
        {
            _autoxform->setCullingActive( true );
            this->dirtyBound();
        }
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
        _horizonCuller->setHorizon( Horizon(*mapNode->getMapSRS()->getEllipsoid()) );
        setHorizonCulling( _horizonCullingRequested );

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
    return _horizonCullingRequested;
}

void
OrthoNode::setHorizonCulling(bool value)
{
    _horizonCullingRequested = value;

    _horizonCuller->setEnabled(
        _horizonCullingRequested &&
        getMapNode() &&
        getMapNode()->isGeocentric() );
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
