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

#include <osgEarthAnnotation/LocalizedNode>
#include <osgEarthAnnotation/Decluttering>
#include <osgEarth/Utils>
#include <osg/MatrixTransform>
#include <osg/AutoTransform>

using namespace osgEarth;
using namespace osgEarth::Annotation;


LocalizedNode::LocalizedNode(const SpatialReference* mapSRS,
                             const osg::Vec3d&       pos,
                             bool                    is2D ) :
_mapSRS        ( mapSRS ),
_horizonCulling( false ),
_autoTransform ( is2D )
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
    }
    _xform->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );

    if ( mapSRS )
    {
        setHorizonCulling( true );
    }
    
    setPosition( pos );
}

void
LocalizedNode::traverse( osg::NodeVisitor& nv )
{
    if ( _autoTransform && nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
       _xform->setCullingActive( true );
    }
    osg::Group::traverse( nv );
}

bool
LocalizedNode::setPosition( const osg::Vec3d& pos )
{
    return setPosition( pos, 0L );
}

bool
LocalizedNode::setPosition( const osg::Vec3d& pos, const SpatialReference* posSRS )
{
    // first transform the point to the map's SRS:
    osg::Vec3d mapPos = pos;
    if ( posSRS && _mapSRS.valid() && !posSRS->transform(pos, _mapSRS.get(), mapPos) )
        return false;

    CullNodeByHorizon* culler = dynamic_cast<CullNodeByHorizon*>(_xform->getCullCallback());

    // update the transform:
    if ( !_mapSRS.valid() )
    {
        if ( _autoTransform )
            static_cast<osg::AutoTransform*>(_xform.get())->setPosition( mapPos );
        else
            static_cast<osg::MatrixTransform*>(_xform.get())->setMatrix( osg::Matrix::translate(pos) );
    }
    else
    {
        osg::Matrixd local2world;
        _mapSRS->createLocal2World( mapPos, local2world );

        if ( _autoTransform )
            static_cast<osg::AutoTransform*>(_xform.get())->setPosition( local2world.getTrans() );
        else
            static_cast<osg::MatrixTransform*>(_xform.get())->setMatrix( local2world );
        
        if ( culler )
            culler->_world = local2world.getTrans();
    }

    return true;
}

osg::Vec3d
LocalizedNode::getPosition() const
{
    return getPosition( 0L );
}

osg::Vec3d
LocalizedNode::getPosition( const SpatialReference* srs ) const
{
    osg::Vec3d world;
    if ( _autoTransform )
        world = static_cast<osg::AutoTransform*>(_xform.get())->getPosition();
    else
        world = static_cast<osg::MatrixTransform*>(_xform.get())->getMatrix().getTrans();

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
}

void
LocalizedNode::setHorizonCulling( bool value )
{
    if ( _horizonCulling != value && _mapSRS.valid() )
    {
        _horizonCulling = value;

        if ( _horizonCulling )
        {
            osg::Vec3d world;
            if ( _autoTransform )
                world = static_cast<osg::AutoTransform*>(_xform.get())->getPosition();
            else
                world = static_cast<osg::MatrixTransform*>(_xform.get())->getMatrix().getTrans();

            _xform->setCullCallback( new CullNodeByHorizon(world, _mapSRS->getEllipsoid()) );
        }
        else
        {
            _xform->removeCullCallback( _xform->getCullCallback() );
        }
    }
}

//------------------------------------------------------------------------


OrthoNode::OrthoNode(const SpatialReference* mapSRS,
                     const osg::Vec3d&       pos ) :
_mapSRS        ( mapSRS ),
_horizonCulling( false )
{
    _autoxform = new osg::AutoTransform();
    _autoxform->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
    _autoxform->setAutoScaleToScreen( true );
    _autoxform->setCullingActive( false ); // for the first pass
    this->addChild( _autoxform.get() );

    _matxform = new osg::MatrixTransform();
    this->addChild( _matxform.get() );

    this->setSingleChildOn( 0 );

    this->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );

    if ( mapSRS )
    {
        setHorizonCulling( true );
    }

    setPosition( pos );
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

        // turn off small feature culling
        cv->setSmallFeatureCullingPixelSize(0.0f);
    }

    for( unsigned pos = 0; pos < _children.size(); ++pos )
    {
        if ( _values[pos] )
            _children[pos]->accept( nv );
    }

    if ( cv )
    {
        this->setCullingActive( true );
    }
}

void
OrthoNode::attach( osg::Node* child )
{
    _autoxform->addChild( child );
    _matxform->addChild( child );
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

    CullNodeByHorizon* culler = dynamic_cast<CullNodeByHorizon*>(this->getCullCallback());

    // update the transform:
    if ( !_mapSRS.valid() )
    {
        _autoxform->setPosition( mapPos );
        _matxform->setMatrix( osg::Matrix::translate(pos) );
    }
    else
    {
        osg::Matrixd local2world;
        _mapSRS->createLocal2World( mapPos, local2world );
        _autoxform->setPosition( local2world.getTrans() );
        _matxform->setMatrix( local2world );
        
        if ( culler )
            culler->_world = local2world.getTrans();
    }

    return true;
}

osg::Vec3d
OrthoNode::getPosition() const
{
    return getPosition( 0L );
}

osg::Vec3d
OrthoNode::getPosition( const SpatialReference* srs ) const
{
    osg::Vec3d world = _autoxform->getPosition();
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
OrthoNode::setAnnotationData( AnnotationData* data )
{
    _annoData = data;
}
